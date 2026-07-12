

#include "pyro_wl_chassis.h"
#include "pyro_dwt_drv.h"
#include "pyro_algo_common.h"
#include "pyro_vofa.h"
#include "pyro_referee.h"
#include <cstring>


#define WHEEL_DISTANCE 0.424f
#define SUPPORT_FORCE_ACC_LPF_RC 0.01f
#define IMU_OFFSET_X  0.2f

namespace pyro
{
float time;
float last_time;

extern referee_drv_t *referee_drv;




wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis", 0, 2048)
{
    memset(&_ctx.data, 0, sizeof(wl_chassis_data_t));
}


void wl_chassis_t::configure(const wl_chassis_deps_t& deps)
{
    _ctx.deps = deps;
}


status_t wl_chassis_t::_init()
{
    auto& data = _ctx.data;
    const auto& deps = _ctx.deps;

   
    if (!deps.kinematic_solver || !deps.power_ctrl || !deps.ins_drv) return PYRO_PARAM_ERROR;
    
 
    memcpy(data.lqr_cof, deps.lqr_coef, sizeof(float) * 48);
    memcpy(data.lqr_cof_over_step, deps.lqr_coef_over_step, sizeof(float) * 48);

    // 初始化动力学求解器
    data.yaw = data.pitch = data.roll = 0.0f;
    data.g_yaw = data.g_pitch = data.g_roll = 0.0f;
    data.a_x = data.a_y = data.a_z = 0.0f;
    data.a_forward = data.a_upward = data.a_upward_lpf = 0.0f;

    //  VMC 转换矩阵初始化
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_init_f32(&data.leg_data[i].T_mat, 2, 2, data.leg_data[i].T_mat_val);
    }

    
    _fsm.init(this, &_state_passive);
    return PYRO_OK;
}

/* ========================================================================== */
/*                   Block 2: 传感器反馈与正运动学解算 (Feedback)                 */
/* ========================================================================== */


void wl_chassis_t::_update_feedback()
{
    auto& data = _ctx.data;
    const auto& deps = _ctx.deps;

    // 同步裁判系统功率
    data.power_data.limit = referee_drv->get_data().robot_status.chassis_power_limit;
    last_time = dwt_drv_t::get_timeline_ms();
   
    supercap_drv_t::cap_feedback_t cap_feedback = supercap_drv_t::get_instance()->get_feedback();
    data.power_data.chassis_power = cap_feedback.chassis_power_cap / 100.0f; 
    data.power_data.cap_power = cap_feedback.cap_power_cap / 100.0f - 250; 
    data.power_data.voltage = cap_feedback.vot_cap / 100.0f; 
    data.power_data.buffer_energy = referee_drv->get_data().power_heat.buffer_energy;

    //  超级电容的命令帧
    data.supercap_cmd.power_referee = 0;
    data.supercap_cmd.power_limit_referee = data.power_data.limit;
    data.supercap_cmd.power_buffer_limit_referee = 60.0f;
    data.supercap_cmd.power_buffer_referee = data.power_data.buffer_energy;
    data.supercap_cmd.kill_chassis_user = 0;
    data.supercap_cmd.speed_up_user_now = 0;

    static uint32_t dwt_cnt;
    static float last_dx[2];

    // imu 数据读取
    if(deps._ins_drv)
    {
        deps._ins_drv->get_rads_b(&data.yaw, &data.pitch, &data.roll);
        deps._ins_drv->get_gyro_b(&data.g_yaw, &data.g_pitch, &data.g_roll);
        deps._ins_drv->get_acc_without_g_b(&data.a_x, &data.a_y, &data.a_z);
    }

    // 关节电机角度和转速
    for(uint8_t i = 0; i < 4; i++) 
    {
        deps.joint_motors[i]->update_feedback();
    }

    data.leg_data[R].theta1 = -deps.joint_motors[RF]->get_current_position() + deps.joint_motor_offset[RF];
    data.leg_data[R].theta2 = -deps.joint_motors[RB]->get_current_position() + deps.joint_motor_offset[RB];
    data.leg_data[L].theta1 =  deps.joint_motors[LF]->get_current_position() + deps.joint_motor_offset[LF];
    data.leg_data[L].theta2 =  deps.joint_motors[LB]->get_current_position() + deps.joint_motor_offset[LB];

    data.leg_data[R].d_theta1 = -deps.joint_motors[RF]->get_current_rotate();
    data.leg_data[R].d_theta2 = -deps.joint_motors[RB]->get_current_rotate();
    data.leg_data[L].d_theta1 =  deps.joint_motors[LF]->get_current_rotate();
    data.leg_data[L].d_theta2 =  deps.joint_motors[LB]->get_current_rotate();

    //  轮毂电机  x,dx
    deps.wheel_motors[R]->update_feedback();
    data.leg_data[R].w = deps.wheel_motors[R]->get_current_rotate()/ deps.reduction_ratio;
    data.leg_data[R].T_w_real = deps.wheel_motors[R]->get_current_torque() * 0.3f / (3591.0f/187.0f) * deps.reduction_ratio;
    data.leg_data[R].dx = data.leg_data[R].w * deps.wheel_radius;
    data.leg_data[R].x += (data.leg_data[R].dx + last_dx[R])/2 * 0.001f;
    last_dx[R] = data.leg_data[R].dx;

    deps.wheel_motors[L]->update_feedback();
    data.leg_data[L].w = deps.wheel_motors[L]->get_current_rotate() / deps.reduction_ratio;
    data.leg_data[L].T_w_real = deps.wheel_motors[L]->get_current_torque() * 0.3f / (3591.0f/187.0f) * deps.reduction_ratio;
    data.leg_data[L].dx = -data.leg_data[L].w * deps.wheel_radius;
    data.leg_data[L].x += (data.leg_data[L].dx + last_dx[L])/2 * 0.001f;
    last_dx[L] = data.leg_data[L].dx;

    // 航向云台
    deps.yaw_motor->update_feedback();
    data.gimbal_yaw = wrap2pi_f32(deps.yaw_motor->get_current_position() + deps.yaw_offset);
    data.gimbal_g_yaw = deps.yaw_motor->get_current_rotate();

    time = dwt_drv_t::get_delta_t(&dwt_cnt);

    //  VMC 正运动学解算 
    for(uint8_t i = 0; i < 2; i++)
    {
        float last_d_l = data.leg_data[i].d_l;
        float last_d_beta = data.leg_data[i].d_beta;
        
        status_t ret = deps.kinematic_solver->solve(
            data.leg_data[i].theta1, data.leg_data[i].theta2,
            data.leg_data[i].d_theta1, data.leg_data[i].d_theta2,
            &data.leg_data[i].phi1, &data.leg_data[i].phi2,
            &data.leg_data[i].alpha, &data.leg_data[i].l,
            &data.leg_data[i].d_l, &data.leg_data[i].d_alpha,
            &data.leg_data[i].d_jx, &data.leg_data[i].d_jy,
            &data.leg_data[i].jx, &data.leg_data[i].jy
        );
        if(ret != PYRO_OK) 
        {
            data.cnt.solver_error++;
        }
         //beta 是腿与竖直线的夹角，gamma是车身与水平面的夹角
        data.leg_data[i].beta = PI / 2 - data.leg_data[i].alpha - data.pitch;
        data.leg_data[i].d_beta = -data.leg_data[i].d_alpha - data.g_pitch;
        data.leg_data[i].gamma = -data.pitch;
        data.leg_data[i].d_gamma = -data.g_pitch;

        data.leg_data[i].d2_beta = (data.leg_data[i].d_beta - last_d_beta) / time;
        data.leg_data[i].d2_l = (data.leg_data[i].d_l - last_d_l) / time;
    }

    //  VMC 雅可比矩阵
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret = deps.kinematic_solver->get_VMC_value(
            data.leg_data[i].theta1,
            data.leg_data[i].theta2,
            data.leg_data[i].phi1,
            data.leg_data[i].phi2,
            data.leg_data[i].l, 
            data.leg_data[i].alpha,
            data.leg_data[i].T_mat.pData
        );
        if(ret != PYRO_OK) 
        {
            data.cnt.solver_error++;
        }
    }

    // 8. 轮式里程计卡尔曼滤波 (KF) 融合
    float kf_u = 0.0f;
    float kf_z[3] = {0.0f, 0.0f, 0.0f};
    float kf_estimated[3] = {0.0f, 0.0f, 0.0f};
    
    data.a_forward = data.a_x * arm_cos_f32(data.pitch) 
                        + data.a_z * arm_sin_f32(data.pitch) 
                        + data.g_yaw * data.g_yaw * IMU_OFFSET_X;
    data.a_upward = -data.a_x * arm_sin_f32(data.pitch) 
                        + data.a_z * arm_cos_f32(data.pitch);
    data.a_upward_lpf = data.a_upward_lpf * SUPPORT_FORCE_ACC_LPF_RC / (time + SUPPORT_FORCE_ACC_LPF_RC)
                        + data.a_upward * time / (time + SUPPORT_FORCE_ACC_LPF_RC);
                 
    float v_obs = (data.leg_data[R].dx + data.leg_data[L].dx) / 2.0f;
    for(uint8_t i = 0; i < 2; i++)
    {
        kf_u = 0.0f;
        kf_z[0] = v_obs;
        kf_z[1] = data.a_forward;
        kf_z[2] = data.g_yaw;   
        deps.wheel_kf[i]->update(kf_z, &kf_u, kf_estimated);
        data.leg_data[i].kf_v = kf_estimated[0];
        data.leg_data[i].kf_a = kf_estimated[1];
        data.leg_data[i].kf_w = kf_estimated[2];
        data.leg_data[i].kf_x += data.leg_data[i].kf_v * 0.001f;
    }
}




void wl_chassis_t::_fsm_execute()
{
    // _cmd = &_current_cmd;
    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd.mode)
        _fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd.mode)
        _fsm.change_state(&_state_active);

    //电容启用
    __decide_cap();          
    _fsm.execute(this);    
    
    time = dwt_drv_t::get_timeline_ms() - last_time;
}

void wl_chassis_t::__send_supercap_command() const
{
    supercap_drv_t::get_instance()->send_cmd(_ctx.data.supercap_cmd);
}

void wl_chassis_t::__decide_cap()
{
    static bool _last_status = false;
    static uint32_t _timer   = 0;
    static bool _delay_done  = false;

    bool current_status = referee_drv->get_data().robot_status.power_management_chassis_output;
    auto& supercap_cmd = _ctx.data.supercap_cmd;

    if (current_status) {
        if (!_last_status) 
        {
            _timer = 0; 
            _delay_done = false;
         }
        if (!_delay_done) {
            if (++_timer >= 1000) {
                _delay_done = true;
                 _timer = 0; 
                supercap_cmd.use_cap = 1; 
                __send_supercap_command();
            }
        } else {
            if (++_timer >= 10) { 
                _timer = 0; supercap_cmd.use_cap = 1; 
                __send_supercap_command(); 
            }
        }
    } else {
        if (_last_status) { supercap_cmd.use_cap = 0; 
            __send_supercap_command(); 
            _delay_done = false; 
            _timer = 0; 
        }
    }
    _last_status = current_status;
}

//函数接口
status_t wl_chassis_t::get_cur_angle(float *r_angle, float *l_angle) {
    if(!l_angle || !r_angle) 
    {
        return PYRO_PARAM_ERROR;
    }
    *r_angle = _ctx.data.leg_data[R].alpha; 
    *l_angle = _ctx.data.leg_data[L].alpha; 
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_d_angle(float *r_d_angle, float *l_d_angle) {
    if(!l_d_angle || !r_d_angle) 
    {
        return PYRO_PARAM_ERROR;
    }
    *r_d_angle = _ctx.data.leg_data[R].d_alpha; 
    *l_d_angle = _ctx.data.leg_data[L].d_alpha; 
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_length(float *r_leg, float *l_leg) {
    if(!l_leg || !r_leg) 
    {
        return PYRO_PARAM_ERROR;
    }
    *r_leg = _ctx.data.leg_data[R].l; 
    *l_leg = _ctx.data.leg_data[L].l; 
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_p_torque(float *r_torque, float *l_torque) {
    if(!r_torque || !l_torque) 
    {
        return PYRO_PARAM_ERROR;
    }
    *r_torque = _ctx.data.leg_data[R].F[1]; 
    *l_torque = _ctx.data.leg_data[L].F[1]; 
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_ins_yaw(float* temp_yaw) {
    if(!temp_yaw) 
    {
        return PYRO_PARAM_ERROR;
    }
    *temp_yaw = _ctx.data.yaw; 
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_x_bias(float* r_x_bias, float* l_x_bias) {
    if(!r_x_bias || !l_x_bias) 
    {
        return PYRO_PARAM_ERROR;
    }
    *r_x_bias = _ctx.data.leg_data[R].x_bias; 
    *l_x_bias = _ctx.data.leg_data[L].x_bias; 
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_beta_bias(float* r_beta_bias, float* l_beta_bias) {
    if(!r_beta_bias || !l_beta_bias) 
    {
        return PYRO_PARAM_ERROR;
    }
    *r_beta_bias = _ctx.data.leg_data[R].beta_bias; 
    *l_beta_bias = _ctx.data.leg_data[L].beta_bias; 
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_gamma_bias(float* gamma_bias) {
    if(!gamma_bias) 
    {
        return PYRO_PARAM_ERROR;
    }
    *gamma_bias = _ctx.data.leg_data[R].gamma_bias; 
    return PYRO_OK;
}

uint8_t wl_chassis_t::get_status_flag(wl_cmd_t::active_mode_t mode)
{
    const auto& flag = _ctx.data.active_mode_flag;
    switch(mode)
    {
        case wl_cmd_t::READY:           return flag.ready;
        case wl_cmd_t::TEST:            return flag.test;
        case wl_cmd_t::REVERSE:         return flag.reverse;
        case wl_cmd_t::OVER_STEP:       return flag.over_step;
        case wl_cmd_t::OVER_STEP_RESET: return flag.over_step_reset;
        case wl_cmd_t::NORMAL:          return flag.normal;
        case wl_cmd_t::CONTROL:         return flag.control;
        case wl_cmd_t::JUMP:            return flag.jump;
        default:                        return 0;
    }
}

status_t wl_chassis_t::clear_status_flag(wl_cmd_t::active_mode_t mode)
{
    auto& flag = _ctx.data.active_mode_flag;
    switch(mode)
    {
        case wl_cmd_t::READY:           flag.ready = 0; break;
        case wl_cmd_t::TEST:            flag.test = 0; break;
        case wl_cmd_t::REVERSE:         flag.reverse = 0; break;
        case wl_cmd_t::OVER_STEP:       flag.over_step = 0; break;
        case wl_cmd_t::OVER_STEP_RESET: flag.over_step_reset = 0; break;
        case wl_cmd_t::NORMAL:          flag.normal = 0; break;
        case wl_cmd_t::CONTROL:         flag.control = 0; break;
        case wl_cmd_t::JUMP:            flag.jump = 0; break;
        default:                        return PYRO_PARAM_ERROR;
    }
    return PYRO_OK;
}

} // namespace pyro