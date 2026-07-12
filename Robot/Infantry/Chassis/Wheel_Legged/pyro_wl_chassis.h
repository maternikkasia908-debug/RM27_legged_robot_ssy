
#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_kin.wl.h"
#include "pyro_supercap_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"
#include "pyro_algo_pid.h"
#include "kalman_filter.h"
#include "pyro_wl_power_ctrl.h"

namespace pyro
{

/* ========================================================================== */
/*                                1. 基础配置与指令层                          */
/* ========================================================================== */



/* --- 配置依赖层 (Deps) --- */
struct wl_chassis_cfg_t
{
 
    wheel_legged_kin_t::phi_k_t phi_k;
    wheel_legged_kin_t::polar_k_t polar_k;
    wheel_legged_kin_t::vmc_k_t vmc_k;
    
    wheel_legged_kin_t _kinematic_solver;

    float *lqr_coef;
    float *lqr_coef_over_step;
    //dji
    float wheel_radius;
    float reduction_ratio;
    float yaw_offset;
    float joint_motor_offset[4];
    
    wl_power_ctrl_cfg_t power_ctrl_cfg;
    wl_power_ctrl_t    _power_ctrl;
   

 
    motor_base_t *joint_motors[4];
    motor_base_t *wheel_motors[2];
    motor_base_t *yaw_motor;

    ins_drv_t         *_ins_drv;

    pid_t *T_pid[2];
    pid_t *d_T_pid[2];
    pid_t *F_pid[2];
    pid_t *d_F_pid[2];

    pid_t *yaw_pid;
    pid_t *g_yaw_pid;
    pid_t *delta_pid;
    pid_t *d_delta_pid;
    pid_t *roll_pid;
    
    kf_t  *wheel_kf[2];

};

/* --- 指令层 (Cmd) --- */
struct wl_cmd_t final : public cmd_base_t
{
    float vx;   
    float vy;  
    float vz;
    float yaw;
    float l_leg;
    float r_leg;
    float l_angle;
    float r_angle;
    
    enum active_mode_t
    {
        NORMAL = 0,
        READY = 1,
        TEST = 2,
        REVERSE = 3,
        OVER_STEP = 4,
        OVER_STEP_READY = 5,
        OVER_STEP_RESET = 6,
        CONTROL = 7,
        SPIN = 8,
        JUMP = 9
    } active_mode, last_active_mode;

    wl_cmd_t() : vx(0), vy(0), vz(0), yaw(0), l_leg(0), r_leg(0), l_angle(0), r_angle(0), active_mode(TEST)
    {
    }
};

/* ========================================================================== */
/*                                2. 状态上下文构建                            */
/* ========================================================================== */

/* --- 数据状态层 (DataCtx) --- */
struct wl_data_ctx_t
{
    // IMU 姿态及解算反馈
    float yaw, pitch, roll;
    float g_yaw, g_pitch, g_roll;
    float a_x, a_y, a_z, a_forward, a_upward, a_upward_lpf;
    float gimbal_yaw, gimbal_g_yaw;

    // 目标及控制偏差
    float yaw_ref;
    float g_yaw_ref;
    float delta_mea;
    float d_delta_mea;
    float d_delta_ref;

    // LQR 系数存储
    float lqr_cof[48];
    float lqr_cof_over_step[48];

    // 底盘与电机参数存储
    float wheel_radius;
    float reduction_ratio;
    float yaw_offset;
    float motor_offset[4];

    // 控制增益
    float T_w_gain;
    float x_gain;
    float T_l_gain; 
    float roll_gain;

    // 腿部状态
    struct leg_data_t
    {
        float theta1, theta2;
        float d_theta1, d_theta2;
        float phi1, phi2;
        float l, ref_l;
        float d_l, d2_l, ref_d_l;
        float alpha, d_alpha, ref_d_alpha;
        float beta, d_beta, d2_beta;
        float gamma, d_gamma;
        float x, dx, w;
        
        arm_matrix_instance_f32 T_mat;
        float T_mat_val[4];
        float T[2];
        float F[2];
        
        float T_w, T_w_balance, T_w_move, T_w_turn, T_w_real, T_w_out;
        float lqr_gain[12];
        float x_gain, d_x_gain;
        float P;
        float jx, jy, d_jx, d_jy;

        float kf_v, kf_a, kf_w, kf_x;
        float predict_power;

        float beta_bias, d_beta_bias;
        float gamma_bias, d_gamma_bias;
        float x_bias, d_x_bias;
    } leg_data[2];

    // 计数器与状态标志
    struct { 
             uint16_t solver_error; 
            } cnt;

    struct { uint8_t is_aerial = 0; 
             uint8_t aerial_cnt = 0; 
             float test = 0; 
            } flag;

    struct {
        uint8_t normal = 0, 
                ready = 0,
                test = 0, 
                reverse = 0, 
                over_step = 0,  
                over_step_reset = 0, 
                control = 0, 
                jump = 0;
            } active_mode_flag;

    // 电容与功率反馈
    supercap_drv_t::chassis_cmd_t supercap_cmd;

    struct {
        float chassis_power;
        float voltage;
        float cap_power;
        float limit;
        float buffer_energy;
            } power_data;
};

/* --- 模块总上下文 (ModuleCtx) --- */
struct wl_chassis_ctx_t
{
    wl_chassis_cfg_t deps;
    wl_data_ctx_t    data;
    wl_cmd_t        *cmd;
};

/* --- 参数聚合 --- */
struct wl_chassis_params_t
{
    using CmdType    = wl_cmd_t;
    using ModuleDeps = wl_chassis_cfg_t;
    using ModuleCtx  = wl_chassis_ctx_t;
};

/* ========================================================================== */
/*                                3. 核心模块类定义                            */
/* ========================================================================== */

class wl_chassis_t final : public module_base_t<wl_chassis_t, wl_chassis_params_t>
{
   friend class module_base_t<wl_chassis_t, wl_chassis_params_t>;
   friend class vofa_drv_t;

public:
   
   enum { 
          RF = 0, 
          RB = 1, 
          LF = 2, 
          LB = 3 
        };
   enum { 
          R = 0, 
          L = 1 
        };

   wl_chassis_t(const wl_chassis_t &)            = delete;
   wl_chassis_t &operator=(const wl_chassis_t &) = delete;

//派生方法
   status_t get_cur_angle(float *r_angle, float *l_angle);
   status_t get_cur_d_angle(float *r_angle, float *l_angle);
   status_t get_cur_length(float *r_leg, float *l_leg);
   status_t get_cur_p_torque(float *r_torque, float *l_torque);
   status_t get_cur_ins_yaw(float* temp_yaw);
   status_t get_cur_x_bias(float* r_x_bias, float* l_x_bias);
   status_t get_cur_beta_bias(float* r_beta_bias, float* l_beta_bias);
   status_t get_cur_gamma_bias(float* gamma_bias);
   uint8_t get_status_flag(wl_cmd_t::active_mode_t mode);
   status_t clear_status_flag(wl_cmd_t::active_mode_t mode);

private:
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    // 基态接口
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    //data context
    wl_chassis_ctx_t _ctx;
    

    /* =====================================================
       FSM 声明 
       ===================================================== */
    class fsm_active_t : public fsm_t<wl_chassis_t>
    {
    public:
        class state_test_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_test;

        class state_ready_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner);
        } _state_ready;

        class state_normal_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_support_force(wl_chassis_t *owner);
        } _state_normal;

        class state_reverse_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_reverse;

        class state_over_step_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner);
        } _state_over_step;

        class state_over_step_ready_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_over_step_ready;

        class state_over_step_reset_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner);
        } _state_over_step_reset;

        class state_control_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_control;

        class state_spin_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_spin;

        class state_jump_t : public state_t<wl_chassis_t> {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_balance(wl_chassis_t *owner);
        } _state_jump;

        void on_enter(wl_chassis_t *owner) override;
        void on_execute(wl_chassis_t *owner) override;
        void on_exit(wl_chassis_t *owner) override;
    } _state_active;

    class state_passive_t : public state_t<wl_chassis_t>
    {
    public:
        void enter(wl_chassis_t *owner) override;
        void execute(wl_chassis_t *owner) override;
        void exit(wl_chassis_t *owner) override;
    } _state_passive;

  
    
    fsm_t<wl_chassis_t> _fsm;

    void __send_supercap_command() const;
    void __decide_cap();
};

}
#endif // __PYRO_WL_CHASSIS_H__