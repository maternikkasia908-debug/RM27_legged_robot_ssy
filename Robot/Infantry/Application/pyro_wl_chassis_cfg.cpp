/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: Refactored to match exact wl_chassis_cfg_t header
 * @Description: Wheel-Legged Chassis Configuration 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

#include "pyro_wl_chassis.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"

using namespace pyro;


#define PHI_K0 22506.0f
#define PHI_K1 475976946.0f
#define PHI_K2 296955904.0f
#define PHI_K3 179021042.0f
#define PHI_K4 18922.0f

#define POLAR_K0 236976927.0f
#define POLAR_K1 21059.0f
#define POLAR_K2 946100000.0f
#define POLAR_K3 100000.0f

#define TRANS_K0 21059.0f
#define TRANS_K1 100000.0f

#if ROBOT_ID == INFANTRY1_ID
#define R_MOTOR1_OFFSET  0.4424f 
#define R_MOTOR2_OFFSET  0.2270f 
#define L_MOTOR1_OFFSET -0.0236f 
#define L_MOTOR2_OFFSET  3.8210f 
#define YAW_OFFSET      -1.9f
#elif ROBOT_ID == INFANTRY2_ID
#define R_MOTOR1_OFFSET  1.2694f
#define R_MOTOR2_OFFSET  1.2610f
#define L_MOTOR1_OFFSET  0.5164f
#define L_MOTOR2_OFFSET  2.3000f
#define YAW_OFFSET      -1.945f
#endif

#define WHEEL_DISTANCE 0.424f
#define CONTROL_PERIOD 0.001f

#define LQR_GAIN \
 -0.9822, 14.2356, -25.1651, 15.4754,-4.0889, 61.2729, -106.1286, 64.5492,-18.8575, -18.5331, 192.4497, -254.9521,-3.4886, -5.0196, 37.7243, -47.2479,6.1991, -140.3664, 90.9413, 26.5639,0.8399, -11.9744, -29.4522, 36.8439,2.1984, -133.0088, 506.8353, -585.9221,9.1394, -577.8490, 2198.9349, -2542.0398,-1.6755, -317.2803, -101.5236, 787.0448,0.4825, -20.7444, -167.8482, 324.9636,-55.8928, 1690.0016, -5614.6267, 6071.1369,-9.3538, 239.3199, -674.7390, 677.7541

#define LQR_COEF_OVER_STEP \
 -1.0981, 19.8929, -49.9394, 46.5243,-4.5492, 84.2808, -207.8295, 192.5193,-21.8057, 21.1337, 84.8880, -164.0295,-3.9946, 0.0755, 27.1950, -41.8240,5.3853, -148.0953, 163.4375, -81.1506,0.7690, -12.9720, -22.7003, 27.9048,-6.2843, -22.0035, 143.7480, -194.7848,-26.6169, -97.0868, 616.6843, -830.3942,17.2288, -565.2392, 1236.8181, -998.1581,6.4658, -104.0931, 205.8637, -147.1518,7.8346, 561.3459, -1928.7994, 2121.5519,-0.8096, 90.8998, -240.2076, 235.7069

float infantry2_lqr_coef[48] = {LQR_GAIN};
float infantry2_lqr_coef_over_step[48] = {LQR_COEF_OVER_STEP};



// 电机
static dm_motor_drv_t s_jm0(
    0x01, 
    0x11, 
    can_hub_t::can1
);
static dm_motor_drv_t s_jm1(
    0x02, 
    0x12, 
    can_hub_t::can1
);
static dm_motor_drv_t s_jm2(
    0x03, 
    0x13, 
    can_hub_t::can2
);
static dm_motor_drv_t s_jm3(
    0x04, 
    0x14, 
    can_hub_t::can2
);

static dji_m3508_motor_drv_t s_wm0(
    dji_motor_tx_frame_t::id_3, 
    can_hub_t::can1
);

static dji_m3508_motor_drv_t s_wm1(
    dji_motor_tx_frame_t::id_2, 
    can_hub_t::can2
);

static dji_gm_6020_motor_drv_t s_yaw_m(
    dji_motor_tx_frame_t::id_5, 
    can_hub_t::can3
);

// PID 
static pid_t s_T_pid_0(
    20.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    40.0f
);
static pid_t s_T_pid_1(
    20.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    40.0f
);
static pid_t s_d_T_pid_0(
    30.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    20.0f
);
static pid_t s_d_T_pid_1(
    30.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    20.0f
);
static pid_t s_F_pid_0(
    80.0f, 
    0.0f, 
    0.0f, 
    50.0f, 
    200.0f
);
static pid_t s_F_pid_1(
    80.0f, 
    0.0f, 
    0.0f, 
    50.0f, 
    200.0f
);
static pid_t s_d_F_pid_0(
    240.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    200.0f
);
static pid_t s_d_F_pid_1(
    240.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    200.0f
);
static pid_t s_yaw_pid(
    4.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    30.0f
);
static pid_t s_g_yaw_pid(
    2.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    40.0f
);
static pid_t s_delta_pid(
    4.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    20.0f
);
static pid_t s_d_delta_pid(
    3.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    20.0f
);
static pid_t s_roll_pid(
    300.0f, 
    0.0f, 
    0.0f, 
    0.0f, 
    60.0f
);

// 卡尔曼滤波
static kf_t s_wheel_kf_0(
    3, 
    1, 
    3, 
    2
);
static kf_t s_wheel_kf_1(
    3, 
    1, 
    3, 
    2
);




wl_chassis_cfg_t infantry2_chassis_cfg = {
    // Kinematic 参数
    .phi_k = {
        .k0 = PHI_K0, .k1 = PHI_K1, .k2 = PHI_K2, .k3 = PHI_K3, .k4 = PHI_K4,
    },
    .polar_k = {
        .k0 = POLAR_K0, .k1 = POLAR_K1, .k2 = POLAR_K2, .k3 = POLAR_K3,
    },
    .vmc_k = {
        .k0 = TRANS_K0, .k1 = TRANS_K1,
    },

    

    .lqr_coef = infantry2_lqr_coef,
    .lqr_coef_over_step = infantry2_lqr_coef_over_step,

    .wheel_radius = 0.06f,
    .reduction_ratio = 13.94f,
    .yaw_offset = YAW_OFFSET,
    .joint_motor_offset = {
        R_MOTOR1_OFFSET, 
        R_MOTOR2_OFFSET, 
        L_MOTOR1_OFFSET,
        L_MOTOR2_OFFSET
    },

    // 功控参数
    .power_ctrl_cfg = {
        .k1 = {
            0.115f, 
            0.167f
        },
        .k2 = {
            2.44f, 
            2.26f
        },
        .k3 = {
            2.0f, 
            2.0f
        },
        .energy_kp = 1.0f,
        .energy_kd = 0.0f,
        .min_max_power = 15.0f,
        .cap_max_bonus = 150.0f,
    },

   

    // 电机指针数组映射 
    .joint_motors = { &s_jm0, &s_jm1, &s_jm2, &s_jm3 },
    .wheel_motors = { &s_wm0, &s_wm1 },
    .yaw_motor = &s_yaw_m,
    
    
    ._ins_drv = nullptr, 

    // PID 指针数组映射
    .T_pid = { &s_T_pid_0, &s_T_pid_1 },
    .d_T_pid = { &s_d_T_pid_0, &s_d_T_pid_1 },
    .F_pid = { &s_F_pid_0, &s_F_pid_1 },
    .d_F_pid = { &s_d_F_pid_0, &s_d_F_pid_1 },
    .yaw_pid = &s_yaw_pid,
    .g_yaw_pid = &s_g_yaw_pid,
    .delta_pid = &s_delta_pid,
    .d_delta_pid = &s_d_delta_pid,
    .roll_pid = &s_roll_pid,

    // KF 指针数组映射
    .wheel_kf = { &s_wheel_kf_0, &s_wheel_kf_1 }
};



// 在 app.cpp 的 infantry2_chassis_init() 中，在 configure 之前调用该函数
void infantry2_chassis_cfg_init()
{
    //  设置电机限幅
    for(int i = 0; i < 4; i++) {
        dm_motor_drv_t* motor = static_cast<dm_motor_drv_t*>(infantry2_chassis_cfg.joint_motors[i]);
        motor->set_rotate_range(-45.0f, 45.0f);
        motor->set_position_range(-3.141593f, 3.141593f);
        motor->set_torque_range(-54.0f, 54.0f);
    }

    //  初始化 KF 矩阵 
    float kf_A[9] = {
        1.0f, CONTROL_PERIOD, 0.0f,
        0.0f, 1.0f, 0.0f, 
        0.0f, 0.0f, 1.0f
    };
    float kf_B[3] = {
        0.0f, 0.0f, 0.0f
    };
    float kf_H[9] = {
        1.0f, 0.0f, 0.0f, 
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
        };
    float kf_G[6] = {
        CONTROL_PERIOD*CONTROL_PERIOD/2.0f, 0.0f, 
        CONTROL_PERIOD,                     0.0f, 
        0.0f,                               CONTROL_PERIOD
        };
    float kf_Q[4] = {
        10000.0f, 0.0f, 
        0.0f,     10000.0f
    };
    float kf_R[9] = {
        0.05f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f, 
        0.0f, 0.0f, 0.005f
    };
    float kf_x_init[3] = {
        0.0f, 0.0f, 0.0f
    };
    float kf_P_init[9] = {
        1.0f, 0.0f, 0.0f, 
        0.0f, 1.0f, 0.0f, 
        0.0f, 0.0f, 1.0f
    };
    
    for(int i = 0; i < 2; i++) {
        infantry2_chassis_cfg.wheel_kf[i]->init(kf_A, kf_B, kf_H, kf_G, kf_Q, kf_R, kf_x_init, kf_P_init);
    }

    
    infantry2_chassis_cfg._ins_drv = ins_drv_t::get_instance();

    
    infantry2_chassis_cfg._kinematic_solver.init(
        &infantry2_chassis_cfg.phi_k, 
        &infantry2_chassis_cfg.polar_k, 
        &infantry2_chassis_cfg.vmc_k
    );

    infantry2_chassis_cfg._power_ctrl.init(&infantry2_chassis_cfg.power_ctrl_cfg);
}