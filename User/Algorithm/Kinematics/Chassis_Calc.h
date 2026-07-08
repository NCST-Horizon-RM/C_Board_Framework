//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef F4_FRAMEWORK_CHASSIS_CALC_H
#define F4_FRAMEWORK_CHASSIS_CALC_H
#include <stdint.h>

typedef struct
{
    float wheel_r;    /* 轮的半径（m）*/
    float chassis_r;         /* 底盘旋转半径（m）*/
    float phi[4];          /* 轮子安装方位角 (rad)*/
    float deceleration_ratio; /* 电机减速比 */
} mecanumInit_typdef;

typedef struct
{
    float wheel_r;    /* 轮的半径（m）*/
    float chassis_r;         /* 底盘旋转半径（m）*/
    float phi[4];          /* 轮子安装方位角 (rad)*/
    float deceleration_ratio; /* 电机减速比 */
} OmniInit_typdef;

uint8_t Mecanum_Init(mecanumInit_typdef *mecanumInitT);
void Mecanum_Calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t);

uint8_t Omni_Init(OmniInit_typdef *OmniInit_t);
void Omni_Calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t);


/* ==================== 舵轮 ==================== */

// 舵轮物理常量结构体
typedef struct {
    float m;                // 底盘质量 (kg)
    float J;                // 转动惯量 (kg*m^2)
    float R;                // 旋转半径 (m)
    float r;                // 轮子半径 (m)
    float Swerve_offset[4]; // 舵轮零点偏角
    float phi[4];           // 轮子安装方位角 (rad)
    float gear_d;           // 驱动电机减速比
} Swerve_Cfg_t;

// 实时单轮调试/物理状态
typedef struct {
    float theta_now;       // 归一化±π的舵轮实际角度 (rad)
    float theta_target;    // 归一化±π的舵轮目标角度 (rad)
    float v_wheel_now;     // 单轮实际线速度 (m/s)
    float v_wheel_target;  // 单轮目标线速度 (m/s)
    float ff_out;          // 单轮前馈输出原始值
} Swerve_Wheel_Debug_t;

// 底盘整体状态结构体
typedef struct {
    Swerve_Cfg_t cfg;

    // 底盘速度状态
    float vx;              // 底盘x轴实际速度 (m/s)
    float vy;              // 底盘y轴实际速度 (m/s)
    float vw;              // 底盘实际角速度 (rad/s)

    // 底盘目标指令
    float vx_target;       // 底盘x轴目标速度 (m/s)
    float vy_target;       // 底盘y轴目标速度 (m/s)
    float vw_target;       // 底盘目标角速度 (rad/s)
    float ax_target;       // 底盘x轴目标加速度 (m/s²)
    float ay_target;       // 底盘y轴目标加速度 (m/s²)
    float aw_target;       // 底盘目标角加速度 (rad/s²)

    Swerve_Wheel_Debug_t wheel[4];
} Swerve_State_t;

// 反馈数据，输入给解算器
typedef struct {
    float steer_angle_rad[4];  // 舵轮当前连续绝对角度 (rad)
    float steer_rpm[4];
    float wheel_rpm[4];        // 驱动轮当前转速 (RPM)
} Swerve_Feedback_t;

// 解算器输出的指令数据
typedef struct {
    float target_steer_angle_rad[4]; // 目标舵向角 (rad)
    float target_wheel_rpm[4];       // 目标驱动轮转速 (RPM)
    float ff_torque_raw[4];          // 前馈控制量
} Swerve_Command_t;

uint8_t Swerve_Init(Swerve_State_t *state);

void Swerve_Forward_Calc(Swerve_State_t *now, const Swerve_Feedback_t *fb);

void Swerve_Inverse_Calc(Swerve_Command_t *cmd, Swerve_State_t *state,
                         float ax, float ay, float aw,
                         float vx, float vy, float vw,
                         const Swerve_Feedback_t *fb);


#endif //F4_FRAMEWORK_CHASSIS_CALC_H