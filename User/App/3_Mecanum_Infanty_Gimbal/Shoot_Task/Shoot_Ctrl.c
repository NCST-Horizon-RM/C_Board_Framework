//
// Created by R0602 on 26-7-10.
//
#include "Shoot_Ctrl.h"
#include "All_define.h"
#include "Message_Center.h"
#include "System_State.h"
#include "Robot_Config.h"
#include "Robot_Cmd.h"
//TODO:发射部分待完成
static Shoot_Ctrl_Block_t shoot_ctrl;
//订阅消息
static Subscriber_t *sys_state_sub;
static Subscriber_t *shoot_cmd_sub;
//底盘本地变量，用于存储订阅消息
static System_State_t local_sys_state;
static Shoot_Cmd_t cmd = {0};
/**
 * @brief 发射机构控制初始化
 * @param MOTOR 发射机构电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Shoot_Control_Init(void)
{
    // 拨盘电机PID参数初始化
    float PID_Bmotor_P[3] = {0.0f,   0.0f,  0.0f};
    PID_Init(&shoot_ctrl.Bmotor_P, 50.0f, 30.0f, PID_Bmotor_P,
        0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_Bmotor_S[3] = {0.0f,   0.0f,   0.0f};
    PID_Init(&shoot_ctrl.Bmotor_S, 30.0f, 2.0f, PID_Bmotor_S,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //两个摩擦轮 PID参数初始化
    float PID_Lfire_S[3] = {0.0f,   0.0f,  0.0f};
    PID_Init(&shoot_ctrl.Lfire_S, 50.0f, 30.0f, PID_Lfire_S,
        0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_Rfire_S[3] = {0.0f,   0.0f,   0.0f};
    PID_Init(&shoot_ctrl.Rfire_S, 30.0f, 2.0f, PID_Rfire_S,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //向系统下发底盘当前状态，准备中
    System_State_Report(ID_SHOOT, STATUS_PREPARING);
    //订阅系统状态、底盘控制指令
    sys_state_sub   = SubRegister("system_state", sizeof(System_State_t));
    shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Cmd_t));
    return 1;
}
/**
 * @brief 单发控制
 */
void Single_Shoot_Control(const Shoot_Motor_Group_t *g_motor)
{

}
/**
 * @brief 连发控制(开启平滑模式)
 */
void Smooth_Shoot_Control(const Shoot_Motor_Group_t *g_motor)
{/*
    //连发
    if (DBUS.Remote.S2 == 1) {
        float raw_freq = 18.0f - (float)DBUS.Remote.Dial / 20.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = true;   // 模式一：开启平滑
    }
    else if (DBUS.Remote.S2 == 2) {
        float raw_freq = 10.0f - (float)DBUS.Remote.Dial / 30.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = false;  // 模式二：关闭平滑
    }*/
}
/**
 * @brief 连发控制(关闭平滑模式)
 */
void Usmooth_Shoot_Control(const Shoot_Motor_Group_t *g_motor)
{
   /* //连发
    if (DBUS.Remote.S2 == 1) {
        float raw_freq = 18.0f - (float)DBUS.Remote.Dial / 20.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = true;   // 模式一：开启平滑
    }
    else if (DBUS.Remote.S2 == 2) {
        float raw_freq = 10.0f - (float)DBUS.Remote.Dial / 30.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = false;  // 模式二：关闭平滑
    }*/
}
/**
 * @brief 卡弹检测
 */
void Automatic_Shoot_Control(const Shoot_Motor_Group_t *g_motor)
{

}
/**
 * @brief 发射控制任务
 */
void Shoot_Control_Task(const Shoot_Motor_Group_t *g_motor)
{

}
/*
void Ctrl_Shoot_Task() {
    if (DBUS.Remote.S2 != 1 && DBUS.Remote.S2 != 2) {
        DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, 0, 0);
        return;
    }
    // 方向切换开关 1 默认方向，-1 整体反转方向
    const int32_t dir_sign = -1;

    static uint8_t last_s1 = 3;
    static uint32_t last_shot_time = 0;
    static float smooth_ref = 0.0f;
    static bool is_init = false;

    if (!is_init) {
        smooth_ref = All_Motor.DJI_2006_bo.DATA.Angle_Infinite;
        g_feeder.target_pos_cnt = (int32_t)(smooth_ref / COUNTS_PER_SHOT);
        is_init = true;
    }
    // 配置不同模式的“控制特性”
    float target_freq = 0.0f;
    bool use_smoothing = false;
    // 两个模式的正常转动方向完全相同，统一使用 dir_sign
    int32_t dir = 1 * dir_sign;



//连发
    if (DBUS.Remote.S2 == 1) {
        float raw_freq = 18.0f - (float)DBUS.Remote.Dial / 20.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = true;   // 模式一：开启平滑
    }
    else if (DBUS.Remote.S2 == 2) {
        float raw_freq = 10.0f - (float)DBUS.Remote.Dial / 30.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = false;  // 模式二：关闭平滑
    }
    // 统一频率限幅与时间判定
    uint32_t now = HAL_GetTick();
    float interval = 1000.0f / target_freq;

    // 统一正常射击触发判定
    if ((DBUS.Remote.S1 == 2 && last_s1 == 3) || (DBUS.Remote.S1 == 1)) {
        if (now - last_shot_time >= (uint32_t)interval) {
            g_feeder.target_pos_cnt += dir;
            last_shot_time = now;
        }
    }
    last_s1 = DBUS.Remote.S1;




    // 分模式处理卡弹（堵转）逻辑
    if (All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] == 1)
    {
        All_Motor.DJI_2006_bo.PID_S.Output = 0.0f;
        All_Motor.DJI_2006_bo.PID_S.Iout = 0.0f;
        float current_exact_pop = All_Motor.DJI_2006_bo.DATA.Angle_Infinite / COUNTS_PER_SHOT;

        if (dir > 0) {
            // 正转时卡弹（例如10.1）：必须去 11，使用 ceilf 向上取整
            g_feeder.target_pos_cnt = (int32_t)ceilf(current_exact_pop);

            // 如果正好卡在整数点（概率极低），强制向前再推一发，避免目标没有更新
            if ((float)g_feeder.target_pos_cnt - current_exact_pop < 0.01f) {
                g_feeder.target_pos_cnt += 1;
            }
        } else {
            // 反转时卡弹（例如 -10.1）：必须去 -11，使用 floorf 向下取整
            g_feeder.target_pos_cnt = (int32_t)floorf(current_exact_pop);

            // 如果正好卡在整数点，强制向前（负方向）再推一发
            if (current_exact_pop - (float)g_feeder.target_pos_cnt < 0.01f) {
                g_feeder.target_pos_cnt -= 1;
            }
        }
        // 让平滑参考值立刻对齐当前实际位置，这样下一帧它会顺着‘前进方向’平滑移动到新的整弹点
        smooth_ref = All_Motor.DJI_2006_bo.DATA.Angle_Infinite;
        // 处理完毕，手动清除堵转标志位
        All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] = 0;
    }
    // 统一目标计算与运动平滑控制 (Ramp 阶跃生成器)
    float final_target = (float)g_feeder.target_pos_cnt * COUNTS_PER_SHOT;

    if (use_smoothing) {
        float step = (target_freq * COUNTS_PER_SHOT) / 1000.0f;

        if (smooth_ref > final_target) {
            smooth_ref -= step;
            if (smooth_ref < final_target) smooth_ref = final_target;
        } else if (smooth_ref < final_target) {
            smooth_ref += step;
            if (smooth_ref > final_target) smooth_ref = final_target;
        }
    } else {
        // 模式二关闭平滑，目标值直接阶跃过去，爆发力最强
        smooth_ref = final_target;
    }

    // 7. 统一底层电机控制与 CAN 发送
    All_Motor.DJI_2006_bo.PID_P.Ref = smooth_ref;
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_P, All_Motor.DJI_2006_bo.DATA.Angle_Infinite, smooth_ref);
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_S, All_Motor.DJI_2006_bo.DATA.Speed_now, All_Motor.DJI_2006_bo.PID_P.Output);

    DJI_Motor_Stuck_Check(&All_Motor.DJI_2006_bo, 6000, 100, 100, 500);
    DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, All_Motor.DJI_2006_bo.PID_S.Output, 0);
}
*/