//
// Created by R0602 on 26-7-10.
//
#include "Shoot_Ctrl.h"
#include "Message_Center.h"
#include "System_State.h"
#include "Robot_Config.h"
#include "Robot_Cmd.h"
#include "Horizon_MATH.h"
#include "VT13.h"
#include "DBUS.h"
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
    shoot_ctrl.motor_ratio=36.0f;
    shoot_ctrl.feed_ratio=2.5f;
    shoot_ctrl.slot_num=8.0f;
    shoot_ctrl.target_hz=18.0f;
    shoot_ctrl.use_smoothing=0;
    shoot_ctrl.dir_sign=-1;
    shoot_ctrl.fire_state=0;
    shoot_ctrl.target_freq=0;
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
void Single_Shoot_Control(void)
{
    shoot_ctrl.Feeder_Count.target_pos_cnt += shoot_ctrl.dir_sign;
}
/**
 * @brief 连发控制
 */
void Smooth_Shoot_Control(void)
{
    //连发
    float raw_freq = shoot_ctrl.target_hz;
    shoot_ctrl.target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
    if (cmd.trigger_auto==1&&VT13.Remote.trigger==1)
    {
        shoot_ctrl.use_smoothing=1;
    }
    else if (cmd.trigger_auto ==1 && VT13.Remote.trigger  ==0)
    {
        shoot_ctrl.use_smoothing=0;
    }
}

/**
 * @brief 卡弹检测TODO
 */
void Automatic_Shoot_Control(const Shoot_Motor_Group_t *g_motor)
{

}
/**
 * @brief 发射总控制任务
 */
void Shoot_Control_Task(const Shoot_Motor_Group_t *g_motor)
{
    // 空指针保护
    if (g_motor == NULL ) {
        System_State_Report(ID_SHOOT, STATUS_ERROR);
        return;
    }
    // 获取所有订阅数据
    SubGetMessage(sys_state_sub, &local_sys_state);
    if (shoot_cmd_sub) SubGetMessage(shoot_cmd_sub, &cmd);
    // 状态汇报
    if (!Is_Group_Online(SHOOT)) {
        System_State_Report(ID_SHOOT, STATUS_LOST);
    }
    else{System_State_Report(ID_SHOOT, STATUS_RUN);}
    // 判断系统状态
    bool is_system_locked = (local_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             local_sys_state.global_mode == GLOBAL_STANDBY ||
                             local_sys_state.global_mode == GLOBAL_INIT_STAGE);
    if (cmd.mode == SHOOT_CMD_SAFE || is_system_locked)
    {
        // 清空PID
            PID_Clear(&shoot_ctrl.Bmotor_P);
            PID_Clear(&shoot_ctrl.Bmotor_S);
            PID_Clear(&shoot_ctrl.Lfire_S );
            PID_Clear(&shoot_ctrl.Rfire_S );
    }
    static uint32_t last_shot_time = 0;
    static bool is_init =false;
    static float smooth_ref = 0.0f;
    shoot_ctrl.Counts_Shoot=shoot_ctrl.motor_ratio*shoot_ctrl.slot_num /shoot_ctrl.slot_num;
//包含摩擦轮是否开启
    if (VT13.Remote.mode_sw  ==0)
    {
        DJI_Motor_Send(&hcan1,0x200,0,0,0,0);
        DJI_Motor_Send(&hcan2,0x200,0,0,0,0);
        shoot_ctrl.fire_state=0;
        return;
    }
    if (VT13.Remote.mode_sw  ==1||VT13.Remote.mode_sw  ==2)
    {
        shoot_ctrl.fire_state==1;
    }
    if (!is_init)
    {
        smooth_ref = g_motor->DJI_2006_bo.Angle_Infinite;
        shoot_ctrl.Feeder_Count.target_pos_cnt=(int32_t)(smooth_ref / shoot_ctrl.slot_num);
        is_init = true;
    }
    //连发模式同时已经开启摩擦轮
    if (cmd.trigger_auto==1&&VT13.Remote.trigger==1)
    {
        Smooth_Shoot_Control();
    }
    //TODO：卡弹检测
    //单发
    // 统一正常射击触发判定
    uint32_t now = HAL_GetTick();
    float interval = 1000.0f / shoot_ctrl.target_freq;
    if ((shoot_ctrl.fire_state==1)&&(cmd.trigger_single ==true))
    {
        if (now - last_shot_time >= (uint32_t)interval)
        {
            Single_Shoot_Control();
            last_shot_time = now;
        }
    }
    // 统一目标计算与运动平滑控制 (Ramp 阶跃生成器)
    float final_target = (float)shoot_ctrl.Feeder_Count.target_pos_cnt * shoot_ctrl.Counts_Shoot;
    //连发
    if ((shoot_ctrl.fire_state==1)&&shoot_ctrl.use_smoothing ==1)//使用平滑模式
    {
        float step = (shoot_ctrl.target_freq * shoot_ctrl.Counts_Shoot) / 1000.0f;
        if (smooth_ref > final_target)
        {
            smooth_ref -= step;
            if (smooth_ref < final_target) smooth_ref = final_target;
        } else if (smooth_ref < final_target)
        {
            smooth_ref += step;
            if (smooth_ref > final_target) smooth_ref = final_target;
        }
    }
    else if((shoot_ctrl.fire_state==1)&&(shoot_ctrl.use_smoothing ==0))
    {
        smooth_ref = final_target;
    }
    else
    {
        smooth_ref = g_motor->DJI_2006_bo.Angle_Infinite;;
    }
    shoot_ctrl.Lfire_S.Ref=cmd.lfriction_rpm;
    shoot_ctrl.Rfire_S.Ref=cmd.rfriction_rpm;
    shoot_ctrl.Bmotor_P.Ref=smooth_ref;
    //pid计算与can发送
    PID_Calculate(&shoot_ctrl.Lfire_S, g_motor->DJI_3508_L.Speed_now, shoot_ctrl.Lfire_S.Ref);
    PID_Calculate(&shoot_ctrl.Rfire_S, g_motor->DJI_3508_R.Speed_now, shoot_ctrl.Rfire_S.Ref);
    PID_Calculate(&shoot_ctrl.Bmotor_P, g_motor->DJI_2006_bo.Angle_Infinite, shoot_ctrl.Bmotor_P.Ref);
    PID_Calculate(&shoot_ctrl.Bmotor_S, g_motor->DJI_2006_bo.Speed_now, shoot_ctrl.Bmotor_P .Output);
    DJI_Motor_Send(&hcan2,0x200,shoot_ctrl.Lfire_S.Output,shoot_ctrl.Rfire_S.Output,0,0);
    DJI_Motor_Send(&hcan1,0x200,0,0,shoot_ctrl.Bmotor_S.Output,0 );
}
//射击检测
#define K_UP             0.673//0.360f   // 上升系数
#define K_DN             0.142//0.059f   // 下降系数
#define TH_FIRE          200.0f   // 触发阈值
#define TH_FIRE_MAX      1200.0f  // 最大触发阈值
#define MIN_SLOPE        80.0f    // 最小斜率阈值
#define RELATIVE_RECOVER 0.25f    // 回升比例
#define TH_RST_SAFE      100.0f   // 复位阈值
#define TIMEOUT_TICKS    14       // 超时上限
#define COOL_DOWN_TICKS  2        // 冷却周期
/*
 *弹丸计数
 */
bool Shoot_Count(float speed1, float speed2)
{
    float val = (fabsf(speed1) + fabsf(speed2)) / 2.0f;
    if (!shoot_ctrl.Det_Count.init)
    {
        shoot_ctrl.Det_Count.base = val;
        shoot_ctrl.Det_Count.last_val = val;
        shoot_ctrl.Det_Count.max_drop_in_round = 0;
        shoot_ctrl.Det_Count.cool_down_cnt = 0;
        shoot_ctrl.Det_Count.init = true;
        return false;
    }
    float slope = shoot_ctrl.Det_Count.last_val - val;
    shoot_ctrl.Det_Count.last_val = val;
    if (val > shoot_ctrl.Det_Count.base) {
        shoot_ctrl.Det_Count.base = (K_UP * val) + (1.0f - K_UP) * shoot_ctrl.Det_Count.base;
    } else {
        shoot_ctrl.Det_Count.base = (K_DN * val) + (1.0f - K_DN) * shoot_ctrl.Det_Count.base;
    }
    float drop = shoot_ctrl.Det_Count.base - val;
    bool shoot_done = false;
    if (shoot_ctrl.Det_Count.cool_down_cnt > 0) {
        shoot_ctrl.Det_Count.cool_down_cnt--;
        shoot_ctrl.Det_Count.armed = false;
        return false;
    }
    if (!shoot_ctrl.Det_Count.armed) {
        if (drop > TH_FIRE && drop < TH_FIRE_MAX && slope > MIN_SLOPE && val>4000 ) {
            shoot_ctrl.Det_Count.armed = true;
            shoot_ctrl.Det_Count.max_drop_in_round = drop;
            shoot_ctrl.Det_Count.t_out = 0;
        }
    } else {
        shoot_ctrl.Det_Count.t_out++;
        if (drop > shoot_ctrl.Det_Count.max_drop_in_round) {
            shoot_ctrl.Det_Count.max_drop_in_round = drop;
        }
        bool condition_relative = (drop < shoot_ctrl.Det_Count.max_drop_in_round * (1.0f - RELATIVE_RECOVER));
        bool condition_absolute = (drop < TH_RST_SAFE);
        if (condition_relative || condition_absolute) {
            shoot_ctrl.Det_Count.armed = false;
            shoot_ctrl.Det_Count.cnt++;
            shoot_ctrl.Det_Count.cool_down_cnt = COOL_DOWN_TICKS;
            shoot_ctrl.Det_Count.max_drop_in_round = 0;
            shoot_done = true;
        }
        else if (shoot_ctrl.Det_Count.init >= TIMEOUT_TICKS) {
            shoot_ctrl.Det_Count.armed = false;
            shoot_ctrl.Det_Count.max_drop_in_round = 0;
        }
    }
    return shoot_done;
}
//TODO:火控待完善