//
// Created by R0602 on 26-7-10.
//

#ifndef SHOOT_CTRL_H
#define SHOOT_CTRL_H
#include <stdint.h>
#include "Robot_Config.h"
#include "Classic_Control.h"

typedef struct {
    float base;              // 动态基准线
    float last_val;          // 记录上一次的转速，用于算斜率
    float max_drop_in_round; // 记录单次触发过程中的最大跌落深度
    bool  armed;             // 触发状态
    uint32_t cnt;            // 计数器
    uint32_t last_cnt;     //上一次的计数器
    uint8_t  t_out;          // 超时计数器
    uint8_t  cool_down_cnt;  // 冷却计数器
    bool  init;              // 初始化标志
} ShootDet_t;
typedef enum {
    FEEDER_STOP = 0,
    FEEDER_SINGLE,
    FEEDER_BURST
} FeederMode_e;
typedef struct {
    FeederMode_e mode;
    int32_t target_pos_cnt;      // 目标弹槽索引 (绝对位置)
    uint8_t last_trigger_state;  // 记录上一次触发状态 (用于单发边缘检测)
    float   target_freq;
} Feeder_t;
typedef struct {
    float motor_ratio;//电机减速比
    float feed_ratio;//齿轮减速比
    float slot_num;//弹槽数
    float target_hz;//目标弹频
    float target_freq;//限幅保护后的目标弹频
    bool use_smoothing ;//是否平滑模式的标志位
    float Counts_Shoot;//电机减速比*齿轮减速比/弹槽数
    float Lfire_speed;
    float Rfire_speed;
    bool fire_state ;
    int32_t dir_sign ;
    PID_t Bmotor_P;
    PID_t Bmotor_S;
    PID_t Lfire_S;
    PID_t Rfire_S;
    ShootDet_t Det_Count;
    Feeder_t Feeder_Count;
} Shoot_Ctrl_Block_t;

uint8_t Shoot_Control_Init(void);
void Shoot_Control_Task(const Shoot_Motor_Group_t *g_motor);
void Single_Shoot_Control(void);
void Smooth_Shoot_Control(void);


#endif //SHOOT_CTRL_H
