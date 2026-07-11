//
// Created by R0602 on 26-7-10.
//

#ifndef SHOOT_CTRL_H
#define SHOOT_CTRL_H
#include <stdint.h>
#include "Robot_Config.h"
#include "Classic_Control.h"
typedef struct {
    PID_t Bmotor_P;
    PID_t Bmotor_S;
    PID_t Lfire_S;
    PID_t Rfire_S;
} Shoot_Ctrl_Block_t;

uint8_t Shoot_Control_Init(void);
void Shoot_Control_Task(const Shoot_Motor_Group_t *g_motor);
void Single_Shoot_Control(const Shoot_Motor_Group_t *g_motor);
void Smooth_Shoot_Control(const Shoot_Motor_Group_t *g_motor);
void Usmooth_Shoot_Control(const Shoot_Motor_Group_t *g_motor);

#endif //SHOOT_CTRL_H
