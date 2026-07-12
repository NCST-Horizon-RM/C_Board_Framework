//
// Created by CaoKangqi on 2026/6/20.
//

#ifndef F4_FRAMEWORK_CHASSIS_CTRL_H
#define F4_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "Chassis_Calc.h"
#include "IMU_Task.h"

typedef struct {
    PID_t Drive_S[4];
    PID_t Follow;
    mecanumInit_typdef Mecanum;
} Chassis_Ctrl_Block_t;

uint8_t Chassis_Control_Init(void);
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, float dt);

#endif //F4_FRAMEWORK_CHASSIS_CTRL_H
