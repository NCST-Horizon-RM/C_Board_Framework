//
// Created by CaoKangqi on 2026/6/20.
//

#ifndef H7_FRAMEWORK_CHASSIS_CTRL_H
#define H7_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "Chassis_Calc.h"
#include "IMU_Task.h"

typedef struct {
    PID_t Drive_S[4];

    mecanumInit_typdef Mecanum;
} Chassis_Ctrl_Block_t;

uint8_t Chassis_Control_Init(void);
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor,
                          const IMU_Data_t *c_imu);

#endif //H7_FRAMEWORK_CHASSIS_CTRL_H
