//
// Created by CaoKangqi on 2026/2/13.
//

#ifndef F4_FRAMEWORK_ALL_DEFINE_H
#define F4_FRAMEWORK_ALL_DEFINE_H

#define DF_READY 1
#define DF_ERROR 0

/// 圆周率
#define PI 3.14159265358979f
/// 角度转弧度
#define DEG2RAD (PI / 180.0f)
/// 弧度转角度
#define RAD2DEG (180.0f / PI)

#define ENCODER_TO_RAD (2.0f * PI / 8192.0f)

#define RPM_TO_RADS    (2.0f * PI / 60.0f)
//设备离线
#define DEVICE_OFFLINE 0
//设备在线
#define DEVICE_ONLINE  1
//电机离线检测时间
#define MOTOR_OFFLINE_TIME 15
//电容离线检测时间
#define CAP_OFFLINE_TIME 15
//遥控离线检测时间
#define DBUS_OFFLINE_TIME 50

#define REFEREE_OFFLINE_TIME 800

#define INIT_ANGLE 0

#endif //F4_FRAMEWORK_ALL_DEFINE_H