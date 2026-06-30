//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef F4_FRAMEWORK_VOFA_H
#define F4_FRAMEWORK_VOFA_H

#include <stdint.h>

#define VOFA_MAX_CHANNELS      20    // JustFloat 单次发送的最大通道数
#define VOFA_TEXT_BUF_SIZE     256   // FireWater 文本协议的最大单行缓冲区大小

void VOFA_JustFloat(uint8_t channels_num, ...);
void VOFA_FireWater(uint8_t channels_num, ...);

#endif //F4_FRAMEWORK_VOFA_H