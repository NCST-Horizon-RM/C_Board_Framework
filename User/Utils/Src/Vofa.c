//
// Created by CaoKangqi on 2026/1/19.
//
#include "Vofa.h"
#include <stdarg.h>
#include <string.h>
#include "usart.h"
#include "usbd_cdc_if.h"

/**
 * @brief VOFA+ JustFloat 协议可变参数发送函数
 * @param channels_num 实际发送的通道数量 (1 ~ VOFA_MAX_CHANNELS)
 * @param ... 具体的 float 数据点
 */
void VOFA_JustFloat(uint8_t channels_num, ...)
{
    if (channels_num == 0 || channels_num > VOFA_MAX_CHANNELS) return;

    static uint8_t send_buf[(VOFA_MAX_CHANNELS * 4) + 4];

    va_list args;
    va_start(args, channels_num);

    float temp_data;
    for (uint8_t i = 0; i < channels_num; i++) {
        temp_data = (float)va_arg(args, double);
        memcpy(&send_buf[i * 4], &temp_data, 4);
    }
    va_end(args);

    uint32_t tail_index = channels_num * 4;
    send_buf[tail_index]     = 0x00;
    send_buf[tail_index + 1] = 0x00;
    send_buf[tail_index + 2] = 0x80;
    send_buf[tail_index + 3] = 0x7F;

    HAL_UART_Transmit_DMA(&huart1,send_buf,tail_index + 4);
    //CDC_Transmit_FS(send_buf, tail_index + 4);
}

/**
 * @brief VOFA+ FireWater 协议可变参数发送函数
 * @details 格式示例: "1.1,3.2,-0.6\n" 或者是 "ch_name:1.1,3.2\n"
 * 遇到换行符 '\n' 时 VOFA+ 才会渲染打印波形。
 */
void VOFA_FireWater(uint8_t channels_num, ...)
{
    if (channels_num == 0) return;

    static char text_buf[VOFA_TEXT_BUF_SIZE];
    uint32_t str_len = 0;

    va_list args;
    va_start(args, channels_num);
    for (uint8_t i = 0; i < channels_num; i++) {
        float temp_data = (float)va_arg(args, double);

        int written = snprintf(&text_buf[str_len], VOFA_TEXT_BUF_SIZE - str_len,
                               (i == channels_num - 1) ? "%.4f" : "%.4f,", temp_data);

        if (written < 0 || (str_len + written) >= VOFA_TEXT_BUF_SIZE - 2) {
            break;
        }
        str_len += written;
    }
    va_end(args);

    text_buf[str_len++] = '\n';
    text_buf[str_len]   = '\0';

    HAL_UART_Transmit_DMA(&huart1,(uint8_t*)text_buf, str_len);
    //CDC_Transmit_FS((uint8_t*)text_buf, str_len);
}