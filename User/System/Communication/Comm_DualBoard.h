//
// Created by CaoKangqi on 2026/6/22.
//

#ifndef H7_FRAMEWORK_COMM_DUALBOARD_H
#define H7_FRAMEWORK_COMM_DUALBOARD_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LINK_CAN,
    LINK_UART
} Comm_Link_e;

#define DUALBOARD_MAX_PAYLOAD 256

/* 定义接收回调函数指针类型 */
typedef void (*DualBoard_Rx_Callback_t)(Comm_Link_e link, uint8_t *data, uint16_t len);

/* 双板 CAN 通信硬件与 ID 配置结构体 */
typedef struct {
    void* can_handle;  // 绑定的 CAN 句柄指针 (例如 &hcan1, &hcan2)
    uint32_t tx_id;       // 发送数据的 CAN ID (例如 0x500)
    uint32_t rx_id;       // 接收数据的 CAN ID (例如 0x501)
} DualBoard_Config_t;

/* 初始化时传入接收回调和硬件配置 */
void DualBoard_Comm_Init(DualBoard_Rx_Callback_t rx_cb, DualBoard_Config_t *config);

uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len);

/* 接收端增加 rx_id 参数用于过滤校验 */
void DualBoard_CAN_Rx(void *can_handle, uint32_t rx_id, uint8_t *data);
void DualBoard_UART_Rx(uint8_t *rx_buf, uint16_t len);

void DualBoard_Task_Poll(void);

#endif //H7_FRAMEWORK_COMM_DUALBOARD_H