//
// Created by CaoKangqi on 2026/6/22.
//
#include "Comm_DualBoard.h"
#include "BSP_CAN.h"
#include "BSP_DWT.h"
#include <string.h>

// CAN 传输层切片状态机
typedef struct {
    uint8_t  buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t total_len;
    uint16_t offset;
    uint8_t  seq;
    uint32_t last_tick;
    bool     is_sending;
} CAN_TP_Tx_State_t;

typedef struct {
    uint8_t  buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t current_len;
    uint8_t  next_seq;
    bool     is_active;
} CAN_TP_Rx_t;

static CAN_TP_Tx_State_t can_tx = {0};
static CAN_TP_Rx_t       can_rx = {0};

/* 保存应用层注册的回调函数和硬件配置 */
static DualBoard_Rx_Callback_t App_Rx_Callback = NULL;
static DualBoard_Config_t      Comm_Config     = {0};

void DualBoard_Comm_Init(DualBoard_Rx_Callback_t rx_cb, DualBoard_Config_t *config)
{
    App_Rx_Callback = rx_cb;

    if (config != NULL) {
        Comm_Config.can_handle = config->can_handle;
        Comm_Config.tx_id      = config->tx_id;
        Comm_Config.rx_id      = config->rx_id;
    }
}

uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len)
{
    if (data_ptr == NULL || len > DUALBOARD_MAX_PAYLOAD || len == 0) return 1;

    if (link == LINK_UART) {
        // 串口发送逻辑
        return 0;
    }
    if (link == LINK_CAN) {
        // 如果没有配置 CAN 句柄，拒绝发送
        if (Comm_Config.can_handle == NULL) return 3;

        if (can_tx.is_sending) return 2; // 正在发送，返回忙碌状态
        can_tx.total_len = len;
        memcpy(can_tx.buf, data_ptr, len);
        can_tx.offset = 0;
        can_tx.seq = 0;
        can_tx.is_sending = true;
        return 0;
    }
    return 1;
}

void DualBoard_Task_Poll(void)
{
    if (!can_tx.is_sending) return;

    while (can_tx.is_sending) {
        uint8_t tx_buf[8];
        uint16_t remain = can_tx.total_len - can_tx.offset;
        uint8_t chunk_size = (remain > 7) ? 7 : (uint8_t)remain;
        uint8_t is_last = (remain <= 7) ? 1 : 0;

        tx_buf[0] = (is_last << 7) | (can_tx.seq & 0x7F);
        memcpy(&tx_buf[1], &can_tx.buf[can_tx.offset], chunk_size);
        if (chunk_size < 7) memset(&tx_buf[1 + chunk_size], 0, 7 - chunk_size);
        // 尝试发送
        if (CAN_Send_Msg(Comm_Config.can_handle, Comm_Config.tx_id, tx_buf, 8) == 0) {
            can_tx.offset += chunk_size;
            can_tx.seq++;

            if (is_last) {
                can_tx.is_sending = false;
                break;
            }
        } else {
            break;
        }
    }
}

void DualBoard_CAN_Rx(void *can_handle, uint32_t rx_id, uint8_t *data)
{
    if (data == NULL) return;

    // ★ 过滤校验：如果配置了对应的句柄和ID，不是发给我的包直接丢弃 ★
    if (Comm_Config.can_handle != NULL && can_handle != Comm_Config.can_handle) return;
    if (rx_id != Comm_Config.rx_id) return;

    uint8_t ctrl = data[0];
    uint8_t is_last = (ctrl >> 7) & 0x01;
    uint8_t seq = ctrl & 0x7F;
    uint8_t payload_len = 7;

    if (seq == 0) {
        can_rx.is_active = true;
        can_rx.current_len = 0;
        can_rx.next_seq = 0;
    }

    if (can_rx.is_active && seq == can_rx.next_seq) {
        if (can_rx.current_len + payload_len <= sizeof(can_rx.buf)) {
            memcpy(&can_rx.buf[can_rx.current_len], &data[1], payload_len);
            can_rx.current_len += payload_len;
            can_rx.next_seq++;

            if (is_last) {
                if (App_Rx_Callback != NULL) {
                    App_Rx_Callback(LINK_CAN, can_rx.buf, can_rx.current_len);
                }
                can_rx.is_active = false;
            }
        } else {
            can_rx.is_active = false;
        }
    }
}