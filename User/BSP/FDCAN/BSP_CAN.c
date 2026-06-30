#include "BSP_CAN.h"
#include "BSP_DWT.h"

/**
 * @brief CAN外设配置函数（经典CAN模式）
 * @param hcan      CAN句柄
 * @param fifo      选择接收FIFO（CAN_RX_FIFO0 或 CAN_RX_FIFO1）
 * @note 配置过滤器接收所有标准帧，开启对应的FIFO中断，启动外设
 */
void CAN_Config(hcan_t *hcan, uint32_t fifo)
{
    // 重置并初始化CAN外设
    if (HAL_CAN_DeInit(hcan) != HAL_OK) Error_Handler();
    if (HAL_CAN_Init(hcan) != HAL_OK) Error_Handler();

    // 配置过滤器（接收所有标准帧）
    CAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.FilterBank = 0;                     // 使用过滤器组0（根据MCU调整）
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;              // 掩码为0，接收所有ID
    sFilterConfig.FilterIdLow  = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow  = 0x0000;
    sFilterConfig.FilterFIFOAssignment = (fifo == CAN_RX_FIFO0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) Error_Handler();

    // 开启中断：使能FIFO消息挂起中断（无需额外全局过滤配置）
    // 中断由NVIC使能，此处仅启动外设并激活通知
    if (HAL_CAN_Start(hcan) != HAL_OK) Error_Handler();
    // 激活RX中断通知（使能FIFO水印等，这里使用标准回调）
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) Error_Handler();
}

/**
 * @brief 字节长度转换为DLC（经典CAN仅支持0~8字节）
 */
uint32_t Bytes_To_DLC(uint32_t len) {
    if (len > 8) len = 8;   // 经典CAN最大8字节
    return len;             // 在HAL_CAN中，DLC直接等于数据长度（0~8）
}

/**
 * @brief DLC转换为字节长度（经典CAN）
 */
uint8_t DLC_To_Bytes(uint32_t dlc) {
    return (dlc > 8) ? 8 : (uint8_t)dlc;
}

/**
 * @brief CAN发送通用函数
 */
uint8_t CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
    CAN_TxHeaderTypeDef TxHeader = {
        .DLC = Bytes_To_DLC(len),
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .StdId = id,
        .ExtId = 0,
        .TransmitGlobalTime = DISABLE
    };
    uint32_t txMailbox;
    return (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &txMailbox) == HAL_OK) ? 0 : 1;
}

/**
 * @brief CAN错误回调函数（Bus-Off恢复）
 */
void HAL_CAN_ErrorCallback(hcan_t *hcan)
{
    // 检查是否进入Bus-Off状态
    if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_ERROR)
    {
        static uint32_t last_recovery_time[3] = {0};
        uint32_t now = HAL_GetTick();

        uint8_t idx = 0;
        uint32_t target_fifo = CAN_RX_FIFO0;
        if (hcan->Instance == CAN1) {
            idx = 0; target_fifo = CAN_RX_FIFO0;
        } else if (hcan->Instance == CAN2) {
            idx = 1; target_fifo = CAN_RX_FIFO1;
        } else {
            return;
        }

        if ((now - last_recovery_time[idx]) > 100) {
            last_recovery_time[idx] = now;
            HAL_CAN_Stop(hcan);
            if (HAL_CAN_Start(hcan) == HAL_OK) {
                HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
                                             CAN_IT_RX_FIFO1_MSG_PENDING);
            }
        }
    }
}


extern const Auto_CAN_Reg_t __start_CAN_Reg_Sec;
extern const Auto_CAN_Reg_t __stop_CAN_Reg_Sec;

void BSP_CAN_Auto_Init(void)
{
    const Auto_CAN_Reg_t *node = &__start_CAN_Reg_Sec;
    for (; node < &__stop_CAN_Reg_Sec; node++)
    {
        // 注意：由于现在在 BSP 内部了，你可以直接操作 BSP_Hash_Table
        // 甚至连 BSP_CAN_Register_Slot 这个函数都不用对外暴露了！
        hcan_t temp_hcan = { .Instance = node->instance };
        BSP_CAN_Register_Slot(&temp_hcan, node->id, node->device_ptr, node->resolve);
    }
}


/* 哈希表 */
#define CAN_HASH_SIZE       16
#define CAN_HASH_MASK       (CAN_HASH_SIZE - 1)
#define CAN_BUS_NUM         3

static BSP_CAN_Hash_Node_t BSP_Hash_Table[CAN_BUS_NUM][CAN_HASH_SIZE] = {0};

static inline uint8_t Get_CAN_Bus_Index(hcan_t *hcan)
{
    if (hcan->Instance == CAN1) return 0;
    if (hcan->Instance == CAN2) return 1;

    return 0;
}

void BSP_CAN_Register_Slot(hcan_t *hcan, uint32_t id, void *device_ptr, void (*callback)(void*, uint8_t*))
{
    uint8_t bus_idx = Get_CAN_Bus_Index(hcan);
    uint32_t hash_idx = id & CAN_HASH_MASK;
    uint32_t start_idx = hash_idx;
    while (BSP_Hash_Table[bus_idx][hash_idx].id != 0) {
        if (BSP_Hash_Table[bus_idx][hash_idx].id == id) break;
        hash_idx = (hash_idx + 1) & CAN_HASH_MASK;
        if (hash_idx == start_idx) return;
    }
    BSP_Hash_Table[bus_idx][hash_idx].id = id;
    BSP_Hash_Table[bus_idx][hash_idx].device_ptr = device_ptr;
    BSP_Hash_Table[bus_idx][hash_idx].resolve = callback;
}

void CAN_App_Frame_Dispatch(hcan_t *hcan, uint32_t identifier, uint8_t *data, uint32_t len)
{
    (void)len;
    uint8_t bus_idx = Get_CAN_Bus_Index(hcan);
    uint32_t hash_idx = identifier & CAN_HASH_MASK;
    uint32_t start_idx = hash_idx;
    while (BSP_Hash_Table[bus_idx][hash_idx].id != 0) {
        if (BSP_Hash_Table[bus_idx][hash_idx].id == identifier) {
            if (BSP_Hash_Table[bus_idx][hash_idx].resolve != NULL) {
                BSP_Hash_Table[bus_idx][hash_idx].resolve(BSP_Hash_Table[bus_idx][hash_idx].device_ptr, data);
            }
            return;
        }
        hash_idx = (hash_idx + 1) & CAN_HASH_MASK;
        if (hash_idx == start_idx) return;
    }
}

CAN_Stats_t can1_stats;
CAN_Stats_t can2_stats;
CAN_Stats_t can3_stats;

/* FIFO处理函数（由于HAL_CAN回调不带ITs，直接读取所有待处理消息） */
static inline void CAN_Rx_FIFO_Process(hcan_t *hcan, uint32_t fifo, CAN_Stats_t *stats)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];   // 经典CAN最大8字节
    while (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, rx_data) == HAL_OK) {
        if (stats) stats->rx_count++;
        CAN_App_Frame_Dispatch(hcan, rx_header.StdId, rx_data, rx_header.DLC);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t *hcan)
{
    CAN_Stats_t *stats = (hcan->Instance == CAN1) ? &can1_stats : NULL;
    CAN_Rx_FIFO_Process(hcan, CAN_RX_FIFO0, stats);
}

void HAL_CAN_RxFifo1MsgPendingCallback(hcan_t *hcan)
{
    CAN_Stats_t *stats = (hcan->Instance == CAN2) ? &can2_stats : NULL;
    CAN_Rx_FIFO_Process(hcan, CAN_RX_FIFO1, stats);
}