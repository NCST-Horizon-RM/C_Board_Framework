//
// Created by CaoKangqi on 2026/6/13.
//

#ifndef F4_FRAMEWORK_BSP_SPI_H
#define F4_FRAMEWORK_BSP_SPI_H

#include "stm32f4xx_hal.h"

void BSP_SPI_Accel_CS(uint8_t state);
void BSP_SPI_Gyro_CS(uint8_t state);

HAL_StatusTypeDef BSP_SPI_Transmit(const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef BSP_SPI_Receive(uint8_t *data, uint16_t size, uint32_t timeout);
SPI_TypeDef *BSP_SPI_GetInstance(void);

#endif //F4_FRAMEWORK_BSP_SPI_H
