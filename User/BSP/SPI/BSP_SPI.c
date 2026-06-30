//
// Created by CaoKangqi on 2026/6/13.
//
#include "BSP_SPI.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
#define BMI088_SPI_HANDLE &hspi2

void BSP_SPI_Accel_CS(uint8_t state) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, (GPIO_PinState)state);
}

void BSP_SPI_Gyro_CS(uint8_t state) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, (GPIO_PinState)state);
}

HAL_StatusTypeDef BSP_SPI_Transmit(const uint8_t *data, uint16_t size, uint32_t timeout) {
    return HAL_SPI_Transmit(BMI088_SPI_HANDLE, (uint8_t *)data, size, timeout);
}

HAL_StatusTypeDef BSP_SPI_Receive(uint8_t *data, uint16_t size, uint32_t timeout) {
    return HAL_SPI_Receive(BMI088_SPI_HANDLE, data, size, timeout);
}

SPI_TypeDef *BSP_SPI_GetInstance(void) {
    return ((SPI_HandleTypeDef *)BMI088_SPI_HANDLE)->Instance;
}