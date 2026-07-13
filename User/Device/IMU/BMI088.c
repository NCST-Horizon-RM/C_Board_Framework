//
// Created by CaoKangqi on 2026/7/6.
//
#include "BMI088.h"

#include <math.h>
#include <string.h>
#include "All_define.h"
#include "BSP_DWT.h"

float acc_res = 0;
float gyr_res = 0;

/* * Hardware Abstraction for Dual CS Pins
 * Replace ACC_CS_PORT/PIN with your actual definitions from main.h
 */
#define ACCEL_CS(state) HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define GYRO_CS(state)  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* internal: write Accel register */
static void Accel_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t cmd[2] = {reg & 0x7F, val};
    BSP_SPI_Accel_CS(0);
    BSP_SPI_Transmit(cmd, 2, 10);
    BSP_SPI_Accel_CS(1);
}

/* internal: read Accel register */
static uint8_t Accel_ReadReg(uint8_t reg) {
    uint8_t addr = reg | 0x80;
    uint8_t val[2] = {0};
    BSP_SPI_Accel_CS(0);
    BSP_SPI_Transmit(&addr, 1, 10);
    BSP_SPI_Receive(val, 2, 10);
    BSP_SPI_Accel_CS(1);
    return val[1];
}

/* internal: write Gyro register */
static void Gyro_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t cmd[2] = {reg & 0x7F, val};
    BSP_SPI_Gyro_CS(0);
    BSP_SPI_Transmit(cmd, 2, 10);
    BSP_SPI_Gyro_CS(1);
}

/* internal: read Gyro register */
static uint8_t Gyro_ReadReg(uint8_t reg) {
    uint8_t addr = reg | 0x80, val = 0;
    BSP_SPI_Gyro_CS(0);
    BSP_SPI_Transmit(&addr, 1, 10);
    BSP_SPI_Receive(&val, 1, 10);
    BSP_SPI_Gyro_CS(1);
    return val;
}

// 二阶滤波器状态结构体
typedef struct {
    float b0, b1, b2; // 前馈系数 (Feedforward)
    float a1, a2;     // 反馈系数 (Feedback)
    float x1, x2;     // 输入历史 x[n-1], x[n-2]
    float y1, y2;     // 输出历史 y[n-1], y[n-2]
} Butterworth2nd_t;

// 滤波函数
float Butterworth2nd_Apply(Butterworth2nd_t *filt, float input) {
    float output = (filt->b0 * input) + (filt->b1 * filt->x1) + (filt->b2 * filt->x2)
                 - (filt->a1 * filt->y1) - (filt->a2 * filt->y2);
    filt->x2 = filt->x1;
    filt->x1 = input;
    filt->y2 = filt->y1;
    filt->y1 = output;
    return output;
}

// 根据采样率 (fs) 和截止频率 (fc) 动态计算参数并初始化
void Butterworth2nd_Init_ByFreq(Butterworth2nd_t *filt, float fs, float fc) {
    // 根据奈奎斯特采样定理，截止频率不能超过采样率的一半
    if (fc >= fs / 2.0f) {
        fc = (fs / 2.0f) - 1.0f;
    }
    // 双线性变换 (Bilinear Transform)
    float omega_c = tanf(PI * fc / fs);
    float omega_c2 = omega_c * omega_c;
    float sqrt2_omega_c = 1.414213562373095f * omega_c; // sqrt(2) * omega_c
    // 归一化常数
    float N = 1.0f + sqrt2_omega_c + omega_c2;
    // 计算前馈系数 b
    filt->b0 = omega_c2 / N;
    filt->b1 = 2.0f * filt->b0;
    filt->b2 = filt->b0;
    // 计算反馈系数 a
    filt->a1 = 2.0f * (omega_c2 - 1.0f) / N;
    filt->a2 = (1.0f - sqrt2_omega_c + omega_c2) / N;
    // 清零历史状态
    filt->x1 = 0.0f; filt->x2 = 0.0f;
    filt->y1 = 0.0f; filt->y2 = 0.0f;
}

static Butterworth2nd_t bmi088_accel_filter[3];
static Butterworth2nd_t bmi088_gyro_filter[3];
static uint8_t is_filter_initialized = 0;

void BMI088_Filter_Init(float sys_fs,float acc_fc,float gyr_fc) {
    for(int i = 0; i < 3; i++) {
        Butterworth2nd_Init_ByFreq(&bmi088_accel_filter[i], sys_fs, acc_fc);
        Butterworth2nd_Init_ByFreq(&bmi088_gyro_filter[i],  sys_fs, gyr_fc);
    }
    is_filter_initialized = 1;
}

uint8_t BMI088_Init(void) {
    uint8_t dummy = 0;
    // ================= Accel 启动流修正 =================
    // 1. 向 Accel 发送盲读，强制使其从 I2C 切换为 SPI 模式
    ACCEL_CS(0);
    uint8_t dummy_addr = REG_ACC_CHIP_ID | 0x80;
    BSP_SPI_Transmit(&dummy_addr, 1, 10);
    BSP_SPI_Receive(&dummy, 1, 10); // 触发 CSB1 上升沿
    ACCEL_CS(1);
    DWT_Delay_ms(5);
    // 2. 检查 Accel 芯片 ID
    if (Accel_ReadReg(REG_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID_VAL) {
        return 1;
    }
    // 3. Accel 软复位
    Accel_WriteReg(REG_ACC_SOFTRESET, BMI088_SOFTRESET_VAL);
    DWT_Delay_ms(50);

    // 先写 ACC_PWR_CONF(0x7C) = 0x00，从挂起切到活动
    Accel_WriteReg(REG_ACC_PWR_CONF, 0x00);
    DWT_Delay_ms(10);
    // 再写 ACC_PWR_CTRL(0x7D) = 0x04，开启加速度计主电源
    Accel_WriteReg(REG_ACC_PWR_CTRL, 0x04);
    DWT_Delay_ms(50);
    // ================= Gyro 启动流 =================
    // 5. 检查 Gyro 芯片 ID
    if (Gyro_ReadReg(REG_GYRO_CHIP_ID) != BMI088_GYRO_CHIP_ID_VAL) {
        return 2;
    }
    // 6. Gyro 软复位
    Gyro_WriteReg(REG_GYRO_SOFTRESET, BMI088_SOFTRESET_VAL);
    DWT_Delay_ms(50);
    // 7. 唤醒 Gyro (写 0x00 进入 Normal Mode)
    Gyro_WriteReg(REG_GYRO_LPM1, 0x00);
    DWT_Delay_ms(30);  // Gyro 唤醒需要 30ms 稳定
    // 8. 配置量程与采样率
    BMI088_SetFormat(ACC_ODR_1600Hz, ACCEL_FS_6G, GYR_ODR_1000Hz_BW_116Hz, GYRO_FS_2000DPS);

    // ================= 中断配置 =================
    // Accel INT1: 推挽输出(0<<1), 低电平有效(0<<2), 开启中断(1<<3)
    Accel_WriteReg(REG_ACC_INT1_IO_CTRL, 0x08);
    Accel_WriteReg(REG_ACC_INT_MAP_DATA, (1 << 2)); // 映射 DRDY 到 INT1
    // Gyro INT3: 推挽输出, 低电平有效 -> 对应 0x00
    Gyro_WriteReg(REG_GYRO_INT3_INT4_IO_CONF, 0x00);
    Gyro_WriteReg(REG_GYRO_CTRL, 0x80);             // 开启 Gyro 的 DRDY
    Gyro_WriteReg(REG_GYRO_INT3_INT4_IO_MAP, 0x01); // 映射 DRDY 到 INT3

    BMI088_Filter_Init(1000,40,100); // 初始化滤波器

    return 1;
}

void BMI088_SetFormat(AccelODR_t a_odr, AccelFS_t a_fsr, GyroODR_BW_t g_odr_bw, GyroFS_t g_fsr) {
    // Accel Config: Must set BIT 7 to 1
    Accel_WriteReg(REG_ACC_CONF, 0x80 | (0x02 << 4) | a_odr);
    Accel_WriteReg(REG_ACC_RANGE, a_fsr);

    // Gyro Config: Must set BIT 7 to 1
    Gyro_WriteReg(REG_GYRO_BANDWIDTH, 0x80 | g_odr_bw);
    Gyro_WriteReg(REG_GYRO_RANGE, g_fsr);

    // Calculate Multipliers
    acc_res = (3.0f * (float)(1 << a_fsr)) / 32768.0f * 9.81f; // 3G, 6G, 12G, 24G
    gyr_res = (2000.0f / (float)(1 << g_fsr)) / 32768.0f * DEG2RAD; // 2000, 1000, 500...
}

uint8_t BMI088_IsDataReady(void) {
     // Accel DRDY check
     return (Accel_ReadReg(REG_ACC_STATUS) & 0x80);
}

void BMI088_read(float gyro[3], float accel[3], float *temperature) {
    uint8_t a_buf[7], g_buf[6], t_buf[3];
    int16_t raw_val;

    // Read Accel (Requires 1 dummy byte)
    uint8_t a_addr = REG_ACC_XOUT_L | 0x80;
    ACCEL_CS(0);
    BSP_SPI_Transmit(&a_addr, 1, 10);
    BSP_SPI_Receive(a_buf, 7, 10); // a_buf[0] is dummy
    ACCEL_CS(1);

    // Read Gyro
    uint8_t g_addr = REG_GYRO_X_L | 0x80;
    GYRO_CS(0);
    BSP_SPI_Transmit(&g_addr, 1, 10);
    BSP_SPI_Receive(g_buf, 6, 10);
    GYRO_CS(1);

    // Read Temp
    uint8_t t_addr = REG_TEMP_M | 0x80;
    ACCEL_CS(0);
    BSP_SPI_Transmit(&t_addr, 1, 10);
    BSP_SPI_Receive(t_buf, 3, 10); // t_buf[0] is dummy
    ACCEL_CS(1);

    accel[0] = (int16_t)((a_buf[2] << 8) | a_buf[1]) * acc_res;
    accel[1] = (int16_t)((a_buf[4] << 8) | a_buf[3]) * acc_res;
    accel[2] = (int16_t)((a_buf[6] << 8) | a_buf[5]) * acc_res;

    gyro[0] = (int16_t)((g_buf[1] << 8) | g_buf[0]) * gyr_res;
    gyro[1] = (int16_t)((g_buf[3] << 8) | g_buf[2]) * gyr_res;
    gyro[2] = (int16_t)((g_buf[5] << 8) | g_buf[4]) * gyr_res;

    raw_val = (int16_t)((t_buf[1] << 3) | (t_buf[2] >> 5));
    if (raw_val > 1023) raw_val -= 2048;
    *temperature = (raw_val * 0.125f) + 23.0f;
}

void BMI088_Read_Fast(float gyro[3], float accel[3], float *temperature) {
    uint8_t a_buf[7] = {0};
    uint8_t g_buf[6] = {0};
    uint8_t t_buf[3] = {0};
    uint8_t addr;

    float raw_accel[3];
    float raw_gyro[3];

    // Read Accel: BMI088 accel SPI has one dummy byte before the 6 data bytes.
    addr = REG_ACC_XOUT_L | 0x80;
    ACCEL_CS(0);
    BSP_SPI_Transmit(&addr, 1, 10);
    BSP_SPI_Receive(a_buf, 7, 10);
    ACCEL_CS(1);

    // Read Gyro: gyro SPI returns data immediately after the address phase.
    addr = REG_GYRO_X_L | 0x80;
    GYRO_CS(0);
    BSP_SPI_Transmit(&addr, 1, 10);
    BSP_SPI_Receive(g_buf, 6, 10);
    GYRO_CS(1);

    // Read Temp separately. TEMP_M/TEMP_L are at 0x22/0x23, not after ACC_XOUT_Z.
    addr = REG_TEMP_M | 0x80;
    ACCEL_CS(0);
    BSP_SPI_Transmit(&addr, 1, 10);
    BSP_SPI_Receive(t_buf, 3, 10);
    ACCEL_CS(1);

    // 1. 获取原始物理数据 (Raw Data)
    raw_accel[0] = (int16_t)((a_buf[2] << 8) | a_buf[1]) * acc_res;
    raw_accel[1] = (int16_t)((a_buf[4] << 8) | a_buf[3]) * acc_res;
    raw_accel[2] = (int16_t)((a_buf[6] << 8) | a_buf[5]) * acc_res;

    raw_gyro[0] = (int16_t)((g_buf[1] << 8) | g_buf[0]) * gyr_res;
    raw_gyro[1] = (int16_t)((g_buf[3] << 8) | g_buf[2]) * gyr_res;
    raw_gyro[2] = (int16_t)((g_buf[5] << 8) | g_buf[4]) * gyr_res;

    // 2. 将原始数据送入滤波器
    if (is_filter_initialized) {
        for(int i = 0; i < 3; i++) {
            accel[i] = Butterworth2nd_Apply(&bmi088_accel_filter[i], raw_accel[i]);
            gyro[i]  = Butterworth2nd_Apply(&bmi088_gyro_filter[i],  raw_gyro[i]);
        }
    } else {
        // 如果未初始化，直接透传
        for(int i = 0; i < 3; i++) {
            accel[i] = raw_accel[i];
            gyro[i]  = raw_gyro[i];
        }
    }

    int16_t temp_raw_val = (int16_t)((t_buf[1] << 3) | (t_buf[2] >> 5));
    if (temp_raw_val > 1023) {
        temp_raw_val -= 2048;
    }

    *temperature = (temp_raw_val * 0.125f) + 23.0f;
}
