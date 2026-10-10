//
// Created by lingxi on 2026/10/8.
//

#ifndef A_SMALL_CAR_GYROSCOPE_H
#define A_SMALL_CAR_GYROSCOPE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// MPU6050 7位物理地址 (AD0=GND)
#define MPU6050_ADDR                (0x68 << 1)

// 关键寄存器地址
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_WHO_AM_I            0x75
#define MPU6050_ACCEL_XOUT_H        0x3B

// 原始数据结构体
typedef struct {
    int16_t Accel_X, Accel_Y, Accel_Z;
    int16_t Temp;
    int16_t Gyro_X, Gyro_Y, Gyro_Z;
} MPU6050_Data_t;

// 函数声明
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

// 姿态解算与航向
void  Gyro_Update(void);
float Gyro_WrapDeg(float a);

#endif //A_SMALL_CAR_GYROSCOPE_H
