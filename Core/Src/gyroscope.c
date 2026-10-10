//
// Created by lingxi on 2026/10/8.
//
#include "gyroscope.h"
#include "global_parameters.h"
#include "i2c.h"

/**
 * @brief 初始化 MPU6050 (带软复位和严格校验)
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check_id = 0;
    uint8_t retries = 3;

    Bt_Printf("MPU6050 Init Start\r\n");
    if (g_car_state.IS_IIC_BUSY) return 2;
    g_car_state.IS_IIC_BUSY = 1;

    HAL_I2C_DeInit(hi2c);
    HAL_Delay(10);
    HAL_I2C_Init(hi2c);
    HAL_Delay(50); // 上电等待，让 MPU6050 内部振荡器稳定

    do {
        // 验证设备 ID
        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I,
                             I2C_MEMADD_SIZE_8BIT, &check_id, 1, 200) == HAL_OK) {
            if (check_id == 0x68 || check_id == 0x69) {

                // 触发软复位，清除所有寄存器到出厂默认状态
                uint8_t reset_cmd = 0x80;
                HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                                  I2C_MEMADD_SIZE_8BIT, &reset_cmd, 1, 200);
                HAL_Delay(100); // 软复位后，芯片需要约 100ms 重启

                // 唤醒 MPU6050 (写入 0x00 退出睡眠模式)
                uint8_t wake_cmd = 0x00;
                HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                                  I2C_MEMADD_SIZE_8BIT, &wake_cmd, 1, 200);

                // 回读验证是否真正唤醒成功
                uint8_t pwr_check = 0xFF;
                HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                                 I2C_MEMADD_SIZE_8BIT, &pwr_check, 1, 100);

                // 如果 SLEEP 位 (Bit 6) 为 0，说明彻底唤醒成功
                if ((pwr_check & 0x40) == 0) {
                    HAL_Delay(10); // 给内部 PLL 时钟源一点稳定时间
                    g_car_state.IS_IIC_BUSY = 0;
                    Bt_Printf("MPU6050 Init OK\r\n");
                    return 0;  // 初始化成功
                }
            }
                             }
        HAL_Delay(10);  // 重试间隔
    } while (--retries);

    g_car_state.IS_IIC_BUSY = 0;
    Bt_Printf("MPU6050 INIT FAILED\r\n");
    return 1;  // 初始化失败
}

/**
 * @brief 一次性读取全部 14 字节数据 (带防沉睡保底)
 */
uint8_t MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data) {
    uint8_t buffer[14] = {0};
    uint8_t pwr_reg = 0;

    // OLED 的 DMA 一次最多独占总线约 24ms（100kHz 下 262 字节）。
    // 这里最多等 30ms，等到总线空闲再读，避免被相位锁定永久饿死。
    // 只有超过 30ms 还忙（真卡死）才闪灯放弃。
    uint32_t wait_start = HAL_GetTick();
    while (g_car_state.IS_IIC_BUSY && (HAL_GetTick() - wait_start) < 30) {
    }
    if (g_car_state.IS_IIC_BUSY) {
        return 2;
    }
    g_car_state.IS_IIC_BUSY = 1;

    // 读取电源寄存器，如果处于睡眠模式(0x40)，强制唤醒
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                         I2C_MEMADD_SIZE_8BIT, &pwr_reg, 1, 100) == HAL_OK) {
        if (pwr_reg & 0x40) { // 检查 SLEEP 位是否为 1
            uint8_t wake_cmd = 0x00;
            HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                              I2C_MEMADD_SIZE_8BIT, &wake_cmd, 1, 100);
            HAL_Delay(10); // 唤醒后给芯片一点时间启动内部时钟
        }
    }

    // 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 字节
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, buffer, 14, 100) != HAL_OK) {
        g_car_state.IS_IIC_BUSY = 0;
        Bt_Printf("MPU6050 Read FAILED\r\n");
        return 1;
    }

    // 拼接高低字节 (MPU6050 是高字节在前)，【注意】必须强转为 int16_t
    data->Accel_X = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->Accel_Y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->Accel_Z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->Temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->Gyro_X  = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->Gyro_Y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->Gyro_Z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    g_car_state.IS_IIC_BUSY = 0;
    return 0;
}

/**
 * @brief 航向角度归一化到 [-180, 180)
 */
float Gyro_WrapDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/**
 * @brief 周期读取 MPU6050，积分 Z 轴陀螺仪得到航向，X 轴加速度估计坡度
 * @note  需以固定周期(约10ms)在循环中调用
 */
void Gyro_Update(void) {
    static uint32_t last_gyro_tick = 0;
    MPU6050_Data_t data;

    if (MPU6050_ReadAll(&hi2c1, &data) != 0) {
        g_car_state.gyro_valid = 0;
        return;
    }
    g_car_state.gyro_valid = 1;

    uint32_t now = HAL_GetTick();
    if (last_gyro_tick == 0) {
        last_gyro_tick = now;   // 首次只取基准时间，不做积分
        return;
    }
    float dt = (float)(now - last_gyro_tick) / 1000.0f;
    last_gyro_tick = now;

    // Z 轴角速度积分 -> 航向（正 = 左转）
    float yaw_rate = (float)data.Gyro_Z / GYRO_LSB_PER_DPS;
    g_car_state.yaw_deg = Gyro_WrapDeg(g_car_state.yaw_deg + yaw_rate * dt);

    // X 轴加速度 -> 俯仰角(坡度)，小角度线性近似: pitch ≈ asin(ax/g)
    g_car_state.pitch_deg = (int16_t)((float)data.Accel_X *
                            (180.0f / (ACCEL_LSB_PER_G * 3.14159265f)));
}