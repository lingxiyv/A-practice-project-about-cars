//
// Created by lingxi on 2026/9/23.
//
#include "global_parameters.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "usart.h"
car_state g_car_state;

void car_state_init(void) {
    g_car_state.IS_IIC_BUSY = 0;

    g_car_state.current_motion_state = CAR_STOP;
    g_car_state.current_mode = MODE_IDLE;
    g_car_state.prev_mode = MODE_IDLE;
    g_car_state.avoid_sub_state = AVOID_IDLE;
    g_car_state.linear_speed_target_cm_s = 0;
    g_car_state.bt_speed_percent = 0;
    g_car_state.linear_speed_cm_s = 0;
    g_car_state.left_speed_cm_s = 0;
    g_car_state.right_speed_cm_s = 0;
    g_car_state.angular_speed = 0;
    g_car_state.left_last_count = 0;
    g_car_state.right_last_count = 0;
    g_car_state.BT_last_tick = 0;
    g_car_state.roam_state_tick = 0;

    g_car_state.yaw_deg = 0.0f;
    g_car_state.target_yaw_deg = 0.0f;
    g_car_state.pitch_deg = 0;
    g_car_state.steer_cm_s = 0;
    g_car_state.gyro_valid = 0;

    g_car_state.turn_start_yaw  = 0.0f;
    g_car_state.turn_target_deg = 0;

    g_car_state.g_scan.state = SCAN_IDLE;
    g_car_state.g_scan.data_valid = 0;
    for (int i = 0; i < 7; i++) {
        g_car_state.g_scan.angles[i] = i*30;
        g_car_state.g_scan.distances[i] = 0;
    }
    g_car_state.g_scan.state_tick = 0;
    g_car_state.g_scan.angle_idx  = 0;
    g_car_state.g_scan.rx_active  = 0;
    g_car_state.g_scan.rx_error   = 0;
    g_car_state.g_scan.rx_total   = 0;
    g_car_state.g_scan.rx_mark    = 0;
    g_car_state.g_scan.rx_buf[0]  = 0;
    g_car_state.g_scan.rx_buf[1]  = 0;
    g_car_state.g_scan.hold_until = 0;
    g_car_state.g_scan.continuous = SCAN_CONTINUOUS;
    g_car_state.g_scan.ok_cnt     = 0;
    g_car_state.g_scan.err_cnt    = 0;
}

// 定义缓冲区实体
Bt_RingBuffer_t g_bt_rx_buf = {0};

// 蓝牙写入数据
void Bt_RingBuf_Put(uint8_t data) {
    g_car_state.BT_last_tick = HAL_GetTick();
    uint16_t next_head = (g_bt_rx_buf.head + 1) % BT_RX_BUF_SIZE;
    if (next_head != g_bt_rx_buf.tail) { // 缓冲区未满
        g_bt_rx_buf.buffer[g_bt_rx_buf.head] = data;
        g_bt_rx_buf.head = next_head;
    }
    // 如果满了，选择丢弃以保护旧数据
}

// 读取数据（主循环调用）
uint8_t Bt_RingBuf_Get(void) {
    if (Bt_RingBuf_IsEmpty()) return 0;
    __disable_irq();
    uint8_t data = g_bt_rx_buf.buffer[g_bt_rx_buf.tail];
    g_bt_rx_buf.tail = (g_bt_rx_buf.tail + 1) % BT_RX_BUF_SIZE;
    __enable_irq();
    return data;
}

// 获取长度与判空
uint16_t Bt_RingBuf_GetCount(void) {
    return (g_bt_rx_buf.head - g_bt_rx_buf.tail + BT_RX_BUF_SIZE) % BT_RX_BUF_SIZE;
}
bool Bt_RingBuf_IsEmpty(void) {
    return g_bt_rx_buf.head == g_bt_rx_buf.tail;
}

void Speed_Calculation_ISR(void) {

    uint16_t right_now_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t left_now_count  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    int16_t right_delta = right_now_count - g_car_state.right_last_count;
    int16_t left_delta  = left_now_count - g_car_state.left_last_count;

    g_car_state.right_last_count = right_now_count;
    g_car_state.left_last_count  = left_now_count;

    g_car_state.left_speed_cm_s  = (int32_t)left_delta * SPEED_CALC_COEFF / SPEED_CALC_DIV;
    g_car_state.right_speed_cm_s = (int32_t)right_delta * SPEED_CALC_COEFF / SPEED_CALC_DIV;
    // 计算底盘中心线速度
    g_car_state.linear_speed_cm_s = (g_car_state.left_speed_cm_s + g_car_state.right_speed_cm_s) / 2;
}

/* 蓝牙发送缓冲与状态。
   原来是 HAL_UART_Transmit 阻塞发送：一条 60~100 字节的状态回传 @9600 要堵住主循环
   60~100ms，陀螺仪/OLED/电机控制全被卡住。改成中断发送，主循环最多只等 30ms。 */
static char g_bt_tx_buf[BT_RX_BUF_SIZE];
static volatile uint8_t g_bt_tx_busy = 0;
static uint32_t g_bt_tx_tick = 0;

void Bt_Tx_Complete_Callback(void) {
    g_bt_tx_busy = 0;
}

void Bt_Printf(const char* fmt, ...) {
    va_list args;
    int len;

    /* 发送卡死兜底：正常几毫秒就发完了，超过 BT_TX_TIMEOUT_MS 说明出问题了 */
    if (g_bt_tx_busy && (HAL_GetTick() - g_bt_tx_tick) > BT_TX_TIMEOUT_MS) {
        HAL_UART_AbortTransmit(&huart2);
        g_bt_tx_busy = 0;
    }

    /* 上一条还没发完：最多等 30ms，超了就放弃这一条，绝不长时间阻塞主循环 */
    if (g_bt_tx_busy) {
        uint32_t start = HAL_GetTick();
        while (g_bt_tx_busy && (HAL_GetTick() - start) < 30) {
        }
        if (g_bt_tx_busy) return;   // 丢掉这一条总比堵住主循环好
    }

    va_start(args, fmt);
    len = vsnprintf(g_bt_tx_buf, sizeof(g_bt_tx_buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len > (int)sizeof(g_bt_tx_buf) - 1) len = sizeof(g_bt_tx_buf) - 1;  // 截断保护

    g_bt_tx_busy = 1;
    g_bt_tx_tick = HAL_GetTick();
    if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)g_bt_tx_buf, (uint16_t)len) != HAL_OK) {
        g_bt_tx_busy = 0;   // 启动失败就直接放弃
    }
}