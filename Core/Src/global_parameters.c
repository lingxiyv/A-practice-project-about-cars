//
// Created by lingxi on 2026/9/23.
//
#include "global_parameters.h"

#include <stdio.h>

car_state g_car_state;

void car_state_init(void) {
    g_car_state.current_motion_state = CAR_STOP;
    g_car_state.linear_speed_target_cm_s = 0;
    g_car_state.linear_speed_cm_s = 0;
    g_car_state.left_speed_cm_s = 0;
    g_car_state.right_speed_cm_s = 0;
    g_car_state.angular_speed = 0;
    g_car_state.voltage_current = 0;
    g_car_state.left_last_count = 0;
    g_car_state.right_last_count = 0;
    g_car_state.sum = 0;
}

// 定义缓冲区实体
Bt_RingBuffer_t g_bt_rx_buf = {0};

// 蓝牙写入数据
void Bt_RingBuf_Put(uint8_t data) {
    uint16_t next_head = (g_bt_rx_buf.head + 1) % BT_RX_BUF_SIZE;
    if (next_head != g_bt_rx_buf.tail) { // 缓冲区未满
        g_bt_rx_buf.buffer[g_bt_rx_buf.head] = data;
        g_bt_rx_buf.head = next_head;
    }
    g_car_state.sum++;
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