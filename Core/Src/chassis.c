//
// Created by lingxi on 2026/9/16.
//
#include "chassis.h"

#include "oled.h"
#include "stdio.h"
#include "string.h"

// 主循环解析逻辑
// 蓝牙发送的指令以 '\n' (0x0A) 结尾
static uint32_t last_rx_tick = 0; // 记录上一次收到数据的时间戳
static uint8_t bt_speed_percent = 0;
#define TIMEOUT_MS 150  // 超时阈值

void Bt_Parse_Commands(void) {
    static char cmd_line[64];
    static uint8_t idx = 0;
    uint32_t current_tick = HAL_GetTick();

    if (!Bt_RingBuf_IsEmpty()) {
        last_rx_tick = current_tick;
        char ch = (char)Bt_RingBuf_Get();

        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                cmd_line[idx] = '\0';

                state_motion target_state = g_car_state.current_motion_state;
                bool is_motion_cmd = false;
                int val = 0;

                if (strcmp(cmd_line, "W") == 0) {
                    target_state = CAR_FORWARD;  is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "A") == 0) {
                    target_state = CAR_LEFT_TURN; is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "S") == 0) {
                    target_state = CAR_BACKWARD; is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "D") == 0) {
                    target_state = CAR_RIGHT_TURN; is_motion_cmd = true;
                }

                else if (sscanf(cmd_line, "speed %d", &val) == 1) {
                    bt_speed_percent = val;
                    if (bt_speed_percent > 100) bt_speed_percent = 100;
                    g_car_state.linear_speed_target_cm_s = (int32_t)bt_speed_percent * MAX_LINEAR_SPEED_CM_S / 100;
                    //如果目标速度极小，说明想停车
                    if (g_car_state.linear_speed_target_cm_s < 30) {
                        g_car_state.linear_speed_target_cm_s = 0;
                    }
                }
                else if (sscanf(cmd_line, "pwmleft %d", &val) == 1) {
                }
                else if (sscanf(cmd_line, "pwmright %d", &val) == 1) {

                }
                if (is_motion_cmd && target_state != g_car_state.current_motion_state) {
                    SET_Action_execution(target_state);
                }

                idx = 0;
            }
        } else {
            if (idx < sizeof(cmd_line) - 1) {
                cmd_line[idx++] = ch;
            }
        }
    }
    else {
        if ((current_tick - last_rx_tick) > TIMEOUT_MS) {
            if (g_car_state.current_motion_state != CAR_STOP) {
                SET_Action_execution(CAR_STOP);
            }
        }
    }
}

void SET_Action_execution(state_motion car_state) {
    g_car_state.current_motion_state = car_state;
    switch (car_state) {
    case CAR_FORWARD: {
        oled_show_string(52,0,"FORWARD  ",12);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    }
    case CAR_BACKWARD: {
        oled_show_string(52,0,"BACKWARD ",12);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_SET);
        break;
    }
    case CAR_LEFT_TURN: {
        oled_show_string(52,0,"LEFT TURN",12);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    }
    case CAR_RIGHT_TURN: {
        oled_show_string(52,0,"RIGHT TURN",12);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_SET);
        break;
    }
    default: {
        // 停止
        oled_show_string(52,0,"STOP      ",12);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    }
    }
}

void PWM_automatic_adjustment(void) {
    // 蓝牙百分比 -> 目标物理速度 (cm/s)
    g_car_state.linear_speed_target_cm_s = (int32_t)bt_speed_percent * MAX_LINEAR_SPEED_CM_S / 100;

    // 根据目标速度，计算一个“基础 PWM”（前馈控制）
    // 一个简单的线性映射：目标速度占最大速度的百分比 = PWM 占最大 PWM 的百分比
    // 即使 Kp=0，电机也能以大致正确的速度跑起来
    int32_t base_pwm = (int32_t)g_car_state.linear_speed_target_cm_s * PWM_MAX_LIMIT / MAX_LINEAR_SPEED_CM_S;

    // 计算左右轮的物理速度误差 (cm/s)
    int32_t left_error  = g_car_state.linear_speed_target_cm_s - g_car_state.left_speed_cm_s;
    int32_t right_error = g_car_state.linear_speed_target_cm_s - g_car_state.right_speed_cm_s;

    // 用物理误差计算“修正量”
    int32_t left_pwm_adjust  = left_error * SPEED_KP;
    int32_t right_pwm_adjust = right_error * SPEED_KP;

    // 基础 PWM + 修正量 = 最终 PWM
    int32_t final_left_pwm  = base_pwm + left_pwm_adjust;
    int32_t final_right_pwm = base_pwm + right_pwm_adjust;

    // 绝对限幅保护
    if (final_left_pwm > PWM_MAX_LIMIT) final_left_pwm = PWM_MAX_LIMIT;
    if (final_left_pwm < PWM_MIN_LIMIT) final_left_pwm = PWM_MIN_LIMIT;
    if (final_right_pwm > PWM_MAX_LIMIT) final_right_pwm = PWM_MAX_LIMIT;
    if (final_right_pwm < PWM_MIN_LIMIT) final_right_pwm = PWM_MIN_LIMIT;

    // 输出给定时器
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)final_right_pwm);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)final_left_pwm);
}