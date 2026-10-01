//
// Created by lingxi on 2026/9/16.
//
#include "chassis.h"

#include "oled.h"
#include "stdio.h"
#include "string.h"

// 主循环解析逻辑
// 蓝牙发送的指令以 '\n' (0x0A) 结尾

void Bt_Parse_Commands(void) {
    static char cmd_line[64];
    static uint8_t idx = 0;
    uint32_t current_tick = HAL_GetTick();
    if ((current_tick - g_car_state.BT_last_tick) > TIMEOUT_MS) {
        if (g_car_state.current_motion_state != CAR_STOP) {
            SET_Action_execution(CAR_STOP);
            return;
        }
    }
    if (!Bt_RingBuf_IsEmpty()) {
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
                    g_car_state.bt_speed_percent = val;
                    if (g_car_state.bt_speed_percent > 100) g_car_state.bt_speed_percent = 100;
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
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        break;
    }
    }
}

void PWM_automatic_adjustment(void) {
    if (g_car_state.current_motion_state == CAR_STOP) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
    // 1. 蓝牙百分比 -> 目标物理速度 (cm/s)
    g_car_state.linear_speed_target_cm_s = (int32_t)g_car_state.bt_speed_percent * MAX_LINEAR_SPEED_CM_S / 100;

    // 2. 基础 PWM（前馈控制）
    int32_t base_pwm = (int32_t)g_car_state.linear_speed_target_cm_s * PWM_MAX_LIMIT / MAX_LINEAR_SPEED_CM_S;

    // 3. 计算左右轮的物理速度误差 (目标 - 实际)
    int32_t left_speed_error  = g_car_state.linear_speed_target_cm_s - g_car_state.left_speed_cm_s;
    int32_t right_speed_error = g_car_state.linear_speed_target_cm_s - g_car_state.right_speed_cm_s;

    // 4. 【新增】差速同步调整逻辑
    int32_t left_sync_adjust = 0;
    int32_t right_sync_adjust = 0;

    // 仅在直线运动（前进、后退）时进行左右轮同步修正
    if (g_car_state.current_motion_state == CAR_FORWARD ||
        g_car_state.current_motion_state == CAR_BACKWARD) {

        // 计算当前左右轮的平均实际速度
        int32_t avg_actual_speed = (g_car_state.left_speed_cm_s + g_car_state.right_speed_cm_s) / 2;

        // 计算各轮相对于平均速度的偏差
        // 如果左轮速度 < 平均速度，说明左轮偏慢，需要增加PWM，所以用 (平均 - 实际)
        int32_t left_sync_error  = avg_actual_speed - g_car_state.left_speed_cm_s;
        int32_t right_sync_error = avg_actual_speed - g_car_state.right_speed_cm_s;

        // 同步比例系数（建议设为 SPEED_KP 的一半或相同，避免过冲）
        #define SYNC_KP (SPEED_KP / 2)

        left_sync_adjust  = left_sync_error * SYNC_KP;
        right_sync_adjust = right_sync_error * SYNC_KP;
    }

    // 5. 最终 PWM = 基础前馈 + 速度误差修正 + 同步修正
    int32_t final_left_pwm  = base_pwm + (left_speed_error * SPEED_KP) + left_sync_adjust;
    int32_t final_right_pwm = base_pwm + (right_speed_error * SPEED_KP) + right_sync_adjust;

    // 6. 绝对限幅保护
    if (final_left_pwm > PWM_MAX_LIMIT) final_left_pwm = PWM_MAX_LIMIT;
    if (final_left_pwm < PWM_MIN_LIMIT) final_left_pwm = PWM_MIN_LIMIT;
    if (final_right_pwm > PWM_MAX_LIMIT) final_right_pwm = PWM_MAX_LIMIT;
    if (final_right_pwm < PWM_MIN_LIMIT) final_right_pwm = PWM_MIN_LIMIT;

    // 7. 输出给定时器
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)final_right_pwm);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)final_left_pwm);
}