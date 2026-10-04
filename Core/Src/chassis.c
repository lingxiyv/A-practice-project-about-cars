//
// Created by lingxi on 2026/9/16.
//
#include "chassis.h"

#include "oled.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#define KEY_PIN_3 GPIO_PIN_3    //连接左前方避障传感器
#define KEY_PIN_4 GPIO_PIN_4    //连接右前方避障传感器
#define KEY_ALL_PINS (GPIO_PIN_3 | GPIO_PIN_4)

// 记录随机漫游时当前动作的预期结束时间
uint32_t roam_action_end_tick = 0;

// 主循环解析逻辑
// 蓝牙发送的指令以 '\n' (0x0A) 结尾

// 切换到手动模式（避障过程中不打断，保持 AVOIDING）
static void Enter_Manual_Mode(void) {
    if (g_car_state.current_mode != MODE_AVOIDING) {
        g_car_state.current_mode = MODE_MANUAL;
    }
}

void Bt_Parse_Commands(void) {
    static char cmd_line[64];
    static uint8_t idx = 0;
    if ((HAL_GetTick() - g_car_state.BT_last_tick) > TIMEOUT_MS && g_car_state.current_mode == MODE_MANUAL) {
        if (g_car_state.current_motion_state != CAR_STOP) {
            SET_Action_execution(CAR_STOP);
            g_car_state.current_mode = MODE_IDLE;
        }
        return;
    }
    if (!Bt_RingBuf_IsEmpty()) {
        char ch = (char)Bt_RingBuf_Get();

        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                cmd_line[idx] = '\0';

                state_motion target_state = g_car_state.current_motion_state;
                bool is_motion_cmd = false;
                int val = 0;

                if (strcmp(cmd_line, "W") == 0 && g_car_state.current_mode != MODE_AVOIDING) {
                    // 检测到障碍物时忽略前进指令
                    Enter_Manual_Mode();
                    target_state = CAR_FORWARD;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "A") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_LEFT_TURN;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "S") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_BACKWARD;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "D") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_RIGHT_TURN;
                    is_motion_cmd = true;
                }
                else if (sscanf(cmd_line, "speed %d", &val) == 1) {
                    Enter_Manual_Mode();
                    g_car_state.bt_speed_percent = val;
                    if (g_car_state.bt_speed_percent > 100) g_car_state.bt_speed_percent = 100;
                }
                else if (strcmp(cmd_line,"AUTO_ROAM") == 0) {
                    g_car_state.current_mode = MODE_AUTO_ROAM;
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
        g_car_state.current_motion_state = CAR_FORWARD;
        g_car_state.roam_state_tick = HAL_GetTick();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    }
    case CAR_BACKWARD: {
        g_car_state.current_motion_state = CAR_BACKWARD;
        g_car_state.roam_state_tick = HAL_GetTick();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_SET);
        break;
    }
    case CAR_LEFT_TURN: {
        g_car_state.current_motion_state = CAR_LEFT_TURN;
        g_car_state.roam_state_tick = HAL_GetTick();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    }
    case CAR_RIGHT_TURN: {
        g_car_state.current_motion_state = CAR_RIGHT_TURN;
        g_car_state.roam_state_tick = HAL_GetTick();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_SET);
        break;
    }
    default: {
        // 停止
        g_car_state.current_motion_state = CAR_STOP;
        g_car_state.roam_state_tick = HAL_GetTick();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    }
    }
}

void PWM_automatic_adjustment(void) {
    if (g_car_state.current_motion_state == CAR_STOP) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        return;
    }
    // 蓝牙百分比 -> 目标物理速度 (cm/s)
    g_car_state.linear_speed_target_cm_s = (int32_t)g_car_state.bt_speed_percent * MAX_LINEAR_SPEED_CM_S / 100;

    // 基础 PWM（前馈控制）
    int32_t base_pwm = (int32_t)g_car_state.linear_speed_target_cm_s * PWM_MAX_LIMIT / MAX_LINEAR_SPEED_CM_S;

    // 计算左右轮的物理速度误差 (目标 - 实际)
    int32_t left_speed_error  = g_car_state.linear_speed_target_cm_s - g_car_state.left_speed_cm_s;
    int32_t right_speed_error = g_car_state.linear_speed_target_cm_s - g_car_state.right_speed_cm_s;

    // 差速同步调整逻辑
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

    // 最终 PWM = 基础前馈 + 速度误差修正 + 同步修正
    int32_t final_left_pwm  = base_pwm + (left_speed_error * SPEED_KP) + left_sync_adjust;
    int32_t final_right_pwm = base_pwm + (right_speed_error * SPEED_KP) + right_sync_adjust;

    // 绝对限幅保护
    if (final_left_pwm > PWM_MAX_LIMIT) final_left_pwm = PWM_MAX_LIMIT;
    if (final_left_pwm < PWM_MIN_LIMIT) final_left_pwm = PWM_MIN_LIMIT;
    if (final_right_pwm > PWM_MAX_LIMIT) final_right_pwm = PWM_MAX_LIMIT;
    if (final_right_pwm < PWM_MIN_LIMIT) final_right_pwm = PWM_MIN_LIMIT;

    // 输出给定时器
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)final_right_pwm);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)final_left_pwm);
}

void Obstacle_Check(void) {
    uint16_t port_state = GPIOB->IDR;
    // 低电平代表检测到障碍物
    bool is_obstacle = ((~port_state) & KEY_ALL_PINS);
    switch (g_car_state.avoid_sub_state) {
    case AVOID_IDLE:
        if (is_obstacle) {
            SET_Action_execution(CAR_STOP);
            g_car_state.prev_mode = g_car_state .current_mode;
            g_car_state.current_mode = MODE_AVOIDING;
            g_car_state.avoid_sub_state = AVOID_STOP;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);  //蜂鸣器提示
        }
        break;
    case AVOID_STOP:
        if (HAL_GetTick() - g_car_state.roam_state_tick > 200) {
            SET_Action_execution(CAR_BACKWARD);
            g_car_state.avoid_sub_state = AVOID_BACKWARD;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);   //关闭蜂鸣器
        }
        break;
    case AVOID_BACKWARD:
        if (HAL_GetTick() - g_car_state.roam_state_tick > 500) {
            if ((~port_state) & KEY_PIN_3) {
                SET_Action_execution(CAR_RIGHT_TURN);
            }
            else if ((~port_state) & KEY_PIN_4) {
                SET_Action_execution(CAR_LEFT_TURN);
            }
            else {
                SET_Action_execution(CAR_LEFT_TURN);
            }
            g_car_state.avoid_sub_state = AVOID_TURN;
        }
        break;
    case AVOID_TURN:
        if (HAL_GetTick() - g_car_state.roam_state_tick > 2000) {
            g_car_state.current_mode = g_car_state.prev_mode;
            SET_Action_execution(CAR_STOP);
            g_car_state.avoid_sub_state = AVOID_IDLE; // 回到空闲，继续检测
        }
        break;
    }
}

void Task_Auto_Roam(void)
{
    if (g_car_state.current_mode != MODE_AUTO_ROAM) return;

    uint32_t now = HAL_GetTick();

    // 还没到切换时间，保持当前动作
    if (now < roam_action_end_tick) return;

    // 到了切换时间，决定下一步做什么
    // 有 70% 的概率继续直行（惯性），30% 的概率改变方向
    uint8_t dice = rand() % 100;

    if (dice < 70) {
        // 直行，持续 1~3 秒（1000~3000ms）
        SET_Action_execution(CAR_FORWARD);
        roam_action_end_tick = now + 1000 + (rand() % 2000);
    }
    else {
        // 改变方向，随机左转或右转，持续 300~800ms
        if (rand() % 2) {
            SET_Action_execution(CAR_LEFT_TURN);
        } else {
            SET_Action_execution(CAR_RIGHT_TURN);
        }
        roam_action_end_tick = now + (300 + (rand() % 500))*3;
    }
}