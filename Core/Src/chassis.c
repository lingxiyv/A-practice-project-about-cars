//
// Created by lingxi on 2026/9/16.
//
#include "chassis.h"

#include "oled.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "gyroscope.h"

#define KEY_PIN_3 GPIO_PIN_3    //连接左前方避障传感器
#define KEY_PIN_4 GPIO_PIN_4    //连接右前方避障传感器
#define KEY_ALL_PINS (GPIO_PIN_3 | GPIO_PIN_4)

// 记录随机漫游时当前动作的预期结束时间
uint32_t roam_action_end_tick = 0;

// 切换到手动模式（避障过程中不打断，保持 AVOIDING）
static void Enter_Manual_Mode(void) {
    if (g_car_state.current_mode != MODE_AVOIDING) {
        g_car_state.current_mode = MODE_MANUAL;
    }
}

// 主循环解析逻辑
// 蓝牙发送的指令以 '\n' (0x0A) 结尾
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
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "A") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_LEFT_TURN;
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "S") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_BACKWARD;
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "D") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_RIGHT_TURN;
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "WA") == 0) {   // 前进+左弧线
                    Enter_Manual_Mode();
                    target_state = CAR_FORWARD;
                    g_car_state.steer_cm_s = -STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "WD") == 0) {   // 前进+右弧线
                    Enter_Manual_Mode();
                    target_state = CAR_FORWARD;
                    g_car_state.steer_cm_s = STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "SA") == 0) {   // 后退+左弧线
                    Enter_Manual_Mode();
                    target_state = CAR_BACKWARD;
                    g_car_state.steer_cm_s = -STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "SD") == 0) {   // 后退+右弧线
                    Enter_Manual_Mode();
                    target_state = CAR_BACKWARD;
                    g_car_state.steer_cm_s = STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (sscanf(cmd_line, "speed %d", &val) == 1) {
                    Enter_Manual_Mode();
                    g_car_state.bt_speed_percent = val;
                    if (g_car_state.bt_speed_percent > 100) g_car_state.bt_speed_percent = 100;
                }
                else if (strcmp(cmd_line,"AUTO_ROAM") == 0) {
                    g_car_state.current_mode = MODE_AUTO_ROAM;
                    if (g_car_state.bt_speed_percent == 0) {
                        g_car_state.bt_speed_percent = ROAM_SPEED_PERCENT;
                    }
                    Ultrasonic_Scan_Start();
                }
                else if (sscanf(cmd_line,"angle %d", &val) == 1) {
                    Enter_Manual_Mode();
                    Servo_control_ultrasonic(val);
                    HAL_Delay(50);
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
        g_car_state.target_yaw_deg = g_car_state.yaw_deg;   // 直行开始锁定当前航向
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    }
    case CAR_BACKWARD: {
        g_car_state.current_motion_state = CAR_BACKWARD;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.target_yaw_deg = g_car_state.yaw_deg;   // 直行开始锁定当前航向
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_SET);
        break;
    }
    case CAR_LEFT_TURN: {
        g_car_state.current_motion_state = CAR_LEFT_TURN;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.steer_cm_s = 0;   // 原地转向不使用差速
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    }
    case CAR_RIGHT_TURN: {
        g_car_state.current_motion_state = CAR_RIGHT_TURN;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.steer_cm_s = 0;   // 原地转向不使用差速
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_SET);
        break;
    }
    default: {
        // 停止
        g_car_state.current_motion_state = CAR_STOP;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.steer_cm_s = 0;
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

    // 蓝牙百分比 -> 目标速度幅值 (cm/s)
    int32_t target_mag = (int32_t)g_car_state.bt_speed_percent * MAX_LINEAR_SPEED_CM_S / 100;
    g_car_state.linear_speed_target_cm_s = target_mag;

    // 方向符号：前进为正、后退为负
    int32_t dir_sign = (g_car_state.current_motion_state == CAR_BACKWARD) ? -1 : 1;

    // 航向保持：仅在直行且无手动转向时生效（避免与弧线转向打架）
    int32_t yaw_adjust = 0;
    if (g_car_state.steer_cm_s == 0 && g_car_state.gyro_valid) {
        float yaw_err = Gyro_WrapDeg(g_car_state.yaw_deg - g_car_state.target_yaw_deg);
        yaw_adjust = (int32_t)(YAW_KP * yaw_err);
    }

    // 转向差速：手动平滑转向 + 航向修正。正 = 右转（左轮更快）
    int32_t steer = (int32_t)g_car_state.steer_cm_s + yaw_adjust;
    // 后退时差速符号需翻转，保证“右转”语义一致
    int32_t steer_eff = (dir_sign > 0) ? steer : -steer;

    // 坡度前馈：上坡加驱动、下坡减驱动；后退时按行进方向翻转符号
    int32_t slope_ff = 0;
    if (g_car_state.pitch_deg > SLOPE_DEADBAND_DEG ||
        g_car_state.pitch_deg < -SLOPE_DEADBAND_DEG) {
        slope_ff = SLOPE_KP * g_car_state.pitch_deg;
    }
    int32_t slope_eff = (dir_sign > 0) ? slope_ff : -slope_ff;

    // 每轮独立目标速度（带符号）
    int32_t left_target  = dir_sign * target_mag + steer_eff;
    int32_t right_target = dir_sign * target_mag - steer_eff;

    // 前馈 PWM 幅值 + 坡度补偿
    int32_t base_pwm = (int32_t)target_mag * PWM_MAX_LIMIT / MAX_LINEAR_SPEED_CM_S + slope_eff;

    // 每轮速度误差修正
    int32_t left_speed_error  = left_target  - g_car_state.left_speed_cm_s;
    int32_t right_speed_error = right_target - g_car_state.right_speed_cm_s;

    int32_t final_left_pwm  = base_pwm + (left_speed_error * SPEED_KP);
    int32_t final_right_pwm = base_pwm + (right_speed_error * SPEED_KP);

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
        if (HAL_GetTick() - g_car_state.roam_state_tick > 1000) {
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

    // 到达切换时间，用最近一次完整扫描做简单寻路
    uint16_t front = g_car_state.g_scan.distances[3];          // 90° 正前方
    uint16_t left_max  = g_car_state.g_scan.distances[0];
    uint16_t right_max = g_car_state.g_scan.distances[6];
    for (int i = 1; i < 3; i++) {
        if (g_car_state.g_scan.distances[i]     > left_max)  left_max  = g_car_state.g_scan.distances[i];
        if (g_car_state.g_scan.distances[6 - i] > right_max) right_max = g_car_state.g_scan.distances[6 - i];
    }
    bool scan_ready = g_car_state.g_scan.data_valid;

    if (scan_ready && front < OBSTACLE_DIST_MM) {
        // 前方被挡：原地转向更开阔的一侧
        if (right_max >= left_max) {
            SET_Action_execution(CAR_RIGHT_TURN);
        } else {
            SET_Action_execution(CAR_LEFT_TURN);
        }
        roam_action_end_tick = now + 600 + (rand() % 400);
    } else {
        // 前方通畅：直行，若一侧明显更开阔则轻微弧线靠过去
        g_car_state.steer_cm_s = 0;
        if (scan_ready && (right_max > SIDE_CLEAR_MM || left_max > SIDE_CLEAR_MM)) {
            if (right_max > left_max)      g_car_state.steer_cm_s =  STEER_ARC_CM_S / 2;
            else if (left_max > right_max) g_car_state.steer_cm_s = -STEER_ARC_CM_S / 2;
        }
        SET_Action_execution(CAR_FORWARD);
        roam_action_end_tick = now + 1000 + (rand() % 2000);
    }

    // 重启扫描，保证运动过程中始终有新鲜的距离数据
    Ultrasonic_Scan_Start();
}

void Servo_control_ultrasonic(uint8_t angle)
{
    uint16_t pulse_width_us;
    if (angle > 180) {
        angle = 180;
    }
    if (angle < 0) {
        angle = 0;
    }
    pulse_width_us = 500 + (uint16_t)((float)angle * 2000.0 / 180.0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width_us);
}

void Ultrasonic_Scan_Update(void) {
    uint32_t now = HAL_GetTick();

    switch (g_car_state.g_scan.state) {
    case SCAN_MOVE_SERVO:
        // 发出舵机指令，立即进入等待状态
        Servo_control_ultrasonic(g_car_state.g_scan.angles[g_car_state.g_scan.angle_idx]);
        g_car_state.g_scan.state = SCAN_SETTLE;
        g_car_state.g_scan.state_tick = now;
        break;

    case SCAN_SETTLE:
        // 等 100ms 让舵机转到位并稳定
        if (now - g_car_state.g_scan.state_tick >= 100) {
            g_car_state.g_scan.state = SCAN_MEASURE;
            g_car_state.g_scan.state_tick = now;
        }
        break;

    case SCAN_MEASURE:
        // 现在舵机已经稳定，测量才有效
        g_car_state.g_scan.distances[g_car_state.g_scan.angle_idx] = Ultrasonic_Measure_Once();

        g_car_state.g_scan.angle_idx++;
        if (g_car_state.g_scan.angle_idx >= 7) {
            g_car_state.g_scan.state = SCAN_COMPLETE;
        } else {
            g_car_state.g_scan.state = SCAN_MOVE_SERVO;
        }
        break;

    case SCAN_COMPLETE:
        g_car_state.g_scan.data_valid = 1;   // 标记已有完整扫描结果
        g_car_state.g_scan.state = SCAN_IDLE;
        break;

    case SCAN_IDLE:
    default:
        break;
    }
}

void Ultrasonic_Scan_Start(void) {
    if (g_car_state.g_scan.state != SCAN_IDLE) return;  // 正在扫描，不重复启动
    g_car_state.g_scan.state = SCAN_MOVE_SERVO;
    g_car_state.g_scan.angle_idx = 0;
    g_car_state.g_scan.data_valid = 0;   // 新一轮扫描开始，旧结果作废
}


uint16_t Ultrasonic_Measure_Once(void) {
    uint8_t tx_cmd = 0x55;
    uint8_t rx_buf[2];

    HAL_UART_Transmit(&huart1, &tx_cmd, 1, 50);

    if (HAL_UART_Receive(&huart1, rx_buf, 2, 100) != HAL_OK) {
        return 0;
    }

    uint16_t distance_mm = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    if (distance_mm > 5000 || distance_mm < 20) {
        return 0;
    }
    return distance_mm;
}

// 0.5Hz 状态回传（yaw 单位为 0.1°）
void Telemetry_Update(void) {
    uint16_t left_max  = g_car_state.g_scan.distances[0];
    uint16_t right_max = g_car_state.g_scan.distances[6];
    for (int i = 1; i < 3; i++) {
        if (g_car_state.g_scan.distances[i]     > left_max)  left_max  = g_car_state.g_scan.distances[i];
        if (g_car_state.g_scan.distances[6 - i] > right_max) right_max = g_car_state.g_scan.distances[6 - i];
    }
    Bt_Printf("[T] m=%d spd=%d steer=%d yaw=%d pit=%d L=%d F=%d R=%d\r\n",
              g_car_state.current_mode,
              g_car_state.linear_speed_cm_s,
              g_car_state.steer_cm_s,
              (int16_t)(g_car_state.yaw_deg * 10.0f),
              g_car_state.pitch_deg,
              left_max, g_car_state.g_scan.distances[3], right_max);
}