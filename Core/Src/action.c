//
// Created by lingxi on 2026/9/16.
//
#include "action.h"
#include "gpio.h"
#include "oled.h"

uint8_t rx_data = 0;
uint8_t rx_cmd = 0;
uint8_t current_action = 'X';
uint8_t new_cmd_flag = 0;
uint32_t last_cmd_tick = 0;

uint8_t Instruction_retrieval(void) {
    // 处理新收到的指令
    uint8_t car_state = STOP;
    if (new_cmd_flag)
    {
        new_cmd_flag = 0; // 清除标志位

        // 只有指令改变时才执行电机驱动，避免频繁重复调用
        if (rx_cmd != current_action)
        {
            current_action = rx_cmd;

            switch (current_action)
            {
            case 'W': car_state = FORWARD;  break;
            case 'S': car_state = BACKWARD; break;
            case 'A': car_state = LEFT_TURN; break;
            case 'D': car_state = RIGHT_TURN; break;
            case 'X': car_state = STOP; break;
            default:  car_state = STOP;     break; // 收到未知字符也停车
            }
        }
    }

    // 心跳超时检测
    // 如果当前时间 - 上一次收到指令的时间 > 500ms
    if ((HAL_GetTick() - last_cmd_tick) > TIMEOUT_MS)
    {
        // 只有在小车还在运动时，才需要触发强制停止
        if (current_action != 'X')
        {
            current_action = 'X';
            // Motor_Stop(); // 强制刹车
        }
    }
    return car_state;
}

void Action_execution(uint8_t car_state) {
    switch (car_state) {
    case FORWARD: {
        oled_show_string(64,0," FORWARD",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);
        break;
    }
    case BACKWARD: {
        oled_show_string(64,0," BACKWARD",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);
        break;
    }
    case LEFT_TURN: {
        oled_show_string(64,0," LEFT    ",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
        break;
    }
    case RIGHT_TURN: {
        oled_show_string(64,0," RIGHT    ",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
        break;
    }
    default: {
        // 停止
        oled_show_string(64,0," STOP     ",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);
        break;
    }
    }
}