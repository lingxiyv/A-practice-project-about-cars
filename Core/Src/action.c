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
static uint8_t last_state = 0xFF;

uint8_t Instruction_retrieval(void)
{
    uint8_t car_state = STOP;

    // 收到新指令：直接覆盖 current_action
    if (new_cmd_flag)
    {
        new_cmd_flag = 0;
        current_action = rx_cmd;
    }

    // 心跳超时：强制停车
    if ((HAL_GetTick() - last_cmd_tick) > TIMEOUT_MS)
    {
        current_action = 'X';
    }

    // 每次调用都根据 current_action 生成 car_state
    switch (current_action)
    {
    case 'W': car_state = FORWARD;    break;
    case 'S': car_state = BACKWARD;   break;
    case 'A': car_state = LEFT_TURN;  break;
    case 'D': car_state = RIGHT_TURN; break;
    case 'X': car_state = STOP;       break;
    default:  car_state = STOP;       break;
    }

    return car_state;
}

void Action_execution(uint8_t car_state) {
    if (car_state == last_state) return;   // 状态没变，什么都不做
    last_state = car_state;

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