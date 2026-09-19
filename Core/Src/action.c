//
// Created by lingxi on 2026/9/16.
//
#include "action.h"
#include "gpio.h"
#include "oled.h"

uint8_t Instruction_retrieval(void) {
    uint8_t car_state = STOP;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) car_state = FORWARD;
    }
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) car_state = BACKWARD;
    }
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) car_state = LEFT_TURN;
    }
    else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) car_state = RIGHT_TURN;
    }
    else car_state = STOP;
    return car_state;
}

void Action_execution(uint8_t car_state) {
    switch (car_state) {
    case FORWARD: {
        oled_show_string(64,24,"FORWARD",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);
        break;
    }
    case BACKWARD: {
        oled_show_string(64,24,"BACKWARD",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);
        break;
    }
    case LEFT_TURN: {
        oled_show_string(64,24,"LEFT    ",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
        break;
    }
    case RIGHT_TURN: {
        oled_show_string(64,24,"RIGHT    ",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
        break;
    }
    default: {
        // 停止
        oled_show_string(64,24,"STOP    ",12);
        oled_refresh_gram();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);
        break;
    }
    }
}