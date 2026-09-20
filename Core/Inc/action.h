//
// Created by lingxi on 2026/9/16.
//

#ifndef A_SMALL_CAR_ACTION_H
#define A_SMALL_CAR_ACTION_H

#include "stdint.h"

#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define LEFT_TURN 3
#define RIGHT_TURN 4

extern uint8_t rx_data;
extern uint8_t rx_cmd;              // 存放当前接收到的指令
extern uint8_t current_action;    // 小车当前执行的动作，默认停止
extern uint32_t last_cmd_tick;      // 上一次收到指令的时间（毫秒）
extern uint8_t new_cmd_flag;        // 新指令标志位
#define TIMEOUT_MS 200           // 超时阈值

uint8_t Instruction_retrieval(void);
void Action_execution(uint8_t car_state);

#endif //A_SMALL_CAR_ACTION_H
