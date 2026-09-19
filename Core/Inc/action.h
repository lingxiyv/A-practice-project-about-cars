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

uint8_t Instruction_retrieval(void);
void Action_execution(uint8_t car_state);

#endif //A_SMALL_CAR_ACTION_H
