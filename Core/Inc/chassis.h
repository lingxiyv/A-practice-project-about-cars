//
// Created by lingxi on 2026/9/16.
//

#ifndef A_SMALL_CAR_CHASSIS_H
#define A_SMALL_CAR_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "global_parameters.h"

#define TIMEOUT_MS 500           // 超时阈值

void Bt_Parse_Commands(void);    // 蓝牙指令解析
void SET_Action_execution(state_motion car_state);  //底盘状态设置与执行
void PWM_automatic_adjustment(void);    //PWM自动调整
void Obstacle_Check(void);       //障碍物检测与自动避障
void Task_Auto_Roam(void);       //随机漫游行动逻辑

#ifdef __cplusplus
}
#endif

#endif //A_SMALL_CAR_CHASSIS_H
