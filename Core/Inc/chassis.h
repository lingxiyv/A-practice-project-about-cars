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
void Servo_control_ultrasonic(uint16_t angle);  //超声波舵机控制(0~180°)
void Ultrasonic_Scan_Update(void);  //超声波扫描状态机，需在 10ms 周期任务里调用
void Ultrasonic_Scan_Start(void);   // 启动一次扫描
void Ultrasonic_Scan_Stop(void);    // 停止自动循环扫描
void Ultrasonic_Trigger(void);      // 触发一次测距（非阻塞，回包靠常驻中断接收）
void Ultrasonic_Rx_Complete(void);  // USART1 收到 1 个字节（在 HAL_UART_RxCpltCallback 里调用）
void Ultrasonic_Rx_Error(void);     // USART1 接收出错（在 HAL_UART_ErrorCallback 里调用）
void Ultrasonic_Rx_Service(void);   // 常驻接收自愈，放到主循环里调用
uint16_t Ultrasonic_Get_Distance(uint8_t index); // 取某个角度的距离(mm)，0=本次无效
void Servo_Test_Update(void);       // 舵机慢速自检，用来单独验证舵机硬件
void Servo_Test_Start(void);        // 启动舵机自检
void Telemetry_Update(void);       // 0.5Hz 状态回传


#ifdef __cplusplus
}
#endif

#endif //A_SMALL_CAR_CHASSIS_H
