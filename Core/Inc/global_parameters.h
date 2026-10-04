//
// Created by lingxi on 2026/9/23.
//

#ifndef A_SMALL_CAR_GLOBAL_PARAMETERS_H
#define A_SMALL_CAR_GLOBAL_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdbool.h"
#include "tim.h"

// 编码器物理线数（单倍频）
#define ENCODER_PPR 14
// 实测减速比
#define GEAR_RATIO_ACTUAL 45.26f
// 综合系数：输出轴转一圈对应的单倍频脉冲数
#define PULSES_PER_REV_OUT (ENCODER_PPR * GEAR_RATIO_ACTUAL) // 约等于 633.7
//轮子周长(cm)
#define WHEEL_PERIMETER 21.1

#define SPEED_CALC_COEFF 2210
#define SPEED_CALC_DIV   2535
//最大PWM
#define PWM_MAX_LIMIT 19999
//最小PWM
#define PWM_MIN_LIMIT 500
//比例系数
#define SPEED_KP 20
//最大速度
#define MAX_LINEAR_SPEED_CM_S 120
//小车运动状态枚举
typedef enum {
    CAR_STOP,
    CAR_FORWARD,
    CAR_BACKWARD,
    CAR_LEFT_TURN,
    CAR_RIGHT_TURN,
}state_motion;

//小车控制模式枚举
typedef enum {
    MODE_IDLE,      //空闲（刚上电，没有收到任何指令）
    MODE_MANUAL,    //蓝牙遥控模式
    MODE_AUTO_ROAM, //自动漫游模式
    MODE_AVOIDING,  //避障中（临时状态）
} CarMode;

//避障状态枚举
typedef enum {
    AVOID_IDLE,      // 正常巡航，无避障动作
    AVOID_STOP,      // 刚检测到障碍，紧急刹车
    AVOID_BACKWARD,  // 正在后退
    AVOID_TURN       // 正在转向
} Avoid_SubState;

typedef struct {
    state_motion current_motion_state;     //当前状态
    CarMode current_mode;                  //当前模式
    CarMode prev_mode;                     //之前的模式
    Avoid_SubState avoid_sub_state;        //避障状态
    uint16_t linear_speed_target_cm_s;     //目标速度
    uint8_t bt_speed_percent;              //目标速度百分比
    int16_t left_speed_cm_s;        // 左轮线速度 (m/s)
    int16_t right_speed_cm_s;       // 右轮线速度 (m/s)
    int16_t linear_speed_cm_s;      // 底盘中心线速度 (m/s)
    int16_t angular_speed;          // 底盘旋转角速度
    uint16_t left_last_count;        //左轮编码器上次读取时间戳
    uint16_t right_last_count;       //右轮编码器上次读取时间戳
    uint32_t BT_last_tick;          //蓝牙上次收到指令时间戳
    uint32_t roam_state_tick;  // 进入当前状态的时间戳
}car_state;

extern car_state g_car_state;
void car_state_init(void);

// 定义蓝牙接收缓冲区大小
#define BT_RX_BUF_SIZE 128

typedef struct {
    uint8_t buffer[BT_RX_BUF_SIZE];
    volatile uint16_t head; // 写指针（中断中更新）
    volatile uint16_t tail; // 读指针（主循环中更新）
} Bt_RingBuffer_t;

extern Bt_RingBuffer_t g_bt_rx_buf;

// 暴露缓冲区操作接口
void Bt_RingBuf_Put(uint8_t data);      // 中断中调用
uint8_t Bt_RingBuf_Get(void);           // 主循环调用
uint16_t Bt_RingBuf_GetCount(void);     // 获取缓冲区有效数据长度
bool Bt_RingBuf_IsEmpty(void);

void Speed_Calculation_ISR(void);       //速度变量更新

#ifdef __cplusplus
}
#endif
#endif //A_SMALL_CAR_GLOBAL_PARAMETERS_H
