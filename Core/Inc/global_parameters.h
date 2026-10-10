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

// 陀螺仪/坡度/转向相关参数
#define GYRO_LSB_PER_DPS    131.0f   // MPU6050 默认 ±250°/s 灵敏度 (LSB per °/s)
#define ACCEL_LSB_PER_G     16384.0f // MPU6050 默认 ±2g 灵敏度 (LSB per g)
#define YAW_KP              3.0f     // 航向保持增益 (cm/s 每度误差)
#define SLOPE_KP            15       // 坡道前馈 (PWM 每度坡度)
#define SLOPE_DEADBAND_DEG  3        // 坡度死区(度)
#define STEER_ARC_CM_S      30       // 弧线转向的差速量(cm/s)
#define OBSTACLE_DIST_MM    300      // 前方判定为障碍的距离(mm)
#define SIDE_CLEAR_MM       400      // 侧方判定为可通行距离(mm)
#define ROAM_SPEED_PERCENT  40       // 自动漫游默认速度百分比

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


typedef enum {
    SCAN_IDLE = 0,
    SCAN_MOVE_SERVO,     // 发出舵机指令
    SCAN_SETTLE,         // 等待舵机稳定（200ms）
    SCAN_MEASURE,        // 触发超声波并读取
    SCAN_COMPLETE,       // 完成一次全扫描
} ScanState_t;

typedef struct {
    ScanState_t state;
    uint32_t    state_tick;
    uint8_t     angle_idx;       // 当前扫描到第几个角度
    uint8_t     angles[7];       // 0°,30°,60°,90°,120°,150°,180°
    uint16_t    distances[7];    // 每个角度测得距离 (mm)
    uint8_t     data_valid;      // 是否有一份完整扫描结果
} Scan_Ctrl_t;

typedef struct {
    volatile uint8_t IS_IIC_BUSY;        //IIC全局忙标志（主循环 + 中断共享，必须 volatile）

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

    float   yaw_deg;          // 当前航向（Z轴陀螺仪积分），正=左转
    float   target_yaw_deg;   // 直行/转向的期望航向
    int16_t pitch_deg;        // 俯仰角(坡度)，正=上坡
    int16_t steer_cm_s;       // 平滑转向差速(cm/s)，正=右转
    uint8_t gyro_valid;       // 陀螺仪是否读到有效数据

    Scan_Ctrl_t g_scan;         //关于舵机和超声波

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
void Bt_Printf(const char* fmt, ...);   //通过蓝牙的数据回传
#ifdef __cplusplus
}
#endif
#endif //A_SMALL_CAR_GLOBAL_PARAMETERS_H
