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

// 陀螺仪零偏标定：上电时静止采样多少次。不标定的话积分出的 yaw 会一直漂，
// 航向保持(YAW_KP)就一直往一边修，车看起来就是“跑偏 / 两轮不同步”
#define GYRO_CALIB_SAMPLES  100

// 舵机(PA8 / TIM1_CH1，50Hz)
#define SERVO_MIN_PULSE_US   500     // 0°   对应脉宽(us)
#define SERVO_MAX_PULSE_US   2400    // 180° 对应脉宽(us)。不要给到 2500us：
                                     // 有些舵机机械行程到不了，会一直堵转“嗡嗡”响却不转
#define SERVO_CENTER_ANGLE   90      // 扫描间隙时舵机停放角度

// 超声波(US-100 / AJ-SR04M 等串口型，接 USART1 PA9/PA10，9600)
#define ULTRASONIC_TRIGGER_BYTE 0x55    // 触发字节，模块回 2 字节：距离高 8 位、低 8 位(mm)
#define ULTRASONIC_MIN_MM    20         // 有效距离下限(mm)
#define ULTRASONIC_MAX_MM    5000       // 有效距离上限(mm)
#define SCAN_SETTLE_MS       100        // 发完舵机角度后等它转到位
#define SCAN_REPLY_TIMEOUT_MS 120       // 等超声波回包超时。US-100 一次测距约 50~60ms
#define SCAN_GAP_MS          800        // 两轮扫描之间的间隔
#define SCAN_MANUAL_HOLD_MS  5000       // 手动指定角度后暂停自动扫描的时间
#define SCAN_CONTINUOUS      1          // 1=上电后自动循环扫描，任何模式下都有新鲜距离数据

// 主循环不允许被长时间卡住的相关保护阈值
#define I2C_DMA_STUCK_MS     50         // OLED 的 DMA 超过这么久没完成就复位 I2C，避免忙标志被永久占用
#define I2C_LOCK_WAIT_MS     20         // 等 I2C 空闲的上限，超时就本次放弃，下一拍再来
#define BT_TX_TIMEOUT_MS     200        // 蓝牙中断发送卡住的兜底时间

// 扫描角度在数组里的下标：0°~60° 在车左侧，90° 是正前方，120°~180° 在车右侧
#define SCAN_IDX_LEFT_FROM   0
#define SCAN_IDX_LEFT_TO     2
#define SCAN_IDX_FRONT       3
#define SCAN_IDX_RIGHT_FROM  4
#define SCAN_IDX_RIGHT_TO    6

// ---- 避障(红外 + 超声波)与按角度转向 ----
#define AVOID_STOP_MS        200    // 撞见障碍后先急停多久
#define AVOID_BACK_MS        500    // 后退多久再开始转向
#define AVOID_TURN_TIMEOUT_MS 4000  // 按角度转向的兜底超时（陀螺仪失效或车转不动时用）
#define AVOID_TURN_DEFAULT_DEG 90   // 默认转向角度
#define AVOID_TURN_SMALL_DEG   60   // 正前方还有余量时，小角度让开
#define AVOID_TURN_LARGE_DEG  120   // 障碍贴得很近时，大角度让开
#define AVOID_TURN_MAX_DEG    150   // 转角上限
#define AVOID_TURN_EXTRA_DEG   30   // 要转过去那一侧也不开阔时，额外加的角度
#define AVOID_NEAR_MM         150   // 判定“障碍很近”的距离(mm)
#define AVOID_MIN_SPEED_CM_S   30   // 避障动作的最低速度(cm/s)：speed 为 0 时也得能转得动

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
    SCAN_SETTLE,         // 等待舵机稳定
    SCAN_MEASURE,        // 已触发超声波，等回包（中断接收，不阻塞主循环）
    SCAN_GAP,            // 两轮扫描之间的间隔（舵机停在中位）
    SCAN_COMPLETE,       // 完成一次全扫描
} ScanState_t;

typedef struct {
    ScanState_t state;
    uint32_t    state_tick;
    uint8_t     angle_idx;       // 当前扫描到第几个角度
    uint8_t     angles[7];       // 0°,30°,60°,90°,120°,150°,180°
    uint16_t    distances[7];    // 每个角度测得距离 (mm)
    uint8_t     data_valid;      // 是否有一份完整扫描结果

    /* 超声波：常驻中断接收 + 非阻塞测距
       注意不要用"中断收满 2 字节"的写法：只要有一个字节因为溢出(ORE)没被及时读走，
       HAL 就会把这次接收整个中止掉，永远等不到第 2 个字节 → 距离全是 0。 */
    uint8_t  rx_buf[2];          // 最近一帧的原始字节（诊断用）
    volatile uint16_t rx_total;  // 累计收到的字节数（中断里自增）
    uint16_t rx_mark;            // 触发测距那一刻的 rx_total
    uint8_t  rx_active;          // 已触发，正在等回包
    uint8_t  rx_error;           // 接收出错（溢出/帧错误）
    uint32_t hold_until;         // 在此之前不启动新一轮扫描（手动指定角度用）
    uint8_t  continuous;         // 1=自动循环扫描
    uint8_t  ok_cnt;             // 本轮成功次数
    uint8_t  err_cnt;            // 本轮失败/超时次数
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

    /* 按角度转向：用陀螺仪积分的航向判断转够了没有，不再靠"转多少毫秒"猜 */
    float   turn_start_yaw;   // 本次转向开始时的航向
    int16_t turn_target_deg;  // 本次转向目标角度，带符号：正=左转，负=右转

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
void Bt_Printf(const char* fmt, ...);   //通过蓝牙的数据回传（中断发送，不阻塞主循环）
void Bt_Tx_Complete_Callback(void);     //蓝牙发送完成，由 HAL_UART_TxCpltCallback 调用
#ifdef __cplusplus
}
#endif
#endif //A_SMALL_CAR_GLOBAL_PARAMETERS_H
