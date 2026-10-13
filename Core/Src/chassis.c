//
// Created by lingxi on 2026/9/16.
//
#include "chassis.h"

#include "oled.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "gyroscope.h"

#define KEY_PIN_3 GPIO_PIN_3    //连接左前方避障传感器
#define KEY_PIN_4 GPIO_PIN_4    //连接右前方避障传感器
#define KEY_ALL_PINS (GPIO_PIN_3 | GPIO_PIN_4)

// 记录随机漫游时当前动作的预期结束时间
uint32_t roam_action_end_tick = 0;

// 切换到手动模式（避障过程中不打断，保持 AVOIDING）
static void Enter_Manual_Mode(void) {
    if (g_car_state.current_mode != MODE_AVOIDING) {
        g_car_state.current_mode = MODE_MANUAL;
    }
}

// 主循环解析逻辑
// 蓝牙发送的指令以 '\n' (0x0A) 结尾
void Bt_Parse_Commands(void) {
    static char cmd_line[64];
    static uint8_t idx = 0;
    if ((HAL_GetTick() - g_car_state.BT_last_tick) > TIMEOUT_MS && g_car_state.current_mode == MODE_MANUAL) {
        if (g_car_state.current_motion_state != CAR_STOP) {
            SET_Action_execution(CAR_STOP);
            g_car_state.current_mode = MODE_IDLE;
        }
        return;
    }
    if (!Bt_RingBuf_IsEmpty()) {
        char ch = (char)Bt_RingBuf_Get();

        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                cmd_line[idx] = '\0';

                state_motion target_state = g_car_state.current_motion_state;
                bool is_motion_cmd = false;
                int val = 0;

                if (strcmp(cmd_line, "W") == 0 && g_car_state.current_mode != MODE_AVOIDING) {
                    // 检测到障碍物时忽略前进指令
                    Enter_Manual_Mode();
                    target_state = CAR_FORWARD;
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "A") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_LEFT_TURN;
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "S") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_BACKWARD;
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "D") == 0) {
                    Enter_Manual_Mode();
                    target_state = CAR_RIGHT_TURN;
                    g_car_state.steer_cm_s = 0;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "WA") == 0) {   // 前进+左弧线
                    Enter_Manual_Mode();
                    target_state = CAR_FORWARD;
                    g_car_state.steer_cm_s = -STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "WD") == 0) {   // 前进+右弧线
                    Enter_Manual_Mode();
                    target_state = CAR_FORWARD;
                    g_car_state.steer_cm_s = STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "SA") == 0) {   // 后退+左弧线
                    Enter_Manual_Mode();
                    target_state = CAR_BACKWARD;
                    g_car_state.steer_cm_s = -STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (strcmp(cmd_line, "SD") == 0) {   // 后退+右弧线
                    Enter_Manual_Mode();
                    target_state = CAR_BACKWARD;
                    g_car_state.steer_cm_s = STEER_ARC_CM_S;
                    is_motion_cmd = true;
                }
                else if (sscanf(cmd_line, "speed %d", &val) == 1) {
                    Enter_Manual_Mode();
                    g_car_state.bt_speed_percent = val;
                    if (g_car_state.bt_speed_percent > 100) g_car_state.bt_speed_percent = 100;
                }
                else if (strcmp(cmd_line,"AUTO_ROAM") == 0) {
                    g_car_state.current_mode = MODE_AUTO_ROAM;
                    if (g_car_state.bt_speed_percent == 0) {
                        g_car_state.bt_speed_percent = ROAM_SPEED_PERCENT;
                    }
                    Ultrasonic_Scan_Start();
                }
                else if (sscanf(cmd_line,"angle %d", &val) == 1) {
                    Enter_Manual_Mode();
                    if (val < 0)   val = 0;
                    if (val > 180) val = 180;
                    Servo_control_ultrasonic((uint16_t)val);
                    /* 手动指定角度：让自动扫描先停一会儿，否则下一轮扫描马上把舵机拽走 */
                    g_car_state.g_scan.state_tick = HAL_GetTick();
                    g_car_state.g_scan.hold_until = HAL_GetTick() + SCAN_MANUAL_HOLD_MS;
                    g_car_state.g_scan.state = SCAN_GAP;
                    g_car_state.g_scan.rx_active = 0;
                }
                else if (strcmp(cmd_line, "servotest") == 0 || strcmp(cmd_line, "SERVOTEST") == 0) {
                    Enter_Manual_Mode();
                    Servo_Test_Start();   // 舵机慢速自检：单独判断舵机硬件好坏
                }
                if (is_motion_cmd && target_state != g_car_state.current_motion_state) {
                    SET_Action_execution(target_state);
                }
                idx = 0;
            }
        } else {
            if (idx < sizeof(cmd_line) - 1) {
                cmd_line[idx++] = ch;
            }
        }
    }
}

void SET_Action_execution(state_motion car_state) {
    g_car_state.current_motion_state = car_state;
    switch (car_state) {
    case CAR_FORWARD: {
        g_car_state.current_motion_state = CAR_FORWARD;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.target_yaw_deg = g_car_state.yaw_deg;   // 直行开始锁定当前航向
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    }
    case CAR_BACKWARD: {
        g_car_state.current_motion_state = CAR_BACKWARD;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.target_yaw_deg = g_car_state.yaw_deg;   // 直行开始锁定当前航向
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_SET);
        break;
    }
    case CAR_LEFT_TURN: {
        g_car_state.current_motion_state = CAR_LEFT_TURN;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.steer_cm_s = 0;   // 原地转向不使用差速
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    }
    case CAR_RIGHT_TURN: {
        g_car_state.current_motion_state = CAR_RIGHT_TURN;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.steer_cm_s = 0;   // 原地转向不使用差速
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_SET);
        break;
    }
    default: {
        // 停止
        g_car_state.current_motion_state = CAR_STOP;
        g_car_state.roam_state_tick = HAL_GetTick();
        g_car_state.steer_cm_s = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    }
    }
}

void PWM_automatic_adjustment(void) {
    if (g_car_state.current_motion_state == CAR_STOP) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        return;
    }

    // 蓝牙百分比 -> 目标速度幅值 (cm/s)
    int32_t target_mag = (int32_t)g_car_state.bt_speed_percent * MAX_LINEAR_SPEED_CM_S / 100;

    /* 避障动作(急停/后退/按角度转向)必须真的能动：速度被设成 0 时给一个最低速度，
       否则避障会卡在原地，按角度转向也永远转不到位 */
    if (g_car_state.current_mode == MODE_AVOIDING && target_mag < AVOID_MIN_SPEED_CM_S) {
        target_mag = AVOID_MIN_SPEED_CM_S;
    }
    g_car_state.linear_speed_target_cm_s = target_mag;

    // 方向符号：前进为正、后退为负
    int32_t dir_sign = (g_car_state.current_motion_state == CAR_BACKWARD) ? -1 : 1;

    /* 航向保持只在直行(前进/后退)时生效：
       原地转向时再去"修航向"会跟转向本身对着干，按角度转向就永远转不到位 */
    int32_t yaw_adjust = 0;
    if ((g_car_state.current_motion_state == CAR_FORWARD ||
         g_car_state.current_motion_state == CAR_BACKWARD) &&
        g_car_state.steer_cm_s == 0 && g_car_state.gyro_valid) {
        float yaw_err = Gyro_WrapDeg(g_car_state.yaw_deg - g_car_state.target_yaw_deg);
        yaw_adjust = (int32_t)(YAW_KP * yaw_err);
    }

    // 转向差速：手动平滑转向 + 航向修正。正 = 右转（左轮更快）
    int32_t steer = (int32_t)g_car_state.steer_cm_s + yaw_adjust;
    // 后退时差速符号需翻转，保证“右转”语义一致
    int32_t steer_eff = (dir_sign > 0) ? steer : -steer;

    // 坡度前馈：上坡加驱动、下坡减驱动；后退时按行进方向翻转符号
    int32_t slope_ff = 0;
    if (g_car_state.pitch_deg > SLOPE_DEADBAND_DEG ||
        g_car_state.pitch_deg < -SLOPE_DEADBAND_DEG) {
        slope_ff = SLOPE_KP * g_car_state.pitch_deg;
    }
    int32_t slope_eff = (dir_sign > 0) ? slope_ff : -slope_ff;

    // 每轮独立目标速度（带符号）
    int32_t left_target  = dir_sign * target_mag + steer_eff;
    int32_t right_target = dir_sign * target_mag - steer_eff;

    // 前馈 PWM 幅值 + 坡度补偿
    int32_t base_pwm = (int32_t)target_mag * PWM_MAX_LIMIT / MAX_LINEAR_SPEED_CM_S + slope_eff;

    // 每轮速度误差修正
    int32_t left_speed_error  = left_target  - g_car_state.left_speed_cm_s;
    int32_t right_speed_error = right_target - g_car_state.right_speed_cm_s;

    int32_t final_left_pwm  = base_pwm + (left_speed_error * SPEED_KP);
    int32_t final_right_pwm = base_pwm + (right_speed_error * SPEED_KP);

    // 绝对限幅保护
    if (final_left_pwm > PWM_MAX_LIMIT)  final_left_pwm  = PWM_MAX_LIMIT;
    if (final_right_pwm > PWM_MAX_LIMIT) final_right_pwm = PWM_MAX_LIMIT;
    if (final_left_pwm < 0)  final_left_pwm  = 0;
    if (final_right_pwm < 0) final_right_pwm = 0;

    /* 只有“确实要求了速度”时才把占空比抬到最小值（为了克服静摩擦）。
       原来是无条件抬到 PWM_MIN_LIMIT：于是目标速度为 0 时(例如刚收到 AUTO_ROAM
       但还没设过 speed)两轮依旧吃到 500 的占空比，车会自己慢慢往前爬。 */
    if (target_mag > 0) {
        if (final_left_pwm > 0 && final_left_pwm < PWM_MIN_LIMIT)   final_left_pwm  = PWM_MIN_LIMIT;
        if (final_right_pwm > 0 && final_right_pwm < PWM_MIN_LIMIT) final_right_pwm = PWM_MIN_LIMIT;
    }

    // 输出给定时器
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)final_right_pwm);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)final_left_pwm);
}

/* 取扫描结果里某一段角度中最远的距离(mm)。返回 0 表示这一段里没有有效数据 */
static uint16_t Scan_Side_Max(uint8_t from, uint8_t to)
{
    uint16_t best = 0;

    for (uint8_t i = from; i <= to && i < 7; i++) {
        if (g_car_state.g_scan.distances[i] > best) {
            best = g_car_state.g_scan.distances[i];
        }
    }
    return best;
}

static uint16_t Scan_Left_Max(void)  { return Scan_Side_Max(SCAN_IDX_LEFT_FROM,  SCAN_IDX_LEFT_TO);  }
static uint16_t Scan_Right_Max(void) { return Scan_Side_Max(SCAN_IDX_RIGHT_FROM, SCAN_IDX_RIGHT_TO); }
static uint16_t Scan_Front(void)     { return g_car_state.g_scan.distances[SCAN_IDX_FRONT];          }

/* 开始一次"按角度转向"：signed_deg 正=左转，负=右转。
   转够没有由陀螺仪积分出的航向判断，不再靠"转多少毫秒"去猜 */
static void Turn_Start(int16_t signed_deg)
{
    if (signed_deg >= 0) {
        SET_Action_execution(CAR_LEFT_TURN);
    } else {
        SET_Action_execution(CAR_RIGHT_TURN);
    }
    g_car_state.turn_target_deg = signed_deg;
    g_car_state.turn_start_yaw  = g_car_state.yaw_deg;
}

/* 这次转向转够了没有。
   用航向变化量的绝对值判断：即使陀螺仪 Z 轴方向跟底盘转向定义相反，
   也能正常结束转向，不至于每次都靠超时才停 */
static bool Turn_Reached(void)
{
    float turned = Gyro_WrapDeg(g_car_state.yaw_deg - g_car_state.turn_start_yaw);
    float target = (float)g_car_state.turn_target_deg;

    if (turned  < 0.0f) turned = -turned;
    if (target  < 0.0f) target = -target;

    return turned >= target;
}

/* 按正前方距离决定"要转多少度"：越近转得越多。front=0 表示没有有效数据 */
static int16_t Avoid_Angle_From_Front(uint16_t front)
{
    if (front == 0)                 return AVOID_TURN_DEFAULT_DEG;  /* 没有测距数据：用默认角度 */
    if (front < AVOID_NEAR_MM)      return AVOID_TURN_LARGE_DEG;    /* 贴得很近，多转一点 */
    if (front < OBSTACLE_DIST_MM)   return AVOID_TURN_DEFAULT_DEG;
    return AVOID_TURN_SMALL_DEG;                                    /* 前方还有余量，小角度让开 */
}

/* 红外 + 超声波一起决定"往哪边转、转多少度"。
   红外(低电平有效)只说明哪一侧被挡住；具体转角由超声波测距决定：
     前方越近 → 转得越多；要转过去的那一侧也不开阔 → 先考虑换边，换不了就加大角度。
   没有任何有效测距数据时，退回原来的默认行为(左转 90°)。 */
static void Avoid_Decide_Turn(uint16_t port_state)
{
    uint16_t front = Scan_Front();
    uint16_t left  = Scan_Left_Max();
    uint16_t right = Scan_Right_Max();

    bool ir_left  = (((~port_state) & KEY_PIN_3) != 0);
    bool ir_right = (((~port_state) & KEY_PIN_4) != 0);
    /* 超声波数据可用 = 这一轮确实量到了距离(非 0 才算数) */
    bool scan_ready = (front > 0) || (left > 0) || (right > 0);

    int16_t angle = AVOID_TURN_DEFAULT_DEG;
    int16_t dir;    /* +1 左转，-1 右转 */

    /* 1) 红外先定方向：哪边被挡住就往反方向让开 */
    if (ir_left && !ir_right) {
        dir = -1;
    } else if (ir_right && !ir_left) {
        dir = +1;
    } else if (scan_ready && (left != right)) {
        dir = (right > left) ? -1 : +1;   /* 两边都被挡住：往开阔的一侧转 */
    } else {
        dir = +1;                          /* 没有参考数据：保持原来的默认左转 */
    }

    /* 2) 有超声波数据时，用距离修正转角 */
    if (scan_ready) {
        angle = Avoid_Angle_From_Front(front);

        /* 要转过去的那一侧也不开阔：能换边就换边，换不了就加大角度 */
        uint16_t turn_side  = (dir > 0) ? left  : right;
        uint16_t other_side = (dir > 0) ? right : left;

        if (turn_side > 0 && turn_side < SIDE_CLEAR_MM) {
            if (other_side > turn_side) {
                dir   = -dir;
                angle = AVOID_TURN_DEFAULT_DEG;
            } else {
                angle += AVOID_TURN_EXTRA_DEG;
            }
        }
    }

    if (angle > AVOID_TURN_MAX_DEG) angle = AVOID_TURN_MAX_DEG;

    Turn_Start((int16_t)(dir * angle));
}

void Obstacle_Check(void) {
    uint16_t port_state = GPIOB->IDR;
    // 低电平代表检测到障碍物
    bool is_obstacle = ((~port_state) & KEY_ALL_PINS);
    uint32_t now = HAL_GetTick();

    switch (g_car_state.avoid_sub_state) {
    case AVOID_IDLE:
        if (is_obstacle) {
            SET_Action_execution(CAR_STOP);
            g_car_state.prev_mode = g_car_state .current_mode;
            g_car_state.current_mode = MODE_AVOIDING;
            g_car_state.avoid_sub_state = AVOID_STOP;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);  //蜂鸣器提示
        }
        break;

    case AVOID_STOP:
        if ((now - g_car_state.roam_state_tick) > AVOID_STOP_MS) {
            SET_Action_execution(CAR_BACKWARD);
            g_car_state.avoid_sub_state = AVOID_BACKWARD;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);   //关闭蜂鸣器
        }
        break;

    case AVOID_BACKWARD:
        if ((now - g_car_state.roam_state_tick) > AVOID_BACK_MS) {
            /* 退够了：先按红外 + 超声波定好转向方向和角度，再开始转 */
            Avoid_Decide_Turn(port_state);
            g_car_state.avoid_sub_state = AVOID_TURN;
        }
        break;

    case AVOID_TURN:
        /* 按角度转向：转够目标角度就结束；陀螺仪失效/车转不动时由超时兜底 */
        if (Turn_Reached() || (now - g_car_state.roam_state_tick) > AVOID_TURN_TIMEOUT_MS) {
            g_car_state.current_mode = g_car_state.prev_mode;
            SET_Action_execution(CAR_STOP);
            g_car_state.avoid_sub_state = AVOID_IDLE; // 回到空闲，继续检测
        }
        break;
    }
}

void Task_Auto_Roam(void)
{
    if (g_car_state.current_mode != MODE_AUTO_ROAM) return;

    uint32_t now = HAL_GetTick();

    /* 正在按角度转向：转够了就立刻结束这次转向，不用等定时器到期 */
    if ((g_car_state.current_motion_state == CAR_LEFT_TURN ||
         g_car_state.current_motion_state == CAR_RIGHT_TURN) && Turn_Reached()) {
        roam_action_end_tick = now;
    }

    // 还没到切换时间，保持当前动作
    if (now < roam_action_end_tick) return;

    // 到达切换时间，用最近一次完整扫描做简单寻路
    uint16_t front = Scan_Front();          // 90° 正前方
    uint16_t left_max  = Scan_Left_Max();   // 0°~60°
    uint16_t right_max = Scan_Right_Max();  // 120°~180°
    bool scan_ready = g_car_state.g_scan.data_valid;

    if (scan_ready && front < OBSTACLE_DIST_MM) {
        // 前方被挡：往更开阔的一侧原地转向，转多少度由前方距离决定
        int16_t dir = (right_max >= left_max) ? -1 : +1;
        Turn_Start((int16_t)(dir * Avoid_Angle_From_Front(front)));
        roam_action_end_tick = now + AVOID_TURN_TIMEOUT_MS;   // 有角度判定，这里只做兜底
    } else {
        // 前方通畅：直行，若一侧明显更开阔则轻微弧线靠过去
        g_car_state.steer_cm_s = 0;
        if (scan_ready && (right_max > SIDE_CLEAR_MM || left_max > SIDE_CLEAR_MM)) {
            if (right_max > left_max)      g_car_state.steer_cm_s =  STEER_ARC_CM_S / 2;
            else if (left_max > right_max) g_car_state.steer_cm_s = -STEER_ARC_CM_S / 2;
        }
        SET_Action_execution(CAR_FORWARD);
        roam_action_end_tick = now + 1000 + (rand() % 2000);
    }

    // 重启扫描，保证运动过程中始终有新鲜的距离数据
    Ultrasonic_Scan_Start();
}

/* ===================== 舵机 ===================== */

void Servo_control_ultrasonic(uint16_t angle)
{
    uint32_t pulse_width_us;

    if (angle > 180U) angle = 180U;

    /* 0~180° 线性映射到 SERVO_MIN_PULSE_US~SERVO_MAX_PULSE_US。
       TIM1 预分频 71 → 计数时钟 1MHz，1 个计数正好 1us，所以 CCR 直接写微秒数 */
    pulse_width_us = SERVO_MIN_PULSE_US +
                     ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180U;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width_us);
}

/* ===================== 超声波(USART1，中断接收) ===================== */

/* US-100 一次测距要 50~60ms 才回包，用阻塞等待会把主循环整个卡住。
   这里用"常驻 1 字节中断接收 + 软件 FIFO"：
     - RXNEIE 一直开着，每来一个字节就在中断里收走，几乎不可能溢出；
     - 扫描状态机只关心"触发之后又来了几个字节"，完全不用等。
   特别注意不能用 HAL_UART_Receive_IT(...,2)：只要有 1 个字节因为溢出没及时读走，
   HAL 就会把整次接收中止，于是永远等不到第 2 个字节，距离全是 0。 */
#define US_RX_FIFO_SIZE 16
static volatile uint8_t s_us_fifo[US_RX_FIFO_SIZE];
static volatile uint8_t s_us_head = 0;
static volatile uint8_t s_us_tail = 0;
static uint8_t s_us_it_byte = 0;          /* 常驻中断接收的单字节缓冲 */

static void Ultrasonic_Rx_Arm(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &s_us_it_byte, 1);
}

/* 丢掉 FIFO 里的旧字节，并清掉接收寄存器/溢出标志里的残留 */
static void Ultrasonic_Flush_Rx(void)
{
    s_us_tail = s_us_head;
    __HAL_UART_CLEAR_OREFLAG(&huart1);    /* 读 SR 再读 DR：清 ORE/RXNE 并丢掉脏字节 */
}

/* 从 FIFO 取一个字节（调用前需确认有新字节） */
static uint8_t Ultrasonic_Pop(void)
{
    uint8_t b = s_us_fifo[s_us_tail];
    s_us_tail = (uint8_t)((s_us_tail + 1) % US_RX_FIFO_SIZE);
    return b;
}

/* 触发一次测距（非阻塞） */
void Ultrasonic_Trigger(void)
{
    uint8_t tx_cmd = ULTRASONIC_TRIGGER_BYTE;
    Scan_Ctrl_t *s = &g_car_state.g_scan;

    Ultrasonic_Flush_Rx();
    s->rx_error  = 0;
    s->rx_mark   = s->rx_total;   /* 记下此刻的计数，之后新增的字节才是本次回包 */
    s->rx_active = 1;

    /* 触发字节只有 1 个 @9600 ≈ 1ms，这点阻塞可以接受 */
    if (HAL_UART_Transmit(&huart1, &tx_cmd, 1, 5) != HAL_OK) {
        s->rx_active = 0;
        s->rx_error  = 1;
    }
}

/* 中断收到 1 个字节（在 HAL_UART_RxCpltCallback 里调用） */
void Ultrasonic_Rx_Complete(void)
{
    uint8_t next = (uint8_t)((s_us_head + 1) % US_RX_FIFO_SIZE);

    if (next != s_us_tail) {          /* FIFO 满就丢新数据，保护旧数据 */
        s_us_fifo[s_us_head] = s_us_it_byte;
        s_us_head = next;
    }
    g_car_state.g_scan.rx_total++;
    Ultrasonic_Rx_Arm();              /* 立刻重新挂上，保持常驻接收 */
}

/* 接收出错（溢出/帧错误）：清标志并把常驻接收重新挂上，否则串口就"收不到"了 */
void Ultrasonic_Rx_Error(void)
{
    g_car_state.g_scan.rx_error = 1;
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    Ultrasonic_Rx_Arm();
}

/* 主循环里调用：万一常驻接收被异常中止，这里自愈 */
void Ultrasonic_Rx_Service(void)
{
    if (huart1.RxState == HAL_UART_STATE_READY) {
        Ultrasonic_Rx_Arm();
    }
}

/* 用最近一帧的两个字节算出距离：高 8 位在前，单位 mm */
static uint16_t Ultrasonic_Parse(void)
{
    uint16_t mm = ((uint16_t)g_car_state.g_scan.rx_buf[0] << 8) | g_car_state.g_scan.rx_buf[1];

    if (mm < ULTRASONIC_MIN_MM || mm > ULTRASONIC_MAX_MM) {
        return 0;
    }
    return mm;
}

uint16_t Ultrasonic_Get_Distance(uint8_t index)
{
    if (index >= 7) return 0;
    return g_car_state.g_scan.distances[index];
}

/* 记下当前角度的结果，并推进到下一个角度 */
static void Ultrasonic_Scan_Next(uint16_t distance_mm)
{
    Scan_Ctrl_t *s = &g_car_state.g_scan;

    s->distances[s->angle_idx] = distance_mm;
    if (distance_mm > 0) s->ok_cnt++;
    else                 s->err_cnt++;

    s->rx_active = 0;
    s->rx_error  = 0;

    s->angle_idx++;
    s->state = (s->angle_idx >= 7) ? SCAN_COMPLETE : SCAN_MOVE_SERVO;
}

void Ultrasonic_Scan_Update(void)
{
    Scan_Ctrl_t *s = &g_car_state.g_scan;
    uint32_t now = HAL_GetTick();

    switch (s->state) {
    case SCAN_MOVE_SERVO:
        /* 先转舵机，等它到位再测，否则量到的还是上一个方向的回声 */
        Servo_control_ultrasonic(s->angles[s->angle_idx]);
        s->state = SCAN_SETTLE;
        s->state_tick = now;
        break;

    case SCAN_SETTLE:
        if ((now - s->state_tick) >= SCAN_SETTLE_MS) {
            s->state = SCAN_MEASURE;
            s->state_tick = now;
            s->rx_active = 0;
        }
        break;

    case SCAN_MEASURE:
        if (!s->rx_active) {
            Ultrasonic_Trigger();          /* 发触发，立即返回 */
            if (s->rx_error) {
                Ultrasonic_Scan_Next(0);   /* 连触发都没发出去 */
            }
            break;                         /* 下一拍再来看回包 */
        }
        /* 触发之后 FIFO 里新增的字节就是本次回包，凑够 2 个就解析 */
        if ((uint16_t)(s->rx_total - s->rx_mark) >= 2) {
            s->rx_buf[0] = Ultrasonic_Pop();
            s->rx_buf[1] = Ultrasonic_Pop();
            Ultrasonic_Scan_Next(Ultrasonic_Parse());
            break;
        }
        if (s->rx_error || (now - s->state_tick) >= SCAN_REPLY_TIMEOUT_MS) {
            Ultrasonic_Flush_Rx();         /* 半截数据清掉，免得影响下一次 */
            Ultrasonic_Scan_Next(0);
        }
        break;

    case SCAN_COMPLETE:
        s->data_valid = 1;                 /* 已有完整扫描结果 */
        Servo_control_ultrasonic(SERVO_CENTER_ANGLE);   /* 间隙期间舵机回中位 */
        s->state_tick = now;
        s->state = (SCAN_CONTINUOUS && s->continuous) ? SCAN_GAP : SCAN_IDLE;
        break;

    case SCAN_GAP:
        if ((now - s->state_tick) >= SCAN_GAP_MS &&
            (int32_t)(now - s->hold_until) >= 0) {
            s->angle_idx  = 0;
            s->data_valid = 0;             /* 新一轮扫描开始，旧结果作废 */
            s->ok_cnt  = 0;
            s->err_cnt = 0;
            s->state = SCAN_MOVE_SERVO;
        }
        break;

    case SCAN_IDLE:
    default:
        break;
    }
}

void Ultrasonic_Scan_Start(void)
{
    Scan_Ctrl_t *s = &g_car_state.g_scan;

    if (s->state != SCAN_IDLE && s->state != SCAN_GAP) return;  /* 正在扫描，不重复启动 */

    s->angle_idx  = 0;
    s->data_valid = 0;
    s->ok_cnt  = 0;
    s->err_cnt = 0;
    s->rx_active = 0;
    s->rx_error  = 0;
    s->hold_until = 0;
    Ultrasonic_Flush_Rx();
    s->state = SCAN_MOVE_SERVO;
}

void Ultrasonic_Scan_Stop(void)
{
    Scan_Ctrl_t *s = &g_car_state.g_scan;

    s->rx_active = 0;
    s->state = SCAN_IDLE;
    Ultrasonic_Flush_Rx();
}

/* ===================== 舵机自检 ===================== */

static uint8_t  s_servo_test_active = 0;
static uint8_t  s_servo_test_step   = 0;
static uint32_t s_servo_test_tick   = 0;

/* 只在中位附近小幅摆动：既看得出舵机在不在动，又不会顶到机械限位 */
static const uint16_t s_servo_test_angles[] = {90, 60, 90, 120, 90};

void Servo_Test_Start(void)
{
    Ultrasonic_Scan_Stop();        /* 先停掉正在进行的扫描 */

    s_servo_test_active = 1;
    s_servo_test_step = 0;
    s_servo_test_tick = 0;

    g_car_state.g_scan.hold_until = HAL_GetTick() + 10000;  /* 自检期间不自动扫描 */
}

void Servo_Test_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (!s_servo_test_active) return;

    if (s_servo_test_tick != 0 && (now - s_servo_test_tick) < 1000) return;  /* 每步停 1s */
    s_servo_test_tick = now;

    if (s_servo_test_step >= (sizeof(s_servo_test_angles) / sizeof(s_servo_test_angles[0]))) {
        s_servo_test_active = 0;
        Servo_control_ultrasonic(SERVO_CENTER_ANGLE);
        Ultrasonic_Scan_Start();   /* 自检结束，恢复自动扫描 */
        return;
    }

    Servo_control_ultrasonic(s_servo_test_angles[s_servo_test_step++]);
}

// 0.5Hz 状态回传（yaw 单位为 0.1°）
void Telemetry_Update(void) {
    uint16_t left_max  = Scan_Left_Max();
    uint16_t right_max = Scan_Right_Max();
    // gv=陀螺仪有没有读到数据  sc=扫描状态机  ok/er=本轮超声成功/失败次数  tt=本次转向目标角度
    Bt_Printf("[T] m=%d spd=%d steer=%d yaw=%d pit=%d gv=%d sc=%d ok=%d er=%d tt=%d L=%d F=%d R=%d\r\n",
              g_car_state.current_mode,
              g_car_state.linear_speed_cm_s,
              g_car_state.steer_cm_s,
              (int16_t)(g_car_state.yaw_deg * 10.0f),
              g_car_state.pitch_deg,
              g_car_state.gyro_valid,
              (int)g_car_state.g_scan.state,
              g_car_state.g_scan.ok_cnt,
              g_car_state.g_scan.err_cnt,
              g_car_state.turn_target_deg,
              left_max, Scan_Front(), right_max);
}