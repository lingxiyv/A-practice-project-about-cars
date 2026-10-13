# A_small_car

基于 STM32F103C8T6（Blue Pill）的智能小车练习项目，使用 STM32CubeMX + HAL 库，CMake 构建。

> ⚠️ **声明：本项目仅用于学习、研究与个人练习，不用于任何商业或生产用途。**

## 功能特性

- **蓝牙遥控**：通过 USART2 接收指令，`W` / `A` / `S` / `D` 分别控制前进 / 左转 / 后退 / 右转，`speed NNN` 调速，指令以 `\n` 或 `\r` 结尾
- **自动漫游**：发送 `AUTO_ROAM` 进入自动漫游模式，小车随机前进 / 转向自主行走
- **障碍避让**：左右前方红外检测到障碍后自动停车 → 后退 → **按角度转向**（用陀螺仪判断转够没有），蜂鸣器提示，避让完成后回到之前的模式；转向角度由超声波测距决定
- **双轮闭环调速**：左右轮各接一个旋转编码器，接入 TIM3 / TIM4 硬件编码器接口（4 倍频解码），实时解算左右轮线速度与底盘中心线速度
- **自动 PWM 调速**：前馈（目标速度→基础 PWM）+ 比例（速度误差修正）+ 左右轮差速同步修正，实时调整两路 PWM 占空比，使实际速度逼近目标速度
- **OLED 显示**：通过硬件 I²C1（DMA 传输）驱动 SSD1306，实时显示运行模式与当前速度（cm/s）
- **按键复位**：PB0 按键将车辆复位到停止状态、PWM 清零、参数复位到默认值
- **运行指示**：PC13 LED 以 1 Hz 闪烁作为心跳指示（MPU6050 初始化失败时改成 5 Hz 快闪，见下）
- **超声波扫描测距**：SG90 舵机（PA8 / TIM1_CH1）带 US-100 超声波（USART1）在 0°~180° 之间循环扫描，7 个角度各测一次距离
- **陀螺仪航向保持**：MPU6050（I²C1）积分 Z 轴角速度得到航向，上电先做静止零偏标定，直行时用航向误差修正左右轮差速
> 注：自动漫游会沿用上一次设定的速度，如果上电后直接发 AUTO_ROAM 目标速度是0，小车不会行动

> 扫描默认是**上电后自动循环**的（`SCAN_CONTINUOUS = 1`）：任何模式下蓝牙回传里都有 L/F/R 距离，
> 舵机每轮扫描之间会在中位停 0.8 s。如果只想按需扫描，把 `global_parameters.h` 里的
> `SCAN_CONTINUOUS` 改成 0（那样只有发 `AUTO_ROAM` 才会扫描）。

> **MPU6050 初始化失败不再死机**：以前会停在 `while(1)` 里，整车（舵机 / 超声波 / 蓝牙 / 电机）
> 一起失能，看起来像"板子坏了"。现在只打印 `MPU6050 Init Failed! run without gyro`、PC13 改成
> 5 Hz 快闪，然后以 `gv=0` 继续运行：失去航向保持，按角度转向会退化成超时兜底（`AVOID_TURN_TIMEOUT_MS`），
> 其它功能照旧。

## 运行模式

| 模式 | 枚举 | 说明 |
| --- | --- | --- |
| 空闲 | `MODE_IDLE` | 上电后的初始状态，等待指令 |
| 手动 | `MODE_MANUAL` | 蓝牙遥控模式 |
| 自动漫游 | `MODE_AUTO_ROAM` | 随机前进 / 转向自主行走 |
| 避障中 | `MODE_AVOIDING` | 临时状态，避让完成后回到之前的模式 |

- 收到 `W` / `A` / `S` / `D` / `speed` 指令会切换到手动模式（避障过程中不被手动指令打断）。
- 收到 `AUTO_ROAM` 切换到自动漫游模式。
- 手动模式下，约 500 ms 未收到任何蓝牙数据会自动停车并回到空闲状态。

### 避障与按角度转向

避障流程：红外触发 → 急停 200 ms（`AVOID_STOP_MS`）→ 后退 500 ms（`AVOID_BACK_MS`）→ 原地转向 → 回到原来的模式。

- **按角度转向**：靠 MPU6050 积分出的航向判断"转够了没有"，不再用固定的 1 s。
  `AVOID_TURN_TIMEOUT_MS`(4000 ms) 只是陀螺仪失效 / 车被卡住转不动时的兜底。
  判断用的是航向变化量的绝对值，所以即使陀螺仪 Z 轴朝向与底盘转向定义相反也能正常结束。
- **转向角度由超声波测距修正**（本轮扫描确实量到距离时才生效）：

  | 正前方距离（`distances[3]`） | 转向角度 |
  | --- | --- |
  | < `AVOID_NEAR_MM`(150 mm) | `AVOID_TURN_LARGE_DEG`(120°) |
  | < `OBSTACLE_DIST_MM`(300 mm) | `AVOID_TURN_DEFAULT_DEG`(90°) |
  | ≥ 300 mm | `AVOID_TURN_SMALL_DEG`(60°) |

- **方向**：先看红外——哪一侧被挡住就往反方向让开；两侧都被挡住时，往超声波更开阔的一侧转。
- **要转过去的一侧也不开阔**（该侧最远距离 < `SIDE_CLEAR_MM` = 400 mm）时：另一侧更开阔就改往另一侧转，
  否则额外多转 `AVOID_TURN_EXTRA_DEG`(30°)，总转角上限 `AVOID_TURN_MAX_DEG`(150°)。
- **没有任何有效测距数据**时，退回原来的默认行为：左转 90°。
- 避障动作有最低速度 `AVOID_MIN_SPEED_CM_S`(30 cm/s)：即使 `speed 0`，避障也能转得动。
- 自动漫游里"前方被挡就转向"同样改成了按角度转，转角由前方距离决定。
- 原地转向时不再叠加航向保持（否则转弯会和"修航向"互相打架，按角度转向永远转不到位）。

## 硬件平台

| 外设 | 型号 / 引脚 |
| --- | --- |
| MCU | STM32F103C8T6（Blue Pill） |
| 电机驱动 | L298N 类（IN1~IN4 → PB13 / PB12 / PB11 / PB10） |
| PWM 调速 | TIM2 CH1（PA0）、CH2（PA1），50 Hz |
| 右轮编码器 | TIM3 编码器模式，A 相 PA6（TIM3_CH1）、B 相 PA7（TIM3_CH2） |
| 左轮编码器 | TIM4 编码器模式，A 相 PB6（TIM4_CH1）、B 相 PB7（TIM4_CH2） |
| 蓝牙模块 | USART2（TX=PA2，RX=PA3），9600 8N1 |
| 扫描舵机 | SG90 类，信号 PA8（TIM1_CH1），50 Hz，脉宽 500~2400 us |
| 超声波 | US-100（串口模式）接 USART1（TX=PA9，RX=PA10），9600，发 `0x55` 回 2 字节距离(mm) |
| 陀螺仪 | MPU6050，硬件 I²C1（与 OLED 共用总线，地址 0x68） |
| 避障传感器 | 左前方 PB3、右前方 PB4，低电平有效 |
| 蜂鸣器 | PB5，检测到障碍时提示 |
| OLED 显示 | 硬件 I²C1（SCL=PB8，SDA=PB9，DMA 传输），SSD1306，地址 0x78 |
| 复位按键 | PB0 |
| 运行指示灯 | PC13 |

> 注：PB3 / PB4 默认复用 JTAG 功能，已通过 `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 释放用于 GPIO。

## 蓝牙指令

指令以换行符（`\n`）或回车（`\r`）结尾（兼容 CRLF），蓝牙串口波特率 9600 8N1。

| 指令 | 说明 |
| --- | --- |
| `W` | 前进 |
| `A` | 左转 |
| `S` | 后退 |
| `D` | 右转 |
| `speed NNN` | 设定目标速度，`NNN` 为目标速度百分比（0~100） |
| `AUTO_ROAM` | 进入自动漫游模式 |
| `angle NNN` | 手动把扫描舵机转到 `NNN` 度（0~180），并暂停自动扫描 5 s |
| `servotest` | 舵机慢速自检（在中位附近 90→60→90→120→90 来回摆动），用来单独判断舵机硬件好坏 |

- `speed NNN` 示例：`speed 50` 表示目标速度为最大速度的 50%（最大速度约 120 cm/s），`speed 100` 为全速；超出 0~100 范围自动截断。
- `AUTO_ROAM` 示例：发送后小车进入自动漫游模式，随机前进（持续 1~3 s）或转向，自主行走。
- 手动模式（`MODE_MANUAL`）下，约 500 ms 未收到任何蓝牙数据会自动停车并回到空闲状态。
- 0.5 Hz 状态回传格式（各项都是整数，`yaw` 单位 0.1°）：
  `[T] m=模式 spd=线速度 steer=转向差速 yaw=航向 pit=坡度 gv=陀螺仪有效 sc=扫描状态 ok=本轮成功次数 er=失败次数 tt=本次转向目标角度 L=左侧最远 F=正前方 R=右侧最远`

## 目录结构

```
├── Core/               # 应用源码（main、chassis、usart、tim、gpio、i2c、oled 等）
├── Drivers/            # STM32F1xx HAL 驱动
├── cmake/              # 工具链与 CubeMX 子工程
├── A_small_car.ioc     # STM32CubeMX 工程配置
├── CMakeLists.txt      # 顶层构建脚本
├── CMakePresets.json   # Debug / Release 预设
└── my-blue-pill.cfg    # OpenOCD 烧录配置（ST-Link V2，SWD）
```

## 构建

依赖：`arm-none-eabi-gcc`、`CMake (>=3.22)`、`Ninja`。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

## 烧录与调试

使用 ST-Link V2 连接目标板，通过 OpenOCD 烧录：

```bash
openocd -f my-blue-pill.cfg \
        -c "program build/Debug/A_small_car.elf verify reset exit"
```

如使用其他调试器（J-Link / DAPLink），替换 `my-blue-pill.cfg` 中的接口文件即可。
