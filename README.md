# A_small_car

基于 STM32F103C8T6（Blue Pill）的智能小车练习项目，使用 STM32CubeMX + HAL 库，CMake 构建。

> ⚠️ **声明：本项目仅用于学习、研究与个人练习，不用于任何商业或生产用途。**

## 功能特性

- **蓝牙遥控**：通过 USART2 接收指令，`W` / `A` / `S` / `D` 分别控制前进 / 左转 / 后退 / 右转，`speed NNN` 调速，指令以 `\n` 或 `\r` 结尾
- **自动漫游**：发送 `AUTO_ROAM` 进入自动漫游模式，小车随机前进 / 转向自主行走
- **障碍避让**：左右前方传感器检测障碍后自动停车 → 后退 → 转向避让，蜂鸣器提示，避让完成后回到之前的模式
- **双轮闭环调速**：左右轮各接一个旋转编码器，接入 TIM3 / TIM4 硬件编码器接口（4 倍频解码），实时解算左右轮线速度与底盘中心线速度
- **自动 PWM 调速**：前馈（目标速度→基础 PWM）+ 比例（速度误差修正）+ 左右轮差速同步修正，实时调整两路 PWM 占空比，使实际速度逼近目标速度
- **OLED 显示**：通过硬件 I²C1（DMA 传输）驱动 SSD1306，实时显示运行模式与当前速度（cm/s）
- **按键复位**：PB0 按键将车辆复位到停止状态、PWM 清零、参数复位到默认值
- **运行指示**：PC13 LED 以 1 Hz 闪烁作为心跳指示
> 注：自动漫游会沿用上一次设定的速度，如果上电后直接发 AUTO_ROAM 目标速度是0，小车不会行动

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

## 硬件平台

| 外设 | 型号 / 引脚 |
| --- | --- |
| MCU | STM32F103C8T6（Blue Pill） |
| 电机驱动 | L298N 类（IN1~IN4 → PB13 / PB12 / PB11 / PB10） |
| PWM 调速 | TIM2 CH1（PA0）、CH2（PA1），50 Hz |
| 右轮编码器 | TIM3 编码器模式，A 相 PA6（TIM3_CH1）、B 相 PA7（TIM3_CH2） |
| 左轮编码器 | TIM4 编码器模式，A 相 PB6（TIM4_CH1）、B 相 PB7（TIM4_CH2） |
| 蓝牙模块 | USART2（TX=PA2，RX=PA3），9600 8N1 |
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

- `speed NNN` 示例：`speed 50` 表示目标速度为最大速度的 50%（最大速度约 120 cm/s），`speed 100` 为全速；超出 0~100 范围自动截断。
- `AUTO_ROAM` 示例：发送后小车进入自动漫游模式，随机前进（持续 1~3 s）或转向，自主行走。
- 手动模式（`MODE_MANUAL`）下，约 500 ms 未收到任何蓝牙数据会自动停车并回到空闲状态。

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
