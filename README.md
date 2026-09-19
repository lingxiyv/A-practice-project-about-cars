# A_small_car

基于 STM32F103C8T6（Blue Pill）的智能小车练习项目，使用 STM32CubeMX + HAL 库，CMake 构建。

## 功能特性

- **方向控制**：按键切换前进 / 后退 / 左转 / 右转 / 停止
- **调速**：ADC 采集电位器电压，经 8 点滑动平均滤波后映射为 PWM 占空比
- **OLED 显示**：实时显示车辆状态与 ADC 数值

## 硬件平台

| 外设 | 型号 / 引脚 |
| --- | --- |
| MCU | STM32F103C8T6（Blue Pill） |
| 电机驱动 | L298N 类（IN1~IN4 → PA4 / PA5 / PA6 / PA7） |
| PWM 调速 | TIM2 CH1（PA0）、CH2（PA1），50 Hz |
| 速度给定 | ADC1 CH8（PB0），电位器 |
| OLED 显示 | I²C（软件模拟，SCL=PA2，SDA=PA3） |
| 按键 | 前进 PB11、后退 PB13、左转 PB15、右转 PA11 |

## 目录结构

```
├── Core/               # 应用源码（main、action、adc、tim、gpio、oled 等）
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
