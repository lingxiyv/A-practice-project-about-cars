# A_small_car

基于 STM32F103C8T6（Blue Pill）的智能小车练习项目，使用 STM32CubeMX + HAL 库，CMake 构建。

## 功能特性

- **蓝牙遥控**：通过 USART2 接收蓝牙指令，W / A / S / D 分别控制前进 / 左转 / 后退 / 右转，收到其他字符或超时自动停车
- **编码器调速**：旋转编码器接入 TIM4 硬件编码器接口（4 倍频解码），在主循环中读取计数值实时增减 PWM 占空比，控制电机转速
- **OLED 显示**：实时显示车辆状态与 PWM 值
- **按键复位**：PB0 按键将 PWM 占空比复位到默认值

## 硬件平台

| 外设 | 型号 / 引脚 |
| --- | --- |
| MCU | STM32F103C8T6（Blue Pill） |
| 电机驱动 | L298N 类（IN1~IN4 → PA4 / PA5 / PA6 / PA7） |
| PWM 调速 | TIM2 CH1（PA0）、CH2（PA1），50 Hz |
| 编码器 | TIM4 编码器模式，A 相 PB6（TIM4_CH1）、B 相 PB7（TIM4_CH2） |
| 蓝牙模块 | USART2（TX=PA2，RX=PA3），9600 8N1 |
| OLED 显示 | I²C（软件模拟，SCL=PB5，SDA=PB4） |
| 复位按键 | PB0 |

## 目录结构

```
├── Core/               # 应用源码（main、action、usart、tim、gpio、oled 等）
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
