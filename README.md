# STM32 两轮自平衡小车

主控：STM32F103C8T6；电机驱动：TB6612FNG。以下接线与当前 CubeMX 工程一致，按模块丝印连接，不代表排针顺序。

## 核心板与电机驱动接线

| TB6612FNG 引脚 | 连接到 |
| --- | --- |
| PWMA | PA8（TIM1_CH1，左轮 PWM） |
| AIN1 | PB12（左轮方向） |
| AIN2 | PB13（左轮方向） |
| PWMB | PA9（TIM1_CH2，右轮 PWM） |
| BIN1 | PB14（右轮方向） |
| BIN2 | PB15（右轮方向） |
| STBY | PB0（高电平使能，低电平待机；不接 5V） |
| VCC | 3.3V 逻辑电源 |
| VM | 独立电机电源正极（方案为稳压 6V，实际电源待确认） |
| GND / PGND | STM32 GND、电机电源负极，共地 |
| AO1 / AO2 | 左电机两根绕组线 |
| BO1 / BO2 | 右电机两根绕组线 |

两路 PWM 均为 20kHz，上电 STBY 为低。电机不接 STM32 电源引脚；电机绕组线须先核实，不能猜测六针连接器的实际顺序。核心板具体供电入口按实物板型确认。

## 下载与串口

- SWD：PA13 → SWDIO，PA14 → SWCLK，GND → 调试器 GND。
- USB 串口：PA2（TX）→ 模块 RX，PA3（RX）→ 模块 TX，并共地；使用 3.3V TTL 电平，115200、8N1。

其余外设见 [完整引脚表](PINOUT.md)，烧录与测试见 [调试指南](CLI_AND_MOTOR_DEBUG.md)。
