# Balance-vehicle 引脚配置

芯片：STM32F103C8T6，LQFP48。HSE 与 PLL 配置保留，SYSCLK 为 72 MHz；PA13/PA14 保留 Serial Wire。

| 用途 | 引脚 | 外设/模式 | 代码标签 |
| --- | --- | --- | --- |
| 左轮编码器 A | PA0 | TIM2_CH1 | LEFT_ENCODER_A |
| 左轮编码器 B | PA1 | TIM2_CH2 | LEFT_ENCODER_B |
| 右轮编码器 A | PA6 | TIM3_CH1 | RIGHT_ENCODER_A |
| 右轮编码器 B | PA7 | TIM3_CH2 | RIGHT_ENCODER_B |
| 左轮 PWM | PA8 | TIM1_CH1 | LEFT_PWM |
| 右轮 PWM | PA9 | TIM1_CH2 | RIGHT_PWM |
| 左电机 AIN1 | PB12 | GPIO 推挽输出 | LEFT_AIN1 |
| 左电机 AIN2 | PB13 | GPIO 推挽输出 | LEFT_AIN2 |
| 右电机 BIN1 | PB14 | GPIO 推挽输出 | RIGHT_BIN1 |
| 右电机 BIN2 | PB15 | GPIO 推挽输出 | RIGHT_BIN2 |
| TB6612 STBY | PB0 | GPIO 推挽输出 | TB6612_STBY |
| MPU6050 SCL | PB6 | I2C1_SCL，复用开漏 | MPU6050_SCL |
| MPU6050 SDA | PB7 | I2C1_SDA，复用开漏 | MPU6050_SDA |
| 调试串口 TX | PA2 | USART2_TX | DEBUG_TX |
| 调试串口 RX | PA3 | USART2_RX | DEBUG_RX |

## 初始参数

- TIM2/TIM3：Encoder TI1+TI2 模式，四倍频计数；PSC=0，ARR=65535，无输入滤波、无内部上下拉。
- TIM1：PWM mode 1，高电平有效；PSC=0，ARR=3599，72 MHz / 3600 = 20 kHz；CH1/CH2 初始 CCR=0。
- I2C1：100 kHz，7 位地址模式。
- USART2：115200 baud，8 数据位、1 停止位、无校验、无硬件流控。
- 电机方向脚和 STBY：初始化输出低电平。

初始化代码由 CubeMX 生成在 `Core/Src/tim.c`、`i2c.c`、`usart.c`、`gpio.c`。引脚宏位于 `Core/Inc/main.h`。

第一阶段应用位于 `App/motor.c`、`App/motor_test.c`：调用 `HAL_TIM_PWM_Start()` 启动 TIM1 CH1/CH2，上电保持 STBY 低；默认仅手动串口点动命令使能驱动。当前命令行烧录、串口与万用表排查步骤见 [CLI_AND_MOTOR_DEBUG.md](CLI_AND_MOTOR_DEBUG.md)，接线原理见 [MOTOR_TEST.md](MOTOR_TEST.md)。

两路编码器已接入并验证：车轮一圈约 1000 个四倍频计数（`ENCODER_COUNTS_PER_REV`），串口每秒输出左右轮计数与 RPM。

## MPU6050 接线（GY-521 模块）

| GY-521 引脚 | 接到 | 说明 |
| --- | --- | --- |
| VCC | STM32 **3.3V** | 先确认手上的模块支持 3.3V 输入，首次联调使用 3.3V 逻辑 |
| GND | 公共 **GND** | 必须与电机驱动共地 |
| SCL | **PB6** | I2C1_SCL，100 kHz，开漏 |
| SDA | **PB7** | I2C1_SDA，100 kHz，开漏 |
| AD0 | **GND** | 选中 7 位地址 `0x68`（接 3.3V 会变成 `0x69`，代码不匹配） |
| INT | 不接 | 当前使用轮询，不用中断 |
| XDA / XCL | 不接 | 仅用于外接磁力计等辅助 I2C 设备 |

注意：

- SCL 和 SDA 各需一个上拉。先核对模块是否已有上拉以及上拉电压。若实板没有上拉且采用 3.3V 逻辑，每条线各加一个约 4.7 kΩ 电阻到 3.3V。避免将总线拉到 5V。
- 模块要**刚性固定**在车体顶层，芯片面朝上，不能靠杜邦线悬空，否则车体晃动会直接污染姿态角。
- 按当前实测安装方向，模块 **Y 轴沿两轮轴线**。`App/mpu6050.h` 选用 `MPU6050_PITCH_AXIS_Y`，两个符号系数均为 `-1`，抬起车头时 `pitch` 增大。重新安装模块后，用 `i` 复核俯仰轴和正负方向。

首次接入时保持电机 VM 断开，模块放平静止后复位。USART2 使用 115200、8N1。启动应显示 `WHO_AM_I=0x68`，随后打印陀螺零偏。静止平放时加速度合量应接近 1g，校准后的角速度应接近 0°/s。

- `g`：读一次 MPU6050。关闭连续采样时，俯仰角由这一帧加速度计算。
- `i`：开启或关闭约 10 Hz 的连续输出，包含互补滤波倾角和换算后的三轴加速度、角速度。缓慢前后倾斜，观察角度是否连续变化。电机点动期间暂停 IMU 读取。
- `G`：车体静止时重新做陀螺零偏校准；
- `d`：同时打印电机驱动状态，便于区分是 IMU 还是电机的问题。

后续 PID（直立环/速度环）尚未添加，未做实物平衡验证。

## 命令行重新生成与构建

先将 CubeMX GUI 中的编辑保存并关闭该项目，再修改 `.ioc`，然后在项目根目录执行：

```bash
bash tools/generate.sh
cmake --preset Debug
cmake --build --preset Debug
```

`tools/generate.sh` 会读取当前项目的 `.ioc` 并调用 CubeMX `-q` 模式重新生成工程；它不负责修改引脚。AI 可以修改 `.ioc` 后执行此脚本，省去 GUI 中逐个点击。

默认使用 PATH 中的 `stm32cubemx`；可通过 `STM32CUBEMX=/path/to/executable` 指定可执行文件。本机 CubeMX 6.18.1 即使使用 `-q` 也需要可用的图形会话。

修改应用代码时使用 `USER CODE BEGIN/END` 区块或独立文件，便于后续重新生成。
