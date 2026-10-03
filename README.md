# STM32 MPU6050 电机姿态控制

## 项目简介

本项目是电控组第一次考核题的 STM32F103C8T6 固件框架，使用 MPU6050 获取姿态，通过 PID 串级控制和 TB6612 驱动 TT 减速电机，并通过 VOFA+ FireWater 查看数据、发送目标值。

## 目录结构

```text
App/                 应用初始化、控制调度和串级控制
BSP/                 电机与编码器板级驱动
Algorithm/           独立 PID 算法
Communication/       VOFA+ FireWater 串口收发与解析
ThirdParty/          MPU6050 驱动、姿态解算和工程适配层
Core/                STM32CubeMX 生成的外设初始化和中断代码
Drivers/             STM32 HAL 与 CMSIS 驱动
Docs/                项目文档
cmake/               CMake 工具链配置
```

## 模块功能

- `App`: `App_Init()` 初始化编码器、MPU6050、电机和 PID；`App_Loop()` 按固定周期运行角度环/速度环并发送当前模式的数据。
- `BSP/motor`: 控制 TB6612 的方向、STBY 和 TIM2 PWM。
- `BSP/encoder`: 读取 TIM3 编码器计数、计数差和计数速度。速度单位是编码器计数/秒。
- `Algorithm/pid`: 纯 C 位置式 PID，包含输出、积分限幅和可选积分分离。
- `Communication/vofa`: 通过 USART1 发送三列浮点数，并按行解析上位机发来的第一个浮点数作为 target。
- `ThirdParty`: 上游 MPU6050 I2C 驱动、Mahony 姿态解算，以及统一的角度/角速度/角加速度读取接口。

## 硬件接线

| 功能 | STM32 引脚 | 工程配置 |
| --- | --- | --- |
| TB6612 AIN1 | PA4 | GPIO 输出 |
| TB6612 AIN2 | PA5 | GPIO 输出 |
| TB6612 PWMA | PA0 | TIM2 CH1 PWM |
| TB6612 STBY | PB12 | GPIO 输出，高电平使能 |
| 编码器 A/B 相 | PA6 / PA7 | TIM3 CH1 / CH2 编码器输入 |
| MPU6050 SCL/SDA | PB6 / PB7 | I2C1 |
| VOFA+ USART1 TX/RX | PA9 / PA10 | 115200, 8N1 |
| LED | PC13 | GPIO 输出 |
| KEY | PB0 | GPIO 输入 |

USB 转串口连接时交叉连接 TX/RX，并与开发板共地。电机电源和 TB6612 逻辑电源应按驱动板及电机规格供电，所有地线共地；不要从 MCU IO 口给电机供电。考核题建议使用合适的外部 5 V 电源，并提醒不要用 12 V 或电脑 USB 口的 5 V 直接供电。当前 STBY 接 PB12，由 MCU 的 3.3 V GPIO 控制使能，不要再把 STBY 直接接到固定 5 V。

MPU6050 驱动默认 AD0 接地、I2C 地址为 `0x68`。`MPU6050_Init()` 会做陀螺仪零偏校准，校准时保持传感器静止。姿态控制角暂时默认为 roll；确认传感器安装方向后，在 `ThirdParty/Inc/mpu6050_api.h` 修改 `MPU6050_CONTROL_ANGLE` 为 roll、pitch 或 yaw 对应宏。

## 编译

需要安装 STM32CubeCLT（包含 ARM GNU 工具链）、CMake 和 Ninja，并确保这些工具在终端 `PATH` 中。在工程根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出固件为 `build/Debug/Project_MPU6050.elf`。

## VOFA+ 使用

VOFA+ 选择 USART1 对应串口，波特率 `115200`、数据位 8、无校验、停止位 1，使用 FireWater/文本数据通道。每帧格式为三列浮点数和换行，例如 `12.5,10.2,300.0`。

接收目标值时发送一个浮点数并以换行结束，例如 `30.0\n`。角度模式下该值单位为度；速度模式下单位为编码器计数/秒。`main.c` 已启动 USART1 单字节中断接收并将字节交给 VOFA 行解析器。

`App/Inc/app.h` 定义的模式可通过调试器或后续按键逻辑切换 `vofa_mode`：

| 模式 | 输出通道 | 控制行为 |
| --- | --- | --- |
| `VOFA_MODE_ANGLE` | 目标角、实际角、PWM 输出 | 角度 PID 产生目标速度，速度 PID 产生 PWM |
| `VOFA_MODE_SPEED` | 目标速度、实际速度、PWM 输出 | 关闭角度环，VOFA target 直接作为速度目标 |
| `VOFA_MODE_ENCODER` | 当前有符号 16 位 TIM3 CNT、0、0 | 保持角度串级控制 |
| `VOFA_MODE_IMU` | X 轴角速度、X 轴角加速度、控制姿态角 | 保持角度串级控制 |

编码器输出是当前 16 位 CNT，不是跨溢出的累计位置。控制周期和 PID 增益、限幅可在 `App/Inc/app.h` 修改；默认值仅供上板调试起点，最终需要根据电机、减速比、编码器和机构实测调整。
