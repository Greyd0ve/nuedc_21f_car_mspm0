# nuedc_e_car_mspm0

本工程是基于 TI MSPM0G3507、Keil MDK、MSPM0 SDK、SysConfig 和 MSPM0 DriverLib 的电赛 E 题自动寻迹小车工程。当前阶段优先适配新版 IO 并做板级外设验证，不默认上车完整循迹。

## 软件结构

| 目录 | 说明 |
| --- | --- |
| `User/` | 主入口与主循环 |
| `App/` | E 题状态机、循迹逻辑、串口命令、板级测试 |
| `Control/` | PID、滤波、速度闭环 |
| `Hardware/` | MSPM0G3507 外设驱动封装 |
| `System/` | 系统节拍、定时器、调度 |
| `keil/` | Keil MDK 工程文件 |

板级逻辑名集中在 `Hardware/Board_Config.h`。如果 SysConfig 重新生成的宏名变化，优先在 `Board_Config.h` 中做映射，不要在 App 层散落 DriverLib 或寄存器调用。

## 新版 IO 映射

### TB6612 电机驱动

| 逻辑名 | 硬件连接 | 说明 |
| --- | --- | --- |
| `MOTOR_L_PWM` | TIMG0-C0 | 左电机 PWM |
| `MOTOR_L_IN1` | B17 | 左电机方向 1 |
| `MOTOR_L_IN2` | B19 | 左电机方向 2 |
| `MOTOR_R_PWM` | TIMG0-C1 | 右电机 PWM |
| `MOTOR_R_IN1` | A16 | 右电机方向 1 |
| `MOTOR_R_IN2` | B24 | 右电机方向 2 |
| `MOTOR_STBY` | 5V | 常使能，不占用 MCU IO |

STBY 当前直接接 5V，软件不能依赖 STBY 关断电机，只能通过 PWM=0 和方向脚安全状态停车。

### 编码器

| 逻辑名 | 目标连接 | 当前代码映射 |
| --- | --- | --- |
| `ENC_L_A` | 左电机 M1 A 相 | PA26，A 路中断 |
| `ENC_L_B` | 左电机 M1 B 相 | PA27，B 路方向采样 |
| `ENC_R_A` | 右电机 M2 A 相 | PA25，A 路中断 |
| `ENC_R_B` | 右电机 M2 B 相 | PA14，B 路方向采样 |

当前默认编码器计数方式：
- A 相 GPIO 中断；
- 默认只统计 A 相上升沿；
- B 相只用于读取电平判断方向；
- `LEFT_ENCODER_DIR` / `RIGHT_ENCODER_DIR` 用于修正左右轮方向。

`ECAR_ENCODER_PULSE_PER_REV = 367.0f` 是在当前代码计数方式，也就是 A 相上升沿计数 + B 相判断方向的条件下，实测得到的轮端每圈脉冲数。若以后改成 A 相双边沿或 AB 四倍频，必须重新实测该值，否则速度和距离都会算错。

### 八路灰度模块

| 逻辑名 | 引脚 | 说明 |
| --- | --- | --- |
| `GRAY_AD2` | PB23 | 多路选择地址位 |
| `GRAY_AD1` | PB10 | 多路选择地址位 |
| `GRAY_AD0` | PB13 | 多路选择地址位 |
| `GRAY_OUT` | PB01 | 灰度输出输入 |

灰度模块当前按 5V 供电记录。`GRAY_OUT` 进入 MSPM0 PB01 前，硬件必须已经通过分压或电平转换降到 3.3V 安全范围。软件无法把 5V GPIO 输入变安全，不能用代码掩盖硬件风险。`GRAY_OUT` 配置为无内部上下拉输入，由模块输出电平决定。`GRAY_AD0/AD1/AD2` 是 MSPM0 普通 3.3V 输出，如果 5V 模块不能可靠识别 3.3V 高电平，需要硬件增加电平转换。

### OLED、I2C、蜂鸣器、LED、按键、舵机、UART

| 外设 | 映射 |
| --- | --- |
| OLED_H8_I2C | H8 1x8 排母前 4 针，GND / 3V3 / SCL(SKC) PB9 / SDA PB8，ECAR_OLED_ENABLE 默认关闭；启用后默认使用 H8 I2C 接法 |
| I2C0 | PA31 SCL / PA28 SDA，MPU6050 六轴传感器 |
| OLED_H8_SPI | 可选 H8 1x8 排母 SPI 模式，SCL PB9 / SDA PB8 / RES PB10 / DC PB11 / CS PB14，默认关闭 |
| BEEP | A07 |
| LED_USER | B04，若串联电阻为 10k，亮度可能偏低 |
| KEY1/SW1 | B14，输入上拉，按下为 0 |
| KEY2/SW2 | B11，输入上拉，按下为 0 |
| KEY3/SW3 | B27，输入上拉，按下为 0 |
| KEY4/SW4 | B26，输入上拉，按下为 0 |
| SERVO1_PWM | PA21 / TIMA0-C0 |
| SERVO2_PWM | PA22 / TIMA0-C1 |
| SERVO3_PWM | PA15 / TIMA0-C2 |
| SERVO4_PWM | PA17 / TIMA0-C3 |
| UART_DEBUG | UART1，TX1/PB6 -> UART1_TX，RX1/PB7 -> UART1_RX，9600，8N1，无校验，无流控 |
| UART0 | 当前不作为测试/调参串口 |
| UART2/UART3 | 备用 |

HC-04 接线：

```text
HC-04 RX -> TX1 / PB6
HC-04 TX -> RX1 / PB7
HC-04 GND -> MSPM0 GND
HC-04 VCC -> 按模块要求供电
```

当前使用 4 针 IIC OLED，直接插在 H8 排母前 4 针时，代码默认 `BOARD_OLED_USE_H8_I2C = 1`，通过 PB9/PB8 软件 I2C 驱动，因此灰度 `GRAY_AD1` 和 `KEY1/KEY2` 正常可用。若后续改用天猛星 H8 SPI OLED，`PB10` 会作为 OLED 复位脚并和 `GRAY_AD1` 冲突，`PB11/PB14` 会作为 OLED DC/CS 并和 `KEY2/KEY1` 冲突。

舵机电源应使用独立大电流供电，并与主控共地。舵机测试默认关闭，需要显式打开宏。

SWDIO/SWCLK 不要配置为普通 GPIO。建议后续硬件把 RST 补到 SWD 接口。

## 后置循迹模块 / 反向车头模式

当前支持通过 `ECAR_REAR_LINE_SENSOR_MODE` 在 `Hardware/Board_Config.h` 中切换机械布局：

```c
#define ECAR_REAR_LINE_SENSOR_MODE 0U   // 原始前置循迹模块布局
#define ECAR_REAR_LINE_SENSOR_MODE 1U   // 后置循迹模块布局，新车头方向相对原来反向
```

后置模式 (`ECAR_REAR_LINE_SENSOR_MODE = 1`) 下：

- 原右轮映射为逻辑左轮，原左轮映射为逻辑右轮；
- 编码器左右互换；
- 电机方向和编码器方向符号同步调整；
- 循迹修正方向 `g_lineTurnSign` 和角点固定转向 `E_CAR_TURN_SIGN` 反号；
- 灰度逻辑顺序 `g_lineReverseOrderF` 置 1 以匹配新车头视角。

首次切换或重新布线后，必须架空车轮验证：

1. `Motor_SetPWM(+150, +150)` 是否驱动小车向新车头方向前进；
2. 手推小车向新车头方向时，`g_leftEncoderDelta` 和 `g_rightEncoderDelta` 是否都为正；
3. 将黑胶带置于灰度模块下方，从新车头方向看，黑线偏左时 `g_lineError` 应为负。

实测结果：
- 首次上电时正向 PWM 使左右轮均向后转；
- 因此后置模式下 `LEFT_MOTOR_DIR / RIGHT_MOTOR_DIR` 已整体取反；
- 验证目标：`Motor_SetPWM(+150, +150)` 驱动小车朝新的前进方向运动。

## 板级测试模式

当前 main 分支默认工作在**正式循迹构建模式**，但不会上电自动跑车：

```c
#define ECAR_BOARD_TEST_MODE       0
#define ECAR_TEST_MOTOR_ENABLE     0
#define ECAR_TEST_IMU_ENABLE       0
```

含义：
- 不进入 board test 专用流程；
- 电机不会自动启动；
- `g_carEnable` 默认仍为 0；
- `main` 不应主动调用 `ECar_Start()`；
- 需要通过按键或明确的启动命令进入运行。

如需板级 IMU 测试，请手动改为：

```c
#define ECAR_BOARD_TEST_MODE       1
#define ECAR_TEST_IMU_ENABLE       1
#define ECAR_TEST_MOTOR_ENABLE     0
```

板级测试模式不进入完整 E 题状态机，不自动启动电机。UART_DEBUG（UART1，TX1/RX1，9600）周期输出类似：

```text
[boot] mspm0 e-car board test
[imu] ok addr=0x68 who=0x68 gz_raw=-12 gz_dps=-0.9 yaw=0.0 healthy=1
```

电机低占空比架空测试需要先把 `ECAR_TEST_MOTOR_ENABLE` 改为 1，并通过串口发送 `[test,arm]` 后才允许 `[test,pwm,left,right]`。默认宏值下，即使收到电机测试命令也会拒绝并保持停车。

## 当前实现状态

已实现或适配：

- 电机 PWM 与方向控制，默认 `Motor_StopAll()`
- 编码器 A 路中断计数、B 路方向判断
- CD4051 灰度 8 路轮询和加权误差
- KEY1~KEY4 输入上拉、软件消抖、短按事件
- UART_DEBUG 调试输出与串口命令解析，当前实际使用 UART1 PB6/PB7，9600 8N1
- A07 蜂鸣器、B04 用户 LED
- TIMA0 四路舵机基础 PWM 接口
- Board test 周期状态输出

保留 stub 或最小接口：

- OLED：默认使用 H8 排母前 4 针的 4 针 IIC SSD1306 驱动，SCL/SKC=PB9、SDA=PB8，初始化自动尝试地址 0x3C/0x3D，board test 默认启用
- MPU6050：使用 GPIO 软件 IIC 驱动（PA28=SDA / PA31=SCL），不依赖 DL_I2C 硬件外设。已支持 WHO_AM_I、陀螺仪原始读取、Z 轴零偏校准、yaw 积分。board test 模式下默认启用 IMU 测试。
- UART2/UART3：用途预留，当前未初始化为独立通信驱动

## SysConfig 复核项

`empty.syscfg` 已同步新版 IO，但当前工程以本地 `ti_msp_dl_config.c/.h` 为准。重新打开 SysConfig 或重新生成代码前，请重点复核：

1. TIMG0-C0/C1 是否仍为电机 PWM。
2. TIMA0-C0~C3 是否对应实际 H7/H15/H8/H14 舵机接口。
3. 编码器 PA26/PA27/PA25/PA14 是否和实际接口接线一致。
4. UART_DEBUG 是否为 UART1：TX1/PB6、RX1/PB7，波特率 9600，8N1。
5. 4 针 OLED 是否插在 H8 前 4 针：GND、3V3、SCL/SKC -> PB9、SDA -> PB8。
6. 灰度 `GRAY_OUT` 到 PB01 前是否已经硬件限压到 3.3V。
7. 若改用 H8 SPI OLED，灰度 `GRAY_AD1` 和 `KEY1/KEY2` 是否已经避开 PB10/PB14/PB11。

## 上车前安全检查

1. 保持 `ECAR_TEST_MOTOR_ENABLE = 0`，先验证 UART、按键、LED、蜂鸣器、灰度和编码器。
2. 架空车轮后再打开电机测试宏，并使用低占空比短时间测试。
3. 确认 `g_carEnable` 默认是 0，目标速度默认是 0，main 不主动调用 `ECar_Start()`。
4. 确认 TB6612 STBY 虽然常使能，但 PWM 初值为 0、方向脚为安全状态。
5. 舵机测试前确认独立供电和共地，避免从主控板取大电流。
6. 灰度 5V 供电时绝不能把 5V OUT 直连 MSPM0 GPIO。
