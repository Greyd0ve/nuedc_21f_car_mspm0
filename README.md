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

当前第一版使用 A 路 GPIO 双边沿中断计数、B 路读取电平判断方向，并保留 `LEFT_ENCODER_DIR` / `RIGHT_ENCODER_DIR` 方向系数。左右 A 相都在 GPIOA 中断组内，ISR 会分别检查 PA26 和 PA25。

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
| OLED_H8_SPI | H8 1x8 排母，SCL PB9 / SDA PB8 / RES PB10 / DC PB11 / CS PB14 |
| I2C0 | PA1 SCL / PA0 SDA，保留给 MPU6050 或外接 4-pin IIC OLED |
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
| UART0 | USB 调试 / printf / board test 日志，当前 115200 |
| UART1 | 预留 K230 / 瞄准模块 |
| UART2/UART3 | 备用 |

H8 的 SPI OLED 接口是天猛星开发板固定连线。当前启用 H8 OLED 时，`PB10` 会作为 OLED 复位脚，和现有 `GRAY_AD1` 冲突；`PB11/PB14` 会作为 OLED DC/CS，和 `KEY2/KEY1` 冲突。因此当前 board test 会跳过灰度读取，并只保留 `KEY3/KEY4` 检测。后续上车前需要重新分配灰度 AD1 和按键 IO，或者改回 PA1/PA0 的 IIC OLED。

舵机电源应使用独立大电流供电，并与主控共地。舵机测试默认关闭，需要显式打开宏。

SWDIO/SWCLK 不要配置为普通 GPIO。建议后续硬件把 RST 补到 SWD 接口。

## 板级测试模式

默认 `App/app_config.h` 中：

```c
#define ECAR_BOARD_TEST_MODE 1
#define ECAR_TEST_MOTOR_ENABLE 0
#define ECAR_TEST_SERVO_ENABLE 0
#define ECAR_TEST_BEEP_ENABLE 1
#define ECAR_TEST_OLED_ENABLE 1
#define ECAR_TEST_IMU_ENABLE 0
```

板级测试模式不进入完整 E 题状态机，不自动启动电机。UART0 周期输出类似：

```text
[boot] mspm0 e-car board test
[key] k1=1 k2=1 k3=1 k4=1
[gray] raw=1,0,0,1,1,0,0,1 err=-2
[enc] left=123 right=120
[pwm] disabled
[servo] disabled
[imu] stub
[oled] h8-spi enabled
```

电机低占空比架空测试需要先把 `ECAR_TEST_MOTOR_ENABLE` 改为 1，并通过串口发送 `[test,arm]` 后才允许 `[test,pwm,left,right]`。默认宏值下，即使收到电机测试命令也会拒绝并保持停车。

## 当前实现状态

已实现或适配：

- 电机 PWM 与方向控制，默认 `Motor_StopAll()`
- 编码器 A 路中断计数、B 路方向判断
- CD4051 灰度 8 路轮询和加权误差
- 按键输入上拉、软件消抖、短按事件；H8 OLED 模式下当前只保留 KEY3/KEY4
- UART0 调试输出与串口命令解析
- A07 蜂鸣器、B04 用户 LED
- TIMA0 四路舵机基础 PWM 接口
- Board test 周期状态输出

保留 stub 或最小接口：

- OLED：默认使用天猛星 H8 1x8 排母的软件 SPI SSD1306 驱动，board test 默认启用
- MPU6050：保留 WHO_AM_I 最小读取接口，默认 board test 不启用
- UART1/UART2/UART3：用途预留，当前未初始化为独立通信驱动

## SysConfig 复核项

`empty.syscfg` 已同步新版 IO，但当前工程以本地 `ti_msp_dl_config.c/.h` 为准。重新打开 SysConfig 或重新生成代码前，请重点复核：

1. TIMG0-C0/C1 是否仍为电机 PWM。
2. TIMA0-C0~C3 是否对应实际 H7/H15/H8/H14 舵机接口。
3. 编码器 PA26/PA27/PA25/PA14 是否和实际接口接线一致。
4. UART0 是否为 USB 调试口，波特率 115200。
5. H8 OLED 是否为 SPI SSD1306 单色屏，且插入方向符合 H8 默认 1=GND、2=3V3。
6. 灰度 `GRAY_OUT` 到 PB01 前是否已经硬件限压到 3.3V。
7. 若启用 H8 OLED，灰度 `GRAY_AD1` 和 `KEY1/KEY2` 是否已经避开 PB10/PB14/PB11。

## 上车前安全检查

1. 保持 `ECAR_TEST_MOTOR_ENABLE = 0`，先验证 UART、按键、LED、蜂鸣器、灰度和编码器。
2. 架空车轮后再打开电机测试宏，并使用低占空比短时间测试。
3. 确认 `g_carEnable` 默认是 0，目标速度默认是 0，main 不主动调用 `ECar_Start()`。
4. 确认 TB6612 STBY 虽然常使能，但 PWM 初值为 0、方向脚为安全状态。
5. 舵机测试前确认独立供电和共地，避免从主控板取大电流。
6. 灰度 5V 供电时绝不能把 5V OUT 直连 MSPM0 GPIO。
