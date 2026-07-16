# mspm0_car_base

MSPM0G3507 小车底层驱动模板工程。保留 MSPM0G3507 小车底层驱动、IO 映射、速度闭环、灰度读取、串口、按键、OLED、IMU、舵机、板级测试能力。

**本工程不包含 E 题/F 题业务状态机。** 开发新题目时，在 `App/` 层新建业务状态机，不要修改底层驱动。

## 软件结构

| 目录 | 说明 |
| --- | --- |
| `User/` | 主入口与主循环 |
| `App/` | app_control(速度闭环)、app_line(循迹)、app_board_test(板测)、app_car_base(模板入口)、app_car_state(全局运行变量) |
| `Control/` | PID |
| `Hardware/` | MSPM0G3507 外设驱动封装 |
| `System/` | 系统节拍、定时器、调度 |
| `keil/` | Keil MDK 工程文件 |

板级逻辑名集中在 `Hardware/Board_Config.h`。

## 安全默认状态

- **上电默认电机不转**：PWM 默认 0，方向脚安全状态。
- **`g_carEnable` 默认 0**：不使能电机输出。
- **不自动启动任何比赛任务**：main 入口使用 `CarBase_Task10ms()` 空任务，不会自动进入任何竞赛状态机。

## IO 映射

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

STBY 直接接 5V，软件只能通过 PWM=0 和方向脚安全状态停车。

### 编码器

| 逻辑名 | 引脚 | 说明 |
| --- | --- | --- |
| `ENC_L_A` | PA26 | 左电机 A 相中断 |
| `ENC_L_B` | PA27 | 左电机 B 相方向采样 |
| `ENC_R_A` | PA25 | 右电机 A 相中断 |
| `ENC_R_B` | PA14 | 右电机 B 相方向采样 |

编码器计数方式：A 相上升沿计数 + B 相电平判断方向。`LEFT_ENCODER_DIR` / `RIGHT_ENCODER_DIR` 修正左右轮方向。

### 八路灰度（CD4051 轮询）

| 逻辑名 | 引脚 | 说明 |
| --- | --- | --- |
| `GRAY_AD2` | PB23 | 地址位 |
| `GRAY_AD1` | PB10 | 地址位 |
| `GRAY_AD0` | PB13 | 地址位 |
| `GRAY_OUT` | PB01 | 灰度输出（需硬件 5V->3.3V 电平转换） |

### 其他外设

| 外设 | 映射 |
| --- | --- |
| OLED (I2C, H8) | SCL=PB9, SDA=PB8, 4针 IIC SSD1306 |
| I2C0 (MPU6050) | PA31 SCL / PA28 SDA |
| BEEP | A07 |
| LED_USER | B04 |
| KEY1~KEY4 | B14 / B11 / B27 / B26, 输入上拉 |
| SERVO1~4 | PA21 / PA22 / PA15 / PA17 (TIMA0-C0~C3) |
| UART_DEBUG | UART1, TX1=PB6, RX1=PB7, 9600 8N1 |
| UART2/UART3 | 备用 |

**电机方向和编码器方向保持原样，本次未做修改。**

## 板级测试模式

```c
#define ECAR_BOARD_TEST_MODE       0
#define ECAR_TEST_MOTOR_ENABLE     0
#define ECAR_TEST_IMU_ENABLE       0
```

- 默认不进入 board test。
- 电机不会自动启动。
- 如需板级 IMU 测试，改为 `ECAR_BOARD_TEST_MODE = 1`, `ECAR_TEST_IMU_ENABLE = 1`。

## CarBase 模板入口

- `CarBase_Init()`：安全初始化，清零 PWM，停止电机。
- `CarBase_Task10ms()`：空任务（仅 `App_Control_ForcePWMZero()`），保证上电电机不转。
- `CarBase_Task100ms()`：串口输出 base 状态（编码器、灰度）。
- `CarBase_Task200ms()`：OLED 占位显示。
- `CarBase_KeyProcess()`：空实现。

## 开发新题目指南

1. 在 `App/` 层新建业务状态机文件（如 `app_my_contest.c/h`）。
2. 在 `app_car_base.h` 中声明新接口，或创建独立的 entry 文件。
3. 将新文件加入 Keil 工程 App 组。
4. 修改 `main.c`，将 `CarBase_Task10ms()` 替换为新的业务入口。
5. **不修改底层驱动。**

## SysConfig 复核项

重新打开 SysConfig 或重新生成代码前，请复核：
1. TIMG0-C0/C1 为电机 PWM。
2. TIMA0-C0~C3 为舵机接口。
3. 编码器 PA26/PA27/PA25/PA14 与实际接线一致。
4. UART_DEBUG = UART1 PB6/PB7，9600 8N1。
5. 灰度 GRAY_OUT 到 PB01 前已硬件限压到 3.3V。

## 上车前安全检查

1. 架空车轮后再打开电机测试宏。
2. 确认 `g_carEnable` 默认 0，目标速度默认 0。
3. 舵机测试前确认独立供电和共地。
4. 灰度 5V 供电时需电平转换。
