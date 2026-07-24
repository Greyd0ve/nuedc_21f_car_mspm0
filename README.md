# mspm0_car_base

MSPM0G3507 小车底层驱动模板工程。保留 MSPM0G3507 小车底层驱动、IO 映射、速度闭环、灰度读取、串口、按键、OLED、IMU、舵机、板级测试能力。

**本工程不包含 E 题/F 题业务状态机。** 开发新题目时，在 `App/` 层新建业务状态机，不要修改底层驱动。

---

## 机械参数

### 通用底盘参数

| 参数 | 符号 | 值 | 说明 |
| --- | --- | --- | --- |
| 驱动轮直径 | `ECAR_WHEEL_DIAMETER_CM` | **6.5 cm** | 65mm 橡胶轮 |
| 驱动轮周长 | `ECAR_WHEEL_CIRCUMFERENCE_CM` | **≈ 20.42 cm** | π × 6.5 |
| 左右轮中心距 (轮距) | `ECAR_TRACK_WIDTH_CM` | **15.9 cm** | 159mm |
| 驱动轴到灰度传感器 | `ECAR_LINE_SENSOR_AXLE_OFFSET_CM` | **17.7 cm** | 177mm，传感器在驱动轴前方 |
| 编码器每厘米计数 | `ECAR_CM_PER_PULSE` | 见下方各模式 | 取决于编码器类型和计数方式 |

### 从车步进电机模式 (D36A)

| 参数 | 符号 | 值 | 说明 |
| --- | --- | --- | --- |
| STEP 脉冲/圈 | `STEPPER_STEP_PER_REV` | **3200** | D36A 细分设定 |
| 编码器计数/圈 | `ENCODER_COUNT_PER_REV` | **4096** | AB 相 4× 正交实测值 |
| STEP 与编码器比值 | — | **1.28** | 4096 / 3200 |
| 编码器每厘米计数 | `ECAR_CM_PER_PULSE` | **≈ 200.59** | 4096 / 20.42 |
| 每计数对应距离 | — | **≈ 0.004985 cm** | 1 / 200.59 |
| 软件满转速 | `STEPPER_FULL_SPEED_RPM` | **600 RPM** | — |
| 满转速 STEP 频率 | `STEPPER_FULL_STEP_FREQ_HZ` | **32000 Hz** | 600 × 3200 / 60 |
| 加速率 | `STEPPER_ACCEL_HZ_PER_TICK` | **20 Hz/ms** | — |
| 减速率 | `STEPPER_DECEL_HZ_PER_TICK` | **30 Hz/ms** | — |
| 最低有效频率 | `STEPPER_MIN_TIMER_FREQ_HZ` | **500 Hz** | 16-bit 定时器周期上限 |

**原地转弯理论值 (步进模式):**

| 转角 | 编码器计数 | STEP 脉冲 | 说明 |
| --- | --- | --- | --- |
| 90° | **2505** | 1957 | `enc = track/2 × (π/2) / circum × 4096` |
| 180° | **5010** | 3914 | 2 × 90° 值 |

公式：`enc = (15.9/2) × θ_rad / 20.42 × 4096`，`step = enc × 3200 / 4096`

### 主车减速电机模式 (TB6612 + JGA25-370B)

| 参数 | 符号 | 值 | 说明 |
| --- | --- | --- | --- |
| 编码器计数/圈 | `ECAR_ENCODER_PULSE_PER_REV` | **367** | A 相上升沿计数实测值 |
| 编码器每厘米计数 | `ECAR_CM_PER_PULSE` | **≈ 18.01** | 367 / 20.42 |
| 每计数对应距离 | — | **≈ 0.0556 cm** | 1 / 18.01 |
| PWM 满量程 | `PWM_MAX_DUTY` | **1000** | 逻辑占空比标尺 |
| PWM 周期 | `MOTOR_PWM_PERIOD_COUNTS` | **1600** | TIMG0, 32MHz → 20kHz |

**原地转弯理论值 (直流电机模式):**

采用 A 相上升沿计数 (367 计数/圈):

| 转角 | 编码器计数 | 说明 |
| --- | --- | --- |
| 90° | **≈ 225** | enc = (15.9/2) × (π/2) / (6.5×π) × 367 |
| 180° | **≈ 449** | 2 × 90° 值 |

---

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

### D36A 步进电机驱动（从车）

| 逻辑名 | 硬件连接 | 说明 |
| --- | --- | --- |
| `STEP_L` | PB15 / TIMG7 CC0 | 左通道 STEP，硬件 50% 占空比脉冲 |
| `DIR_L` | PB18 | 左通道方向，高电平=物理右轮前进 |
| `STEP_R` | PB16 / TIMG8 CC1 | 右通道 STEP，硬件 50% 占空比脉冲 |
| `DIR_R` | PB25 | 右通道方向，低电平=物理左轮前进 |
| EN1/EN2 | 硬件常态使能 | 不占用 MCU IO |

> **通道交叉映射**：D36A 接线中 STEP_L/DIR_L 实际驱动物理右轮，STEP_R/DIR_R 实际驱动物理左轮。方向宏 `LEFT_STEPPER_DIR_SIGN = +1`, `RIGHT_STEPPER_DIR_SIGN = -1`。

### TB6612 电机驱动（主车/旧版）

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

**步进模式（从车，4× 正交解码，GPIOB）：**

| 逻辑名 | 引脚 | 说明 |
| --- | --- | --- |
| `ENC_L_A` | PB05 | 左编码器 A 相，双边沿中断 |
| `ENC_L_B` | PB12 | 左编码器 B 相，双边沿中断 |
| `ENC_R_A` | PB08 | 右编码器 A 相，双边沿中断 |
| `ENC_R_B` | PB00 | 右编码器 B 相，双边沿中断 |

> 16 项查表法 4× 正交解码，4096 计数/圈。非法跳变计数支持丢步检测。

**直流电机模式（主车，A 相上升沿计数，GPIOA）：**

| 逻辑名 | 引脚 | 说明 |
| --- | --- | --- |
| `ENC_L_A` | PA26 | 左电机 A 相中断 |
| `ENC_L_B` | PA27 | 左电机 B 相方向采样 |
| `ENC_R_A` | PA25 | 右电机 A 相中断 |
| `ENC_R_B` | PA14 | 右电机 B 相方向采样 |

> A 相上升沿计数 + B 相电平判断方向，367 计数/圈。

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

## 2021 F 题基础循迹框架

当前在 `App/app_21f_car.c` 中实现了 2021 电赛 F 题"智能送药小车"基础去程循迹框架。

### 按键功能

| 按键 | 功能 |
| --- | --- |
| K1 | 选择目标病房号 1~8 循环，LED 三位二进制提示房号 |
| K2 | 启动送药任务 |
| K3 | 紧急停车 |
| K4 | 复位到初始状态 |

### 病房路线

- **1~4 号**：单路口单转弯（SIMPLE 路线），1/2号入口就近路口，3/4号远端路口。
- **5~8 号**：双路口双转弯（FAR 路线），主线 240cm 后第一次路口，转弯后 70cm 第二次路口。
- **当前返回逻辑未实现**，到达病房后等待 3 秒进入 FINISH。

### 重要参数

- `F21_TURN_90_PULSE = 0`：**需要实测后填写**。当前为 0 时，转弯状态会安全停车并串口输出 `[f21,fault,turn90_not_set]`。
- `F21_LINE_BASE_SPEED_CMPS = 10.0`：直线循迹基础速度。
- `F21_TURN_PWM = 140`：转弯 PWM。

### LED 房号提示

非阻塞三位二进制显示：500ms 亮 = 1，100ms 亮 = 0，位间 100ms 灭。

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
