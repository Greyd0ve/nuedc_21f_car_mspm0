# AGENTS.md

## 操作约束

禁止批量删除文件或目录。

不要使用：

- `del /s`
- `rd /s`
- `rmdir /s`
- `Remove-Item -Recurse`
- `rm -rf`

需要删除文件时，只能一次删除一个明确路径的文件。

正确示例：

```powershell
Remove-Item "C:\path\to\file.txt"
```

如果需要批量删除文件，应停止操作，并询问用户，让用户手动删除。

## 项目背景

本项目用于 2025/2026 电赛 E 题：简易自行瞄准装置。

系统分为两部分：

- 自动寻迹小车
- 瞄准模块 / 云台视觉模块

赛题要求小车的寻迹和电机控制必须使用 TI MSPM0 系列 MCU 完成。当前选择 MSPM0G3507 作为小车底盘主控，主要负责小车运动控制；视觉、靶心识别、激光点识别、二维云台控制后续可交给 K230 / OpenMV / CanMV 等模块完成。

## 主控硬件环境

```text
主控芯片：TI MSPM0G3507
开发板：立创·天猛星 MSPM0G3507 核心开发板
用途：电赛 E 题自动寻迹小车底盘控制
```

MSPM0G3507 侧主要负责：

```text
灰度循迹
TB6612FNG 电机驱动
编码器测速
左右轮速度闭环
循迹 PD / PID
计圈停车
OLED 显示
按键交互
蜂鸣器 / LED 状态提示
UART 与瞄准模块通信
```

## 软件开发环境

```text
IDE：Keil MDK
Keil 版本：V5.39.0.0
编译器：ArmClang V6.21
MSPM0 SDK：mspm0_sdk_2_02_00_05
SysConfig：1.21.1
调试器：J-Link / LINK-OB
底层库：MSPM0 DriverLib
工程配置方式：Keil + SysConfig + DriverLib
```

本机实际路径：

```text
Keil 安装目录：D:\Keil_v5_38a
Keil uVision：D:\Keil_v5_38a\UV4\UV4.exe
ArmClang：D:\Keil_v5_38a\ARM\ARMCLANG\bin\armclang.exe
armlink：D:\Keil_v5_38a\ARM\ARMCLANG\bin\armlink.exe
fromelf：D:\Keil_v5_38a\ARM\ARMCLANG\bin\fromelf.exe
MSPM0 SDK：D:\ti2\mspm0_sdk_2_02_00_05
SysConfig：C:\ti\sysconfig_1.21.1
SysConfig CLI：C:\ti\sysconfig_1.21.1\sysconfig_cli.bat
DriverLib Keil 库：D:\ti2\mspm0_sdk_2_02_00_05\source\ti\driverlib\lib\keil\m0p\mspm0g1x0x_g3x0x\driverlib.a
CMSIS include：D:\ti2\mspm0_sdk_2_02_00_05\source\third_party\CMSIS\Core\Include
```

已核对：

```text
armclang --version：MDK-ARM Lite 5.39 / Arm Compiler for Embedded 6.21
SysConfig CLI --version：1.21.1+3772
Keil 工程 pCCUsed：6210000::V6.21::ARMCLANG
```

开发逻辑：

```text
SysConfig：配置引脚、定时器、PWM、UART、I2C、GPIO 等外设
DriverLib：提供 MSPM0 底层外设驱动接口
Keil MDK：负责编译、下载、调试
J-Link / LINK-OB：负责 SWD 下载和在线调试
```

当前不是裸寄存器从零写，而是基于 TI 官方 SDK + SysConfig 自动生成配置 + DriverLib 调外设的方式开发。

Keil 工程注意事项：

```text
工程使用 keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx。
SysConfig 文件为 empty.syscfg。
ti_msp_dl_config.c / ti_msp_dl_config.h 是 SysConfig 生成文件，不写业务逻辑。
新增 .c 文件后必须加入 Keil 工程，否则 Keil 不会编译。
UART0 保留为调试串口。
修改 IO 时必须同步检查原理图、SysConfig、代码宏定义、PCB 丝印和实际接线。
上电默认电机不应自动启动，PWM 默认应为 0。
```

## 当前工程对应硬件外设

```text
电机驱动：TB6612FNG
驱动电机：2 个 JGA25-370B 12V 编码减速电机
循迹模块：8 路灰度模块，CD4051 轮询结构
显示模块：优先使用天猛星 H8 1x8 排母上的 SPI SSD1306 OLED
姿态模块：MPU6050，I2C 接口
交互输入：4 个按键
状态输出：蜂鸣器、自用 LED
通信接口：UART0 调试，UART1 预留给 K230 / 瞄准模块
供电：12V 锂电池输入，降压得到 5V 和 3.3V
```

关键外设分配：

```text
UART0：建议用于 printf、日志输出、调试
UART1：建议用于和 K230 / 瞄准模块通信
H8 OLED：SCL PB9 / SDA PB8 / RES PB10 / DC PB11 / CS PB14
I2C0：PA1 SCL / PA0 SDA，保留给 MPU6050 或外接 4-pin IIC OLED
TIMG0：左右电机 PWM
TIMA0：预留 4 路舵机 PWM
TIMG8 / TIMG12：编码器输入相关
```

## 推荐软件工程结构

```text
User/       主入口 main 和主循环
App/        E 题状态机、循迹逻辑、串口命令
Control/    PID、滤波、速度闭环、循迹控制
Hardware/   MSPM0G3507 外设驱动封装
System/     定时器、系统节拍、任务调度
keil/       Keil 工程文件
```

推荐模块划分：

```text
Motor       TB6612 电机驱动
Encoder     编码器计数与测速
Grayscale   8 路灰度轮询
Key         按键扫描
Serial      UART 通信
OLED        OLED 显示
IMU         MPU6050 姿态读取
BeepLed     蜂鸣器和 LED
Servo       舵机 PWM 输出
Timer       系统节拍与调度
```

## 开发注意事项

### SysConfig 与 IO 保持一致

当前最新版 IO 涉及：

```text
TIMG0-C0 / TIMG0-C1：电机 PWM
TIMA0-C0 ~ TIMA0-C3：舵机 PWM
TIMG8 / TIMG12：编码器输入
H8 OLED：PB9 / PB8 / PB10 / PB11 / PB14
I2C0：PA1 / PA0，MPU6050 或备用 IIC OLED
UART0~UART3：调试和通信
PB23 / PB10 / PB13 / PB01：灰度模块
B14 / B11 / B27 / B26：按键
A07：蜂鸣器
B04：LED
```

每次修改原理图或 IO，都要同步检查：

```text
原理图 IO
SysConfig 配置
代码宏定义
PCB 丝印
实际接线
```

这些位置只要有一个不一致，就可能出现“代码没问题但硬件不动”的情况。

### H8 OLED IO 冲突

天猛星 H8 1x8 排母用于 SPI SSD1306 OLED 时，固定占用：

```text
H8-1 GND
H8-2 3V3
H8-3 SCL -> PB9
H8-4 SDA -> PB8
H8-5 RES -> PB10
H8-6 DC  -> PB11
H8-7 CS  -> PB14
```

当前旧 IO 中 PB10 同时是 `GRAY_AD1`，PB11/PB14 同时是 `KEY2/KEY1`。启用 H8 OLED 时，灰度 AD1 和 KEY1/KEY2 必须重新分配或暂时停用，否则 OLED、灰度和按键会互相抢 IO。

### UART0 保留调试

合理分配：

```text
UART0：电脑串口调试 / printf / 日志
UART1：K230 / 瞄准模块通信
UART2：蓝牙调参 / 备用
UART3：备用
```

不建议把 UART0 长期让给 K230，因为会失去最方便的调试输出通道。

### 5V 外设信号风险

MSPM0G3507 的 GPIO 不能随便承受 5V 输入。

需要重点确认：

```text
灰度模块 OUT
OLED / MPU6050 的 I2C 上拉
外部串口模块
舵机信号与供电
```

如果灰度模块使用 5V 供电，OUT 可能输出 5V，需要确认模块输出电平，或改 3.3V 供电 / 加分压 / 加电平转换。

## 项目一句话总结

当前开发环境是：以立创·天猛星 MSPM0G3507 为主控，使用 Keil MDK V5.39 + ArmClang V6.21 + MSPM0 SDK 2.02.00.05 + SysConfig 1.21.1 + DriverLib 进行开发，通过 J-Link / LINK-OB 下载调试，目标是完成电赛 E 题自动寻迹小车底盘控制部分。
