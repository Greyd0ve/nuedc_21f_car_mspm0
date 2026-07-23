# VISION TRACK STAGE 1 REPORT

**Date:** 2026-07-23
**Target:** firmware_withK230/master_car only

---

## 1. 修改文件列表

| 文件 | 操作 | 说明 |
|------|------|------|
| `App/app_vision_link.h` | 新建 | K230 视觉帧解析模块头文件 |
| `App/app_vision_link.c` | 新建 | K230 视觉帧解析模块实现 |
| `App/app_vision_track.h` | 新建 | 视觉循迹状态机头文件 |
| `App/app_vision_track.c` | 新建 | 第一阶状态枚举 + 视觉控制实现 |
| `App/app_config.h` | 修改 | 新增 `F21_VISION_TRACK_TEST_MODE` 宏 (设为 1) |
| `User/main.c` | 修改 | 新增 `#elif F21_VISION_TRACK_TEST_MODE` 分支 |
| `keil/...uvprojx` | 修改 | 将两个 .c 文件加入 App 分组 |

**未修改:** Serial.c/h, SysConfig, ti_msp_dl_config.c/h, IO 映射, 电机方向, 其他三个工程。

---

## 2. 新增模块职责

- **app_vision_link**：从 UART0 环形缓冲区读取 K230 发送的 `[trk1,...]` 帧，解析并校验字段，存储最新有效帧，统计有效帧计数/解析错误计数，提供 `[mode,track]`/`[mode,idle]` 发送接口。

- **app_vision_track**：第一阶段视觉循迹状态机 (IDLE→ACQUIRE→RUN→LOST→STOP)，按键交互，视觉有效性检查，横向+航向偏差控制计算。

---

## 3. 最终通信协议

K230 → MSPM0: `[trk1,seq,ey,ea,width,flags,jdist,conf]`

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| trk1 | 标识 | — | 固定协议标识 |
| seq | uint16 | 0~65535 | 帧序号 |
| ey | int16 | -1000~1000 | 横向偏差 (Q1000) |
| ea | int16 | -1800~1800 | 航向偏差 (0.1°) |
| width | uint16 | 0~5000 | 道路宽度 (Q1000) |
| flags | uint8 | 0~255 | 边界/支路/故障标志 |
| jdist | uint16 | 0~65535 | 路口距离 (mm) |
| conf | uint8 | 0~100 | 视觉可信度 |

MSPM0 → K230: `[mode,track]` / `[mode,idle]`

---

## 4. 第一阶段状态机

```
IDLE →(K2)→ ACQUIRE →(3 valid frames)→ RUN
RUN  →(data invalid)→ LOST →(3 valid frames)→ RUN
[any]→(K3)→ STOP →(K4)→ IDLE

在 IDLE 状态电机停止；STOP 时发送 `[mode,idle]`；ACQUIRE 时发送 `[mode,track]` 并等待 3 个有效帧后自动进入 RUN；LOST 立即停车但自动重捕获。
```

---

## 5. 按键功能

| 按键 | 功能 |
|------|------|
| K1 | 无功能 |
| K2 | 启动视觉循迹 (发送 [mode,track], 进入 ACQUIRE) |
| K3 | 紧急停车 (发送 [mode,idle], 进入 STOP) |
| K4 | 复位 (回到 IDLE, 清除所有缓存和历史) |

---

## 6. 视觉有效性规则

有效视觉帧必须同时满足:
- frameValid = 1
- 帧龄 ≤ 150ms
- flags 不含 0x80 (视觉故障)
- flags 同时包含 0x01 + 0x02 (左右边界同时有效)
- confidence ≥ 70
- ey 和 ea 在合法范围

任一不满足立即视为无效 → 进入 LOST 停车。

---

## 7. 控制公式和符号约定

```
turnCmd = VISION_TRACK_SIGN * (K_Y * ey + K_A * ea + K_D * dEy)
VISION_TRACK_SIGN = -1.0f

leftTarget  = forwardSpeed - turnSpeed
rightTarget = forwardSpeed + turnSpeed
```

正的 turnSpeed → 右轮更快，左轮更慢 (车向左转)。
dEy 仅在收到新 seq 时更新。

---

## 8. 默认控制参数

| 参数 | 值 | 说明 |
|------|------|------|
| VISION_TRACK_BASE_SPEED_CMPS | 10.0 | 基础前进速度 |
| VISION_TRACK_TURN_LIMIT_CMPS | 6.0 | 最大转向修正 |
| VISION_TRACK_KY | 0.004 | 横向比例增益 |
| VISION_TRACK_KA | 0.02 | 航向比例增益 |
| VISION_TRACK_KD | 0.001 | 横向微分增益 |
| VISION_TRACK_TURN_SIGN | -1.0 | 转向方向符号 |
| VISION_TRACK_MIN_CONFIDENCE | 70 | 最低置信度 |
| VISION_TRACK_TIMEOUT_MS | 200 | 超时停车阈值 |
| VISION_TRACK_FRESH_LIMIT_MS | 150 | 数据新鲜度阈值 |
| VISION_TRACK_ACQUIRE_FRAMES | 3 | 获取连续有效帧数 |

所有参数统一定义在 `App/app_vision_track.h` 中。

---

## 9. 是否跳过灰度初始化

是。当 `F21_VISION_TRACK_TEST_MODE=1` 时，`main.c` 不调用 `Grayscale_Init()` 和 `App_Line_GPIOForceInit()`。原代码保留在 `#else` 分支中，设回 0 即可恢复。

---

## 10. 是否关闭NRF和双车协同

是。视觉测试模式中，不初始化 `App_Radio_Init()`, `F21Car_Init()`, `F21Coop_Init()`, `App_TaskMode_Init()`。原代码保留在 `#else` 分支中。

---

## 11. Keil Rebuild 结果

```
Build target 'master_car_withK230'
0 Error(s), 0 Warning(s)
SysConfig: OK
```

---

## 12. AXF 和 HEX 路径

- AXF: `firmware_withK230/master_car/keil/Objects/master_car_withK230.axf` (268256 bytes)
- HEX: `firmware_withK230/master_car/keil/Objects/master_car_withK230.hex` (35916 bytes)

---

## 13. 已验证内容

- 编译通过 (0 Error, 0 Warning)
- 新模块文件正确加入 Keil 工程
- 宏 `F21_VISION_TRACK_TEST_MODE=1` 生效
- main.c 走视觉测试分支 (#elif)
- 原始完整 F 题代码完整保留在 #else 中

---

## 14. 未进行的实车验证

- K230 UART 帧解析未在硬件上测试
- 视觉循迹控制参数未上车调参
- 自动重捕获 (LOST→RUN) 未实测
- 按键响应时序未实测
- K1 无功能（已按要求）

---

## 15. 后续需要调节的参数

- `VISION_TRACK_KY`, `VISION_TRACK_KA`, `VISION_TRACK_KD` — 需要上手实测
- `VISION_TRACK_BASE_SPEED_CMPS` — 根据实际速度闭环表现调整
- `VISION_TRACK_TURN_LIMIT_CMPS` — 避免过冲
- `VISION_TRACK_MIN_CONFIDENCE` — 根据 K230 输出质量调高/调低
- `VISION_TRACK_FRESH_LIMIT_MS` / `VISION_TRACK_TIMEOUT_MS` — 根据 K230 帧率调整
