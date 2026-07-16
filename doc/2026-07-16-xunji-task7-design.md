# xunji 通用循迹模块 + task7 测试任务 — 设计规格

> 设计日期：2026-07-16

---

## 一、概述

将灰度循迹逻辑封装为独立的 `xunji` 模块（位于 `Drivers/xunji/`），提供阻塞式循迹函数 `xunji_run()`。创建 `task7` 对模块进行测试验证。后续 `task2neo` 的 BC 弧线段可直接复用该模块。

---

## 二、xunji 模块

### 2.1 文件

| 文件 | 职责 |
|------|------|
| `Drivers/xunji/xunji.h` | 宏定义、回调类型、函数声明 |
| `Drivers/xunji/xunji.c` | `xunji_run()` 实现 |

### 2.2 架构

- **阻塞式**：调用后内部循环直到满足停止条件或超时才返回
- **带显示回调**：每轮 PID 循环调用 `display_fn` 刷新 OLED
- **参数全部宏定义**：速度和 PID 参数在 `xunji.h` 中定义

### 2.3 宏定义

```c
#define XUNJI_SPEED         130      // 基础循迹速度 (0~999)
#define XUNJI_KP            7.0f     // P 系数
#define XUNJI_KI            0.015f   // I 系数
#define XUNJI_KD            2.0f     // D 系数
#define XUNJI_DIFF_LIMIT    200      // 差速限幅
#define XUNJI_TIMEOUT_S     60       // 超时 (秒)
#define XUNJI_YAW_THRESHOLD 140.0f   // 停止角度阈值
#define XUNJI_WHITE_DEBOUNCE 2       // 全白去抖次数
#define XUNJI_LOOP_DELAY_MS 10       // 循环周期 (ms)
```

### 2.4 回调类型

```c
typedef void (*xunji_display_fn)(float yaw, float delta_yaw,
                                  const uint8_t *sensors,
                                  int16_t left, int16_t right);
```

### 2.5 函数签名

```c
bool xunji_run(xunji_display_fn display_fn);
```

- **返回值**：`true` = 正常到达终点，`false` = 超时
- **参数**：`display_fn` 为 `NULL` 时不刷新 OLED

### 2.6 控制策略

- **误差计算**：查表优先 —— 加权质心兜底，理想中心 3.5（灰度4/5 = s[3]/s[4]）
- **PID**：增量式 PD，带积分限幅和差速限幅
- **脱线**：全白时清零积分和微分，直行保持
- **停止条件**：`|yaw - start_yaw| > XUNJI_YAW_THRESHOLD` 且灰度全白去抖 `XUNJI_WHITE_DEBOUNCE` 次
- **超时**：`XUNJI_TIMEOUT_S` 秒后强制返回 `false`

### 2.7 依赖

- `Grayscale.h` — 8 路灰度传感器
- `Motor.h` — 电机控制
- `wit.h` — 陀螺仪 yaw 角
- `clock.h` — `tick_ms` 计时

---

## 三、task7 测试任务

### 3.1 文件

| 文件 | 职责 |
|------|------|
| `Task/task7.h` | `Task7_Run()` 声明 |
| `Task/task7.c` | 测试流程 + `display_fn` 回调实现 |

### 3.2 测试流程

```
3-2-1 倒计时 (每次显示 yaw，间隔 700ms)
  → "GO!" 停留 300ms
  → 直接调用 xunji_run(task7_display)
  → 显示结果，停留 4s
  → task_flag = 0
```

### 3.3 OLED 显示（循迹过程中回调刷新）

| 行 | 内容 | 说明 |
|----|------|------|
| 行 0 | `Task7` | 任务标识 |
| 行 2 | `T:xx.xs` | 耗时 |
| 行 4 | `Y:xxx.x D:xxx` | yaw 角 + 角度变化量 |
| 行 7 | `S:xxxxxxxx L:xxx R:xxx` | 8 路传感器 + 左右 PWM |

### 3.4 结束显示

- 正常到达：`Done! T:xx.xs`
- 超时：`Timeout! T:xx.xs`

---

## 四、task2neo 复用

task2neo 的 BC 弧线段（阶段2）改为调用 `xunji_run(NULL)`，替代现有的内联循迹循环。

---

## 五、参数调优指南

| 现象 | 调整 |
|------|------|
| 震荡 | 降低 `XUNJI_KP` 或加大 `XUNJI_KD` |
| 转弯不够 | 加大 `XUNJI_KP` |
| 超调冲出赛道 | 加大 `XUNJI_KD`，降低 `XUNJI_KI` |
| 反应迟钝 | 加大 `XUNJI_KP`，降低 `XUNJI_KI` |
| 过早停车 | 加大 `XUNJI_YAW_THRESHOLD` 或 `XUNJI_WHITE_DEBOUNCE` |
| 过站不停 | 降低 `XUNJI_YAW_THRESHOLD` 或检查 yaw 读数 |
