# 技术变更：原地旋转模块 (Rotate)

> 日期：2026-07-13  
> 类型：新增功能

## 概述

新增 `Drivers/Rotate/` 模块，实现**原地旋转至指定角度**的闭环控制能力。车体通过差速驱动原地转向，陀螺仪 yaw 角作为反馈，增量式 PID 闭环控制，自动处理 ±180° 环绕跳变。

## API

| 函数 | 说明 |
|------|------|
| `Rotate_Init()` | 使用默认参数初始化 |
| `Rotate_Config(cfg)` | 自定义 PID / 速度 / 容忍度 / 显示回调 |
| `Rotate_To(target_yaw)` | 阻塞旋转到目标角度，返回 `true`=到位 / `false`=超时 |
| `Rotate_Stop()` | 立即刹停 |

## 关键技术设计

### 1. 增量式角度 PID

```
Δu = Kp×(e-e₁) + Ki×e + Kd×(e-2e₁+e₂)
```

积分输出限幅 ±200，防止 windup。PID 输出直接映射为左右电机差速（left = -steer, right = +steer）。

### 2. ±180° 环绕双值比较

陀螺仪 yaw 在 ±180° 处跳变。误差计算时同时评估两条路径：

```
error₁ = target - current          // 直接路径
error₂ = error₁ ± 360              // 绕环路径
取 |error| 更小者                 // 保证走最短弧
```

到位检测、PID 误差输入均使用此双值比较，确保跨 ±180° 边界时不绕远路。

### 3. 连续容忍到位检测

单次入容忍不算到位，需连续 `ROTATE_OK_COUNT`（默认 5 次，约 50ms）保持在容忍范围内才判定完成，避免抖动误判。

### 4. 超时保护

`Rotate_To()` 内置超时（默认 15s），超时自动刹停返回 `false`，不会死循环。

### 5. 显示回调机制

`Rotate_Config_t.display_fn` 提供一个可选回调，每 PID 周期（约 10ms）调用一次，传入实时 yaw、目标角度、左右 PWM 值，便于任务注册 OLED 刷新逻辑。不注册时默认 `NULL`，零开销。

## 默认参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `ROTATE_DEFAULT_SPEED` | 300 | PWM 速度 (0~999) |
| `ROTATE_DEFAULT_KP` | 7.0 | 角度 PID P 系数 |
| `ROTATE_DEFAULT_KI` | 2.0 | 角度 PID I 系数 |
| `ROTATE_DEFAULT_KD` | 3.0 | 角度 PID D 系数 |
| `ROTATE_DEFAULT_TOLERANCE` | 1.5° | 到位容忍度 |
| `ROTATE_DEFAULT_TIMEOUT` | 15000ms | 超时时间 |
| `ROTATE_OK_COUNT` | 5 | 连续容忍次数 |
| `ROTATE_I_LIMIT` | ±200 | 积分限幅 |

## 使用示例

### 简单用法

```c
Rotate_Init();
bool ok = Rotate_To(90.0f);  // 旋转到 90°，阻塞直到到位或超时
```

### 自定义配置 + OLED 实时显示

```c
static void my_display(float yaw, float target, int16_t left, int16_t right) {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Rotating", 8);
    sprintf(buf, "Y:%6.1f -> %.0f", yaw, target);
    OLED_ShowString(0, 2, (uint8_t *)buf, 16);
}

Rotate_Config_t cfg = {
    .Kp = 7.0f, .Ki = 2.0f, .Kd = 3.0f,
    .speed = 300,
    .tolerance = 1.5f,
    .timeout_ms = 15000,
    .display_fn = my_display,
};
Rotate_Config(&cfg);
Rotate_To(179.0f);
```

## 受影响文件

| 文件 | 变更 |
|------|------|
| `Drivers/Rotate/rotate.h` | 新增 — 模块头文件 |
| `Drivers/Rotate/rotate.c` | 新增 — 模块实现 |
| `Task/task6.c` | 新增 — 旋转功能验证任务 |
| `Task/task6.h` | 新增 — Task6 头文件 |
| `main.c` | 修改 — 注册 Task6 调度入口 |
