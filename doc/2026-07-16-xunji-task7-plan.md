# xunji + task7 Implementation Plan

> **For agentic workers:** Execute this plan task-by-task in order. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建独立的 `xunji` 通用循迹模块 + `task7` 测试任务

**Architecture:** `xunji_run()` 为阻塞式循迹函数，内部 PD 控制循环直到 |Δyaw|>140° 且全白去抖 2 次。每轮循环回调 `display_fn` 刷新 OLED。task7 提供倒计时 + 循迹 + 结果显示的完整测试流程。

**Tech Stack:** C (MSPM0G3507), DriverLib, 8 路灰度传感器, WIT 陀螺仪

## Global Constraints

- 项目路径: `d:\TI\project\REACT\3507base`
- 编码风格: 4 空格缩进, 英文注释, 文件头 Doxygen 格式
- Motor_SetSpeed 签名: `void Motor_SetSpeed(int16_t speedA, int16_t speedB)`, PWM 范围 0-999
- OLED: 128x64, `OLED_ShowString(page, col, data, font_size)`, 8px 字体每行 16 列
- Grayscale: `GRAYSCALE_CHANNELS=8`, `Grayscale_Sensor_Read_All(sensors)` 读入 1=黑线
- 陀螺仪: `wit_data.yaw` 为当前 yaw 角, ±180° 范围
- 计时: `tick_ms` 全局毫秒计数器, `mspm0_delay_ms(ms)` 阻塞延时

---

### Task 1: 创建 `Drivers/xunji/xunji.h`

**Files:**

- Create: `Drivers/xunji/xunji.h`

**Interfaces:**

- Produces: 宏定义 `XUNJI_*`, 回调类型 `xunji_display_fn`, 函数声明 `xunji_run()`

- [ ] **Step 1: 创建文件**

```c
/**
 * @file  xunji.h
 * @brief 通用灰度循迹模块: 沿黑线循迹直到 |Δyaw|>阈值 且全白去抖
 *
 * 使用方式:
 *   task7 测试:  xunji_run(my_display_fn);
 *   task2neo BC段: xunji_run(NULL);
 */

#ifndef __XUNJI_H
#define __XUNJI_H

#include <stdbool.h>
#include <stdint.h>

/* ===== 可调参数 ===== */
#define XUNJI_SPEED          130      /* 基础循迹速度 (0~999 PWM)          */
#define XUNJI_KP             7.0f     /* P 系数                            */
#define XUNJI_KI             0.015f   /* I 系数                            */
#define XUNJI_KD             2.0f     /* D 系数                            */
#define XUNJI_DIFF_LIMIT     200      /* 差速输出限幅                       */
#define XUNJI_TIMEOUT_S      60       /* 超时保护 (秒)                      */
#define XUNJI_YAW_THRESHOLD  140.0f   /* 停止条件: |Δyaw| 阈值 (°)         */
#define XUNJI_WHITE_DEBOUNCE 2        /* 停止条件: 全白去抖次数             */
#define XUNJI_LOOP_DELAY_MS  10       /* 控制循环周期 (ms)                  */

/* ===== 显示回调类型 ===== */
typedef void (*xunji_display_fn)(float yaw, float delta_yaw,
                                  const uint8_t *sensors,
                                  int16_t left, int16_t right);

/* ===== 接口函数 ===== */

/**
 * @brief 阻塞式循迹, 直到满足停止条件或超时
 * @param display_fn 每轮循环回调, NULL 则不刷新 OLED
 * @return true=正常到达终点, false=超时
 */
bool xunji_run(xunji_display_fn display_fn);

#endif /* __XUNJI_H */
```

- [ ] **Step 2: 验证** — 检查无编译错误（需等全部文件就绪后一起编译）

---

### Task 2: 创建 `Drivers/xunji/xunji.c`

**Files:**

- Create: `Drivers/xunji/xunji.c`

**Interfaces:**

- Consumes: `xunji.h` 宏定义, `Grayscale.h`, `Motor.h`, `wit.h`, `clock.h`
- Produces: `xunji_run()` 实现

- [ ] **Step 1: 创建文件**

```c
/**
 * @file  xunji.c
 * @brief 通用灰度循迹模块实现
 *
 * 控制策略:
 *   - 误差计算: 8 路灰度加权质心法, 相对中心位置 3.5 计算偏移
 *   - PID: 增量式 PD, P 处理即时偏差、D 抑制震荡、I 累积弯道偏置
 *   - 脱线: 全白时清零积分和微分, 直行保持
 *   - 停止: |Δyaw| > XUNJI_YAW_THRESHOLD 且全白去抖 XUNJI_WHITE_DEBOUNCE 次
 */

#include "xunji.h"

#include "Grayscale.h"
#include "Motor.h"
#include "clock.h"
#include "ti_msp_dl_config.h"
#include "wit.h"

/*----------------------------------------------------------------------
 * 灰度加权质心误差
 *   传感器布局: 索引 0=最左, 7=最右
 *   理想中心: 4.5 (传感器 4 和 5 之间)
 *   返回值: 负=偏左需右转, 正=偏右需左转, 范围约 [-7, +7]
 *----------------------------------------------------------------------*/
static float gray_error(const uint8_t *sensors) {
  int16_t sum_pos = 0;
  uint8_t sum_cnt = 0;
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++) {
    if (sensors[i]) {
      sum_pos += (int16_t)i;
      sum_cnt++;
    }
  }
  if (sum_cnt == 0)
    return 0.0f;

  float center = (float)sum_pos / (float)sum_cnt;
  return (center - 4.5f) * 2.0f;
}

/*----------------------------------------------------------------------
 * 阻塞式循迹主循环
 *----------------------------------------------------------------------*/
bool xunji_run(xunji_display_fn display_fn) {
  uint32_t start_tick = tick_ms;
  float start_yaw = wit_data.yaw;
  uint8_t sensors[GRAYSCALE_CHANNELS];
  float integral = 0.0f;
  float prev_error = 0.0f;
  uint8_t white_count = 0;
  int16_t left, right;

  while (1) {
    /* ---- 超时保护 ---- */
    if ((tick_ms - start_tick) / 1000 >= XUNJI_TIMEOUT_S) {
      Motor_Stop();
      return false;
    }

    Grayscale_Sensor_Read_All(sensors);

    /* ---- 计算角度变化 (处理 ±180° 跳变) ---- */
    float delta_yaw = wit_data.yaw - start_yaw;
    if (delta_yaw > 180.0f)
      delta_yaw -= 360.0f;
    if (delta_yaw < -180.0f)
      delta_yaw += 360.0f;

    /* ---- 统计黑线数量 ---- */
    uint8_t black_cnt = 0;
    for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++)
      if (sensors[i])
        black_cnt++;

    /* ---- 停止条件: |Δyaw| > 阈值 且 全白去抖 ---- */
    if (black_cnt == 0 && (delta_yaw > XUNJI_YAW_THRESHOLD ||
                           delta_yaw < -XUNJI_YAW_THRESHOLD)) {
      white_count++;
      if (white_count >= XUNJI_WHITE_DEBOUNCE) {
        Motor_Stop();
        return true;
      }
    } else {
      white_count = 0;
    }

    /* ---- 循迹 PD 控制 ---- */
    if (black_cnt > 0) {
      float err = gray_error(sensors);
      integral += err;
      float derivative = err - prev_error;
      prev_error = err;

      int16_t diff = (int16_t)(XUNJI_KP * err + XUNJI_KI * integral +
                               XUNJI_KD * derivative);
      if (diff > XUNJI_DIFF_LIMIT)
        diff = XUNJI_DIFF_LIMIT;
      if (diff < -XUNJI_DIFF_LIMIT)
        diff = -XUNJI_DIFF_LIMIT;

      left = XUNJI_SPEED + diff;
      right = XUNJI_SPEED - diff;
    } else {
      /* 全白脱线: 清零积分和微分, 直行保持 */
      integral = 0.0f;
      prev_error = 0.0f;
      left = XUNJI_SPEED;
      right = XUNJI_SPEED;
    }

    /* ---- PWM 限幅 ---- */
    if (left > 999)
      left = 999;
    if (left < 0)
      left = 0;
    if (right > 999)
      right = 999;
    if (right < 0)
      right = 0;

    Motor_SetSpeed(left, right);

    /* ---- OLED 回调 ---- */
    if (display_fn)
      display_fn(wit_data.yaw, delta_yaw, sensors, left, right);

    mspm0_delay_ms(XUNJI_LOOP_DELAY_MS);
  }
}
```

- [ ] **Step 2: 验证** — 编译检查无错误

---

### Task 3: 创建 `Task/task7.h` + `Task/task7.c`

**Files:**

- Create: `Task/task7.h`
- Create: `Task/task7.c`

**Interfaces:**

- Consumes: `xunji.h`, `main.h`, `wit.h`, `oled_software_i2c.h`
- Produces: `Task7_Run()`

- [ ] **Step 1: 创建 task7.h**

```c
/**
 * @file  task7.h
 * @brief Task7: 调用 xunji 模块测试灰度循迹功能
 */

#ifndef __TASK7_H
#define __TASK7_H

void Task7_Run(void);

#endif /* __TASK7_H */
```

- [ ] **Step 2: 创建 task7.c**

```c
/**
 * @file  task7.c
 * @brief Task7: 倒计时 → xunji_run() 循迹 → 显示结果
 *
 * OLED 显示 (循迹中):
 *   行0: Task7
 *   行2: Y:xxx.x (实时 yaw)
 *   行4: D:xxx.x (Δyaw)
 *   行6: 8 路灰度传感器状态
 *   行7: L:xxx R:xxx (左右 PWM)
 */

#include "task7.h"

#include "main.h"
#include "oled_software_i2c.h"
#include "ti_msp_dl_config.h"
#include "wit.h"
#include "xunji.h"

#include <stdio.h>

/* ===== 可调参数 ===== */
#define DONE_DELAY_MS 4000 /* 完成/超时后显示停留 (ms) */

/* ===== 模块级静态变量 ===== */
static char oled_buf[32];

/*----------------------------------------------------------------------
 * 循迹过程实时 OLED 刷新回调
 *----------------------------------------------------------------------*/
static void task7_display(float yaw, float delta_yaw,
                           const uint8_t *sensors,
                           int16_t left, int16_t right) {
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);

  sprintf(oled_buf, "Y:%6.1f", (double)yaw);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);

  sprintf(oled_buf, "D:%6.1f", (double)delta_yaw);
  OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);

  sprintf(oled_buf, "%d%d%d%d%d%d%d%d", sensors[0], sensors[1], sensors[2],
          sensors[3], sensors[4], sensors[5], sensors[6], sensors[7]);
  OLED_ShowString(0, 6, (uint8_t *)oled_buf, 8);

  sprintf(oled_buf, "L:%4d R:%4d", left, right);
  OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
}

/*----------------------------------------------------------------------
 * Task7 主逻辑
 *----------------------------------------------------------------------*/
void Task7_Run(void) {
  bool ok;
  uint32_t start_tick;

  /* ---- 3-2-1 倒计时 (显示 yaw) ---- */
  for (uint8_t i = 3; i > 0; i--) {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);
    sprintf(oled_buf, "Start in %d...", i);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);
    sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);
    mspm0_delay_ms(700);
  }

  /* ---- GO! ---- */
  OLED_Clear();
  OLED_ShowString(0, 2, (uint8_t *)"GO!", 16);
  mspm0_delay_ms(300);

  /* ---- 发车循迹 ---- */
  OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);
  start_tick = tick_ms;

  ok = xunji_run(task7_display);

  /* ---- 显示结果 ---- */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);
  sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
  OLED_ShowString(0, 4, (uint8_t *)(ok ? "Done!" : "Timeout!"), 12);
  sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
  OLED_ShowString(0, 6, (uint8_t *)oled_buf, 16);
  mspm0_delay_ms(DONE_DELAY_MS);

  task_flag = 0;
}
```

- [ ] **Step 3: 验证** — 编译检查无错误

---

### Task 4: 修改 `main.c` 注册 task7

**Files:**

- Modify: `main.c`

**Interfaces:**

- Consumes: `Task/task7.h`
- Produces: `task_flag=7` → `Task7_Run()`

- [ ] **Step 1: 在 `main.c` 头部添加 `#include "Task/task7.h"`**

在第 41 行 `#include "Task/task6.h"` 之后追加:

```c
#include "Task/task7.h"
```

- [ ] **Step 2: 在 `switch(task_flag)` 中添加 `case 7:`**

在第 105 行 `case 6:` 的 `break;` 之后追加:

```c
    case 7:
      Task7_Run();
      break;
```

- [ ] **Step 3: 编译并验证**

- [ ] **Step 4: 修改 `task_flag = 5` 为 `task_flag = 7` 测试 task7**

在 CCS 中编译下载后，上电即运行 task7；或保持 `task_flag = 5` 后在调试器中改为 7。
