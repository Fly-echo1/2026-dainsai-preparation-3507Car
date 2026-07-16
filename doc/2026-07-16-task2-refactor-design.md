# task2 重构 — 基于 xunji + rotate 模块

> 设计日期：2026-07-16

---

## 一、概述

用已验证的 `xunji` 循迹模块和 `rotate` 旋转模块重写 `task2.c`，替代原有的内联 PID/查表逻辑，从 773 行缩减到约 200 行。

## 二、站点标志控制

```c
flagA=1, flagB=0, flagC=0, flagD=0
// 前一个 flag=1 且满足到站条件 → 当前 flag 置 1
```

## 三、完整流程

```
AB直行(陀螺仪AnglePID, 目标0°)
  → B: 连续20次检测到黑线 → flagB=1
  → 立刻切 xunji_run(NULL) + 非阻塞蜂鸣0.5s

BC弧线(xunji, 停止条件: |Δyaw|>150°+全白去抖20次)
  → C: xunji停 → flagC=1
  → 非阻塞蜂鸣0.5s + Rotate_To(179°)
  → rotate到位后立刻进CD直行

CD直行(陀螺仪AnglePID, 目标179°)
  → D: 连续20次检测到黑线 → flagD=1
  → 立刻切 xunji_run(NULL) + 非阻塞蜂鸣0.5s

DA弧线(xunji, 停止条件同上)
  → A: xunji停 → 停车 + 蜂鸣 + 显示耗时 → task_flag=0
```

## 四、站点判据

| 站点 | 判据 | 动作 |
|------|------|------|
| B | 任意黑线, 连续20次去抖 | flagB=1, 切xunji, 蜂鸣0.5s |
| C | xunji自带: \|Δyaw\|>150°+全白20次 | flagC=1, 蜂鸣0.5s, Rotate_To(179°) |
| D | 任意黑线, 连续20次去抖 | flagD=1, 切xunji, 蜂鸣0.5s |
| A | xunji自带: \|Δyaw\|>150°+全白20次 | 停车, 蜂鸣, 显示结果 |

## 五、依赖

- `Drivers/xunji/xunji.h` — xunji_run()
- `Drivers/Rotate/rotate.h` — Rotate_To()
- AnglePID（内联，复用 existing 增量式 PID）
- `Grayscale.h`, `Motor.h`, `wit.h`, `clock.h`
- `oled_software_i2c.h` — OLED 显示
- `main.h` — task_flag
