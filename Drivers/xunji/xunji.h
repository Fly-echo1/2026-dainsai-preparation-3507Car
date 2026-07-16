/**
 * @file  xunji.h
 * @brief 通用灰度循迹模块: 沿黑线循迹直到 |Δyaw|>阈值 且全白去抖
 *
 * 使用方式:
 *   task7 测试:    xunji_run(my_display_fn);
 *   task2neo BC段: xunji_run(NULL);
 */

#ifndef __XUNJI_H
#define __XUNJI_H

#include <stdbool.h>
#include <stdint.h>

/* ===== 可调参数 ===== */
#define XUNJI_SPEED 150            /* 基础循迹速度 (0~999 PWM)          */
#define XUNJI_KP 30.0f             /* P 系数                            */
#define XUNJI_KI 0.001f              /* I 系数                            */
#define XUNJI_KD 1.0f              /* D 系数                            */
#define XUNJI_DIFF_LIMIT 200       /* 差速输出限幅                       */
#define XUNJI_TIMEOUT_S 30         /* 超时保护 (秒)                      */
#define XUNJI_YAW_THRESHOLD 150.0f /* 停止条件: |Δyaw| 阈值 (°)         */
#define XUNJI_WHITE_DEBOUNCE 20     /* 停止条件: 全白去抖次数             */
#define XUNJI_LOOP_DELAY_MS 2      /* 控制循环周期 (ms)                  */

/* ===== 显示回调类型 ===== */
typedef void (*xunji_display_fn)(float yaw, float delta_yaw,
                                 const uint8_t *sensors, int16_t left,
                                 int16_t right);

/* ===== 接口函数 ===== */

/**
 * @brief 阻塞式循迹, 直到满足停止条件或超时
 * @param display_fn 每轮循环回调, NULL 则不刷新 OLED
 * @return true=正常到达终点, false=超时
 */
bool xunji_run(xunji_display_fn display_fn);

#endif /* __XUNJI_H */
