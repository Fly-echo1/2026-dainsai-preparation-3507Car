/**
 * @file  rotate.h
 * @brief 原地旋转模块 — 角度 PID 闭环控制，自动处理 ±180° 环绕。
 *
 * 使用示例:
 *   Rotate_Init();                          // 默认配置
 *   Rotate_To(90.0f);                       // 阻塞旋转到 90°
 *   Rotate_Config_t cfg = {...};
 *   Rotate_Config(&cfg);                    // 自定义 PID / 速度 / 容忍度
 *   Rotate_To(-45.0f);                      // 旋转到 -45°
 */

#ifndef __ROTATE_H
#define __ROTATE_H

#include <stdbool.h>
#include <stdint.h>

/* ===== 默认参数 ===== */
#define ROTATE_DEFAULT_SPEED 300      /* 原地旋转 PWM 速度 (0~999)     */
#define ROTATE_DEFAULT_KP 1.0f       /* 角度 PID P 系数              */
#define ROTATE_DEFAULT_KI 0.4f       /* 角度 PID I 系数              */
#define ROTATE_DEFAULT_KD 10.0f        /* 角度 PID D 系数              */
#define ROTATE_DEFAULT_TOLERANCE 0.3f /* 到位容忍度 (°)               */
#define ROTATE_DEFAULT_TIMEOUT 15000  /* 超时时间 (ms)                 */
#define ROTATE_OK_COUNT 5             /* 连续容忍次数 (ms=次数×10)      */
#define ROTATE_I_LIMIT 150            /* 积分限幅 (±)               */
#define ROTATE_OUT_LIMIT 300          /* 输出限幅 (±) 匹配 SPEED    */

/* ===== 显示回调类型 ===== */
typedef void (*Rotate_DisplayFn)(float yaw, float target, int16_t left,
                                 int16_t right);

/* ===== 配置结构体 ===== */
typedef struct {
  float Kp;                    /* 角度 PID P 系数                    */
  float Ki;                    /* 角度 PID I 系数                    */
  float Kd;                    /* 角度 PID D 系数                    */
  int16_t speed;               /* 原地旋转 PWM 速度 (0~999)          */
  float tolerance;             /* 到位容忍度 (°)                     */
  uint32_t timeout_ms;         /* 超时时间 (ms)                      */
  Rotate_DisplayFn display_fn; /* 可选: 每轮循环调用的显示回调    */
} Rotate_Config_t;

/* ===== API ===== */

/**
 * @brief 使用默认参数初始化旋转模块。
 */
void Rotate_Init(void);

/**
 * @brief 自定义旋转参数。
 * @param cfg  配置结构体指针。
 */
void Rotate_Config(const Rotate_Config_t *cfg);

/**
 * @brief 原地旋转到目标 yaw 角（阻塞式，含超时保护）。
 *        自动处理 ±180° 环绕跳变。
 * @param target_yaw  目标 yaw 角 (°)
 * @return true=到位, false=超时未到位
 */
bool Rotate_To(float target_yaw);

/**
 * @brief 立即停止旋转（两轮刹停）。
 */
void Rotate_Stop(void);

#endif /* __ROTATE_H */
