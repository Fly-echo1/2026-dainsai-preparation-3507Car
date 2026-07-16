/**
 * @file  rotate.c
 * @brief 原地旋转模块实现 — 角度 PID 增量式闭环控制，±180° 环绕处理。
 *
 * 依赖: WIT(陀螺仪) / Motor(电机) / clock(延时)
 */

#include "rotate.h"

#include "Motor.h"
#include "clock.h"
#include "wit.h"

/* ===== 角度 PID 结构 ===== */
typedef struct {
  float Kp, Ki, Kd;
  float error, last_error;
  float integral;
} AnglePID_t;

/* ===== 内部函数声明 ===== */
static void AnglePID_Init(AnglePID_t *pid, float Kp, float Ki, float Kd);
static void AnglePID_Reset(AnglePID_t *pid);
static float AnglePID_Calc(AnglePID_t *pid, float target, float current);

/* ===== 模块级静态变量 ===== */
static AnglePID_t pid;
static Rotate_Config_t cfg;

/*===========================================================================
 * 角度 PID — 位置式 + ±180° 环绕
 *===========================================================================*/

static void AnglePID_Init(AnglePID_t *pid, float Kp, float Ki, float Kd) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->integral = 0.0f;
}

static void AnglePID_Reset(AnglePID_t *pid) {
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->integral = 0.0f;
}

static float AnglePID_Calc(AnglePID_t *pid, float target, float current) {
  /* 双值比较: 取最短路径误差 */
  float error = target - current;
  float alt = error > 0.0f ? error - 360.0f : error + 360.0f;
  float abs_error = error > 0.0f ? error : -error;
  float abs_alt = alt > 0.0f ? alt : -alt;
  error = abs_alt < abs_error ? alt : error;

  pid->error = error;

  /* 积分: 累积误差 (抗饱和限幅) */
  pid->integral += error;
  if (pid->integral > (float)ROTATE_I_LIMIT)
    pid->integral = (float)ROTATE_I_LIMIT;
  if (pid->integral < -(float)ROTATE_I_LIMIT)
    pid->integral = -(float)ROTATE_I_LIMIT;

  /* 位置式: u = Kp*e + Ki*∫e + Kd*(e−e₁) */
  float output = pid->Kp * error + pid->Ki * pid->integral +
                 pid->Kd * (error - pid->last_error);

  pid->last_error = error;

  /* 输出限幅 */
  if (output > (float)ROTATE_OUT_LIMIT)
    output = (float)ROTATE_OUT_LIMIT;
  if (output < -(float)ROTATE_OUT_LIMIT)
    output = -(float)ROTATE_OUT_LIMIT;

  return output;
}

/*===========================================================================
 * 旋转模块 API
 *===========================================================================*/

/**
 * @brief 使用默认参数初始化旋转模块。
 */
void Rotate_Init(void) {
  Rotate_Config_t default_cfg = {
      .Kp = ROTATE_DEFAULT_KP,
      .Ki = ROTATE_DEFAULT_KI,
      .Kd = ROTATE_DEFAULT_KD,
      .speed = ROTATE_DEFAULT_SPEED,
      .tolerance = ROTATE_DEFAULT_TOLERANCE,
      .timeout_ms = ROTATE_DEFAULT_TIMEOUT,
      .display_fn = NULL,
  };
  Rotate_Config(&default_cfg);
}

/**
 * @brief 自定义旋转参数。
 */
void Rotate_Config(const Rotate_Config_t *new_cfg) {
  cfg = *new_cfg;
  AnglePID_Init(&pid, cfg.Kp, cfg.Ki, cfg.Kd);
}

/**
 * @brief 原地旋转到目标 yaw 角（阻塞式）。
 *        left=反转 right=正转 → yaw 增大方向（左转/逆时针）。
 */
bool Rotate_To(float target_yaw) {
  uint32_t start_tick;
  uint8_t ok_count;
  float steer;
  int16_t left, right;

  AnglePID_Reset(&pid);
  start_tick = tick_ms;
  ok_count = 0;

  while (1) {
    /* ---- LED 周期翻转 (调试用) ---- */
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_LED1_PIN);

    /* 超时检测 */
    if (tick_ms - start_tick >= cfg.timeout_ms) {
      Rotate_Stop();
      return false;
    }

    /* PID 计算 */
    steer = AnglePID_Calc(&pid, target_yaw, wit_data.yaw);

    /* 原地旋转: left=-steer right=steer → 左转(CCW) yaw增大 */
    left = -(int16_t)steer;
    right = (int16_t)steer;

    /* 限幅 */
    if (left > cfg.speed)
      left = cfg.speed;
    if (left < -cfg.speed)
      left = -cfg.speed;
    if (right > cfg.speed)
      right = cfg.speed;
    if (right < -cfg.speed)
      right = -cfg.speed;

    Motor_SetSpeed(left, right);

    /* 显示回调 */
    if (cfg.display_fn) {
      cfg.display_fn(wit_data.yaw, target_yaw, left, right);
    }

    /* 到位检测: 双值比较取最短路径误差 */
    float diff = target_yaw - wit_data.yaw;
    float alt = diff > 0.0f ? diff - 360.0f : diff + 360.0f;
    float abs_diff = diff > 0.0f ? diff : -diff;
    float abs_alt = alt > 0.0f ? alt : -alt;
    if (abs_alt < abs_diff)
      diff = alt;
    if (diff < cfg.tolerance && diff > -cfg.tolerance) {
      ok_count++;
      if (ok_count >= ROTATE_OK_COUNT) {
        Rotate_Stop();
        return true;
      }
    } else {
      ok_count = 0;
    }

    mspm0_delay_ms(10);
  }
}

/**
 * @brief 立即停止旋转。
 */
void Rotate_Stop(void) { Motor_Stop(); }
