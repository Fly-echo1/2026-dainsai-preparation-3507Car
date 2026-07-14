/**
 * @file  task1.c
 * @brief Task 1 实现: A→B 直线行驶，陀螺仪 Yaw 角度 PID 保持航向，
 *        灰度传感器检测黑线到站停车，蜂鸣器响 1 秒。
 *
 * 依赖驱动: WIT(陀螺仪) / Motor(电机) / Grayscale(灰度) / OLED / BEEP
 */

#include "task1.h"

#include "Grayscale.h"
#include "Motor.h"
#include "clock.h"
#include "main.h"
#include "oled_software_i2c.h"
#include "ti_msp_dl_config.h"
#include "wit.h"

#include <stdbool.h>
#include <stdio.h>

/* ===== 可调参数 ===== */
#define BASE_SPEED 150  /* 直线基础速度 (0~999 PWM)               */
#define STEER_LIMIT 350 /* 转向修正量限幅                         */

#define ANGLE_KP 9.0f /* 角度 PID 比例系数                      */
#define ANGLE_KI 0.8f /* 角度 PID 积分系数                      */
#define ANGLE_KD 0.5f /* 角度 PID 微分系数                      */

#define STATION_DEBOUNCE 4 /* 黑线检测次数 (累加不重置)              */
#define LOOP_DELAY_MS 10   /* 控制循环周期 (ms)                      */
#define TASK_TIMEOUT_S 15  /* 超时时间 (秒)                          */

/* ===== 角度 PID ===== */
typedef struct {
  float Kp, Ki, Kd;
  float error, last_error, last2_error;
  float output;
  float out_min, out_max;
} AnglePID_t;

/* ===== 内部函数声明 ===== */
static void AnglePID_Init(AnglePID_t *pid, float Kp, float Ki, float Kd);
static void AnglePID_SetLimit(AnglePID_t *pid, float min, float max);
static void AnglePID_Reset(AnglePID_t *pid);
static float AnglePID_Calc(AnglePID_t *pid, float target, float current);
static bool IsStation(void);

/* ===== 模块级静态变量 ===== */
static AnglePID_t pid_angle;
static uint8_t station_count;
static char oled_buf[32];
static volatile uint8_t testflag = 0; /* 1=纯PID调试(不停), 0=正常模式 */

/*===========================================================================
 * 角度 PID — 增量式 + ±180° 环绕处理
 *===========================================================================*/

static void AnglePID_Init(AnglePID_t *pid, float Kp, float Ki, float Kd) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->last2_error = 0.0f;
  pid->output = 0.0f;
  pid->out_min = -1e6f;
  pid->out_max = 1e6f;
}

static void AnglePID_SetLimit(AnglePID_t *pid, float min, float max) {
  pid->out_min = min;
  pid->out_max = max;
}

static void AnglePID_Reset(AnglePID_t *pid) {
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->last2_error = 0.0f;
  pid->output = 0.0f;
}

/**
 * @brief 增量式角度 PID，自动处理 ±180° 环绕。
 * @param target  目标角度 (°)
 * @param current 当前 Yaw 角 (°)
 * @return PID 输出 (已限幅)
 */
static float AnglePID_Calc(AnglePID_t *pid, float target, float current) {
  /* 误差归一化到 (-180, 180] */
  float error = target - current;
  while (error > 180.0f)
    error -= 360.0f;
  while (error < -180.0f)
    error += 360.0f;

  pid->error = error;

  /* 增量式: Δu = Kp*(e-e?) + Ki*e + Kd*(e-2e?+e?) */
  pid->output += pid->Kp * (error - pid->last_error) + pid->Ki * error +
                 pid->Kd * (error - 2.0f * pid->last_error + pid->last2_error);

  pid->last2_error = pid->last_error;
  pid->last_error = error;

  /* 输出限幅 */
  if (pid->output > pid->out_max)
    pid->output = pid->out_max;
  if (pid->output < pid->out_min)
    pid->output = pid->out_min;

  return pid->output;
}

/*===========================================================================
 * 站点检测 — 灰度黑线去抖
 *===========================================================================*/

/**
 * @brief 检测 B 站点: 任一路灰度检测到黑线，连续 STATION_DEBOUNCE 次确认。
 * @return true=到站, false=未到
 */
static bool IsStation(void) {
  uint8_t sensors[GRAYSCALE_CHANNELS];
  Grayscale_Sensor_Read_All(sensors);

  bool any_black = false;
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++) {
    if (sensors[i]) {
      any_black = true;
      break;
    }
  }

  if (any_black) {
    station_count++;
    if (station_count >= STATION_DEBOUNCE) {
      station_count = 0;
      return true;
    }
  }
  /* 不重置: 单传感器短暂触发也能累加达标 */
  return false;
}

/*===========================================================================
 * Task 1 主逻辑
 *===========================================================================*/

void Task1_Run(void) {
  uint32_t start_tick;
  float start_yaw;
  int16_t left, right, steer;

  /* 初始化 PID */
  AnglePID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD);
  AnglePID_SetLimit(&pid_angle, (float)(-STEER_LIMIT), (float)STEER_LIMIT);
  AnglePID_Reset(&pid_angle);
  station_count = 0;

  /* OLED 提示 */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task1: A -> B", 12);

  /* 3-2-1 倒计时，给用户瞄准目标方向的时间 */
  for (uint8_t i = 3; i > 0; i--) {
    sprintf(oled_buf, "Start in %d...", i);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);
    mspm0_delay_ms(700);
  }
  OLED_Clear();
  OLED_ShowString(0, 2, (uint8_t *)"GO!", 16);
  mspm0_delay_ms(300);

  /* 显示任务标识 */
  OLED_ShowString(0, 0, (uint8_t *)"Task1", 8);

  /* 记录起始航向 (瞄准方向) */
  start_yaw = wit_data.yaw;
  start_tick = tick_ms;

  /* 主控制循环 */
  while (1) {
    /* 超时保护 (testflag=0 时生效) */
    if (!testflag && (tick_ms - start_tick) / 1000 >= TASK_TIMEOUT_S) {
      Motor_Stop();
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"Task1", 8);
      OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      mspm0_delay_ms(4000);
      task_flag = 0;
      return;
    }

    /* 站点检测 (testflag=0 时生效) */
    if (!testflag && IsStation()) {
      Motor_Stop();

      /* 蜂鸣器响 1 秒 */
      DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
      mspm0_delay_ms(1000);
      DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);

      /* OLED 显示到站信息 */
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"Task1", 8);
      sprintf(oled_buf, "Arrived! Y:%.1f", (double)wit_data.yaw);
      OLED_ShowString(0, 4, (uint8_t *)oled_buf, 8);
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      mspm0_delay_ms(4000);
      task_flag = 0;
      return;
    }

    /* 角度 PID 计算转向修正量 */
    steer = (int16_t)AnglePID_Calc(&pid_angle, start_yaw, wit_data.yaw);

    /* 差速合成: 车偏左(yaw↓)→error>0→steer>0→右轮加速修正 */
    left = BASE_SPEED - steer;
    right = BASE_SPEED + steer;

    /* 限幅到 PWM 范围 [0, 999] */
    if (left > 999)
      left = 999;
    if (left < 0)
      left = 0;
    if (right > 999)
      right = 999;
    if (right < 0)
      right = 0;

    Motor_SetSpeed(left, right);

    /* OLED 显示耗时 + Yaw + steer */
    sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
    sprintf(oled_buf, "Y:%6.1f S:%d", (double)wit_data.yaw, steer);
    OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

    mspm0_delay_ms(LOOP_DELAY_MS);
  }
}
