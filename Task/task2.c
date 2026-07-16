/**
 * @file  task2.c
 * @brief Task2: A→B→C→D→A 完整一圈 (xunji + rotate 重构版)
 *
 *   AB直行(角度PID, 目标0°) → B站 → BC弧线(xunji) → C站
 *   → C调头(Rotate_To 179°) → CD直行(角度PID, 目标179°) → D站
 *   → DA弧线(xunji) → A站 → 停车
 */

#include "task2.h"

#include "Grayscale.h"
#include "Motor.h"
#include "clock.h"
#include "main.h"
#include "oled_software_i2c.h"
#include "rotate.h"
#include "ti_msp_dl_config.h"
#include "wit.h"
#include "xunji.h"

#include <stdbool.h>
#include <stdio.h>

/* ===== 可调参数 ===== */
#define BASE_SPEED 150   /* 直线基础速度 (0~999 PWM)         */
#define STEER_LIMIT 350  /* 转向修正限幅                      */
#define LOOP_DELAY_MS 10 /* 控制循环周期 (ms)                 */
#define TIMEOUT_S 80     /* 全局超时 (秒)                     */
#define BEEP_MS 500      /* 蜂鸣时长 (ms)                     */

#define ANGLE_KP 9.0f /* 角度 PID P */
#define ANGLE_KI 0.8f /* 角度 PID I */
#define ANGLE_KD 0.5f /* 角度 PID D */

#define AB_TARGET 0.0f   /* AB直行目标 yaw */
#define CD_TARGET 179.0f /* CD直行目标 yaw */
#define ROTATE_TARGET 179.0f

#define STATION_DEBOUNCE 5 /* B/D 站黑线去抖次数 */

/* ===== 角度 PID (增量式 + ±180°环绕) ===== */
typedef struct {
  float Kp, Ki, Kd;
  float error, last_error, last2_error;
  float output;
  float out_min, out_max;
} AnglePID_t;

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

static float AnglePID_Calc(AnglePID_t *pid, float target, float current) {
  float error = target - current;
  while (error > 180.0f)
    error -= 360.0f;
  while (error < -180.0f)
    error += 360.0f;

  pid->error = error;
  pid->output += pid->Kp * (error - pid->last_error) + pid->Ki * error +
                 pid->Kd * (error - 2.0f * pid->last_error + pid->last2_error);

  pid->last2_error = pid->last_error;
  pid->last_error = error;

  if (pid->output > pid->out_max)
    pid->output = pid->out_max;
  if (pid->output < pid->out_min)
    pid->output = pid->out_min;

  return pid->output;
}

/* ===== 非阻塞蜂鸣 ===== */
static void beep_nonblock(uint32_t ms) {
  DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
  uint32_t start = tick_ms;
  while ((tick_ms - start) < ms)
    ;
  DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
}

/* ===== 到站检测: 连续 N 次黑线 ===== */
static bool IsStation(const uint8_t *sensors, uint8_t *count) {
  bool any = false;
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++)
    if (sensors[i]) {
      any = true;
      break;
    }
  if (any) {
    (*count)++;
    if (*count >= STATION_DEBOUNCE) {
      *count = 0;
      return true;
    }
  }
  return false;
}

/* ===== 模块级静态变量 ===== */
static AnglePID_t pid_angle;
static char oled_buf[32];

/*===========================================================================
 * Task2 主逻辑
 *===========================================================================*/

void Task2_Run(void) {
  uint32_t start_tick;
  uint8_t flagA = 1, flagB = 0, flagC = 0, flagD = 0;
  uint8_t station_count;
  uint8_t sensors[GRAYSCALE_CHANNELS];
  float target_yaw;
  int16_t left, right, steer;

  /* 初始化角度 PID */
  AnglePID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD);
  AnglePID_SetLimit(&pid_angle, (float)(-STEER_LIMIT), (float)STEER_LIMIT);

  /* OLED 提示 */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);

  /* 3-2-1 倒计时 */
  for (uint8_t i = 3; i > 0; i--) {
    sprintf(oled_buf, "Start in %d...", i);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);
    mspm0_delay_ms(700);
  }
  OLED_Clear();
  OLED_ShowString(0, 2, (uint8_t *)"GO!", 16);
  mspm0_delay_ms(300);
  OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);

  start_tick = tick_ms;

  /*==================================================================
   * 阶段1: AB 直行 → B 站
   *==================================================================*/
  {
    target_yaw = wit_data.yaw; // 锁定出发方向
    station_count = 0;
    OLED_ShowString(48, 0, (uint8_t *)"AB", 8);

    AnglePID_Reset(&pid_angle);
    while (!flagB) {
      /* 超时 */
      if ((tick_ms - start_tick) / 1000 >= TIMEOUT_S) {
        Motor_Stop();
        OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
        mspm0_delay_ms(4000);
        task_flag = 0;
        return;
      }

      Grayscale_Sensor_Read_All(sensors);

      /* B 站检测 */
      if (flagA && IsStation(sensors, &station_count)) {
        flagB = 1;
        OLED_ShowString(48, 0, (uint8_t *)"B!", 8);
        break;
      }

      steer = (int16_t)AnglePID_Calc(&pid_angle, target_yaw, wit_data.yaw);
      left = BASE_SPEED - steer;
      right = BASE_SPEED + steer;

      if (left > 999)
        left = 999;
      if (left < 0)
        left = 0;
      if (right > 999)
        right = 999;
      if (right < 0)
        right = 0;

      Motor_SetSpeed(left, right);

      /* OLED 跳帧: 每10轮刷一次, 避免阻塞 */
      {
        static uint8_t skip = 0;
        if (++skip >= 10) {
          skip = 0;
          sprintf(oled_buf, "T:%4.1fs",
                  (double)(tick_ms - start_tick) / 1000.0);
          OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
          sprintf(oled_buf, "Y:%6.1f S:%d", (double)wit_data.yaw, steer);
          OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
        }
      }

      mspm0_delay_ms(LOOP_DELAY_MS);
    }
  }

  /*==================================================================
   * 阶段2: BC 弧线 → C 站 (xunji)
   *==================================================================*/
  {
    OLED_ShowString(48, 0, (uint8_t *)"BC", 8);
    beep_nonblock(BEEP_MS);
    xunji_run(NULL);
    /* xunji 停 = C 站 */
    flagC = 1;
    OLED_ShowString(48, 0, (uint8_t *)"C!", 8);
  }

  /*==================================================================
   * 阶段3: C 调头 → Rotate_To(179°)
   *==================================================================*/
  {
    OLED_ShowString(48, 0, (uint8_t *)"ROT", 8);
    beep_nonblock(BEEP_MS);
    Rotate_Init();
    Rotate_To(ROTATE_TARGET);
    OLED_ShowString(48, 0, (uint8_t *)"OK", 8);
  }

  /*==================================================================
   * 阶段4: CD 直行 → D 站
   *==================================================================*/
  {
    target_yaw = CD_TARGET;
    station_count = 0;
    AnglePID_Reset(&pid_angle);
    OLED_ShowString(48, 0, (uint8_t *)"CD", 8);

    while (!flagD) {
      if ((tick_ms - start_tick) / 1000 >= TIMEOUT_S) {
        Motor_Stop();
        OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
        mspm0_delay_ms(4000);
        task_flag = 0;
        return;
      }

      Grayscale_Sensor_Read_All(sensors);

      /* D 站检测 (同 B) */
      if (flagC && IsStation(sensors, &station_count)) {
        flagD = 1;
        OLED_ShowString(48, 0, (uint8_t *)"D!", 8);
        break;
      }

      steer = (int16_t)AnglePID_Calc(&pid_angle, target_yaw, wit_data.yaw);
      left = BASE_SPEED - steer;
      right = BASE_SPEED + steer;

      if (left > 999)
        left = 999;
      if (left < 0)
        left = 0;
      if (right > 999)
        right = 999;
      if (right < 0)
        right = 0;

      Motor_SetSpeed(left, right);

      /* OLED 跳帧: 每10轮刷一次, 避免阻塞 */
      {
        static uint8_t skip = 0;
        if (++skip >= 10) {
          skip = 0;
          sprintf(oled_buf, "T:%4.1fs",
                  (double)(tick_ms - start_tick) / 1000.0);
          OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
          sprintf(oled_buf, "Y:%6.1f S:%d", (double)wit_data.yaw, steer);
          OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
        }
      }

      mspm0_delay_ms(LOOP_DELAY_MS);
    }
  }

  /*==================================================================
   * 阶段5: DA 弧线 → A 站 (xunji)
   *==================================================================*/
  {
    OLED_ShowString(48, 0, (uint8_t *)"DA", 8);
    beep_nonblock(BEEP_MS);
    xunji_run(NULL);
    /* xunji 停 = A 站, 全部 flag 已为 1 */
  }

  /*==================================================================
   * 到达 A, 结束
   *==================================================================*/
  Motor_Stop();
  beep_nonblock(BEEP_MS);

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);
  OLED_ShowString(0, 4, (uint8_t *)"Arrived!", 12);
  sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
  mspm0_delay_ms(4000);
  task_flag = 0;
}
