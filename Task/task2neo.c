/**
 * @file  task2neo.c
 * @brief Task2neo:
 *   阶段1 (Task1): 陀螺仪直行→B站停车 (蜂鸣+延时)
 *   阶段2 (弧线):  灰度PI巡线→C点(全白+yaw>140°)停车
 *
 * 依赖驱动: WIT(陀螺仪) / Motor(电机) / Grayscale(灰度) / OLED / BEEP
 */

#include "task2neo.h"

#include "Grayscale.h"
#include "Motor.h"
#include "clock.h"
#include "main.h"
#include "oled_software_i2c.h"
#include "ti_msp_dl_config.h"
#include "wit.h"

#include "rotate.h"

#include <stdbool.h>
#include <stdio.h>

/* ===== 可调参数 ===== */
#define BASE_SPEED 150    /* 直线基础速度 (0~999 PWM)               */
#define ARC_SPEED 130     /* 弧线巡线速度 (0~999 PWM)               */
#define TURN_SPEED 60     /* 阶段3调头速度 (0~999 PWM)              */
#define ARC_DIFF 28       /* 弧线基础差速 (4/5检测到时右转差速)     */
#define RECOVERY_DIFF 10  /* 1检测到时减弱右转的差速               */
#define RECOVERY_BOOST 20 /* 1检测到时加速增量                     */
#define ARC_KP 0.01f   /* 弧线 P 系数 (传感器偏移→差速修正)     */
#define ARC_KI 0.00015f   /* 弧线 I 系数                           */
#define STEER_LIMIT 350   /* 转向修正量限幅                         */

#define ANGLE_KP 9.0f /* 角度 PID 比例系数                      */
#define ANGLE_KI 0.8f /* 角度 PID 积分系数                      */
#define ANGLE_KD 0.5f /* 角度 PID 微分系数                      */

#define STATION_DEBOUNCE 2     /* 站点去抖次数                           */
#define LOOP_DELAY_MS 10       /* 控制循环周期 (ms)                      */
#define TASK1_TIMEOUT_S 40     /* 阶段1超时 (秒)                        */
#define TASK2_TIMEOUT_S 60     /* 阶段2超时 (秒)                        */
#define C_YAW_THRESHOLD 140.0f /* C点yaw阈值                          */
#define ROTATE_SPEED 100       /* 原地旋转速度 (0~999)                */
#define ROTATE_TOLERANCE 5.0f  /* 旋转到位容忍度 (°)                  */

/* ===== 角度 PID (同 Task1) ===== */
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

static uint8_t count_black(const uint8_t *sensors);
static int8_t find_first_black(const uint8_t *sensors);
static bool IsStation(const uint8_t *sensors, uint8_t *count);

/* ===== 模块级静态变量 ===== */
static AnglePID_t pid_angle;
static char oled_buf[32];

/*===========================================================================
 * 角度 PID — 增量式 + ±180° 环绕处理 (同 Task1)
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

/*===========================================================================
 * 灰度传感器辅助函数
 *===========================================================================*/

static uint8_t count_black(const uint8_t *sensors) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++)
    if (sensors[i])
      c++;
  return c;
}

static int8_t find_first_black(const uint8_t *sensors) {
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++)
    if (sensors[i])
      return (int8_t)i;
  return -1;
}

/**
 * @brief 站点检测: 任一路灰度检测到黑线，连续 STATION_DEBOUNCE 次确认。
 *        累加不重置，同 Task1 策略。
 */
static bool IsStation(const uint8_t *sensors, uint8_t *count) {
  bool any_black = false;
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++) {
    if (sensors[i]) {
      any_black = true;
      break;
    }
  }
  if (any_black) {
    (*count)++;
    if (*count >= STATION_DEBOUNCE) {
      *count = 0;
      return true;
    }
  }
  return false;
}

/*===========================================================================
 * Task2neo 主逻辑
 *===========================================================================*/

void Task2neo_Run(void) {
  uint32_t start_tick;
  float start_yaw, target_yaw;
  int16_t left, right, steer;
  uint8_t station_count;
  uint8_t sensors[GRAYSCALE_CHANNELS];
  int8_t black_idx;
  float integral;
  bool recovery;

  /* 初始化角度 PID */
  AnglePID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD);
  AnglePID_SetLimit(&pid_angle, (float)(-STEER_LIMIT), (float)STEER_LIMIT);
  AnglePID_Reset(&pid_angle);
  station_count = 0;

  /* OLED 提示 */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task2neo", 8);

  /* 3-2-1 倒计时 */
  for (uint8_t i = 3; i > 0; i--) {
    sprintf(oled_buf, "Start in %d...", i);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);
    mspm0_delay_ms(700);
  }
  OLED_Clear();
  OLED_ShowString(0, 2, (uint8_t *)"GO!", 16);
  mspm0_delay_ms(300);

  OLED_ShowString(0, 0, (uint8_t *)"T2neo", 8);

  /*==================================================================
   * 阶段1: 陀螺仪直行 (同 Task1) → B站停车
   *==================================================================*/
  OLED_ShowString(48, 0, (uint8_t *)"AB", 8);

  start_yaw = target_yaw = wit_data.yaw;
  start_tick = tick_ms;

  while (1) {
    /* 超时保护 */
    if ((tick_ms - start_tick) / 1000 >= TASK1_TIMEOUT_S) {
      Motor_Stop();
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"T2neo", 8);
      OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      mspm0_delay_ms(4000);
      task_flag = 0;
      return;
    }

    Grayscale_Sensor_Read_All(sensors);

    /* B站检测: 任一路黑线去抖 */
    if (IsStation(sensors, &station_count)) {
      Motor_Stop();

      /* 蜂鸣器响 1 秒 */
      DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
      mspm0_delay_ms(1000);
      DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);

      OLED_ShowString(48, 0, (uint8_t *)"B!", 8);
      sprintf(oled_buf, "Y:%.1f T:%.1f", (double)wit_data.yaw,
              (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 4, (uint8_t *)oled_buf, 8);
      break; /* 进入阶段2 */
    }

    /* 角度 PID 直行 */
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

    /* OLED 显示 */
    sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
    sprintf(oled_buf, "Y:%6.1f S:%d", (double)wit_data.yaw, steer);
    OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

    mspm0_delay_ms(LOOP_DELAY_MS);
  }

  /*==================================================================
   * 阶段2: 灰度PI弧线巡线 → C点停车
   *==================================================================*/
  OLED_ShowString(48, 0, (uint8_t *)"BC", 8);

  static const int8_t SENSOR_ERROR[8] = {
      -3, -2, -1, 0, 0, +1, +2, +3}; /* 传感器1 2 3 4 5 6 7 8 */

  start_tick = tick_ms;
  station_count = 0;
  integral = 0.0f;
  recovery = false;

  while (1) {
    /* 超时保护 */
    if ((tick_ms - start_tick) / 1000 >= TASK2_TIMEOUT_S) {
      Motor_Stop();
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"T2neo", 8);
      OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      mspm0_delay_ms(4000);
      task_flag = 0;
      return;
    }

    Grayscale_Sensor_Read_All(sensors);
    uint8_t black_cnt = count_black(sensors);

    /* C点: 全白 + |yaw|>140° → 停车进入阶段3 */
    if (black_cnt == 0 &&
        (wit_data.yaw > C_YAW_THRESHOLD || wit_data.yaw < -C_YAW_THRESHOLD)) {
      station_count++;
      if (station_count >= STATION_DEBOUNCE) {
        Motor_Stop();
        OLED_ShowString(48, 0, (uint8_t *)"C!", 8);
        sprintf(oled_buf, "Y:%.1f", (double)wit_data.yaw);
        OLED_ShowString(0, 4, (uint8_t *)oled_buf, 8);
        mspm0_delay_ms(500);
        break; /* 进入阶段3 */
      }
    } else {
      station_count = 0;
    }

    /* 恢复模式进出: 1触发进入, 4触发退出 */
    if (sensors[0])
      recovery = true;
    if (sensors[3])
      recovery = false;

    if (recovery) {
      int16_t spd = ARC_SPEED + RECOVERY_BOOST;
      if (spd > 999)
        spd = 999;
      left = spd + RECOVERY_DIFF;
      right = spd - RECOVERY_DIFF;
      integral = 0.0f;
    } else {
      black_idx = find_first_black(sensors);
      if (black_idx >= 0 && black_idx < GRAYSCALE_CHANNELS) {
        int8_t err = SENSOR_ERROR[(uint8_t)black_idx];
        integral += (float)err;
        int16_t diff =
            (int16_t)(ARC_DIFF + ARC_KP * (float)err + ARC_KI * integral);
        left = ARC_SPEED + diff;
        right = ARC_SPEED - diff;
      } else {
        integral = 0.0f;
        left = ARC_SPEED;
        right = ARC_SPEED;
      }
    }

    if (left > 999)
      left = 999;
    if (left < 0)
      left = 0;
    if (right > 999)
      right = 999;
    if (right < 0)
      right = 0;

    Motor_SetSpeed(left, right);

    /* OLED 显示 */
    sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
    sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
    OLED_ShowString(0, 6, (uint8_t *)oled_buf, 8);
    sprintf(oled_buf, "S:%d%d%d%d%d%d%d%d", sensors[0], sensors[1], sensors[2],
            sensors[3], sensors[4], sensors[5], sensors[6], sensors[7]);
    OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

    mspm0_delay_ms(LOOP_DELAY_MS);
  }

  /*==================================================================
   * 阶段3: 调头旋转 (开环/闭环由 task2neo.h 宏选择)
   *==================================================================*/
#if defined(TURN_MODE_CLOSED_LOOP)
  /* ----- 闭环PID调头 (使用 rotate 模块默认参数) ----- */
  OLED_ShowString(48, 0, (uint8_t *)"ROT", 8);

  Rotate_Init();
  Rotate_To(TURN_TARGET_YAW);

  OLED_ShowString(48, 0, (uint8_t *)"OK", 8);
  mspm0_delay_ms(200);

#elif defined(TURN_MODE_OPEN_LOOP)
  /* ----- 开环调头 ----- */
  OLED_ShowString(48, 0, (uint8_t *)"CT", 8);

  Motor_SetSpeed(TURN_SPEED - 15, TURN_SPEED + 15);
  mspm0_delay_ms(200);
  Motor_Stop();
  mspm0_delay_ms(200);

#else
  /* 默认: 开环调头 */
  OLED_ShowString(48, 0, (uint8_t *)"CT", 8);

  Motor_SetSpeed(TURN_SPEED - 15, TURN_SPEED + 15);
  mspm0_delay_ms(300);
  Motor_Stop();
  mspm0_delay_ms(200);
#endif

  /*==================================================================
   * 阶段4: 陀螺仪直行 (重复AB段)
   *==================================================================*/
  OLED_ShowString(48, 0, (uint8_t *)"AB2", 8);

  AnglePID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD);
  AnglePID_SetLimit(&pid_angle, (float)(-STEER_LIMIT), (float)STEER_LIMIT);
  target_yaw = wit_data.yaw;
  start_tick = tick_ms;
  station_count = 0;

  while (1) {
    /* 超时保护 */
    if ((tick_ms - start_tick) / 1000 >= TASK1_TIMEOUT_S) {
      Motor_Stop();
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"T2neo", 8);
      OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      mspm0_delay_ms(4000);
      task_flag = 0;
      return;
    }

    Grayscale_Sensor_Read_All(sensors);

    /* 到站检测 (再次B站) */
    if (IsStation(sensors, &station_count)) {
      Motor_Stop();

      DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
      mspm0_delay_ms(1000);
      DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);

      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"T2neo", 8);
      OLED_ShowString(0, 4, (uint8_t *)"Arrived!", 12);
      sprintf(oled_buf, "Y:%.1f T:%.1f", (double)wit_data.yaw,
              (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 8);
      mspm0_delay_ms(4000);
      task_flag = 0;
      return;
    }

    /* 角度 PID 直行 */
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

    /* OLED 显示 */
    sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
    sprintf(oled_buf, "Y:%6.1f S:%d", (double)wit_data.yaw, steer);
    OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

    mspm0_delay_ms(LOOP_DELAY_MS);
  }
}
