/**
 * @file  task2.c
 * @brief Task 2 实现: A→B→C→D→A 完整一圈。
 *        flag=1: AB 直线 (陀螺仪 Yaw PID 保持航向，检测黑线到 B 站)
 *        flag=2: BC 弧线 (灰度查表 + 陀螺仪角速度 PID，检测全白到 C 站)
 *        flag=3: CD 直线 (反向航向 PID，检测右侧黑线到 D 站)
 *        flag=4: DA 弧线 (灰度查表 + 角速度 PID，检测全白到 A 站)
 *
 * 依赖驱动: WIT(陀螺仪) / Motor(电机) / Grayscale(灰度) / OLED / BEEP
 */

#include "task2.h"

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
#define BASE_SPEED 150    /* 直线基础速度 (0~999 PWM)               */
#define ARC_SPEED 100     /* 弧线巡线速度 (0~999 PWM)               */
#define ARC_DIFF 25       /* 弧线基础差速 (4/5检测到时右转差速)     */
#define RECOVERY_DIFF 10  /* 1检测到时减弱右转的差速               */
#define RECOVERY_BOOST 20 /* 1检测到时加速增量                     */
#define ARC_KP 0.00012f   /* 弧线 P 系数 (传感器偏移→差速修正)     */
#define ARC_KI 0.03f      /* 弧线 I 系数                           */
#define ARC_KD 0.0f       /* 弧线 D 系数 (暂为0)                    */
#define STEER_LIMIT 350   /* 转向修正量限幅                         */
#define OMEGA_LIMIT 100   /* 角速度 PID 输出限幅                    */

#define ANGLE_KP 9.0f /* 角度 PID 比例系数                      */
#define ANGLE_KI 0.8f /* 角度 PID 积分系数                      */
#define ANGLE_KD 0.5f /* 角度 PID 微分系数                      */

#define OMEGA_KP 1.0f  /* 角速度 PID 比例系数                    */
#define OMEGA_KI 0.02f /* 角速度 PID 积分系数                    */
#define OMEGA_KD 0.0f  /* 角速度 PID 微分系数 (仅P)              */

/* 站点检测: B/D 点黑线去抖 (累加不重置, 同 Task1 策略) */
#define STATION_DEBOUNCE_BLACK 4
/* 站点检测: C/A 点全白去抖 (累加不重置) */
#define STATION_DEBOUNCE_WHITE 150

#define LOOP_DELAY_MS 10  /* 控制循环周期 (ms)                      */
#define TASK_TIMEOUT_S 60 /* 超时时间 (秒)                          */

/* 脱线检测: 弧线段全白持续次数阈值 */
#define OFFTRACK_DEBOUNCE 5

/* ===== 角度 PID (同 Task1) ===== */
typedef struct {
  float Kp, Ki, Kd;
  float error, last_error, last2_error;
  float output;
  float out_min, out_max;
} AnglePID_t;

/* ===== 角速度 PID (标准增量式，无需 ±180° 环绕) ===== */
typedef struct {
  float Kp, Ki, Kd;
  float error, last_error, last2_error;
  float output;
  float out_min, out_max;
} OmegaPID_t;

/* ===== 内部函数声明 ===== */
static void AnglePID_Init(AnglePID_t *pid, float Kp, float Ki, float Kd);
static void AnglePID_SetLimit(AnglePID_t *pid, float min, float max);
static void AnglePID_Reset(AnglePID_t *pid);
static float AnglePID_Calc(AnglePID_t *pid, float target, float current);

static void OmegaPID_Init(OmegaPID_t *pid, float Kp, float Ki, float Kd);
static void OmegaPID_SetLimit(OmegaPID_t *pid, float min, float max);
static void OmegaPID_Reset(OmegaPID_t *pid);
static float OmegaPID_Calc(OmegaPID_t *pid, float error);

static uint8_t count_black(const uint8_t *sensors);
static uint8_t count_right_black(const uint8_t *sensors);
static int8_t find_first_black(const uint8_t *sensors);

/* ===== 模块级静态变量 ===== */
static AnglePID_t pid_angle;
static OmegaPID_t pid_omega;
static char oled_buf[32];
static volatile uint8_t testflag = 0; /* 1=仅圆弧循迹调参, 0=正常模式 */

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

/**
 * @brief 增量式角度 PID，自动处理 ±180° 环绕。
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
 * 角速度 PID — 标准增量式 (无需角度环绕)
 *===========================================================================*/

static void OmegaPID_Init(OmegaPID_t *pid, float Kp, float Ki, float Kd) {
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

static void OmegaPID_SetLimit(OmegaPID_t *pid, float min, float max) {
  pid->out_min = min;
  pid->out_max = max;
}

static void OmegaPID_Reset(OmegaPID_t *pid) {
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->last2_error = 0.0f;
  pid->output = 0.0f;
}

static float OmegaPID_Calc(OmegaPID_t *pid, float error) {
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
 * 灰度传感器辅助函数
 * 传感器1=索引0(AD0=AD1=AD2=0), 传感器8=索引7(AD0=AD1=AD2=1)
 *===========================================================================*/

/** @brief 统计黑线传感器数量 */
static uint8_t count_black(const uint8_t *sensors) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++)
    if (sensors[i])
      c++;
  return c;
}

/** @brief 统计右侧 4 路 (传感器1~4, 索引 0~3) 黑线数量 */
static uint8_t count_right_black(const uint8_t *sensors) {
  uint8_t c = 0;
  for (uint8_t i = 0; i <= 3; i++)
    if (sensors[i])
      c++;
  return c;
}

/** @brief 找到第一个黑线传感器索引 (从传感器1到传感器8)，未找到返回 -1 */
static int8_t find_first_black(const uint8_t *sensors) {
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++)
    if (sensors[i])
      return (int8_t)i;
  return -1;
}

/*===========================================================================
 * 灰度查表: 传感器索引 → 目标角速度 (°/s)
 * 传感器1=索引0, 传感器8=索引7
 * 正值=右转, 负值=左转
 *===========================================================================*/
static const float SENSOR_TO_OMEGA[GRAYSCALE_CHANNELS] = {
    -16.0f, /* 0: 传感器1 → 左转   (线在左边, 减轻右转) */
    -7.0f,  /* 1: 传感器2 → 左转                        */
    -3.0f,  /* 2: 传感器3 → 微左转                      */
    -3.0f,  /* 3: 传感器4 → 稍左转                      */
    3.0f,   /* 4: 传感器5 → 稍右转                      */
    5.0f,   /* 5: 传感器6 → 微右转                      */
    10.0f,  /* 6: 传感器7 → 右转                        */
    20.0f,  /* 7: 传感器8 → 快速右转 (线在右边, 加强右转) */
};

/*===========================================================================
 * Task 2 主逻辑
 *===========================================================================*/

void Task2_Run(void) {
  uint32_t start_tick;
  float start_yaw, target_yaw;
  int16_t left, right, steer, omega_out;
  uint8_t flag;
  uint8_t station_count;
  uint8_t sensors[GRAYSCALE_CHANNELS];
  int8_t black_idx;
  float target_omega;
  uint8_t offtrack_count;

  /* 初始化 PID */
  AnglePID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD);
  AnglePID_SetLimit(&pid_angle, (float)(-STEER_LIMIT), (float)STEER_LIMIT);

  OmegaPID_Init(&pid_omega, OMEGA_KP, OMEGA_KI, OMEGA_KD);
  OmegaPID_SetLimit(&pid_omega, (float)(-OMEGA_LIMIT), (float)OMEGA_LIMIT);

  /* testflag=1: 仅圆弧循迹调参, 跳过AB直线, 无超时/站点检测 */
  if (testflag) {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task2 Tune", 8);
    OLED_ShowString(0, 2, (uint8_t *)"Arc Only", 12);
    mspm0_delay_ms(500);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);
    OLED_ShowString(48, 0, (uint8_t *)"BC", 8);

    start_tick = tick_ms;
    offtrack_count = 0;
    float integral = 0.0f; /* I项积分累加 */
    bool recovery = false; /* 传感器1触发时的恢复模式 */
    static const int8_t SENSOR_ERROR[8] = {
        -3, -2, -1, 0, 0, +1, +2, +3}; /* 传感器1 2 3 4 5 6 7 8 */

    while (1) {
      Grayscale_Sensor_Read_All(sensors);
      uint8_t black_cnt = count_black(sensors);

      /* 脱线检测: 全白持续 → 半速直行 */
      if (black_cnt == 0) {
        offtrack_count++;
        if (offtrack_count >= OFFTRACK_DEBOUNCE) {
          int16_t slow = ARC_SPEED / 2;
          if (slow < 20)
            slow = 20;
          Motor_SetSpeed(slow, slow);
          OLED_ShowString(0, 7, (uint8_t *)"OffTrack", 8);
          mspm0_delay_ms(LOOP_DELAY_MS);
          continue;
        }
      } else {
        offtrack_count = 0;
      }

      /* 恢复模式进出: 1触发进入, 4触发退出 */
      if (sensors[0])
        recovery = true;
      if (sensors[3])
        recovery = false;

      if (recovery) {
        /* 传感器1触发: 减轻右转 + 加速, 把线拉回中心 */
        int16_t spd = ARC_SPEED + RECOVERY_BOOST;
        if (spd > 999)
          spd = 999;
        left = spd + RECOVERY_DIFF;
        right = spd - RECOVERY_DIFF;
        integral = 0.0f; /* 重置积分 */
      } else {
        /* 正常PI控制 */
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

      /* OLED: 耗时 + 传感器状态 + L/R */
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      sprintf(oled_buf, "S:%d%d%d%d%d%d%d%d L%dR%d", sensors[0], sensors[1],
              sensors[2], sensors[3], sensors[4], sensors[5], sensors[6],
              sensors[7], left, right);
      OLED_ShowString(0, 6, (uint8_t *)oled_buf, 8);
      if (black_idx >= 0) {
        sprintf(oled_buf, "%s e:%d I:%.0f", recovery ? "R" : "N",
                SENSOR_ERROR[(uint8_t)black_idx], integral);
        OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
      }

      mspm0_delay_ms(LOOP_DELAY_MS);
    }
  }

  /* OLED 提示 */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task2: 1 Lap", 12);

  /* 3-2-1 倒计时 */
  for (uint8_t i = 3; i > 0; i--) {
    sprintf(oled_buf, "Start in %d...", i);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);
    mspm0_delay_ms(700);
  }
  OLED_Clear();
  OLED_ShowString(0, 2, (uint8_t *)"GO!", 16);
  mspm0_delay_ms(300);

  /* 显示任务标识 */
  OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);

  /* 记录起始航向 (瞄准方向, 即 A→B 方向 0°) */
  start_yaw = wit_data.yaw;
  start_tick = tick_ms;

  /*==================================================================
   * flag=1: AB 直线 → B 站
   *   阶段1: 陀螺仪 Yaw PID 直行, 任一路黑线消抖 → 蜂鸣, 不停车
   *   阶段2: 继续直行, 等 P4(索引3) 检测到黑线 → 切换弧线模式
   *==================================================================*/
  {
    flag = 1;
    station_count = 0;
    bool b_confirmed = false; /* B站消抖已确认, 等待摆正 */
    uint32_t beep_start = 0;  /* 蜂鸣开始时刻 */
    AnglePID_Reset(&pid_angle);
    target_yaw = start_yaw;

    OLED_ShowString(48, 0, (uint8_t *)"AB", 8);

    while (1) {
      /* 超时保护 */
      if ((tick_ms - start_tick) / 1000 >= TASK_TIMEOUT_S) {
        Motor_Stop();
        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);
        OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
        sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
        OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
        mspm0_delay_ms(4000);
        task_flag = 0;
        return;
      }

      Grayscale_Sensor_Read_All(sensors);

      /* 蜂鸣器非阻塞关闭 (每循环检查, 不卡控制) */
      if (beep_start && (tick_ms - beep_start) >= 300) {
        DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
        beep_start = 0;
      }

      if (!b_confirmed) {
        /* 阶段1: 任一路黑线 → B 站消抖, 蜂鸣后继续直行 */
        if (count_black(sensors) > 0) {
          station_count++;
          if (station_count >= STATION_DEBOUNCE_BLACK) {
            station_count = 0;
            b_confirmed = true;
            /* 蜂鸣器非阻塞: 记录时刻, 设GPIO后继续循环 */
            beep_start = tick_ms;
            DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
            OLED_ShowString(48, 0, (uint8_t *)"B!", 8);
            /* 不停车, 继续直行等传感器4~8检测到黑线 */
          }
        }
        /* 不重置: 同 Task1 策略 */
      } else {
        /* 阶段2: B已确认, 等传感器4~8(索引3~7)检测到黑线 → 切换弧线 */
        if (sensors[3] || sensors[4] || sensors[5] || sensors[6] ||
            sensors[7]) {
          station_count++;
          if (station_count >= STATION_DEBOUNCE_BLACK) {
            DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
            break; /* 车已摆正, 进入 BC 弧线段 */
          }
        }
        /* 不重置: 累加到 4~8 达标 */
      }

      /* 角度 PID */
      steer = (int16_t)AnglePID_Calc(&pid_angle, target_yaw, wit_data.yaw);

      /* 差速合成 */
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

      /* OLED: 耗时 + Yaw + steer */
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      sprintf(oled_buf, "Y:%6.1f S:%d", (double)wit_data.yaw, steer);
      OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

      mspm0_delay_ms(LOOP_DELAY_MS);
    }
  }

  /*==================================================================
   * flag=2: BC 弧线 → C 站
   *   策略: PI+恢复模式 差速巡线
   *   站点: 全部 8 路白线 (全白) → C 站
   *==================================================================*/
  {
    flag = 2;
    station_count = 0;
    offtrack_count = 0;
    float integral = 0.0f;
    bool recovery = false;
    static const int8_t SENSOR_ERROR[8] = {-3, -2, -1, 0, 0, +1, +2, +3};

    OLED_ShowString(48, 0, (uint8_t *)"BC", 8);

    while (1) {
      /* 超时保护 */
      if ((tick_ms - start_tick) / 1000 >= TASK_TIMEOUT_S) {
        Motor_Stop();
        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);
        OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
        sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
        OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
        mspm0_delay_ms(4000);
        task_flag = 0;
        return;
      }

      /* 读取灰度 */
      Grayscale_Sensor_Read_All(sensors);
      uint8_t black_cnt = count_black(sensors);

      /* 站点检测: 全白 → C 站 */
      if (black_cnt == 0) {
        station_count++;
        if (station_count >= STATION_DEBOUNCE_WHITE) {
          station_count = 0;
          Motor_Stop();
          DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
          mspm0_delay_ms(300);
          DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
          OLED_ShowString(48, 0, (uint8_t *)"C!", 8);
          mspm0_delay_ms(500);
          break;
        }
      } else {
        station_count = 0;
      }

      /* 丢线计数 */
      if (black_cnt == 0) {
        offtrack_count++;
      } else {
        offtrack_count = 0;
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

      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      sprintf(oled_buf, "%s L%dR%d", recovery ? "R" : "N", left, right);
      OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

      mspm0_delay_ms(LOOP_DELAY_MS);
    }
  }

  /*==================================================================
   * flag=3: CD 直线 → D 站
   *   策略: 陀螺仪 Yaw PID 保持反向航向 (start_yaw+180°),
   *         右侧 4 路黑线 → D 站
   *==================================================================*/
  {
    flag = 3;
    station_count = 0;
    AnglePID_Reset(&pid_angle);
    target_yaw = start_yaw + 180.0f;
    /* 归一化 */
    while (target_yaw > 180.0f)
      target_yaw -= 360.0f;
    while (target_yaw < -180.0f)
      target_yaw += 360.0f;

    OLED_ShowString(48, 0, (uint8_t *)"CD", 8);

    while (1) {
      /* 超时保护 */
      if ((tick_ms - start_tick) / 1000 >= TASK_TIMEOUT_S) {
        Motor_Stop();
        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);
        OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
        sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
        OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
        mspm0_delay_ms(4000);
        task_flag = 0;
        return;
      }

      /* 站点检测: 右侧 4 路黑线 → D 站 */
      Grayscale_Sensor_Read_All(sensors);
      if (count_right_black(sensors) > 0) {
        station_count++;
        if (station_count >= STATION_DEBOUNCE_BLACK) {
          station_count = 0;
          Motor_Stop();
          DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
          mspm0_delay_ms(300);
          DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
          OLED_ShowString(48, 0, (uint8_t *)"D!", 8);
          mspm0_delay_ms(500);
          break;
        }
      }

      /* 角度 PID */
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

      /* OLED */
      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      sprintf(oled_buf, "Y:%6.1f S:%d", (double)wit_data.yaw, steer);
      OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

      mspm0_delay_ms(LOOP_DELAY_MS);
    }
  }

  /*==================================================================
   * flag=4: DA 弧线 → A 站
   *   策略: PI+恢复模式 差速巡线
   *   站点: 全部 8 路白线 (全白) → A 站 → 停车
   *==================================================================*/
  {
    flag = 4;
    station_count = 0;
    offtrack_count = 0;
    float integral = 0.0f;
    bool recovery = false;
    static const int8_t SENSOR_ERROR[8] = {-3, -2, -1, 0, 0, +1, +2, +3};

    OLED_ShowString(48, 0, (uint8_t *)"DA", 8);

    while (1) {
      /* 超时保护 */
      if ((tick_ms - start_tick) / 1000 >= TASK_TIMEOUT_S) {
        Motor_Stop();
        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);
        OLED_ShowString(0, 4, (uint8_t *)"Timeout!", 12);
        sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
        OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
        mspm0_delay_ms(4000);
        task_flag = 0;
        return;
      }

      /* 读取灰度 */
      Grayscale_Sensor_Read_All(sensors);
      uint8_t black_cnt = count_black(sensors);

      /* 站点检测: 全白 → A 站 (终点) */
      if (black_cnt == 0) {
        station_count++;
        if (station_count >= STATION_DEBOUNCE_WHITE) {
          station_count = 0;
          Motor_Stop();

          /* 蜂鸣器响 1 秒 (到站提示) */
          DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
          mspm0_delay_ms(1000);
          DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);

          /* OLED 显示到站信息 */
          OLED_Clear();
          OLED_ShowString(0, 0, (uint8_t *)"Task2", 8);
          OLED_ShowString(0, 4, (uint8_t *)"Lap Done!", 12);
          sprintf(oled_buf, "T:%4.1fs",
                  (double)(tick_ms - start_tick) / 1000.0);
          OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
          mspm0_delay_ms(4000);
          task_flag = 0;
          return;
        }
      } else {
        station_count = 0;
      }

      /* 丢线计数 */
      if (black_cnt == 0) {
        offtrack_count++;
      } else {
        offtrack_count = 0;
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

      sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      sprintf(oled_buf, "%s L%dR%d", recovery ? "R" : "N", left, right);
      OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);

      mspm0_delay_ms(LOOP_DELAY_MS);
    }
  }
}
