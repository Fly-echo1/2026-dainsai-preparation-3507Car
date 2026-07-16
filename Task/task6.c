/**
 * @file  task6.c
 * @brief Task6: 启动等3s → 指向179° (靠近±180°跳变边界)
 *        OLED 全程显示当前 yaw 角 + 左右 PWM
 *
 * 依赖: rotate(原地旋转模块) / OLED / WIT
 */

#include "task6.h"

#include "main.h"
#include "oled_software_i2c.h"
#include "rotate.h"
#include "ti_msp_dl_config.h"
#include "wit.h"

#include <stdio.h>

/* ===== 可调参数 ===== */
#define WAIT_S 3           /* 启动后等待时间 (秒)            */
#define TARGET_YAW 179     /* 目标 yaw 角                   */
#define DONE_DELAY_MS 4000 /* 完成/超时后显示停留 (ms)   */
#define OLED_SKIP 128      /* OLED 跳帧: 每 N 轮刷新一次  */

/* ===== 模块级静态变量 ===== */
static char oled_buf[32];

/* ===== 旋转过程实时 OLED 刷新 ===== */
static void rotate_display(float yaw, float target, int16_t left,
                           int16_t right) {
  static uint8_t skip = 0;
  if (++skip < OLED_SKIP)
    return;
  skip = 0;

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
  sprintf(oled_buf, "Y:%6.1f", (double)yaw);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
  sprintf(oled_buf, "-> %.0f", (double)target);
  OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);
  sprintf(oled_buf, "L:%4d R:%4d", left, right);
  OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
}

/*===========================================================================
 * Task6 主逻辑
 *===========================================================================*/

void Task6_Run(void) {
  bool ok;

  /* 初始化旋转模块 + 显示回调 (每轮 PID 循环刷 OLED) */
  {
    Rotate_Config_t cfg = {
        .Kp = ROTATE_DEFAULT_KP,
        .Ki = ROTATE_DEFAULT_KI,
        .Kd = ROTATE_DEFAULT_KD,
        .speed = ROTATE_DEFAULT_SPEED,
        .tolerance = ROTATE_DEFAULT_TOLERANCE,
        .timeout_ms = ROTATE_DEFAULT_TIMEOUT,
        .display_fn = rotate_display,
    };
    Rotate_Config(&cfg);
  }

  /* 3-2-1 倒计时, 期间显示 yaw */
  for (uint8_t i = 2; i > 0; i--) {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
    sprintf(oled_buf, "Start in %d...", i);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);
    sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);
    mspm0_delay_ms(700);
  }
  OLED_Clear();
  OLED_ShowString(0, 2, (uint8_t *)"GO!", 16);
  mspm0_delay_ms(300);

  /* 行0始终显示 Task6 */
  OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);

  /* ===== 阶段1: 179° 测试 (注释, 之后测) ===== */
  /*
  for (uint8_t i = WAIT_S; i > 0; i--) {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
    sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
    sprintf(oled_buf, "Wait %ds...", i);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);
    mspm0_delay_ms(1000);
  }

  ok = Rotate_To(TARGET_YAW);

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
  sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
  OLED_ShowString(0, 4, (uint8_t *)(ok ? "Done! 179" : "Timeout!"), 12);
  mspm0_delay_ms(DONE_DELAY_MS);
  */

  /* ===== 阶段2: 阶梯旋转 0→-30→-60→-90→-120→-180 ===== */
  {
    static const float targets[] = {-30, -60, -90, -120, -180};
    for (uint8_t i = 0; i < 5; i++) {
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
      sprintf(oled_buf, "-> %.0f", (double)targets[i]);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      sprintf(oled_buf, "Step %d/5", i + 1);
      OLED_ShowString(0, 4, (uint8_t *)oled_buf, 12);
      mspm0_delay_ms(500);

      ok = Rotate_To(targets[i]);

      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
      sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
      OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
      sprintf(oled_buf, "%s %.0f", ok ? "OK" : "TO", (double)targets[i]);
      OLED_ShowString(0, 4, (uint8_t *)oled_buf, 12);
      mspm0_delay_ms(800);
    }
  }

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
  OLED_ShowString(0, 4, (uint8_t *)"All Done!", 12);
  mspm0_delay_ms(DONE_DELAY_MS);
  task_flag = 0;
}
