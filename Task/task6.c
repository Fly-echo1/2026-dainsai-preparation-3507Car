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

/* ===== 模块级静态变量 ===== */
static char oled_buf[32];

/* ===== 旋转过程实时 OLED 刷新 ===== */
static void rotate_display(float yaw, float target, int16_t left,
                           int16_t right) {
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
  for (uint8_t i = 3; i > 0; i--) {
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

  /* 启动后等 3s, 显示 yaw + 倒计时 */
  for (uint8_t i = WAIT_S; i > 0; i--) {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
    sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
    sprintf(oled_buf, "Wait %ds...", i);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);
    mspm0_delay_ms(1000);
  }

  /* 旋转到 179° */
  ok = Rotate_To(TARGET_YAW);

  /* 显示结果 */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task6", 8);
  sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
  OLED_ShowString(0, 4, (uint8_t *)(ok ? "Done! 179" : "Timeout!"), 12);
  mspm0_delay_ms(DONE_DELAY_MS);
  task_flag = 0;
}
