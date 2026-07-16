/**
 * @file  task7.c
 * @brief Task7: 倒计时 → xunji_run() 循迹 → 显示结果
 *
 * OLED 显示 (循迹中):
 *   行0: Task7
 *   行2: Y:xxx.x (实时 yaw)
 *   行4: D:xxx.x (Δyaw)
 *   行6: 8 路灰度传感器状态
 *   行7: L:xxx R:xxx (左右 PWM)
 */

#include "task7.h"

#include "main.h"
#include "oled_software_i2c.h"
#include "ti_msp_dl_config.h"
#include "wit.h"
#include "xunji.h"

#include <stdio.h>

/* ===== 可调参数 ===== */
#define DONE_DELAY_MS 4000 /* 完成/超时后显示停留 (ms) */

/* ===== 模块级静态变量 ===== */
static char oled_buf[32];

/*----------------------------------------------------------------------
 * 循迹过程实时 OLED 刷新回调
 *----------------------------------------------------------------------*/
static void task7_display(float yaw, float delta_yaw, const uint8_t *sensors,
                          int16_t left, int16_t right) {
  static uint8_t skip = 0;
//  if (++skip < 1000)
  if (1)
    return;
  skip = 0;

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);

  sprintf(oled_buf, "Y:%6.1f", (double)yaw);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);

  sprintf(oled_buf, "D:%6.1f", (double)delta_yaw);
  OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);

  sprintf(oled_buf, "%d%d%d%d%d%d%d%d", sensors[0], sensors[1], sensors[2],
          sensors[3], sensors[4], sensors[5], sensors[6], sensors[7]);
  OLED_ShowString(0, 6, (uint8_t *)oled_buf, 8);

  sprintf(oled_buf, "L:%4d R:%4d", left, right);
  OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
}

/*----------------------------------------------------------------------
 * Task7 主逻辑
 *----------------------------------------------------------------------*/
void Task7_Run(void) {
  bool ok;
  uint32_t start_tick;

  /* ---- 3-2-1 倒计时 (显示 yaw) ---- */
  for (uint8_t i = 3; i > 0; i--) {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);
    sprintf(oled_buf, "Start in %d...", i);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 12);
    sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);
    mspm0_delay_ms(700);
  }

  /* ---- GO! ---- */
  OLED_Clear();
  OLED_ShowString(0, 2, (uint8_t *)"GO!", 16);
  mspm0_delay_ms(300);

  /* ---- 发车循迹 ---- */
  OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);
  start_tick = tick_ms;

  ok = xunji_run(task7_display);

  /* ---- 显示结果 ---- */
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Task7", 8);
  sprintf(oled_buf, "T:%4.1fs", (double)(tick_ms - start_tick) / 1000.0);
  OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);
  OLED_ShowString(0, 4, (uint8_t *)(ok ? "Done!" : "Timeout!"), 12);
  sprintf(oled_buf, "Y:%6.1f", (double)wit_data.yaw);
  OLED_ShowString(0, 6, (uint8_t *)oled_buf, 16);
  mspm0_delay_ms(DONE_DELAY_MS);

  task_flag = 0;
}
