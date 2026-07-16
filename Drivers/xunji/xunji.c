/**
 * @file  xunji.c
 * @brief 通用灰度循迹模块实现
 *
 * 控制策略:
 *   - 误差计算: 查表优先, 未匹配则加权质心兜底, 理想中心 4.5
 *   - PID: 增量式 PD, P 处理即时偏差、D 抑制震荡、I 累积弯道偏置
 *   - 脱线: 全白时清零积分和微分, 直行保持
 *   - 停止: |Δyaw| > XUNJI_YAW_THRESHOLD 且全白去抖 XUNJI_WHITE_DEBOUNCE 次
 */

#include "xunji.h"

#include "Grayscale.h"
#include "Motor.h"
#include "clock.h"
#include "ti_msp_dl_config.h"
#include "wit.h"

/*----------------------------------------------------------------------
 * 查表 + 加权质心混合误差计算
 *   传感器布局: s[0]=最左, s[7]=最右, 1=黑线, 0=白线
 *   理想中心: 3.5 (物理灰度4=s[3] 与灰度5=s[4] 之间)
 *   误差: 负=偏左(需右转), 正=偏右(需左转)
 *
 *   查表覆盖常见单/双/三传感器组合, 可直接修改返回值调参
 *   查表未匹配到的模式自动用加权质心兜底
 *----------------------------------------------------------------------*/
static float gray_error(const uint8_t *s) {
  /* ===== 查表: 在此修改偏差值 ===== */

  /* -- 单传感器 -- */
  if (s[0] && !s[1] && !s[2] && !s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -7.0f; // 10000000 灰度1
  if (!s[0] && s[1] && !s[2] && !s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -5.0f; // 01000000 灰度2
  if (!s[0] && !s[1] && s[2] && !s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -3.0f; // 00100000 灰度3
  if (!s[0] && !s[1] && !s[2] && s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -1.0f; // 00010000 灰度4
  if (!s[0] && !s[1] && !s[2] && !s[3] && s[4] && !s[5] && !s[6] && !s[7])
    return 1.0f; // 00001000 灰度5
  if (!s[0] && !s[1] && !s[2] && !s[3] && !s[4] && s[5] && !s[6] && !s[7])
    return 3.0f; // 00000100 灰度6
  if (!s[0] && !s[1] && !s[2] && !s[3] && !s[4] && !s[5] && s[6] && !s[7])
    return 5.0f; // 00000010 灰度7
  if (!s[0] && !s[1] && !s[2] && !s[3] && !s[4] && !s[5] && !s[6] && s[7])
    return 7.0f; // 00000001 灰度8

  /* -- 相邻双传感器 -- */
  if (s[0] && s[1] && !s[2] && !s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -7.0f; // 11000000 灰度1+2
  if (!s[0] && s[1] && s[2] && !s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -5.0f; // 01100000 灰度2+3
  if (!s[0] && !s[1] && s[2] && s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -3.0f; // 00110000 灰度3+4
  if (!s[0] && !s[1] && !s[2] && s[3] && s[4] && !s[5] && !s[6] && !s[7])
    return 0.0f; // 00011000 灰度4+5 理想
  if (!s[0] && !s[1] && !s[2] && !s[3] && s[4] && s[5] && !s[6] && !s[7])
    return 3.0f; // 00001100 灰度5+6
  if (!s[0] && !s[1] && !s[2] && !s[3] && !s[4] && s[5] && s[6] && !s[7])
    return 5.0f; // 00000110 灰度6+7
  if (!s[0] && !s[1] && !s[2] && !s[3] && !s[4] && !s[5] && s[6] && s[7])
    return 7.0f; // 00000011 灰度7+8

  /* -- 相邻三传感器 (宽黑线) -- */
  if (s[0] && s[1] && s[2] && !s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -7.0f; // 11100000 灰度1+2+3
  if (!s[0] && s[1] && s[2] && s[3] && !s[4] && !s[5] && !s[6] && !s[7])
    return -4.0f; // 01110000 灰度2+3+4
  if (!s[0] && !s[1] && s[2] && s[3] && s[4] && !s[5] && !s[6] && !s[7])
    return -1.0f; // 00111000 灰度3+4+5
  if (!s[0] && !s[1] && !s[2] && s[3] && s[4] && s[5] && !s[6] && !s[7])
    return 1.0f; // 00011100 灰度4+5+6
  if (!s[0] && !s[1] && !s[2] && !s[3] && s[4] && s[5] && s[6] && !s[7])
    return 4.0f; // 00001110 灰度5+6+7
  if (!s[0] && !s[1] && !s[2] && !s[3] && !s[4] && s[5] && s[6] && s[7])
    return 7.0f; // 00000111 灰度6+7+8

  /* ===== 兜底: 未匹配 → 加权质心 ===== */
  int16_t sum_pos = 0;
  uint8_t sum_cnt = 0;
  for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++) {
    if (s[i]) {
      sum_pos += (int16_t)i;
      sum_cnt++;
    }
  }
  if (sum_cnt == 0)
    return 0.0f;
  float center = (float)sum_pos / (float)sum_cnt;
  return (center - 3.5f) * 2.0f;
}

/*----------------------------------------------------------------------
 * 阻塞式循迹主循环
 *----------------------------------------------------------------------*/
bool xunji_run(xunji_display_fn display_fn) {
  uint32_t start_tick = tick_ms;
  float start_yaw = wit_data.yaw;
  uint8_t sensors[GRAYSCALE_CHANNELS];
  float integral = 0.0f;
  float prev_error = 0.0f;
  uint8_t white_count = 0;
  int16_t left, right;

  while (1) {
    /* ---- LED 周期翻转 (调试用) ---- */
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_LED1_PIN);

    /* ---- 超时保护 ---- */
    if ((tick_ms - start_tick) / 1000 >= XUNJI_TIMEOUT_S) {
      Motor_Stop();
      return false;
    }

    Grayscale_Sensor_Read_All(sensors);

    /* ---- 计算角度变化 (处理 ±180° 跳变) ---- */
    float delta_yaw = wit_data.yaw - start_yaw;
    if (delta_yaw > 180.0f)
      delta_yaw -= 360.0f;
    if (delta_yaw < -180.0f)
      delta_yaw += 360.0f;

    /* ---- 统计黑线数量 ---- */
    uint8_t black_cnt = 0;
    for (uint8_t i = 0; i < GRAYSCALE_CHANNELS; i++)
      if (sensors[i])
        black_cnt++;

    /* ---- 停止条件: |Δyaw| > 阈值 且 全白去抖 ---- */
    if (black_cnt == 0 &&
        (delta_yaw > XUNJI_YAW_THRESHOLD || delta_yaw < -XUNJI_YAW_THRESHOLD)) {
      white_count++;
      if (white_count >= XUNJI_WHITE_DEBOUNCE) {
        Motor_Stop();
        return true;
      }
    } else {
      white_count = 0;
    }

    /* ---- 循迹 PD 控制 ---- */
    if (black_cnt > 0) {
      float err = gray_error(sensors);
      integral += err;
      float derivative = err - prev_error;
      prev_error = err;

      int16_t diff = (int16_t)(XUNJI_KP * err + XUNJI_KI * integral +
                               XUNJI_KD * derivative);
      if (diff > XUNJI_DIFF_LIMIT)
        diff = XUNJI_DIFF_LIMIT;
      if (diff < -XUNJI_DIFF_LIMIT)
        diff = -XUNJI_DIFF_LIMIT;

      left = XUNJI_SPEED + diff;
      right = XUNJI_SPEED - diff;
    } else {
      /* 全白脱线: 清零积分和微分, 直行保持 */
      integral = 0.0f;
      prev_error = 0.0f;
      left = XUNJI_SPEED;
      right = XUNJI_SPEED;
    }

    /* ---- PWM 限幅 ---- */
    if (left > 999)
      left = 999;
    if (left < 0)
      left = 0;
    if (right > 999)
      right = 999;
    if (right < 0)
      right = 0;

    Motor_SetSpeed(left, right);

    /* ---- OLED 回调 ---- */
    if (display_fn)
      display_fn(wit_data.yaw, delta_yaw, sensors, left, right);

    mspm0_delay_ms(XUNJI_LOOP_DELAY_MS);
  }
}
