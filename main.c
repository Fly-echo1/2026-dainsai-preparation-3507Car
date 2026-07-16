/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"
#include "Grayscale.h"
#include "Motor.h"
#include "Task/task1.h"
#include "Task/task2.h"
#include "Task/task2neo.h"
#include "Task/task3.h"
#include "Task/task4.h"
#include "Task/task6.h"
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"

uint8_t oled_buffer[32];
uint8_t Grayscale_buffer[8];
uint8_t sensor_values[GRAYSCALE_CHANNELS];  // 定义数组存储8路数据
char grayscale_str[GRAYSCALE_CHANNELS + 1]; // 用于OLED显示的字符串
volatile uint8_t task_flag = 0;             /* 任务选择: 0=停止, 1~4=Task */
void uart0_send_char(char ch);              // 串口0发送单个字符
void uart0_send_string(char *str);          // 串口0发送字符串

//向左转yaw+，向右转yaw-
int main(void) {
  SYSCFG_DL_init();
  SysTick_Init();

  // MPU6050_Init();
  OLED_Init();
  // Ultrasonic_Init();
  // BNO08X_Init();
  WIT_Init();
  Motor_Init();
  // VL53L0X_Init();
  // LSM6DSV16X_Init();
  // IMU660RB_Init();

  /* Don't remove this! */
  Interrupt_Init();
  DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
  delay_cycles(3200000);
  DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
  OLED_ShowString(0, 7, (uint8_t *)"WIT Demo", 8);

  OLED_ShowString(0, 0, (uint8_t *)"Pitch", 8);
  OLED_ShowString(0, 2, (uint8_t *)" Roll", 8);
  OLED_ShowString(0, 4, (uint8_t *)"  Yaw", 8);

  OLED_ShowString(16 * 6, 3, (uint8_t *)"Accel", 8);
  OLED_ShowString(17 * 6, 4, (uint8_t *)"Gyro", 8);

  // 模式选择
  task_flag = 6;
  // 模式选择
  while (1) {
    switch (task_flag) {
    case 1:
      Task1_Run();
      break;
    case 2:
      Task2_Run();
      break;
    case 3:
      Task3_Run();
      break;
    case 4:
      Task4_Run();
      break;
    case 5:
      Task2neo_Run();
      break;
    case 6:
      Task6_Run();
      break;
    default:
      /* WIT Demo 模式 */
      sprintf((char *)oled_buffer, "%-6.1f", wit_data.pitch);
      OLED_ShowString(5 * 8, 0, oled_buffer, 16);
      sprintf((char *)oled_buffer, "%-6.1f", wit_data.roll);
      OLED_ShowString(5 * 8, 2, oled_buffer, 16);
      sprintf((char *)oled_buffer, "%-6.1f", wit_data.yaw);
      OLED_ShowString(5 * 8, 4, oled_buffer, 16);

      sprintf((char *)oled_buffer, "%6d", wit_data.ax);
      OLED_ShowString(15 * 6, 0, oled_buffer, 8);
      sprintf((char *)oled_buffer, "%6d", wit_data.ay);
      OLED_ShowString(15 * 6, 1, oled_buffer, 8);
      sprintf((char *)oled_buffer, "%6d", wit_data.az);
      OLED_ShowString(15 * 6, 2, oled_buffer, 8);

      sprintf((char *)oled_buffer, "%6d", wit_data.gx);
      OLED_ShowString(15 * 6, 5, oled_buffer, 8);
      sprintf((char *)oled_buffer, "%6d", wit_data.gy);
      OLED_ShowString(15 * 6, 6, oled_buffer, 8);
      sprintf((char *)oled_buffer, "%6d", wit_data.gz);
      OLED_ShowString(15 * 6, 7, oled_buffer, 8);
      Grayscale_Sensor_Read_All(sensor_values);
      for (int i = 0; i < GRAYSCALE_CHANNELS; i++) {
        grayscale_str[i] = sensor_values[i] ? '1' : '0';
      }
      grayscale_str[GRAYSCALE_CHANNELS] = '\0';
      OLED_ShowString(0, 7, (uint8_t *)grayscale_str, 8);
      uart0_send_string(grayscale_str);
      uart0_send_string("\r\n");
      break;
    }
  }
}

// 串口发送单个字符
void uart0_send_char(char ch) {
  // 当串口0忙的时候等待，不忙的时候再发送传进来的字符
  while (DL_UART_isBusy(UART_BLUETEETH_INST) == true)
    ;
  // 发送单个字符
  DL_UART_Main_transmitData(UART_BLUETEETH_INST, ch);
}
// 串口发送字符串
void uart0_send_string(char *str) {
  // 当前字符串地址不在结尾 并且 字符串首地址不为空
  while (*str != 0 && str != 0) {
    // 发送字符串首地址中的字符，并且在发送完成之后首地址自增
    uart0_send_char(*str++);
  }
}
