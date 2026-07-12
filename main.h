#ifndef _MAIN_H_
#define _MAIN_H_

#include "clock.h"
#include "interrupt.h"

#include "bno08x_uart_rvc.h"
#include "imu660rb.h"
#include "lsm6dsv16x.h"
#include "mpu6050.h"
#include "oled_hardware_i2c.h"
#include "oled_hardware_spi.h"
#include "oled_software_i2c.h"
#include "oled_software_spi.h"
#include "ultrasonic_capture.h"
#include "ultrasonic_gpio.h"
#include "vl53l0x.h"
#include "wit.h"

#include <stdint.h>

/* 全局任务选择变量 (定义在 main.c) */
extern volatile uint8_t task_flag;

#endif /* #ifndef _MAIN_H_ */
