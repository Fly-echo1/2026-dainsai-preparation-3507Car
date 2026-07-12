/**
 * @file  Motor.h
 * @brief TN6612 dual DC motor driver (channels A / B).
 *
 * Speed range: -1000 … 0 … +1000.
 *  - Positive → forward, negative → reverse.
 *  - Direction is set via GPIO_MOTOR (LEFT_1/LEFT_2 for A, RIGHT_1/RIGHT_2 for
 * B).
 *  - PWM duty = |speed| / 1000  (PWM period is 1000, configured in SysConfig).
 *
 * GPIO / PWM peripherals are initialised by SYSCFG_DL_init(); Motor_Init() only
 * ensures both motors are stopped.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Macros
 *===========================================================================*/

#define MOTOR_SPEED_MAX 999    /**< Full-speed forward               */
#define MOTOR_SPEED_MIN (-999) /**< Full-speed reverse               */

/*===========================================================================
 * Public API
 *===========================================================================*/

/**
 * @brief Initialise motor driver.
 * @note  GPIO and PWM are already set up by SYSCFG_DL_init().
 *        This function stops both motors to ensure a safe starting state.
 */
void Motor_Init(void);

/**
 * @brief Set Motor A (left) speed.
 * @param speed  -1000 (full reverse) … 0 (stop) … +1000 (full forward).
 */
void MotorA_SetSpeed(int16_t speed);

/**
 * @brief Set Motor B (right) speed.
 * @param speed  -1000 (full reverse) … 0 (stop) … +1000 (full forward).
 */
void MotorB_SetSpeed(int16_t speed);

/**
 * @brief Set speed for both motors simultaneously.
 * @param speedA  Motor A speed, -999 … +999.
 * @param speedB  Motor B speed, -999 … +999.
 */
void Motor_SetSpeed(int16_t speedA, int16_t speedB);

/**
 * @brief Stop both motors immediately (coast stop).
 */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
