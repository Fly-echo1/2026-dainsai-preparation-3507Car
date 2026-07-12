/**
 * @file  Motor.c
 * @brief TN6612 dual DC motor driver implementation.
 *
 * Hardware mapping (SysConfig):
 *  - Motor A direction : LEFT_1 (PA17, IN1),  LEFT_2 (PA18, IN2)
 *  - Motor A speed     : PWM_A  (TIMA0,  PB8, period = 1000)
 *  - Motor B direction : RIGHT_1(PB9,  IN1),  RIGHT_2(PA15, IN2)
 *  - Motor B speed     : PWM_B  (TIMG12, PA14, period = 1000)
 *
 * Control logic per channel:
 *  speed > 0 → IN1=H, IN2=L  (forward)
 *  speed < 0 → IN1=L, IN2=H  (reverse)
 *  speed = 0 → IN1=L, IN2=L  (coast stop)
 *
 * PWM compare value = |speed| (period = 1000 → 0.1 % duty resolution).
 */

#include "Motor.h"

/*===========================================================================
 * Internal helpers — direction control (static inline for zero overhead)
 *===========================================================================*/

/**
 * @brief Set Motor A H-bridge direction pins.
 */
static void MotorA_SetDirection(int16_t speed) {
  if (speed > 0) {
    /* Forward */
    DL_GPIO_setPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
  } else if (speed < 0) {
    /* Reverse */
    DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
  } else {
    /* Coast stop — both low */
    DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
  }
}

/**
 * @brief Set Motor B H-bridge direction pins.
 */
static void MotorB_SetDirection(int16_t speed) {
  if (speed > 0) {
    /* Forward */
    DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
  } else if (speed < 0) {
    /* Reverse */
    DL_GPIO_setPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
  } else {
    /* Coast stop — both low */
    DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
  }
}

/*===========================================================================
 * Public API
 *===========================================================================*/

/**
 * @brief Initialise motor driver.
 *
 * GPIO and PWM modules are already configured by SYSCFG_DL_init().
 * This function simply stops both motors for a safe power-on state.
 */
void Motor_Init(void) { Motor_Stop(); }

/**
 * @brief Set Motor A speed.
 *
 * @param speed  -1000 … +1000. Values outside this range are clamped.
 */
void MotorA_SetSpeed(int16_t speed) {
  uint16_t duty;

  /* Clamp */
  if (speed > MOTOR_SPEED_MAX) {
    speed = MOTOR_SPEED_MAX;
  } else if (speed < MOTOR_SPEED_MIN) {
    speed = MOTOR_SPEED_MIN;
  }

  /* Direction pins */
  MotorA_SetDirection(speed);

  /* PWM duty = |speed| (period = 1000, range 0 … 1000) */
  duty = (speed >= 0) ? (uint16_t)speed : (uint16_t)(-speed);
  DL_TimerA_setCaptureCompareValue(PWM_A_INST, duty, GPIO_PWM_A_C0_IDX);
}

/**
 * @brief Set Motor B speed.
 *
 * @param speed  -1000 … +1000. Values outside this range are clamped.
 */
void MotorB_SetSpeed(int16_t speed) {
  uint16_t duty;

  /* Clamp */
  if (speed > MOTOR_SPEED_MAX) {
    speed = MOTOR_SPEED_MAX;
  } else if (speed < MOTOR_SPEED_MIN) {
    speed = MOTOR_SPEED_MIN;
  }

  /* Direction pins */
  MotorB_SetDirection(speed);

  /* PWM duty = |speed| (period = 1000, range 0 … 1000) */
  duty = (speed >= 0) ? (uint16_t)speed : (uint16_t)(-speed);
  DL_TimerG_setCaptureCompareValue(PWM_B_INST, duty, GPIO_PWM_B_C0_IDX);
}

/**
 * @brief Set speed for both motors simultaneously.
 * @param speedA  Motor A speed, -999 … +999.
 * @param speedB  Motor B speed, -999 … +999.
 */
void Motor_SetSpeed(int16_t speedA, int16_t speedB) {
  MotorA_SetSpeed(speedA);
  MotorB_SetSpeed(speedB);
}

/**
 * @brief Stop both motors (coast — all H-bridge switches off).
 */
void Motor_Stop(void) {
  /* Motor A */
  DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
  DL_GPIO_clearPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
  DL_TimerA_setCaptureCompareValue(PWM_A_INST, 0, GPIO_PWM_A_C0_IDX);

  /* Motor B */
  DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
  DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
  DL_TimerG_setCaptureCompareValue(PWM_B_INST, 0, GPIO_PWM_B_C0_IDX);
}
