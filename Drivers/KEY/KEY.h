/**
 * @file  KEY.h
 * @brief 7-key driver — all keys are active-low (pressed = GND, released = pull-up to VCC).
 *
 * Pin definitions below must match your hardware connections.
 * Change the PORT / PIN / IOMUX macros per key as needed.
 */

#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Pin definitions — adjust to match your PCB
 *===========================================================================*/

/* KEY 1 : PA3 */
#define KEY1_PORT  (GPIOA)
#define KEY1_PIN   DL_GPIO_PIN_3
#define KEY1_IOMUX IOMUX_PINCM8

/* KEY 2 : PA4 */
#define KEY2_PORT  (GPIOA)
#define KEY2_PIN   DL_GPIO_PIN_4
#define KEY2_IOMUX IOMUX_PINCM9

/* KEY 3 : PA5 */
#define KEY3_PORT  (GPIOA)
#define KEY3_PIN   DL_GPIO_PIN_5
#define KEY3_IOMUX IOMUX_PINCM10

/* KEY 4 : PA6 */
#define KEY4_PORT  (GPIOA)
#define KEY4_PIN   DL_GPIO_PIN_6
#define KEY4_IOMUX IOMUX_PINCM11

/* KEY 5 : PA7 */
#define KEY5_PORT  (GPIOA)
#define KEY5_PIN   DL_GPIO_PIN_7
#define KEY5_IOMUX IOMUX_PINCM14

/* KEY 6 : PA12 */
#define KEY6_PORT  (GPIOA)
#define KEY6_PIN   DL_GPIO_PIN_12
#define KEY6_IOMUX IOMUX_PINCM34

/* KEY 7 : PA13 */
#define KEY7_PORT  (GPIOA)
#define KEY7_PIN   DL_GPIO_PIN_13
#define KEY7_IOMUX IOMUX_PINCM35

/*===========================================================================
 * Public types & API
 *===========================================================================*/

/** Number of keys */
#define KEY_COUNT 7

/** Key IDs */
typedef enum {
    KEY_1 = 0,
    KEY_2 = 1,
    KEY_3 = 2,
    KEY_4 = 3,
    KEY_5 = 4,
    KEY_6 = 5,
    KEY_7 = 6
} KEY_ID_t;

/**
 * @brief Initialise all 7 key GPIOs as digital inputs with internal pull-ups.
 * @note  Call once after SYSCFG_DL_init().
 */
void KEY_Init(void);

/**
 * @brief Read a single key state.
 * @param key  Key ID (KEY_1 … KEY_7).
 * @return 1 = pressed (low level), 0 = released.
 */
uint8_t KEY_Read(KEY_ID_t key);

/**
 * @brief Read all 7 keys at once.
 * @return 8-bit bitmask: bit i = 1 → KEY_(i+1) is pressed.
 */
uint8_t KEY_ReadAll(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */
