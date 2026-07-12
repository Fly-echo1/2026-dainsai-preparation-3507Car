/**
 * @file  KEY.c
 * @brief 7-key driver implementation (active-low inputs with internal pull-ups).
 */

#include "KEY.h"

/*===========================================================================
 * Internal types
 *===========================================================================*/

/** Descriptor for a single key's GPIO */
typedef struct {
    GPIO_Regs         *port;   /**< GPIO port register base      */
    uint32_t           pin;    /**< GPIO pin mask                */
    IOMUX_PINCM        iomux;  /**< IOMUX pin-configuration index */
} key_pin_t;

/*===========================================================================
 * Pin lookup table — indexed by KEY_ID_t
 *===========================================================================*/

static const key_pin_t key_pins[KEY_COUNT] = {
    [KEY_1] = { .port = KEY1_PORT, .pin = KEY1_PIN, .iomux = KEY1_IOMUX },
    [KEY_2] = { .port = KEY2_PORT, .pin = KEY2_PIN, .iomux = KEY2_IOMUX },
    [KEY_3] = { .port = KEY3_PORT, .pin = KEY3_PIN, .iomux = KEY3_IOMUX },
    [KEY_4] = { .port = KEY4_PORT, .pin = KEY4_PIN, .iomux = KEY4_IOMUX },
    [KEY_5] = { .port = KEY5_PORT, .pin = KEY5_PIN, .iomux = KEY5_IOMUX },
    [KEY_6] = { .port = KEY6_PORT, .pin = KEY6_PIN, .iomux = KEY6_IOMUX },
    [KEY_7] = { .port = KEY7_PORT, .pin = KEY7_PIN, .iomux = KEY7_IOMUX },
};

/*===========================================================================
 * Public API
 *===========================================================================*/

/**
 * @brief Initialise all 7 key GPIOs as digital inputs with internal pull-ups.
 *
 * Each pin is configured:
 *  - Direction   : input
 *  - Resistor    : pull-up (so idle = HIGH, pressed = LOW)
 *  - Inversion   : disabled
 *  - Hysteresis  : disabled (digital I/O)
 *  - Wake-up     : disabled
 */
void KEY_Init(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        DL_GPIO_initDigitalInputFeatures(
            key_pins[i].iomux,
            DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR_PULL_UP,
            DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP_DISABLE);
    }
}

/**
 * @brief Read a single key state.
 *
 * @note  No de-bounce is performed here — handle in application if needed.
 *
 * @param key  Key ID (KEY_1 … KEY_7).
 * @return 1 = pressed (low level), 0 = released (high level).
 */
uint8_t KEY_Read(KEY_ID_t key)
{
    if (key >= KEY_COUNT) {
        return 0;
    }
    /* Active-low: pin reads 0 → pressed */
    return (DL_GPIO_readPins(key_pins[key].port, key_pins[key].pin) == 0) ? 1 : 0;
}

/**
 * @brief Read all 7 keys at once.
 *
 * Bit mapping: bit 0 = KEY_1, bit 1 = KEY_2, …, bit 6 = KEY_7.
 * A '1' in the bitmask means the corresponding key is pressed.
 *
 * @return 8-bit bitmask of pressed keys.
 */
uint8_t KEY_ReadAll(void)
{
    uint8_t i;
    uint8_t mask = 0;
    for (i = 0; i < KEY_COUNT; i++) {
        if (DL_GPIO_readPins(key_pins[i].port, key_pins[i].pin) == 0) {
            mask |= (1U << i);
        }
    }
    return mask;
}
