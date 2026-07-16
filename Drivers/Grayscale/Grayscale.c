#include "Grayscale.h"
#include "ti_msp_dl_config.h"

// #define GRAYSCALE_CHANNELS 8

static void _select_channel(uint8_t channel) // 0 - 7
{
  // 确保 channel 在有效范围内（可选，用于调试）
  if (channel > 7) {
    return;
  }

  switch (channel) {
  case 0:
    DL_GPIO_clearPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;

  case 1:
    DL_GPIO_setPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;

  case 2:
    DL_GPIO_clearPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;

  case 3:
    DL_GPIO_setPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;

  case 4:
    DL_GPIO_clearPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;

  case 5:
    DL_GPIO_setPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_clearPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;

  case 6:
    DL_GPIO_clearPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;

  case 7:
    DL_GPIO_setPins(GPIO_Grayscale_AD0_PORT, GPIO_Grayscale_AD0_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD1_PORT, GPIO_Grayscale_AD1_PIN);
    DL_GPIO_setPins(GPIO_Grayscale_AD2_PORT, GPIO_Grayscale_AD2_PIN);
    break;
  }
}

uint8_t Read_OUT_value(void) {
  if (DL_GPIO_readPins(GPIO_Grayscale_READ_PORT, GPIO_Grayscale_READ_PIN) > 0) {
    return 1;
  } else {
    return 0;
  }
}

void Grayscale_Sensor_Read_All(uint8_t *sensor_values) {
  uint8_t i;
  for (i = 0; i < GRAYSCALE_CHANNELS; i++) {
    _select_channel(i);
    sensor_values[i] = Read_OUT_value();
  }
}

/*
// static void _select_channel(uint8_t channel) // 0 - 7
// {
//     switch (channel)
//     {
//     case 0:
//         DL_GPIO_clearPins(GPIO_Grayscale_PORT, GPIO_Grayscale_AD0_PIN );
//         DL_GPIO_clearPins(GPIO_Grayscale_PORT, GPIO_Grayscale_AD1_PIN );
//         DL_GPIO_clearPins(GPIO_Grayscale_PORT, GPIO_Grayscale_AD2_PIN );
//         break;
//     case 1:

//         break;
//     case 2:

//         break;
//     case 3:

//         break;
//     case 4:
//     }
// }
*/
