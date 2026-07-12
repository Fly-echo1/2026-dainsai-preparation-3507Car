#ifndef GRAYSCALE_H
#define GRAYSCALE_H
#define GRAYSCALE_CHANNELS 8
#include "ti_msp_dl_config.h"

// void Grayscale(Mat &img);
void Grayscale_Sensor_Read_All(uint8_t* sensor_values);
uint8_t Read_OUT_value(void);

#endif