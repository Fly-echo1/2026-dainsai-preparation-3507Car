#ifndef	__BSP_TRACK_H__
#define __BSP_TRACK_H__

#include "ti_msp_dl_config.h"
#define L3	!DL_GPIO_readPins(TRACK_S6_PORT, TRACK_S6_PIN)
#define L2	!DL_GPIO_readPins(TRACK_S1_PORT, TRACK_S1_PIN)
#define L1	!DL_GPIO_readPins(TRACK_S2_PORT, TRACK_S2_PIN)
#define M0	!DL_GPIO_readPins(TRACK_S3_PORT, TRACK_S3_PIN)
#define R1	!DL_GPIO_readPins(TRACK_S4_PORT, TRACK_S4_PIN)
#define R2	!DL_GPIO_readPins(TRACK_S5_PORT, TRACK_S5_PIN)
#define R3	!DL_GPIO_readPins(TRACK_S7_PORT, TRACK_S7_PIN)
void track_init(void);
int track_scan(void);
int track_control(void);

#endif
