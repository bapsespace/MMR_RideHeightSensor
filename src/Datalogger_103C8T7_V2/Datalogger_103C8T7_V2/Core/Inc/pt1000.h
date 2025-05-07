/*
 * pt1000.h
 *
 *  Created on: May 6, 2024
 *      Author: lucad
 */

#ifndef INC_PT1000_H_
#define INC_PT1000_H_

#include "stdint.h"

#define PT1000_LEN 20

#define PT1000_INT_MODE 0x00
#define PT1000_EQN_MODE 0x01

#define PT1000_ERR_OK 0x00
#define PT1000_ERR_R_OVER 0xFF
#define PT1000_ERR_R_UNDER 0xFE
#define PT1000_ERR_MODE 0xFD
#define PT1000_ERR_INTERPOLATION 0xFC
#define PT1000_ERR_EXT_RES_MODE 0xFB
#define PT1000_ERR_EXT_RES_VALUE 0xFA
#define PT1000_ERR_ADC_RAW_ZER0 0xF9
#define PT1000_ERR_ADC_RAW_MAX 0xF8


#define PT1000_EXT_RES_PULLUP 0x00
#define PT1000_EXT_RES_PULLDOWN 0x01

#define PT1000_MAX_RES 1609.0f
#define PT1000_MIN_RES 924.0f

#define PT1000_EXT_RES_MAX 1000000
#define PT1000_EXT_RES_MIN 0

uint8_t pt1000_getTemperature(float extRes, uint8_t extResMode, uint8_t mode, uint16_t adcRaw, float* temperature);

#endif /* INC_PT1000_H_ */
