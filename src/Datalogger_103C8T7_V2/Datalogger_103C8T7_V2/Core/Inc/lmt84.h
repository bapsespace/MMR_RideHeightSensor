/*
 * lmt84.h
 *
 *  Created on: May 3, 2024
 *      Author: lucad
 */

#ifndef INC_LMT84_H_

#include "stdint.h"

#define LMT84_MAX_VOLT 1.299f
#define LMT84_MIN_VOLT 0.183f

#define LMT84_LEN 41

#define LMT84_INT_MODE 0x00
#define LMT84_SQRT_MODE 0x01

#define LMT84_ERR_OK 0x00
#define LMT84_ERR_V_OVER 0xFF
#define LMT84_ERR_V_UNDER 0xFE
#define LMT84_ERR_MODE 0xFD
#define LMT84_ERR_INTERPOLATION 0xFC

float LMT84_getTemperature(uint8_t mode, float voltage, float* temp);
uint8_t LMT84_isVoltageValid(float voltage);


#endif /* INC_LMT84_H_ */
