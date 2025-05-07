/*
 * lmt84.c
 *
 *  Created on: May 4, 2024
 *      Author: lucad
 */

#include "lmt84.h"

float LMT84_getTemperature(uint8_t mode, float voltage, float* temp){
	*temp = 0;
	if (mode == LMT84_INT_MODE){
		if (voltage > LMT84_MAX_VOLT){
			return LMT84_ERR_V_OVER;
		} else if (voltage < LMT84_MIN_VOLT){
			return LMT84_ERR_V_UNDER;
		} else {
			float volts[LMT84_LEN] = {1.299, 1.273, 1.247, 1.221, 1.194, 1.168, 1.141, 1.114, 1.088, 1.061, 1.034, 1.007, 0.980, 0.952, 0.925,
							   0.898, 0.871, 0.843, 0.816, 0.788, 0.760, 0.732, 0.704, 0.676, 0.647, 0.619, 0.591, 0.562, 0.534, 0.505,
							   0.476, 0.448, 0.419, 0.390, 0.361, 0.332, 0.302, 0.273, 0.243, 0.213, 0.183};
			float temps[LMT84_LEN] = {	 -50,   -45,   -40,   -35,   -30,   -25,   -20,   -15,   -10,    -5,     0,     5,    10,    15,    20,
								  25,    30,    35,    40,    45,    50,    55,    60,    65,    70,    75,    80,    85,    90,    95,
								 100,   105,   110,   115,   120,   125,   130,   135,   140,   145,   150};
			if ((sizeof(volts)/sizeof(volts[0])) != (sizeof(temps)/sizeof(temps[0]))){
				return LMT84_ERR_INTERPOLATION;
			} else {
				uint8_t index = 0;
				for (uint8_t i = 0; i < LMT84_LEN; i++){
					if (volts[i] <= voltage) {
						index = i;
						break;
					}
				}
				if (index == 0){
					*temp = -50.0f;
				} else {
					*temp = temp[index-1] + (temp[index-1] - temp[index]) * (voltage - volts[index-1]) / (volts[index-1] - volts[index]);
				}
				return *temp;
			}
		}
	} else if (mode == LMT84_SQRT_MODE){
		#include <math.h>
		*temp = (5.506f - sqrtf(30.316036f + 0.00704f * (870.6f - voltage*1000)))/(-0.00352f) + 30;
		return *temp;
	} else {
		return LMT84_ERR_MODE;
	}
}

uint8_t LMT84_isVoltageValid(float voltage){
	if (voltage > LMT84_MAX_VOLT){
		return LMT84_ERR_V_OVER;
	} else if (voltage < LMT84_MIN_VOLT){
		return LMT84_ERR_V_UNDER;
	} else {
		return LMT84_ERR_OK;
	}
}

