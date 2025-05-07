/*
 * pt1000.c
 *
 *  Created on: May 6, 2024
 *      Author: lucad
 */

#include "pt1000.h"

uint8_t pt1000_getTemperature(float extRes, uint8_t extResMode, uint8_t mode, uint16_t adcRaw, float* temperature){
	float ptRes = 0;
	if ((extRes < PT1000_EXT_RES_MIN) || (extRes > PT1000_EXT_RES_MAX)){
		return PT1000_ERR_EXT_RES_VALUE;
	} else {
		if (extResMode == PT1000_EXT_RES_PULLDOWN){
			if (adcRaw <= 10){
				*temperature = -10.0f;
				return PT1000_ERR_ADC_RAW_ZER0;
			} else {
				ptRes = extRes * (4095.0f/adcRaw - 1);
			}
		} else if (extResMode == PT1000_EXT_RES_PULLUP){
			if (adcRaw >= 4085){
				*temperature = -10.0f;
				return PT1000_ERR_ADC_RAW_MAX;
			} else {
				ptRes = extRes * adcRaw / (4095.0f - adcRaw);
			}
		} else {
			*temperature = -10.0f;
			return PT1000_ERR_EXT_RES_MODE;
		}
	}
	if (ptRes < PT1000_MIN_RES){
		*temperature = -10.0f;
		return PT1000_ERR_R_UNDER;
	} else if (ptRes > PT1000_MAX_RES){
		*temperature = -10.0f;
		return PT1000_ERR_R_OVER;
	} else {
		if (mode == PT1000_INT_MODE) {
			float res[PT1000_LEN] = {  924,  962, 1000, 1038, 1076, 1095, 1114, 1152, 1190, 1228,
									  1266, 1304, 1342, 1380, 1419, 1457, 1495, 1533, 1571, 1609};
			float temps[PT1000_LEN] = {-20,  -10,    0,   10,   20,   25,   30,   40,   50,   60,
										70,   80,   90,  100,  110,  120,  130,  140,  150,  160};
			if ((sizeof(res)/sizeof(res[0])) != (sizeof(temps)/sizeof(temps[0]))){
				*temperature = -10.0f;
				return PT1000_ERR_INTERPOLATION;
			} else {
				uint8_t index = 0;
				for (uint8_t i = 0; i < PT1000_LEN; i++){
					if (res[i] < ptRes) {
						index = i;
						break;
					}
				}
				if (index == 0){
					*temperature = -20.0f;
				} else {
					*temperature = temps[index-1] + (temps[index-1] - temps[index]) * (ptRes - res[index-1]) / (res[index-1] - res[index]);
				}
				return PT1000_ERR_OK;
			}
		} else if (mode == PT1000_EQN_MODE){
			*temperature = (ptRes - 1000)/3.8f;
			return PT1000_ERR_OK;
		} else {
			*temperature = -10.0f;
			return PT1000_ERR_MODE;
		}
	}
}
