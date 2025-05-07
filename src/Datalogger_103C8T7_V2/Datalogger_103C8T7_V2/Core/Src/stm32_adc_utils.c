/*
 * stm32_adc_utils.c
 *
 *  Created on: May 5, 2024
 *      Author: lucad
 */

#include "stm32_adc_utils.h"

float getInternalTemperatureF103(float voltage){
	return ((F103_OFFSET - voltage)*1000/F103_SLOPE + 25);
}

float getMaxVoltageF103(uint16_t adcRaw){
	return 4095 * F103_VREF/adcRaw;
}

uint8_t calibrateADC(ADC_HandleTypeDef* hadc){
	uint32_t startTick = HAL_GetTick();
	while (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK){
		if (HAL_GetTick() - startTick <= 1000){
			return ADC_UTILS_ERR_TIMEOUT;
		}
	}
	return ADC_UTILS_ERR_OK;
}
