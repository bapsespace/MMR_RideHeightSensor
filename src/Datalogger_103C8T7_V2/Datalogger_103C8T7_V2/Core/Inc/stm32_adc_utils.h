/*
 * stm32_adc_utils.h
 *
 *  Created on: May 5, 2024
 *      Author: lucad
 */

#ifndef INC_STM32_ADC_UTILS_H_

#include "stdint.h"
#include "main.h"

#define F103_SLOPE 4.3f //slope in mV/°C
#define F103_OFFSET 1.43f //offset in V
#define F103_VREF 1.22f

#define ADC_UTILS_ERR_OK 0x00
#define ADC_UTILS_ERR_TIMEOUT 0xFF


float getInternalTemperatureF103(float voltage);
float getMaxVoltageF103(uint16_t adcRaw);
uint8_t calibrateADC(ADC_HandleTypeDef* hadc);

#endif /* INC_STM32_ADC_UTILS_H_ */
