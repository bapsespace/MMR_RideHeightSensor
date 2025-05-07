/*
 * functions.h
 *
 *  Created on: 6 mag 2023
 *      Author: lucad
 */

#ifndef INC_FUNCTIONS_H_

#define INC_FUNCTIONS_H_

#include "stdint.h"
#include "stdbool.h"
#include "main.h"


uint8_t getI2CAddress(uint8_t address, uint8_t mode);
void scanForI2CDevices(I2C_HandleTypeDef* hi2c1, uint8_t* ret);
bool areDevicesReady(uint8_t* allAddresses);
uint8_t ADCStartSyncCommand(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber);
uint8_t ADCSoftwareReset(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber);
uint8_t ADCGetRegistersValue(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber, uint8_t regNumber, int16_t* regValue);
uint8_t ADCSetRegistersValue(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber, uint8_t regNumber, uint8_t regValue);
uint8_t ADCGetDataValue(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber, int32_t* regValue, uint8_t dataType);
void setLEDColor(uint8_t ledColor);
float getValueFromVoltageData(int32_t data);
float getValueFromTemperatureData(int32_t data);
float getValueFromTestVoltageData(int32_t data);
uint8_t linearInterpolation(float* yValues, uint8_t syValues, float* xValues, uint8_t sxValues, float x, float* res);
uint8_t getADCInternalTemperature(I2C_HandleTypeDef* hi2c1, TIM_HandleTypeDef* htim1, uint8_t adcNumber, float* dataValue, bool* timElapsedFlag);
uint8_t getADCReferenceVoltage(I2C_HandleTypeDef* hi2c1, TIM_HandleTypeDef* htim1, uint8_t adcNumber, float* dataValue, bool* timElapsedFlag);
uint8_t getADCChannelVoltage(I2C_HandleTypeDef* hi2c1, TIM_HandleTypeDef* htim1, uint8_t adcNumber, uint8_t dataChannel, float* dataValue, bool* timElapsedFlag);
uint16_t sortTemperatures(uint8_t index, float* temperatures, uint8_t* errors, int scale);

#endif /* INC_FUNCTIONS_H_ */
