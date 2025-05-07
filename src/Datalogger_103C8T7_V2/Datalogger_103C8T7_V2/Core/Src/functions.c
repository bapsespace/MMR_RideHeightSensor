/*
 * functions.c
 *
 *  Created on: 6 mag 2023
 *      Author: lucad
 */

#include "functions.h"
#include "constants.h"


uint8_t getI2CAddress(uint8_t address, uint8_t mode){
	// Left shift to retrieve address in i2c format and add R/W flag
	// I2C_ADDR_ERR -> 0xFF
	// I2C_MODE_ERR -> 0xFE

	if (address > 128){
		return I2C_ADDR_ERR;
	} else {
		if (mode > 2){
			return I2C_MODE_ERR;
		} else {
			return ((address << 1) + mode);
		}
	}
}

void scanForI2CDevices(I2C_HandleTypeDef* hi2c1, uint8_t* ret){
	// Returns the status for all devices connected to i2c bus with the HAL_StatusTypeDef convention
	// HAL_OK -> 0
	// HAL_ERROR -> 1
	// HAL_BUSY -> 2
	// HAL_TIMEOUT -> 3

	for (uint8_t i = 0; i < 128; i++){
		ret[i] = HAL_I2C_IsDeviceReady(hi2c1, (uint16_t) getI2CAddress(i, I2C_MODE_WRITE), 3, HAL_MAX_DELAY);
	}
}

bool areDevicesReady(uint8_t* allAddresses){
	// Returns true if all devices with address in allAddresses array are ready

	uint8_t I2CDevices[I2C_DEVICE_NUMBER] = {ADC1_ADDR, ADC2_ADDR};
	bool allDevicesOk = true;
	for (uint8_t i = 0; i < I2C_DEVICE_NUMBER; i++){
		if (allAddresses[I2CDevices[i]] != HAL_OK){
			allDevicesOk = false;
		}
	}
	return allDevicesOk;
}

uint8_t ADCSoftwareReset(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber){
	// Software reset the ADC, returning following flags
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01

	// Return error if ADC number is not 1 or 2
	if (!((adcNumber == 1) || (adcNumber == 2))){
		return I2C_ADC_WRONG_NUMBER;
	}
	// Get address of relative ADC and in case of error retrieve an error flag
	uint8_t adcAddress;
	if (adcNumber == 1){
		adcAddress = getI2CAddress(ADC1_ADDR, I2C_MODE_WRITE);
		if (adcAddress == I2C_ADDR_ERR){
			return I2C_ADC_WRONG_ADDRESS;
		}
	} else {
		adcAddress = getI2CAddress(ADC2_ADDR, I2C_MODE_WRITE);
		if (adcAddress == I2C_ADDR_ERR){
			return I2C_ADC_WRONG_ADDRESS;
		}
	}
	//Send reset command to ADC and in case of error retrieve the relative flag
	uint8_t addr = I2C_ADC_RESET_COMMAND;
	uint8_t ret = HAL_I2C_Master_Transmit(hi2c1, (uint16_t) adcAddress, &addr, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		return I2C_ADC_TRANSMIT_ERROR;
	} else {
		HAL_Delay(1);
		return I2C_ADC_OK;
	}
}

uint8_t ADCStartSyncCommand(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber){
	// Send to the ADC the START/SYNC command to start synchronous and asynchronous conversion
	// Send this command to start the conversion before reading the data
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01

	// Return error if ADC number is not 1 or 2
	if (!((adcNumber == 1) || (adcNumber == 2))){
		return I2C_ADC_WRONG_NUMBER;
	}
	// Get address of relative ADC and in case of error retrieve an error flag
	uint8_t adcAddress;
	if (adcNumber == 1){
		adcAddress = getI2CAddress(ADC1_ADDR, I2C_MODE_WRITE);
		if (adcAddress == I2C_ADDR_ERR){
			return I2C_ADC_WRONG_ADDRESS;
		}
	} else {
		adcAddress = getI2CAddress(ADC2_ADDR, I2C_MODE_WRITE);
		if (adcAddress == I2C_ADDR_ERR){
			return I2C_ADC_WRONG_ADDRESS;
		}
	}
	//Send reset command to ADC and in case of error retrieve the relative flag
	uint8_t addr = I2C_ADC_START_SYNC_COMMAND;
	uint8_t ret = HAL_I2C_Master_Transmit(hi2c1, (uint16_t) adcAddress, &addr, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		return I2C_ADC_TRANSMIT_ERROR;
	} else {
		return I2C_ADC_OK;
	}
}

uint8_t ADCGetRegistersValue(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber, uint8_t regNumber, int16_t* regValue){
	// Send to the ADC the target register value to read the working parameters of the device
	// Use this command to read ADC register
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01
	// I2C_ADC_RECEIVE_ERROR -> 0x04
	// I2C_ADC_REG_NUMBER_ERROR -> 0x05


	if (!((adcNumber == 1) || (adcNumber == 2))){
		*regValue = I2C_ADC_REG_ERROR;
		return I2C_ADC_WRONG_NUMBER;
	}
	// Get address of relative ADC and in case of error retrieve an error flag
	uint8_t adcAddressTransmit, adcAddressReceive;
	if (adcNumber == 1){
		adcAddressTransmit = getI2CAddress(ADC1_ADDR, I2C_MODE_WRITE);
		adcAddressReceive = getI2CAddress(ADC1_ADDR, I2C_MODE_READ);
		if ((adcAddressTransmit == I2C_ADDR_ERR) || (adcAddressReceive == I2C_ADDR_ERR)){
			*regValue = I2C_ADC_REG_ERROR;
			return I2C_ADC_WRONG_ADDRESS;
		}
	} else {
		adcAddressTransmit = getI2CAddress(ADC2_ADDR, I2C_MODE_WRITE);
		adcAddressReceive = getI2CAddress(ADC2_ADDR, I2C_MODE_READ);
		if ((adcAddressTransmit == I2C_ADDR_ERR) || (adcAddressReceive == I2C_ADDR_ERR)){
			*regValue = I2C_ADC_REG_ERROR;
			return I2C_ADC_WRONG_ADDRESS;
		}
	}

	// Check register
	if (regNumber > 3){
		*regValue = I2C_ADC_REG_ERROR;
		return I2C_ADC_REG_NUMBER_ERROR;
	}

	// Send the command byte
	uint8_t regCommand = ((I2C_ADC_READ_REGISTER_COMMAND << 4) | (regNumber << 2));
	uint8_t ret = HAL_I2C_Master_Transmit(hi2c1, (uint16_t) adcAddressTransmit, &regCommand, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		*regValue = I2C_ADC_REG_ERROR;
		return I2C_ADC_TRANSMIT_ERROR;
	} else {
		HAL_Delay(1);
		ret = HAL_I2C_Master_Receive(hi2c1, (uint16_t) (adcAddressReceive), (uint8_t*) regValue, 1, HAL_MAX_DELAY);
		if (ret != HAL_OK){
			*regValue = I2C_ADC_REG_ERROR;
			return I2C_ADC_RECEIVE_ERROR;
		} else {
			return I2C_ADC_OK;
		}

	}
}

uint8_t ADCSetRegistersValue(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber, uint8_t regNumber, uint8_t regValue){
	// Send to the ADC the target register value to modify the working parameters of the device
	// Use this command to write ADC register
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01
	// I2C_ADC_RECEIVE_ERROR -> 0x04
	// I2C_ADC_REG_NUMBER_ERROR -> 0x05


	// Check adc number
	if (!((adcNumber == 1) || (adcNumber == 2))){
		return I2C_ADC_WRONG_NUMBER;
	}


	// Get address of relative ADC and in case of error retrieve an error flag
	uint8_t adcAddress;
	if (adcNumber == 1){
		adcAddress = getI2CAddress(ADC1_ADDR, I2C_MODE_WRITE);
		if (adcAddress == I2C_ADDR_ERR){
			return I2C_ADC_WRONG_ADDRESS;
		}
	} else {
		adcAddress = getI2CAddress(ADC2_ADDR, I2C_MODE_WRITE);
		if (adcAddress == I2C_ADDR_ERR){
			return I2C_ADC_WRONG_ADDRESS;
		}
	}

	// Check register
	if (regNumber > 3){
		return I2C_ADC_REG_NUMBER_ERROR;
	}

	// Send the command bytes (0100 rrxx dddd dddd)
	uint8_t regCommand[2] = {0};
	regCommand[0] = ((I2C_ADC_WRITE_REGISTER_COMMAND << 4) | (regNumber << 2));
	regCommand[1] = regValue;
	uint8_t ret = HAL_I2C_Master_Transmit(hi2c1, (uint16_t) adcAddress, regCommand, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		return I2C_ADC_TRANSMIT_ERROR;
	} else {
		return I2C_ADC_OK;
	}
}

uint8_t ADCGetDataValue(I2C_HandleTypeDef* hi2c1, uint8_t adcNumber, int32_t* dataValue, uint8_t dataType){
	// Send to the ADC the command to read the data (voltage or temperature) depending to the selected mode
	// Use this command to read ADC data
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01
	// I2C_ADC_RECEIVE_ERROR -> 0x04
	// I2C_ADC_REG_NUMBER_ERROR -> 0x05


	if (!((adcNumber == 1) || (adcNumber == 2))){
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_NUMBER;
	}

	if (!((dataType == I2C_ADC_DATA_TEMP) || (dataType == I2C_ADC_DATA_VOLT))){
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_DATA;
	}


	// Get address of relative ADC and in case of error retrieve an error flag
	uint8_t adcAddressTransmit, adcAddressReceive;
	if (adcNumber == 1){
		adcAddressTransmit = getI2CAddress(ADC1_ADDR, I2C_MODE_WRITE);
		adcAddressReceive = getI2CAddress(ADC1_ADDR, I2C_MODE_READ);
		if ((adcAddressTransmit == I2C_ADDR_ERR) || (adcAddressReceive == I2C_ADDR_ERR)){
			*dataValue = I2C_ADC_DATA_ERROR;
			return I2C_ADC_WRONG_ADDRESS;
		}
	} else {
		adcAddressTransmit = getI2CAddress(ADC2_ADDR, I2C_MODE_WRITE);
		adcAddressReceive = getI2CAddress(ADC2_ADDR, I2C_MODE_READ);
		if ((adcAddressTransmit == I2C_ADDR_ERR) || (adcAddressReceive == I2C_ADDR_ERR)){
			*dataValue = I2C_ADC_DATA_ERROR;
			return I2C_ADC_WRONG_ADDRESS;
		}
	}

	// Send the command byte
	uint8_t addr = I2C_ADC_READ_DATA_COMMAND;
	uint8_t ret = HAL_I2C_Master_Transmit(hi2c1, (uint16_t) adcAddressTransmit, &addr, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_TRANSMIT_ERROR;
	} else {
		uint8_t data[2] = {0x00, 0x00};
		ret = HAL_I2C_Master_Receive(hi2c1, (uint16_t) (adcAddressReceive), data, 2, HAL_MAX_DELAY);
		if (ret != HAL_OK){
			*dataValue = I2C_ADC_DATA_ERROR;
			return I2C_ADC_RECEIVE_ERROR;
		} else {
			// Return 16 bit voltage or 14-bit temperature (left shifted)
			if (dataType == I2C_ADC_DATA_TEMP){
				*dataValue = ((data[0] >> 2) << 8) | (data[1]);
				return I2C_ADC_OK;
			} else {
				*dataValue = data[0] << 8 | data[1];
				return I2C_ADC_OK;
			}
		}
	}
}

void setLEDColor(uint8_t ledColor){
	switch (ledColor){
		case LED_RGB_RED:
			HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, GPIO_PIN_SET);
			return;
		case LED_RGB_BLUE:
			HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, GPIO_PIN_RESET);
			return;
		case LED_RGB_GREEN:
			HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, GPIO_PIN_SET);
			return;
		case LED_RGB_GREEN_BLUE:
			HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, GPIO_PIN_RESET);
			return;
		case LED_RGB_OFF:
			HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, GPIO_PIN_SET);
			return;
		case LED_RGB_ON:
			HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, GPIO_PIN_RESET);
			return;
		default:
			HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, GPIO_PIN_RESET);
	}
}

float getValueFromVoltageData(int32_t data){
	// If data < 0x7FFF i have a negative number else i have a positive one
	// For negative number i have to evaluate the two's complement
	data = (uint16_t) data;
	if (data <= 0x7FFF){
		return (data * 0.000152587890625f);
	} else {
		return ((((~data) + 1) * 0.000152587890625f) * (-1));
	}
}

float getValueFromTemperatureData(int32_t data){
	data = (uint16_t) data;
	if (data & (1 << 13)){
		return ((((~data) + 1) * 0.03125f) * (-1));
	} else {
		return (data * 0.03125f);
	}
}

float getValueFromTestVoltageData(int32_t data){
	// If data < 0x7FFF i have a negative number else i have a positive one
	// For negative number i have to evaluate the two's complement
	// Code follows function: getValueFromVoltageData(int32_t data)
	// In this case uses internal 2.048V and function provided is (AVDD - AVSS)/4
	data = (uint16_t) data;
	if (data <= 0x7FFF){
		return (data * 0.00025f);
	} else {
		return ((((~data) + 1) * 0.00025f) * (-1));
	}
}

uint8_t linearInterpolation(float* yValues, uint8_t syValues, float* xValues, uint8_t sxValues, float x, float* res){

	// Use this function to evaluate the temperature based on the sensor voltage read
	// yValues -> Temperature Array
	// xValues -> Voltage Array
	// x       -> Voltage read by the ADC

	// Check if all arrays are the same length
	if (syValues != sxValues){
		*res = -1.0f;
		return INT_ERR_LEN_ARRAYS;
	} else {
		if ((x <= ADC_MIN_VOLTAGE) || (x >= ADC_MAX_VOLTAGE)){
			*res = -1.0f;
			return INT_ERR_VOLTAGE;
		}

		uint8_t index1, index2 = 0;
		// index 1 is the index before the target
		// index 2 is the index after the target

		for (uint8_t j = 0; j < sxValues; j++)	{
			// Find the interval between the x value is found
			if (x >= xValues[j]) {
				index2++;
			} else {
				break;
			}
		}

		// Evaluate indexes
		index1 = index2 - 1;

		// Check indexes if are lower than zero or higher than the size of the array
		if (index1 >= 0 && index2 >= 0 && index1 < sxValues && index2 < sxValues){
			*res = yValues[index1] + ((x - xValues[index1])/(xValues[index2] - xValues[index1]))*(yValues[index2] - yValues[index1]);
			return INT_OK;
		} else {
			*res = -1.0f;
			return INT_ERR_INDEX;
		}
	}
}


uint8_t getADCInternalTemperature(I2C_HandleTypeDef* hi2c1, TIM_HandleTypeDef* htim1, uint8_t adcNumber, float* dataValue, bool* timElapsedFlag){
	// Retrieve the internal temperature of the ADC in °C
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01
	// I2C_ADC_RECEIVE_ERROR -> 0x04
	// I2C_ADC_REG_NUMBER_ERROR -> 0x05
	// I2C_ADC_WRONG_DATA -> 0x06
	// I2C_ADC_WRONG_SETUP -> 0x07
	// I2C_ADC_ERROR_START_SYNC_CMD -> 0x08
	// I2C_ADC_ERROR_RDATA_CMD -> 0x09

	// Check ADC number
	if (!((adcNumber == 1) || (adcNumber == 2))){
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_NUMBER;
	}

	// Check setup is successful
	uint8_t retValues[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	retValues[0] = ADCSetRegistersValue(hi2c1, adcNumber, 0x00, REG_0_DEFAULT);
	retValues[1] = ADCSetRegistersValue(hi2c1, adcNumber, 0x01, REG_1_TS);
	retValues[2] = ADCSetRegistersValue(hi2c1, adcNumber, 0x02, REG_2_DEFAULT);
	retValues[3] = ADCSetRegistersValue(hi2c1, adcNumber, 0x03, REG_3_DEFAULT);

	if ((retValues[0] == I2C_ADC_OK) && (retValues[1] == I2C_ADC_OK) && (retValues[2] == I2C_ADC_OK) && (retValues[3] == I2C_ADC_OK)){
		if (ADCStartSyncCommand(hi2c1, adcNumber) == I2C_ADC_OK){
				while (HAL_GPIO_ReadPin(ADC1_NDRDY_GPIO_Port, ADC1_NDRDY_Pin) != GPIO_PIN_RESET){
					*timElapsedFlag = false;
					HAL_TIM_Base_Start_IT(htim1);
					while (timElapsedFlag == false){
					}
				}
				int32_t res = 0;
				if (ADCGetDataValue(hi2c1, 1, &res, I2C_ADC_DATA_TEMP) != I2C_ADC_OK){
					*dataValue = I2C_ADC_DATA_ERROR;
					return I2C_ADC_ERROR_RDATA_CMD;
				} else {
					*dataValue = (float) getValueFromTemperatureData(res);
					return I2C_ADC_OK;
				}
		} else {
			*dataValue = I2C_ADC_DATA_ERROR;
			return I2C_ADC_ERROR_START_SYNC_CMD;
		}
	} else {
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_SETUP;
	}
}

uint8_t getADCReferenceVoltage(I2C_HandleTypeDef* hi2c1, TIM_HandleTypeDef* htim1, uint8_t adcNumber, float* dataValue, bool* timElapsedFlag){
	// Retrieve the internal temperature of the ADC in °C
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01
	// I2C_ADC_RECEIVE_ERROR -> 0x04
	// I2C_ADC_REG_NUMBER_ERROR -> 0x05
	// I2C_ADC_WRONG_DATA -> 0x06
	// I2C_ADC_WRONG_SETUP -> 0x07
	// I2C_ADC_ERRROR_START_SYNC_CMD -> 0x08
	// I2C_ADC_ERROR_RDATA_CMD -> 0x09

	// Check ADC number
	if (!((adcNumber == 1) || (adcNumber == 2))){
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_NUMBER;
	}

	// Check setup is successful
	uint8_t retValues[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	retValues[0] = ADCSetRegistersValue(hi2c1, adcNumber, 0x00, REG_0_VDD);
	HAL_Delay(5);
	retValues[1] = ADCSetRegistersValue(hi2c1, adcNumber, 0x01, REG_1_VDD);
	HAL_Delay(5);
	retValues[2] = ADCSetRegistersValue(hi2c1, adcNumber, 0x02, REG_2_DEFAULT);
	HAL_Delay(5);
	retValues[3] = ADCSetRegistersValue(hi2c1, adcNumber, 0x03, REG_3_DEFAULT);
	HAL_Delay(5);

	if ((retValues[0] == I2C_ADC_OK) && (retValues[1] == I2C_ADC_OK) && (retValues[2] == I2C_ADC_OK) && (retValues[3] == I2C_ADC_OK)){
		if (ADCStartSyncCommand(hi2c1, adcNumber) == I2C_ADC_OK){
				while (HAL_GPIO_ReadPin(ADC1_NDRDY_GPIO_Port, ADC1_NDRDY_Pin) != GPIO_PIN_RESET){
					*timElapsedFlag = false;
					HAL_TIM_Base_Start_IT(htim1);
					while (timElapsedFlag == false){
					}
				}
				int32_t res = 0;
				if (ADCGetDataValue(hi2c1, 1, &res, I2C_ADC_DATA_VOLT) != I2C_ADC_OK){
					*dataValue = I2C_ADC_DATA_ERROR;
					return I2C_ADC_ERROR_RDATA_CMD;
				} else {
					*dataValue = (float) getValueFromTestVoltageData(res);
					return I2C_ADC_OK;
				}
		} else {
			*dataValue = I2C_ADC_DATA_ERROR;
			return I2C_ADC_ERROR_START_SYNC_CMD;
		}
	} else {
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_SETUP;
	}
}

uint8_t getADCChannelVoltage(I2C_HandleTypeDef* hi2c1, TIM_HandleTypeDef* htim1, uint8_t adcNumber, uint8_t dataChannel, float* dataValue, bool* timElapsedFlag){
	// Retrieve the voltage of a target single ended channel
	// I2C_ADC_OK -> 0x00
	// I2C_ADC_TRANSMIT_ERROR -> 0x03
	// I2C_ADC_WRONG_ADDRESS -> 0x02
	// I2C_ADC_WRONG_NUMBER -> 0x01
	// I2C_ADC_RECEIVE_ERROR -> 0x04
	// I2C_ADC_REG_NUMBER_ERROR -> 0x05
	// I2C_ADC_WRONG_DATA -> 0x06
	// I2C_ADC_WRONG_SETUP -> 0x07
	// I2C_ADC_ERRROR_START_SYNC_CMD -> 0x08
	// I2C_ADC_ERROR_RDATA_CMD -> 0x09
	// I2C_ADC_WRONG_CHANNEL -> 0x10

	// Check ADC number
	if (!((adcNumber == 1) || (adcNumber == 2))){
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_NUMBER;
	}

	// Check channel number
	if ((dataChannel < 0) || (dataChannel > 3)){
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_CHANNEL;
	}

	uint8_t regChannel[4] = {REG_0_CH0, REG_0_CH1, REG_0_CH2, REG_0_CH3};

	// Check setup is successful
	uint8_t retValues[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	retValues[0] = ADCSetRegistersValue(hi2c1, adcNumber, 0x00, regChannel[dataChannel]);
	retValues[1] = ADCSetRegistersValue(hi2c1, adcNumber, 0x01, REG_1_DEFAULT);
	retValues[2] = ADCSetRegistersValue(hi2c1, adcNumber, 0x02, REG_2_DEFAULT);
	retValues[3] = ADCSetRegistersValue(hi2c1, adcNumber, 0x03, REG_3_DEFAULT);

	if ((retValues[0] == I2C_ADC_OK) && (retValues[1] == I2C_ADC_OK) && (retValues[2] == I2C_ADC_OK) && (retValues[3] == I2C_ADC_OK)){
		if (ADCStartSyncCommand(hi2c1, adcNumber) == I2C_ADC_OK){
			if (adcNumber == 1){
				while (HAL_GPIO_ReadPin(ADC1_NDRDY_GPIO_Port, ADC1_NDRDY_Pin) != GPIO_PIN_RESET){
					*timElapsedFlag = false;
					HAL_TIM_Base_Start_IT(htim1);
					while (timElapsedFlag == false){
					}
				}
			} else if (adcNumber == 2){
				while (HAL_GPIO_ReadPin(ADC2_NDRDY_GPIO_Port, ADC2_NDRDY_Pin) != GPIO_PIN_RESET){
					*timElapsedFlag = false;
					HAL_TIM_Base_Start_IT(htim1);
					while (timElapsedFlag == false){
					}
				}
			}
			int32_t res = 0;
			if (ADCGetDataValue(hi2c1, adcNumber, &res, I2C_ADC_DATA_VOLT) != I2C_ADC_OK){
				*dataValue = I2C_ADC_DATA_ERROR;
				return I2C_ADC_ERROR_RDATA_CMD;
			} else {
				*dataValue = getValueFromVoltageData(res);
				return I2C_ADC_OK;
			}
		} else {
			*dataValue = I2C_ADC_DATA_ERROR;
			return I2C_ADC_ERROR_START_SYNC_CMD;
		}
	} else {
		*dataValue = I2C_ADC_DATA_ERROR;
		return I2C_ADC_WRONG_SETUP;
	}
}

uint16_t sortTemperatures(uint8_t index, float* temperatures, uint8_t* errors, int scale){
	if (errors[index] == TEMP_ERR_OK){
		return ((uint16_t) (temperatures[index] * scale));
	} else {
		if (errors[index] == TEMP_ERR_OV){
			return CAN_ERR_OV;
		} else if (errors[index] == TEMP_ERR_UV){
			return CAN_ERR_UV;
		} else {
			return CAN_ERR_IGNORE;
		}
	}
}



