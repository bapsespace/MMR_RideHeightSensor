/*
 * constants.h
 *
 *  Created on: 17 mag 2023
 *      Author: lucad
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define SEGMENT_NUMBER 1 // 1 or 2 or 3
#define SEGMENT_1_CAN_ID 0x200
#define SEGMENT_2_CAN_ID 0x210
#define SEGMENT_3_CAN_ID 0x220

#define LOOP_TIME	150			//loop time in ms

#define ADC1_ADDR 0x40
#define ADC2_ADDR 0x45
#define EEPROM_ADDR 0x50


#define I2C_MODE_WRITE 0x00
#define I2C_MODE_READ 0x01

#define I2C_DEVICE_NUMBER 2
#define I2C_MODE_ERR 0xFE
#define I2C_ADDR_ERR 0xFF

#define I2C_ADC_OK 0x00
#define I2C_ADC_WRONG_NUMBER 0x01
#define I2C_ADC_WRONG_ADDRESS 0x02
#define I2C_ADC_TRANSMIT_ERROR 0x03
#define I2C_ADC_RECEIVE_ERROR 0x04
#define I2C_ADC_REG_NUMBER_ERROR 0x05
#define I2C_ADC_WRONG_DATA 0x06
#define I2C_ADC_WRONG_SETUP 0x07
#define I2C_ADC_ERROR_START_SYNC_CMD 0x08
#define I2C_ADC_ERROR_RDATA_CMD 0x09
#define I2C_ADC_WRONG_CHANNEL 0x10


// Register address map
// x  : DO NOT CARE
// rr : Register address

#define I2C_ADC_RESET_COMMAND        	0b00000110 // 0000 011x
#define I2C_ADC_START_SYNC_COMMAND   	0b00001000 // 0000 100x
#define I2C_ADC_POWERDOWN_COMMAND    	0b00000010 // 0000 001x
#define I2C_ADC_READ_DATA_COMMAND	 	0b00010000 // 0001 xxxx
#define I2C_ADC_READ_REGISTER_COMMAND   0b0010     // 0010 rrxx
#define I2C_ADC_WRITE_REGISTER_COMMAND  0b0100     // 0100 rrxx

#define I2C_ADC_REG_ERROR -1
#define I2C_ADC_DATA_ERROR -2

#define ADC_MIN_VOLTAGE 0.10f
#define ADC_MAX_VOLTAGE 4.90f
#define ADC_OVERSAMPLING 1

// RGB Color code
#define LED_RGB_RED 0x00
#define LED_RGB_GREEN 0x01
#define LED_RGB_BLUE 0x02

#define LED_RGB_GREEN_BLUE 0x03

#define LED_RGB_ON 0xFE
#define LED_RGB_OFF 0xFF

// Register Default Settings
#define REG_0_DEFAULT 0x01
#define REG_1_DEFAULT 0xD4
#define REG_2_DEFAULT 0x00
#define REG_3_DEFAULT 0x00

// Register Channel Settings
#define REG_0_CH0 0x81
#define REG_0_CH1 0x91
#define REG_0_CH2 0xA1
#define REG_0_CH3 0xB1
#define REG_0_VDD 0xD1
#define REG_1_VDD 0xD0
#define REG_1_TS  0xD1

// Linear Interpolation Errors
#define INT_OK 0x00
#define INT_ERR_LEN_ARRAYS 0x01
#define INT_ERR_INDEX 0x02
#define INT_ERR_VOLTAGE 0xFF

// Temperature channel information
#define TEMP_CH_DIS 0x00
#define TEMP_CH_EN 0x01

// Temperature error channel information
#define TEMP_ERR_OK 0x00
#define TEMP_ERR_OV 0x01
#define TEMP_ERR_UV 0x02
#define TEMP_ERR_IGNORE 0x03

#define CAN_ERR_OV 0xFFFF
#define CAN_ERR_UV 0xFFFE
#define CAN_ERR_IGNORE 0xFFFD

// Retrieved data type
#define I2C_ADC_DATA_TEMP 0x00
#define I2C_ADC_DATA_VOLT 0x01


#endif /* INC_CONSTANTS_H_ */
