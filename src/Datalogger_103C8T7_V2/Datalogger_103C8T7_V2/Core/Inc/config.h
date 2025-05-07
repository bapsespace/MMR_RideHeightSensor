/*
 * config.h
 *
 *  Created on: May 6, 2024
 *      Author: lucad
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

//board type
#define BOARD_TYPE_WITH_ANALOG 			0x01
#define BOARD_TYPE_WITHOUT_ANALOG 		0x02

//PT1000 external resistance
#define PT_EXT_RES 						1000

//DMA configurations
#define DMA_CIRC_BUFF_LEN 				5
#define DMA_EXCLUDED_CYCLES 			5

#define DMA_EXCLUDED_CYCLES_YES 		0x01
#define DMA_EXCLUDED_CYCLES_NO 			0x02

//CAN Bus configurations
#define CAN_SLOW_ADDR 					0x310
#define CAN_FAST_ADDR 					0x314


#define CAN_MICRO_TEMP_SCALE 			100
#define CAN_PCB_TEMP_SCALE 				100
#define CAN_PT1_SCALE 					100
#define CAN_PT2_SCALE 					100

//ADS1115 configurations
#define ADS_ADDRESS 					(0b01001001 << 1)
//#define ADS_ADDRESS 					0x4B
#define ADS_CONVERSION_REGISTER 		0b00000000
#define ADS_CONFIGURATION_REGISTER 		0b00000001

#define ADS_CONVERSION_FACTOR 			0.1875f 			//mV/bit conversion ratio


#endif /* INC_CONFIG_H_ */
