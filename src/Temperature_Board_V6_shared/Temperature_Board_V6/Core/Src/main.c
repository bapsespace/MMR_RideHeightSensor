/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "functions.h"
#include "constants.h"

#include "string.h"// For memset()

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

bool adc_error_flag = false;
bool tim_elapsed_flag = false;

CAN_TxHeaderTypeDef ptxHeader_temp_1_4;     // Temperature 1 - 4 header
CAN_TxHeaderTypeDef ptxHeader_temp_5_8;     // Temperature 5 - 8 header
CAN_TxHeaderTypeDef ptxHeader_temp_9_12;    // Temperature 9 - 12 header
CAN_TxHeaderTypeDef ptxHeader_temp_13_16;   // Temperature 13 - 16 header

uint8_t sortingIndexArray[16] = {0, 2, 5, 7, 1, 3, 4, 6, 8, 10, 13, 15, 9, 11, 12, 14};
uint8_t payload_temp_1_4[8] = { 0 };
uint8_t payload_temp_5_8[8] = { 0 };
uint8_t payload_temp_9_12[8] = { 0 };
uint8_t payload_temp_13_16[8] = { 0 };

uint32_t txMailbox;

// Temperature chart data
float tempData[141] = { 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
		5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0, 10.5, 11.0, 11.5,
		12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5,
		18.0, 18.5, 19.0, 19.5, 20.0, 20.5, 21.0, 21.5, 22.0, 22.5, 23.0, 23.5,
		24.0, 24.5, 25.0, 25.5, 26.0, 26.5, 27.0, 27.5, 28.0, 28.5, 29.0, 29.5,
		30.0, 30.5, 31.0, 31.5, 32.0, 32.5, 33.0, 33.5, 34.0, 34.5, 35.0, 35.5,
		36.0, 36.5, 37.0, 37.5, 38.0, 38.5, 39.0, 39.5, 40.0, 40.5, 41.0, 41.5,
		42.0, 42.5, 43.0, 43.5, 44.0, 44.5, 45.0, 45.5, 46.0, 46.5, 47.0, 47.5,
		48.0, 48.5, 49.0, 49.5, 50.0, 50.5, 51.0, 51.5, 52.0, 52.5, 53.0, 53.5,
		54.0, 54.5, 55.0, 55.5, 56.0, 56.5, 57.0, 57.5, 58.0, 58.5, 59.0, 59.5,
		60.0, 60.5, 61.0, 61.5, 62.0, 62.5, 63.0, 63.5, 64.0, 64.5, 65.0, 65.5,
		66.0, 66.5, 67.0, 67.5, 68.0, 68.5, 69.0, 69.5, 70.0 };

float voltData[141] = { 0.096808f, 0.127701f, 0.158988f, 0.190666f, 0.222730f,
		0.255293f, 0.287996f, 0.321190f, 0.354751f, 0.388675f, 0.422955f,
		0.457586f, 0.492562f, 0.527876f, 0.563524f, 0.599496f, 0.635788f,
		0.672391f, 0.709298f, 0.746502f, 0.783994f, 0.821767f, 0.859812f,
		0.898121f, 0.936685f, 0.975495f, 1.014543f, 1.053818f, 1.093311f,
		1.133013f, 1.172914f, 1.213005f, 1.253275f, 1.293714f, 1.334313f,
		1.375060f, 1.415946f, 1.456960f, 1.498092f, 1.539331f, 1.580667f,
		1.622089f, 1.663586f, 1.705148f, 1.746765f, 1.788425f, 1.830119f,
		1.871835f, 1.913563f, 1.955294f, 1.997016f, 2.038719f, 2.080393f,
		2.122029f, 2.163615f, 2.205143f, 2.246604f, 2.287983f, 2.329276f,
		2.370487f, 2.411563f, 2.452539f, 2.493391f, 2.534110f, 2.574688f,
		2.615117f, 2.655388f, 2.695494f, 2.735426f, 2.775178f, 2.814742f,
		2.854111f, 2.893278f, 2.932235f, 2.970978f, 3.009499f, 3.047792f,
		3.085852f, 3.123673f, 3.161249f, 3.198575f, 3.235647f, 3.272459f,
		3.309007f, 3.345287f, 3.381294f, 3.417038f, 3.452475f, 3.487642f,
		3.522522f, 3.557112f, 3.591409f, 3.625411f, 3.659114f, 3.692518f,
		3.725619f, 3.758415f, 3.790906f, 3.823093f, 3.854963f, 3.886527f,
		3.917780f, 3.948721f, 3.979349f, 4.009664f, 4.039664f, 4.069351f,
		4.098724f, 4.127783f, 4.156528f, 4.184960f, 4.213077f, 4.240882f,
		4.268375f, 4.295557f, 4.322429f, 4.348991f, 4.375246f, 4.401193f,
		4.426834f, 4.452171f, 4.477216f, 4.501938f, 4.526371f, 4.550505f,
		4.574344f, 4.597887f, 4.621137f, 4.644097f, 4.666817f, 4.689153f,
		4.711253f, 4.733069f, 4.754604f, 4.775860f, 4.796837f, 4.817535f,
		4.837954f, 4.858086f, 4.877918f, 4.897422f };

float volt[16] = { 0 };

float oversamplingVoltages[ADC_OVERSAMPLING] = { 0 };
uint8_t oversampligStatus[ADC_OVERSAMPLING] = { 0 };

float oversamplingSum = 0.0f;
float temp[16] = { 0 };
uint16_t sortedTemp[16] = {0};

uint8_t tempChannelError[16] = { 0 }; //Array to store temperature errors (OK, UNDERVOLTAGE, OVERVOLTAGE)
uint8_t sizeVoltData = sizeof(voltData) / sizeof(voltData[0]);
uint8_t sizeTempData = sizeof(tempData) / sizeof(tempData[0]);

uint8_t ind = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	// Start CAN Bus communication
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	// External ADC Setup
	// Set ADCs in reset mode
	HAL_GPIO_WritePin(ADC1_NRST_GPIO_Port, ADC1_NRST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADC2_NRST_GPIO_Port, ADC2_NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(25);

	// Set ADCs in NOT reset mode
	HAL_GPIO_WritePin(ADC1_NRST_GPIO_Port, ADC1_NRST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADC2_NRST_GPIO_Port, ADC2_NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(25);

	// MUXs setup
	// Set MUXs in NOT enabled mode
	HAL_GPIO_WritePin(MUX1_EN_GPIO_Port, MUX1_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MUX1_SEL_GPIO_Port, MUX1_SEL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MUX2_EN_GPIO_Port, MUX2_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MUX2_SEL_GPIO_Port, MUX2_SEL_Pin, GPIO_PIN_RESET);

	// CAN Bus headers setup
	uint16_t canId = 0x100;
	if (SEGMENT_NUMBER == 1){
		canId = SEGMENT_1_CAN_ID;
	} else if (SEGMENT_NUMBER == 2){
		canId = SEGMENT_2_CAN_ID;
	} else if (SEGMENT_NUMBER == 3){
		canId = SEGMENT_3_CAN_ID;
	}
	// Header for temp 1 - 4
	ptxHeader_temp_1_4.IDE = CAN_ID_STD;
	ptxHeader_temp_1_4.RTR = CAN_RTR_DATA;
	ptxHeader_temp_1_4.StdId = canId;
	ptxHeader_temp_1_4.DLC = 8;

	// Header for temp 5 - 8
	ptxHeader_temp_5_8.IDE = CAN_ID_STD;
	ptxHeader_temp_5_8.RTR = CAN_RTR_DATA;
	ptxHeader_temp_5_8.StdId = canId + 1;
	ptxHeader_temp_5_8.DLC = 8;

	// Header for temp 9 - 12
	ptxHeader_temp_9_12.IDE = CAN_ID_STD;
	ptxHeader_temp_9_12.RTR = CAN_RTR_DATA;
	ptxHeader_temp_9_12.StdId = canId + 2;
	ptxHeader_temp_9_12.DLC = 8;

	// Header for temp 13 - 16
	ptxHeader_temp_13_16.IDE = CAN_ID_STD;
	ptxHeader_temp_13_16.RTR = CAN_RTR_DATA;
	ptxHeader_temp_13_16.StdId = canId + 3;
	ptxHeader_temp_13_16.DLC = 8;

	// RGB LED Check
	setLEDColor(LED_RGB_RED);
	HAL_Delay(500);
	setLEDColor(LED_RGB_GREEN);
	HAL_Delay(500);
	setLEDColor(LED_RGB_BLUE);
	HAL_Delay(500);
	setLEDColor(LED_RGB_OFF);

	// Assign relative ignore channel to the relative segment
	uint8_t tempChannelInformation[16] = {0}; //Array to store temperature values and channel infos (ENABLED or DISABLED)
	// Temp array 1 3 6 8 | 2 4 5 7 | 9 11 14 16 | 10 12 13 15
	uint8_t SEGMENT_1_IGNORE_CH[16] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t SEGMENT_2_IGNORE_CH[16] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0};
	uint8_t SEGMENT_3_IGNORE_CH[16] = {1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1};

	if (SEGMENT_NUMBER == 1){
		memcpy(tempChannelInformation, SEGMENT_1_IGNORE_CH, 16 * sizeof(uint8_t));
	} else if (SEGMENT_NUMBER == 2){
		memcpy(tempChannelInformation, SEGMENT_2_IGNORE_CH, 16 * sizeof(uint8_t));
	} else if (SEGMENT_NUMBER == 3){
		memcpy(tempChannelInformation, SEGMENT_3_IGNORE_CH, 16 * sizeof(uint8_t));
	}

	// Check that Data Ready pin is HIGH
	// HAL_GPIO_ReadPin(ADC1_NDRDY_GPIO_Port, ADC1_NDRDY_Pin) == GPIO_PIN_SET;

	// Scan for I2C Devices
	uint8_t allI2CDevices[128] = { 0 };
	scanForI2CDevices(&hi2c1, allI2CDevices);
	if (areDevicesReady(allI2CDevices))
	{
		// If all devices are ready soft reset ADC1 and ADC2
		if ((ADCSoftwareReset(&hi2c1, 1) == I2C_ADC_OK) && (ADCSoftwareReset(&hi2c1, 2) == I2C_ADC_OK))
		{
			// If no error occurred during software reset continue with setup reading memory

			for (uint8_t j = 0; j < 2; j++)
			{
				// Loop Checking and starting both ADCs


				// Memory variables
				int16_t memoryReg[4] = { 0x00FF, 0x00FF, 0x00FF, 0x00FF };
				uint8_t memoryErr[4] = { 0 };

				for (uint8_t i = 0; i < 4; i++) {
					memoryErr[i] = ADCGetRegistersValue(&hi2c1, j+1, i, &memoryReg[i]);
				}

				// Check default memory registers value and error occurred during reading
				if ((memoryReg[0] == 0) && (memoryReg[1] == 0)
						&& (memoryReg[2] == 0) && (memoryReg[3] == 0)
						&& (memoryErr[0] == I2C_ADC_OK)
						&& (memoryErr[1] == I2C_ADC_OK)
						&& (memoryErr[2] == I2C_ADC_OK)
						&& (memoryErr[3] == I2C_ADC_OK)) {

					// Set ADC Register 0x00 with the following configuration -> 0x01
					// PGA_BYPASS -> 0b1     	: Disabled and bypassed
					// GAIN 		-> 0b0     	: Gain = 1 (default)
					// MUX 		-> 0b0000  	: AINP = AIN0, AINN = AIN1 (default)

					// Set ADC Register 0x01 with the following configuration -> 0x6C
					// TS 		-> 0b0	   	: Temperature sensor mode disabled (default)
					// VREF		-> 0b11		: Analog supply (AVDD – AVSS) used as reference
					// CM 		-> 0b0		: Single-shot conversion mode (default)
					// MODE       -> 0b1      : Turbo mode (512-kHz modulator clock)
					// DR         -> 0b100    : 2000 SPS

					// Set ADC Register 0x02 with the following configuration -> 0x00
					// IDAC       -> 0b000    : Off (default)
					// BCS        -> 0b0      : Current sources off (default)
					// CRC        -> 0b00     : Disabled (default)
					// DCNT       -> 0b00		: Conversion counter disabled (default)
					// DRDY		-> 0b0		: No new conversion result available (default)

					// Set ADC Register 0x03 with he following configuration -> 0x00
					// RESERVED   -> 0b00		: Always write 0
					// I2MUX		-> 0b000	: IDAC2 disabled (default)
					// I1MUX      -> 0b000	: IDAC1 disabled (default)

					uint8_t retValues[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
					retValues[0] = ADCSetRegistersValue(&hi2c1, j + 1, 0x00, REG_0_DEFAULT);
					retValues[1] = ADCSetRegistersValue(&hi2c1, j + 1, 0x01, REG_1_DEFAULT);
					retValues[2] = ADCSetRegistersValue(&hi2c1, j + 1, 0x02, REG_2_DEFAULT);
					retValues[3] = ADCSetRegistersValue(&hi2c1, j + 1, 0x03, REG_3_DEFAULT);

					// Check return data to evaluate some errors
					for (uint8_t i = 0; i < 4; i++) {
						if (retValues[i] != I2C_ADC_OK) {
							adc_error_flag = true;
						}
					}


					// Default configuration finished

					// Optional measurements
					// getADCInternalTemperature(&hi2c1, &htim1, 1, &temperature, &tim_elapsed_flag);
					// getADCReferenceVoltage(&hi2c1, &htim1, 1, &voltage, &tim_elapsed_flag);
				} else {
					// Else loop toggling the RED LED
					adc_error_flag = true;
				}
			}

			if (!adc_error_flag) {
				setLEDColor(LED_RGB_GREEN);
			}
			// Add different delay for each segment number
			if (SEGMENT_NUMBER == 2){
				HAL_Delay(50);
			} else if (SEGMENT_NUMBER == 3){
				HAL_Delay(100);
			}

			// Set mux enable to LOW to enable sensors and start with SEL pin LOW
			HAL_GPIO_WritePin(MUX1_EN_GPIO_Port, MUX1_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX1_SEL_GPIO_Port, MUX1_SEL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX2_EN_GPIO_Port, MUX2_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX2_SEL_GPIO_Port, MUX2_SEL_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);

		} else {
			// Else loop toggling the RED LED
			adc_error_flag = true;
		}

	} else {
		// Else loop toggling the RED LED
		adc_error_flag = true;
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (adc_error_flag) {
			// Add special can message
			setLEDColor(LED_RGB_RED);
			HAL_Delay(500);
			setLEDColor(LED_RGB_OFF);
			HAL_Delay(500);
		} else {


			// Starting measurement of temperatures NTC 1, 3, 6 and 8 for ADC1 and 9, 11, 14 and 16 for ADC2

			// For debug purposes turn ON the blue led while the green led is ON
			setLEDColor(LED_RGB_GREEN);

			// Loop twice for both adcs
			for (uint8_t k = 0; k < 2; k++){

				for (uint8_t j = 0; j < 4; j++) {
					oversamplingSum = 0.0f;
					for (uint8_t i = 0; i < ADC_OVERSAMPLING; i++) {
						//HAL_Delay(10);
						getADCChannelVoltage(&hi2c1, &htim1, k + 1, j, &oversamplingVoltages[i], &tim_elapsed_flag);
						oversamplingSum += oversamplingVoltages[i];
					}

					// Evaluate the correct index
					// for j = 0, 1, 2, 3
					// k = 0 -> ADC 1 -> 0, 1, 2, 3
					// k = 1 -> ADC 2 -> 8, 9, 10, 11
					ind = j + (8 * k);

					// Evaluate the average of oversampling data
					volt[ind] = oversamplingSum / ADC_OVERSAMPLING;

					// Evaluate if some error occurred
					if (tempChannelInformation[ind] == TEMP_CH_DIS) {
						tempChannelError[ind] = TEMP_ERR_IGNORE;
					} else if (volt[ind] <= ADC_MIN_VOLTAGE) {
						tempChannelError[ind] = TEMP_ERR_UV;
					} else if (volt[ind] >= ADC_MAX_VOLTAGE) {
						tempChannelError[ind] = TEMP_ERR_OV;
					} else {
						tempChannelError[ind] = TEMP_ERR_OK;
					}

					linearInterpolation(tempData, sizeTempData, voltData, sizeVoltData, volt[ind], &temp[ind]);
				}
			}

			// Starting measurement of temperatures NTC 2, 4, 5 and 7 for ADC1 and 10, 12, 13 and 15 for ADC2
			HAL_GPIO_TogglePin(MUX1_SEL_GPIO_Port, MUX1_SEL_Pin);
			HAL_GPIO_TogglePin(MUX2_SEL_GPIO_Port, MUX2_SEL_Pin);


			HAL_Delay(10);

			// For debug purposes turn OFF the blue led while the green led is ON
			setLEDColor(LED_RGB_OFF);

			// Loop twice for both adcs
			for (uint8_t k = 0; k < 2; k++){

				for (uint8_t j = 0; j < 4; j++) {
					oversamplingSum = 0.0f;
					//HAL_Delay(10);
					for (uint8_t i = 0; i < ADC_OVERSAMPLING; i++) {
						getADCChannelVoltage(&hi2c1, &htim1, k + 1, j, &oversamplingVoltages[i], &tim_elapsed_flag);
						oversamplingSum += oversamplingVoltages[i];
					}

					// Evaluate the correct index
					// for j = 0, 1, 2, 3
					// k = 0 -> ADC 1 -> 4, 5, 6, 7
					// k = 1 -> ADC 2 -> 12, 13, 14, 15

					ind = j + (8 * k) + 4;

					// Evaluate the average of oversampling data
					volt[ind] = oversamplingSum / ADC_OVERSAMPLING;

					// Evaluate if some error occurred
					if (tempChannelInformation[ind] == TEMP_CH_DIS) {
						tempChannelError[ind] = TEMP_ERR_IGNORE;
					} else if (volt[ind] <= ADC_MIN_VOLTAGE) {
						tempChannelError[ind] = TEMP_ERR_UV;
					} else if (volt[ind] >= ADC_MAX_VOLTAGE) {
						tempChannelError[ind] = TEMP_ERR_OV;
					} else {
						tempChannelError[ind] = TEMP_ERR_OK;
					}

					linearInterpolation(tempData, sizeTempData, voltData, sizeVoltData, volt[ind], &temp[ind]);

				}
			}
			// Temp array 1 3 6 8 | 2 4 5 7 | 9 11 14 16 | 10 12 13 15

			// Sorting temps
			for (uint8_t i = 0; i < 16; i++){
				//sortedTemp[i] = sortTemperatures(sortingIndexArray[i], temp, tempChannelError, 100);
				sortedTemp[sortingIndexArray[i]] = sortTemperatures(i, temp, tempChannelError, 100);
			}

			// Filling CAN arrays
			payload_temp_1_4[0] = (uint8_t) (sortedTemp[0] >> 8);
			payload_temp_1_4[1] = (uint8_t) sortedTemp[0];
			payload_temp_1_4[2] = (uint8_t) (sortedTemp[1] >> 8);
			payload_temp_1_4[3] = (uint8_t) sortedTemp[1];
			payload_temp_1_4[4] = (uint8_t) (sortedTemp[2] >> 8);
			payload_temp_1_4[5] = (uint8_t) sortedTemp[2];
			payload_temp_1_4[6] = (uint8_t) (sortedTemp[3] >> 8);
			payload_temp_1_4[7] = (uint8_t) sortedTemp[3];

			payload_temp_5_8[0] = (uint8_t) (sortedTemp[4] >> 8);
			payload_temp_5_8[1] = (uint8_t) sortedTemp[4];
			payload_temp_5_8[2] = (uint8_t) (sortedTemp[5] >> 8);
			payload_temp_5_8[3] = (uint8_t) sortedTemp[5];
			payload_temp_5_8[4] = (uint8_t) (sortedTemp[6] >> 8);
			payload_temp_5_8[5] = (uint8_t) sortedTemp[6];
			payload_temp_5_8[6] = (uint8_t) (sortedTemp[7] >> 8);
			payload_temp_5_8[7] = (uint8_t) sortedTemp[7];

			payload_temp_9_12[0] = (uint8_t) (sortedTemp[8] >> 8);
			payload_temp_9_12[1] = (uint8_t) sortedTemp[8];
			payload_temp_9_12[2] = (uint8_t) (sortedTemp[9] >> 8);
			payload_temp_9_12[3] = (uint8_t) sortedTemp[9];
			payload_temp_9_12[4] = (uint8_t) (sortedTemp[10] >> 8);
			payload_temp_9_12[5] = (uint8_t) sortedTemp[10];
			payload_temp_9_12[6] = (uint8_t) (sortedTemp[11] >> 8);
			payload_temp_9_12[7] = (uint8_t) sortedTemp[11];

			payload_temp_13_16[0] = (uint8_t) (sortedTemp[12] >> 8);
			payload_temp_13_16[1] = (uint8_t) sortedTemp[12];
			payload_temp_13_16[2] = (uint8_t) (sortedTemp[13] >> 8);
			payload_temp_13_16[3] = (uint8_t) sortedTemp[13];
			payload_temp_13_16[4] = (uint8_t) (sortedTemp[14] >> 8);
			payload_temp_13_16[5] = (uint8_t) sortedTemp[14];
			payload_temp_13_16[6] = (uint8_t) (sortedTemp[15] >> 8);
			payload_temp_13_16[7] = (uint8_t) sortedTemp[15];

			HAL_CAN_AddTxMessage(&hcan1, &ptxHeader_temp_1_4, payload_temp_1_4, &txMailbox);
			HAL_Delay(5);
			HAL_CAN_AddTxMessage(&hcan1, &ptxHeader_temp_5_8, payload_temp_5_8, &txMailbox);
			HAL_Delay(5);
			HAL_CAN_AddTxMessage(&hcan1, &ptxHeader_temp_9_12, payload_temp_9_12, &txMailbox);
			HAL_Delay(5);
			HAL_CAN_AddTxMessage(&hcan1, &ptxHeader_temp_13_16, payload_temp_13_16, &txMailbox);
			// Index:          0   1   2   3   4   5   6   7
			// Payload 1-4  : T01 T01 T02 T02 T03 T03 T04 T04
			// Payload 5-8  : T05 T05 T06 T06 T07 T07 T08 T08
			// Payload 9-12 : T09 T09 T10 T10 T11 T11 T12 T12
			// Payload 13-16: T13 T13 T14 T14 T15 T15 T16 T16

			HAL_Delay(200);

			HAL_GPIO_TogglePin(MUX1_SEL_GPIO_Port, MUX1_SEL_Pin);
			HAL_GPIO_TogglePin(MUX2_SEL_GPIO_Port, MUX2_SEL_Pin);

			HAL_Delay(10); //If filters are used set to 50ms
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 32;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = ENABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = ENABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300CC0;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MUX2_EN_Pin|MUX2_SEL_Pin|ADC2_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RGB_LED_R_Pin|RGB_LED_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUX1_EN_Pin|MUX1_SEL_Pin|ADC1_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOOT0_GPIO_Port, BOOT0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MUX2_EN_Pin MUX2_SEL_Pin ADC2_NRST_Pin RGB_LED_R_Pin
                           RGB_LED_B_Pin */
  GPIO_InitStruct.Pin = MUX2_EN_Pin|MUX2_SEL_Pin|ADC2_NRST_Pin|RGB_LED_R_Pin
                          |RGB_LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC2_NDRDY_Pin */
  GPIO_InitStruct.Pin = ADC2_NDRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC2_NDRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_LED_G_Pin MUX1_EN_Pin MUX1_SEL_Pin ADC1_NRST_Pin */
  GPIO_InitStruct.Pin = RGB_LED_G_Pin|MUX1_EN_Pin|MUX1_SEL_Pin|ADC1_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC1_NDRDY_Pin */
  GPIO_InitStruct.Pin = ADC1_NDRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC1_NDRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT0_Pin */
  GPIO_InitStruct.Pin = BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOT0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**

 @brief  Period elapsed callback in non blocking mode
 @note   This function is called  when TIM6 interrupt took place, inside
 HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 a global variable "uwTick" used as application time base.
 @param  htim : TIM handle
 @retval None
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// USER CODE BEGIN Callback 0 /

	// USER CODE END Callback 0 /

	// USER CODE BEGIN Callback 1 /
	tim_elapsed_flag = true;
	HAL_TIM_Base_Stop(&htim1);
	// USER CODE END Callback 1 */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		setLEDColor(LED_RGB_RED);
		HAL_Delay(500);
		setLEDColor(LED_RGB_OFF);
		HAL_Delay(500);

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
