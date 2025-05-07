/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "config.h"
#include "lmt84.h"
#include "pt1000.h"
#include "stm32_adc_utils.h"
#include <string.h>

#include "functions.h"
#include "constants.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOARD_TYPE BOARD_TYPE_WITHOUT_ANALOG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
//Debug variables
uint32_t pippo = 0;
float pluto = 0.0f;
uint8_t paperino = 0;
uint8_t paperone = 0;
uint8_t error_I2C_test = 0;

//DMA variables
uint8_t dmaIndex = 0;
uint8_t dmaExcludedCyclesState = DMA_EXCLUDED_CYCLES_NO;
uint32_t dmaCounter = 0;
uint32_t dmaRawADC[5] = {0};

//Analog variables
uint16_t vRefRaw[DMA_CIRC_BUFF_LEN] = {0};
uint16_t microTempRaw[DMA_CIRC_BUFF_LEN] = {0};
uint16_t pcbTempRaw[DMA_CIRC_BUFF_LEN] = {0};
uint16_t pt1Raw[DMA_CIRC_BUFF_LEN] = {0};
uint16_t pt2Raw[DMA_CIRC_BUFF_LEN] = {0};

float vRef = 0.0f;
float microTemp = 0.0f;
float pcbTemp = 0.0f;
float pt1 = 0.0f;
float pt2 = 0.0f;

uint16_t pcbTempBuff[DMA_CIRC_BUFF_LEN] = {0};
uint16_t microTempBuff[DMA_CIRC_BUFF_LEN] = {0};
uint16_t vRefBuff[DMA_CIRC_BUFF_LEN] = {0};
uint16_t pt1Buff[DMA_CIRC_BUFF_LEN] = {0};
uint16_t pt2Buff[DMA_CIRC_BUFF_LEN] = {0};

//CAN Bus headers, mailbox and dataframes
CAN_TxHeaderTypeDef pHeaderHighFreq;
CAN_TxHeaderTypeDef pHeaderLowFreq;
uint32_t pTxMailbox;
CAN_FilterTypeDef sFilterConfig;
uint8_t pDataLowFreq[8] = {0};
uint8_t pDataHighFreq[8] = {0};

//I2C Analog variables
uint8_t adsConfigBuffer[3] = {0x00};
int16_t adsRawValues[4] = {0x00};
uint8_t adsRetBuffer[2] = {0x00};
float adsVoltageValues[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float adsVread3_Right = 0.0f;
float adsVread2_Left = 0.0f;
uint8_t adsMuxConfiguration[4] = {0b11000001, 0b11010001, 0b11100001, 0b11110001};
uint8_t adsNextIndex = 0;
float adsCh0[DMA_CIRC_BUFF_LEN] = {0.0f};
float adsCh1[DMA_CIRC_BUFF_LEN] = {0.0f};
float adsCh2[DMA_CIRC_BUFF_LEN] = {0.0f};
float adsCh3[DMA_CIRC_BUFF_LEN] = {0.0f};
float adsAverageCh[4] = {0.0f};
uint8_t adsIndex = 0;
uint32_t adsCounter = 0;

float heightSensor1 = 0.0f, heightSensor2 = 0.0f;
float tempBrakeSensor1 =0.0f, empBrakeSensor2 = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //ADC setup commands
  while (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK){}
  HAL_ADC_Start_DMA(&hadc1, dmaRawADC, 5);

  //CAN Bus setup command
  sFilterConfig.FilterActivation = CAN_FILTER_DISABLE;
  sFilterConfig.FilterBank = 18;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0x446<<5;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0x446<<5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.SlaveStartFilterBank = 20;

  //HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  pHeaderHighFreq.IDE = CAN_ID_STD;
  pHeaderHighFreq.StdId = CAN_FAST_ADDR;
  pHeaderHighFreq.RTR = CAN_RTR_DATA;
  pHeaderHighFreq.DLC = 8;

  pHeaderLowFreq.IDE = CAN_ID_STD;
  pHeaderLowFreq.StdId = CAN_SLOW_ADDR;
  pHeaderLowFreq.RTR = CAN_RTR_DATA;
  pHeaderLowFreq.DLC = 8;

  HAL_CAN_Start(&hcan);

  //Timers start command
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  //I2C ADC setup as:
  //- Single shot conversion
  //- Channel 0 (AINP = AIN0 and AINN = GND) -> In vector only this changes
  //- Full scale of 6144mV
  //- Single shot conversion mode
  //- Sampling frequency of 860SPS
  //- Traditional comparator mode
  //- Active Low comparator
  //- Non latching comparator
  //- Disable comparator

  adsConfigBuffer[0] = ADS_CONFIGURATION_REGISTER;
  adsConfigBuffer[1] = adsMuxConfiguration[0];
  adsConfigBuffer[2] = 0b11100011;

  if (HAL_I2C_Master_Transmit(&hi2c1, ADS_ADDRESS, adsConfigBuffer, 3, 100) != HAL_OK){
	  error_I2C_test = 2;
	  Error_Handler();
  }

  HAL_Delay(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for (uint8_t i = 0; i < 4; i++){

		//Get conversion data
		if (HAL_I2C_Mem_Read(&hi2c1, ADS_ADDRESS, ADS_CONVERSION_REGISTER, 1, adsRetBuffer, 2, 10) != HAL_OK){
			error_I2C_test = 3;
			Error_Handler();
		}

		//Decode converted data
		adsRawValues[i] = (adsRetBuffer[0] << 8 | adsRetBuffer[1]);
		adsVoltageValues[i] = adsRawValues[i] * ADS_CONVERSION_FACTOR;

		if (adsVoltageValues[i] < 560.0f) {
			adsVoltageValues[i] = 560.0f;
		} else if (adsVoltageValues[i] > 2800.0f) {
	  	  	  adsVoltageValues[i] = 2800.0f;
  	  	}



		//Prepare for next conversion
		adsNextIndex = (i+1)%4;
		adsConfigBuffer[1] = adsMuxConfiguration[adsNextIndex];

		//Send to ADC the configuration for next channel read
		if (HAL_I2C_Master_Transmit(&hi2c1, ADS_ADDRESS, adsConfigBuffer, 3, 10) != HAL_OK){
			error_I2C_test = 1;
			Error_Handler();
		}

		//Wait for conversion 860sps -> around 1ms minimum wait
		HAL_Delay(2);
	}

	adsVread2_Left = adsVoltageValues[2]/1000;
	adsVread3_Right = adsVoltageValues[3]/1000;

	//Place values in the circular buffer
	adsIndex = adsCounter % DMA_CIRC_BUFF_LEN;
	adsCh0[adsIndex] = adsVoltageValues[0];
	adsCh1[adsIndex] = adsVoltageValues[1];
	adsCh2[adsIndex] = adsVoltageValues[2];
	adsCh3[adsIndex] = adsVoltageValues[3];
	adsCounter++;


	/////////////////////// NEW ADC

	I2C_HandleTypeDef hi2c1;
	TIM_HandleTypeDef htim1;
	float oversamplingVoltages = { 0 };
	bool tim_elapsed_flag = 0;

	getADCChannelVoltage(&hi2c1, &htim1, 0, 0, &oversamplingVoltages, &tim_elapsed_flag);




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 200-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_G_Pin|LED_R_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//DMA interrupt handler -> Each time DMA completes all conversions save them in a custom circular buffer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if ((dmaCounter > DMA_EXCLUDED_CYCLES) || (dmaExcludedCyclesState == DMA_EXCLUDED_CYCLES_YES)){
		dmaIndex = dmaCounter % DMA_CIRC_BUFF_LEN;
		dmaExcludedCyclesState = DMA_EXCLUDED_CYCLES_YES;
		pcbTempRaw[dmaIndex] = dmaRawADC[0];
		pt1Raw[dmaIndex] = dmaRawADC[1];
		pt2Raw[dmaIndex] = dmaRawADC[2];
		microTempRaw[dmaIndex] = dmaRawADC[3];
		vRefRaw[dmaIndex] = dmaRawADC[4];
	}
	dmaCounter++;
}

//TIM elapsed callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2){
		//Tim2 @ 1Hz to blink green led for i'm alive state
		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
	} else if (htim->Instance == TIM3){
		//Tim3 @ 5Hz to send board info and eventually (if enabled) pt1000 data

		//Send over can if more than 2*DMA_CIRC_BUFF_LEN are elapsed
		if(dmaCounter > (DMA_EXCLUDED_CYCLES + DMA_CIRC_BUFF_LEN)){
			//Copy data of DMA into buffers (to avoid that DMA updates data while elaborating it
			memcpy(pcbTempBuff, pcbTempRaw, sizeof(pcbTempRaw));
			memcpy(vRefBuff, vRefRaw, sizeof(vRefRaw));
			memcpy(microTempBuff, microTempRaw, sizeof(microTempRaw));
			memcpy(pt1Buff, pt1Raw, sizeof(pt1Raw));
			memcpy(pt2Buff, pt2Raw, sizeof(pt2Raw));

			//For each element of array evaluate its correct value using vref to calculate fullscale
			float tmp, fullScale=0;
			for (uint8_t i=0; i < DMA_CIRC_BUFF_LEN; i++){
				paperino = i;
				pluto = 0;
				fullScale = getMaxVoltageF103(vRefBuff[i]);
				pluto = (((float) pcbTempBuff[i]) / 4095.0f * fullScale);
				LMT84_getTemperature(LMT84_SQRT_MODE, ((float) pcbTempBuff[i] / 4095.0f * fullScale), &tmp);
				pcbTemp+=tmp;
				microTemp += getInternalTemperatureF103(((float) microTempBuff[i]) / 4095.0f * fullScale);
				if (BOARD_TYPE == BOARD_TYPE_WITH_ANALOG){
					pt1000_getTemperature(10000, PT1000_EXT_RES_PULLDOWN, PT1000_EQN_MODE, pt1Buff[i], &tmp);
					pt1 += tmp;
					pt1000_getTemperature(10000, PT1000_EXT_RES_PULLDOWN, PT1000_EQN_MODE, pt2Buff[i], &tmp);
					pt2 += tmp;
				}
				pippo++;
			}

			//Evaluate averages
			pcbTemp /= DMA_CIRC_BUFF_LEN;
			microTemp /= DMA_CIRC_BUFF_LEN;
			if (BOARD_TYPE == BOARD_TYPE_WITH_ANALOG){
				pt1 /= DMA_CIRC_BUFF_LEN;
				pt2 /= DMA_CIRC_BUFF_LEN;
			}

			//Assign data values to can bytes
			pDataLowFreq[0] = (uint8_t) (((uint16_t) (pcbTemp * CAN_PCB_TEMP_SCALE)) >> 8);
			pDataLowFreq[1] = (uint8_t) (pcbTemp * CAN_PCB_TEMP_SCALE);
			pDataLowFreq[2] = (uint8_t) (((uint16_t) (microTemp * CAN_MICRO_TEMP_SCALE)) >> 8);
			pDataLowFreq[3] = (uint8_t) (microTemp * CAN_MICRO_TEMP_SCALE);

			//Check if board is programmed for analog sensors and if values are coherent
			if (BOARD_TYPE == BOARD_TYPE_WITH_ANALOG){
				if (pt1 >= 0){
					pDataLowFreq[4] = (uint8_t) (((uint16_t) (pt1 * CAN_PT1_SCALE)) >> 8);
					pDataLowFreq[5] = (uint8_t) (pt1 * CAN_PT1_SCALE);
				} else {
					pDataLowFreq[4] = 0xFF;
					pDataLowFreq[5] = 0xFF;
				}
				if (pt2 >= 0){
					pDataLowFreq[6] = (uint8_t) (((uint16_t) (pt2 * CAN_PT2_SCALE)) >> 8);
					pDataLowFreq[7] = (uint8_t) (pt2 * CAN_PT2_SCALE);
				} else {
					pDataLowFreq[6] = 0xFF;
					pDataLowFreq[7] = 0xFF;
				}

			} else {
				pDataLowFreq[4] = 0x00;
				pDataLowFreq[5] = 0x00;
				pDataLowFreq[6] = 0x00;
				pDataLowFreq[7] = 0x00;
			}

			//Send data
			HAL_CAN_AddTxMessage(&hcan, &pHeaderLowFreq, pDataLowFreq, &pTxMailbox);
		}

	} else if (htim->Instance == TIM4){
		//Tim4 @ 50Hz to send I2C Analog values

		//Evaluate averages
		for (uint8_t i = 0; i < DMA_CIRC_BUFF_LEN; i++){
			adsAverageCh[0] += adsCh0[i];
			adsAverageCh[1] += adsCh1[i];
			adsAverageCh[2] += adsCh2[i];
			adsAverageCh[3] += adsCh3[i];
		}
		adsAverageCh[0] /= DMA_CIRC_BUFF_LEN;
		adsAverageCh[1] /= DMA_CIRC_BUFF_LEN;
		adsAverageCh[2] /= DMA_CIRC_BUFF_LEN;
		adsAverageCh[3] /= DMA_CIRC_BUFF_LEN;

		//Assign data values to CAN bytes (all values are stored in mV)
												//Rangemm		//RangeV
		heightSensor1 = ((adsVoltageValues[2]) * (104.0f/(2795.0f-560.0f))-10.0f);
		heightSensor2 = ((adsVoltageValues[3]) * (104.0f/(2795.0f-560.0f))-10.0f);
		tempBrakeSensor1 = map(adsVoltageValues[0], 0, 5, 0, 1000);
		tempBrakeSensor2 = map(adsVoltageValues[1], 0, 5, 0, 1000);

		//il calcolo della conversione è ((adsVoltageValues[3]) * (104.0f/(2795.0f-560.0f))-10.0f)

		// la centralina legge prima i bit da 0 a 16 e da 16 a 32


		pDataHighFreq[0] = (uint8_t) (((uint16_t) (heightSensor1 * 10)) >> 8); //lohigh
		pDataHighFreq[1] = (uint8_t) (heightSensor1 * 10);

		//i primi due byte sono heightSensor2
		pDataHighFreq[2] = (uint8_t) (((uint16_t) (heightSensor2 * 10)) >> 8); //loHigh
		pDataHighFreq[3] = (uint8_t) (heightSensor2 * 10);

		//
		pDataHighFreq[4] = (uint8_t) (((uint16_t) tempBrakeSensor1) >> 8); //loHigh
		pDataHighFreq[5] = (uint8_t) (tempBrakeSensor1);


		pDataHighFreq[6] = (uint8_t) (((uint16_t) tempBrakeSensor2) >> 8); //loHigh
		pDataHighFreq[7] = (uint8_t) tempBrakeSensor2;
		//Send data
		HAL_CAN_AddTxMessage(&hcan, &pHeaderHighFreq, pDataHighFreq, &pTxMailbox);
	}
}




/* x è il valore di partenza, che deve essere convertito.
 * in_min e in_max è il range del sensore in voltaggio
 * out_min e out_max sono i valoi per cui verranno mappati in_min e in_max.
 * es= se in_min = 0 e in_max = 5; out_min = 100 e out_max = 200. allora
 * se x=0 verrà automaticamente mappato a 100
 */
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1){
	  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	  HAL_Delay(1000);
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
