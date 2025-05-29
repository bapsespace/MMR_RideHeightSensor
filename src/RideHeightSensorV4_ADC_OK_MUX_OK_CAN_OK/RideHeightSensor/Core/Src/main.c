/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "core_cm3.h"  // for __NOP
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//ADC
#define ADS1119_ADDR     (0x40 << 1)  // 7-bit address + R/W bit
#define ADS1119_REG_CONFIG   0x40
#define ADS1119_CMD_START    0x08
#define ADS1119_CMD_READ     0x10
#define ADS1119_CMD_RESET    0x06

//Testato funziona NON MODIFICARE
#define CONFIG_AIN0   0b01101110 // AIN0 - AGND
#define CONFIG_AIN1   0b10001110 //	AIN1 - AGND
#define CONFIG_AIN2   0b10101110 //	AIN2 - AGND
#define CONFIG_AIN3   0b11001110 // AIN3 - AGND


#define MUX1_A0_B GPIO_PIN_5
#define MUX1_A1_B GPIO_PIN_4

#define MUX2_A0_B GPIO_PIN_11
#define MUX2_A1_B GPIO_PIN_10



#define CAN_ID_MAIN_DATA 0x315 // ID messaggio ride height (2byte l'uno): left + rigt + adc1 + adc2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

// Global variables for timing flags
volatile uint8_t flag_1ms = 0;
volatile uint8_t flag_10ms = 0;
volatile uint8_t flag_100ms = 0;
volatile uint8_t flag_1000ms = 0;

// Counter variables
volatile uint16_t ms_counter = 0;

// Globals for debug timing
volatile uint32_t exec_time_1ms = 0; // should not exceed 50_000
volatile uint32_t exec_time_10ms = 0; // should not exceed 500_000
volatile uint32_t exec_time_100ms = 0; // should not exceed 5_000_000
volatile uint32_t exec_time_1000ms = 0; // should not exceed 50_000_000

// Helper macro to compute time difference with wraparound
#define SYSTICK_DIFF(start, end) ((start >= end) ? (start - end) : (SysTick->LOAD - end + start))





// CAN
CAN_TxHeaderTypeDef can_main_data_TxHeader;
uint32_t can_TxMailbox = 0;
uint8_t can_main_data[8] = {0x00};


// ADC
int16_t pt_analog_in_raw[8] = {0};
int16_t adc_analog_in_raw[2] = {0};
int16_t height_right_analog_in_raw = 0;
int16_t height_left_analog_in_raw = 0;

// Converted millivolt values
int32_t pt_analog_in_mv[8] = {0};
int32_t adc_analog_in_mv[2] = {0};
int32_t height_right_analog_in_mv = 0;
int32_t height_left_analog_in_mv = 0;

// Calibrated distance outputs (in mm)
float distance_right_mm = 0.0f;
float distance_left_mm = 0.0f;

float pt_analog_in_temp[8] = {0.0f};



// CONFIG byte for differential AIN0-AIN1, Gain = 1, 20 SPS, continuous mode
uint8_t config_data[2] = {0x01, 0b00000100};  // Register address + config byte



//LOOKUP TABLES
// Structure for a lookup table
typedef struct {
    const int32_t *input_values;  // Array of input values (mV)
    const float *output_values;   // Array of output values (mm or °C)
    uint8_t size;                 // Number of entries in the table
} LookupTable_t;



// Table for height sensor RIGHT
static const int32_t height_right_input_mv[] = {7, 660, 1260, 2047};
static const float height_right_output_mm[] = {16.0f, 28.0f, 39.0f, 51.0f};
LookupTable_t height_right_table = {
    .input_values = height_right_input_mv,
    .output_values = height_right_output_mm,
    .size = 4
};

// Table for height sensor LEFT
static const int32_t height_left_input_mv[] = {7, 660, 1260, 2047};
static const float height_left_output_mm[] = {16.0f, 28.0f, 39.0f, 51.0f};
LookupTable_t height_left_table = {
    .input_values = height_left_input_mv,
    .output_values = height_left_output_mm,
    .size = 4
};


// Table for temperature sensor
static const int32_t temp_input_mv[] = {0, 1000, 2000, 3000};
static const float temp_output_c[] = {-10.0f, 25.0f, 60.0f, 100.0f};
LookupTable_t temp_table = {
    .input_values = temp_input_mv,
    .output_values = temp_output_c,
    .size = 4
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void ADS1119_Reset(I2C_HandleTypeDef *hi2c);
void ADS1119_Start(I2C_HandleTypeDef *hi2c);
void ADS1119_Config(I2C_HandleTypeDef *hi2c, int port_number);
int16_t ADS1119_Read(I2C_HandleTypeDef *hi2c);
int32_t ADS1119_ConvertToMillivolts(int16_t raw_value);

void SetMUX(uint8_t mux_value);

void ConvertMilliVoltsToDistance(void);
void ConvertMilliVoltsToTemperature(void);

void ConvertAllAdcToMillivolts(void);

void Analog_Read_ALL(void);

float LookupWithInterpolation(const LookupTable_t *table, int32_t input);


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

void CAN_Config(void);

void SendMainDataToCan(void);

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
  MX_I2C1_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  ADS1119_Reset(&hi2c1);
  ADS1119_Start(&hi2c1);

  HAL_Delay(10);

  //HAL_CAN_Stop(&hcan);
  HAL_CAN_Start(&hcan);

  // Start TIM1 in interrupt mode
  HAL_TIM_Base_Start_IT(&htim1);


  while (1) {
      uint32_t start, end;

      if (flag_1ms) {
          flag_1ms = 0;
          start = SysTick->VAL;

          // 1ms tasks here

          end = SysTick->VAL;
          exec_time_1ms = SYSTICK_DIFF(start, end);
      }

      if (flag_10ms) {
          flag_10ms = 0;
          start = SysTick->VAL;

          // 10ms tasks
          Analog_Read_ALL();
          ConvertAllAdcToMillivolts();
          ConvertMilliVoltsToDistance();
          ConvertMilliVoltsToTemperature();
          SendMainDataToCan();

          end = SysTick->VAL;
          exec_time_10ms = SYSTICK_DIFF(start, end);
      }

      if (flag_100ms) {
          flag_100ms = 0;
          start = SysTick->VAL;

          // 100ms tasks

          end = SysTick->VAL;
          exec_time_100ms = SYSTICK_DIFF(start, end);
      }

      if (flag_1000ms) {
          flag_1000ms = 0;
          start = SysTick->VAL;

          // 1 second tasks

          end = SysTick->VAL;
          exec_time_1000ms = SYSTICK_DIFF(start, end);
      }
  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO clock if not already
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7; // SCL and SDA
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB11 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ADS1119_Reset(I2C_HandleTypeDef *hi2c){
    uint8_t reset_cmd = ADS1119_CMD_RESET;
    HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, &reset_cmd, 1, HAL_MAX_DELAY);

    HAL_Delay(5);  // Allow reset time
}


void ADS1119_Start(I2C_HandleTypeDef *hi2c){
    // 3. Start conversion
    uint8_t start_cmd = ADS1119_CMD_START;
    HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, &start_cmd, 1, HAL_MAX_DELAY);
}

void ADS1119_Config(I2C_HandleTypeDef *hi2c, int port_number){

	   // 2. Configure for AIN0-AIN1, gain = 1, 20SPS, continuous, internal ref
	    uint8_t config_data[2] ;
	    config_data[0] = ADS1119_REG_CONFIG;

	    if (port_number==0)
	    	config_data[1] = CONFIG_AIN0;
	    if (port_number==1)
			config_data[1] = CONFIG_AIN1;
	    if (port_number==2)
			config_data[1] = CONFIG_AIN2;
	    if (port_number==3)
			config_data[1] = CONFIG_AIN3;

	    HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, config_data, 2, HAL_MAX_DELAY);

}

int16_t ADS1119_Read(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    // 5. Read conversion result
    uint8_t read_cmd = ADS1119_CMD_READ;
    ret = HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, &read_cmd, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return -10003;

    ret = HAL_I2C_Master_Receive(hi2c, ADS1119_ADDR, data, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) return -10004;

    int16_t adc_result = (data[0] << 8) | data[1];
    return adc_result;
}

void Analog_Read_ALL(void){


	ADS1119_Config(&hi2c1,0);
	ADS1119_Start(&hi2c1);

	SetMUX(0b00000000);
    adc_analog_in_raw[0] = ADS1119_Read(&hi2c1); 	//ADC1

    SetMUX(0b00000101);
    // NC

    SetMUX(0b00001010);
    pt_analog_in_raw[0] = ADS1119_Read(&hi2c1); 	//PT1

    SetMUX(0b00001111);
    adc_analog_in_raw[1] = ADS1119_Read(&hi2c1); 	//ADC2



	ADS1119_Config(&hi2c1,1);
	ADS1119_Start(&hi2c1);

	SetMUX(0b00000000);
	pt_analog_in_raw[1] = ADS1119_Read(&hi2c1);	//PT2

    SetMUX(0b00000101);
    pt_analog_in_raw[7] = ADS1119_Read(&hi2c1);	//PT8

    SetMUX(0b00001010);
    pt_analog_in_raw[3] = ADS1119_Read(&hi2c1);	//PT4

    SetMUX(0b00001111);
    pt_analog_in_raw[5] = 0;//ADS1119_Read(&hi2c1,1);	//PT6 PCB ERROR NOT CONNECTED



	ADS1119_Config(&hi2c1,2);
	ADS1119_Start(&hi2c1);

	SetMUX(0b00000000);
	pt_analog_in_raw[2] = ADS1119_Read(&hi2c1);	//PT3

    SetMUX(0b00000101);
    pt_analog_in_raw[4] = ADS1119_Read(&hi2c1);	//PT5

    SetMUX(0b00001010);
    pt_analog_in_raw[6] = ADS1119_Read(&hi2c1);	//PT7

    SetMUX(0b00001111);
    height_left_analog_in_raw = ADS1119_Read(&hi2c1);//Height Left


	ADS1119_Config(&hi2c1,3);
	ADS1119_Start(&hi2c1);

	SetMUX(0b00000000);
	// NC

    SetMUX(0b00000101);
    height_right_analog_in_raw = ADS1119_Read(&hi2c1);//Height Right

    SetMUX(0b00001010);
    // NC

    SetMUX(0b00001111);
    // NC

}

void SetMUX(uint8_t mux_value) {
    // Extract each bit from the mux_value
    uint8_t mux1_a0 = (mux_value >> 0) & 0x01;
    uint8_t mux1_a1 = (mux_value >> 1) & 0x01;
    uint8_t mux2_a0 = (mux_value >> 2) & 0x01;
    uint8_t mux2_a1 = (mux_value >> 3) & 0x01;

    // Set the GPIOs accordingly
    HAL_GPIO_WritePin(GPIOB, MUX1_A0_B, mux1_a0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MUX1_A1_B, mux1_a1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MUX2_A0_B, mux2_a0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MUX2_A1_B, mux2_a1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    for(int i = 0; i<10000; i++){ // at least 8000
    	//empty loop to allow time for MUX to set
    	__NOP();
    }

}

void ConvertAllAdcToMillivolts(void) {
    // Convert PT analog inputs (8 channels)
    for(int i = 0; i < 8; i++) {
        pt_analog_in_mv[i] = pt_analog_in_raw[i] / 16;
    }

    // Convert ADC analog inputs (2 channels)
    for(int i = 0; i < 2; i++) {
        adc_analog_in_mv[i] = adc_analog_in_raw[i] / 16;
    }

    // Convert height sensors
    height_right_analog_in_mv = height_right_analog_in_raw / 16;
    height_left_analog_in_mv = height_left_analog_in_raw / 16;
}


int32_t ADS1119_ConvertToMillivolts(int16_t raw_value) {
    // 2.048V = 2048mV, 32768 = 2^15
    // Calculation: (raw_value * 2048) / 32768
    // Which simplifies to: raw_value / 16
    return (int32_t)raw_value / 16;
    // Or even simpler: return raw_value / 16;
}

void ConvertMilliVoltsToDistance(void) {
    distance_right_mm = LookupWithInterpolation(&height_right_table, height_right_analog_in_mv);
    distance_left_mm = LookupWithInterpolation(&height_left_table, height_left_analog_in_mv);
}

void ConvertMilliVoltsToTemperature(void){
    for(int i = 0; i < 8; i++) {
        pt_analog_in_temp[i] = LookupWithInterpolation(&temp_table, pt_analog_in_mv[i]);
    }
}


float LookupWithInterpolation(const LookupTable_t *table, int32_t input) {
    // Check for input below table range
    if(input <= table->input_values[0]) {
        return table->output_values[0];
    }

    // Check for input above table range
    if(input >= table->input_values[table->size-1]) {
        return table->output_values[table->size-1];
    }

    // Find the interval where input lies
    uint8_t i;
    for(i = 0; i < table->size-1; i++) {
        if(input < table->input_values[i+1]) {
            break;
        }
    }

    // Linear interpolation
    float slope = (table->output_values[i+1] - table->output_values[i]) /
                  (float)(table->input_values[i+1] - table->input_values[i]);

    return table->output_values[i] + slope * (input - table->input_values[i]);
}

void SendMainDataToCan(void){

    // Initialize the header fields
	can_main_data_TxHeader.StdId = CAN_ID_MAIN_DATA;          // Standard ID
	can_main_data_TxHeader.ExtId = 0x00;           // Not using extended ID
	can_main_data_TxHeader.RTR = CAN_RTR_DATA;     // Data frame (not remote)
	can_main_data_TxHeader.IDE = CAN_ID_STD;       // Standard ID
	can_main_data_TxHeader.DLC = 8;                // Send 8 bytes
	can_main_data_TxHeader.TransmitGlobalTime = DISABLE;


	can_main_data[0] = (uint8_t) (((uint16_t) (distance_right_mm * 10)) >> 8); //lohigh
	can_main_data[1] = (uint8_t) (distance_right_mm * 10);

	//i primi due byte sono heightSensor2
	can_main_data[2] = (uint8_t) (((uint16_t) (distance_left_mm * 10)) >> 8); //loHigh
	can_main_data[3] = (uint8_t) (distance_left_mm * 10);

	//
	can_main_data[4] = (uint8_t) (((uint16_t) adc_analog_in_mv[0]) >> 8); //loHigh
	can_main_data[5] = (uint8_t) (adc_analog_in_mv[0]);


	can_main_data[6] = (uint8_t) (((uint16_t) adc_analog_in_mv[1]) >> 8); //loHigh
	can_main_data[7] = (uint8_t) adc_analog_in_mv[1];


	if(HAL_CAN_AddTxMessage(&hcan, &can_main_data_TxHeader, can_main_data, &can_TxMailbox)!= HAL_OK)
	{
	Error_Handler();
	}
}


// Timer interrupt callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)  // Check if the interrupt came from TIM1
    {
        flag_1ms = 1;  // Set 1ms flag

        ms_counter++;

        // Every 10ms (when counter is multiple of 10)
        if (ms_counter % 10 == 0) {
            flag_10ms = 1;
        }

        // Every 100ms (when counter is multiple of 100)
        if (ms_counter % 100 == 0) {
            flag_100ms = 1;
        }

        // Every 1000ms (when counter is multiple of 1000)
        if (ms_counter % 1000 == 0) {
            flag_1000ms = 1;
            ms_counter = 0;  // Reset counter to avoid overflow
        }
    }
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
