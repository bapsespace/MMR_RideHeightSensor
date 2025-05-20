/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_can.h"  // CAN standard per STM32F1
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADS1119_ADDR     (0x40 << 1)  // 7-bit address + R/W bit
#define ADS1119_REG_CONFIG   0x40
#define ADS1119_CMD_START    0x08
#define ADS1119_CMD_READ     0x10
#define ADS1119_CMD_RESET    0x06

//Testato funziona NON MODIFICARE
#define CONFIG_AIN0   0b01100000 // AIN0 - AGND
#define CONFIG_AIN1   0b10000000 // AIN1 - AGND
#define CONFIG_AIN2   0b10100000 // AIN2 - AGND
#define CONFIG_AIN3   0b11000000 // AIN3 - AGND

#define MUX1_A0_B GPIO_PIN_5
#define MUX1_A1_B GPIO_PIN_4

#define MUX2_A0_B GPIO_PIN_11
#define MUX2_A1_B GPIO_PIN_10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// adc raw value from pin AIN0 AIN1 AIN2 AIN3
int16_t adc_raw[4] = {0,0,0,0};

// PCB analog pin raw value from multiplexer integration with adc
int16_t pt_analog_in_raw[8] = {0,0,0,0,0,0,0,0};
int16_t adc_analog_in_raw[2] = {0,0};
int16_t height_right_analog_in_raw = 0;
int16_t height_left_analog_in_raw = 0;

// PCB analog pin converted to voltage value from PCB raw
float adc_voltage[4] = {0,0,0,0};

float aux_analog_in_voltage[8] = {0,0,0,0,0,0,0,0};
float adc_analog_in_voltage[2] = {0,0};
float height_left_analog_in_voltage = 0;
float height__left_analog_in_voltage = 0;

// CONFIG byte for differential AIN0-AIN1, Gain = 1, 20 SPS, continuous mode
uint8_t config_data[2] = {0x01, 0b00000100};  // Register address + config byte

// Variabili CAN
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
uint32_t TxMailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADS1119_Reset(I2C_HandleTypeDef *hi2c);
void ADS1119_Start(I2C_HandleTypeDef *hi2c);
int16_t ADS1119_Read(I2C_HandleTypeDef *hi2c, int port_number);
void Analog_Read_ALL();

// Prototipo funzione callback CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
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
  /* USER CODE BEGIN 2 */
  ADS1119_Reset(&hi2c1);
  ADS1119_Start(&hi2c1);

  // Configure the CAN TX header
  TxHeader.StdId = 0x123;       // Standard CAN ID (11-bit)
  TxHeader.ExtId = 0x00;        // 0 per standard ID
  TxHeader.IDE = CAN_ID_STD;    // Standard ID (not extended)
  TxHeader.RTR = CAN_RTR_DATA;  // Data frame (not remote)
  TxHeader.DLC = 8;             // 8 bytes of data
  TxHeader.TransmitGlobalTime = DISABLE;

  // Start CAN
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  // Activate CAN RX notifications
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Analog_Read_ALL();

    // Invia i dati CAN
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
      Error_Handler();
    }

    HAL_Delay(100);  // Ritardo per evitare di saturare il bus CAN
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/* USER CODE BEGIN 4 */

void ADS1119_Reset(I2C_HandleTypeDef *hi2c){
    uint8_t reset_cmd = ADS1119_CMD_RESET;
    HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, &reset_cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(5);  // Allow reset time
}

void ADS1119_Start(I2C_HandleTypeDef *hi2c){
    uint8_t start_cmd = ADS1119_CMD_START;
    HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, &start_cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(60);
}

int16_t ADS1119_Read(I2C_HandleTypeDef *hi2c, int port_number) {
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    uint8_t config_data[2];
    config_data[0] = ADS1119_REG_CONFIG;

    if (port_number==0)
        config_data[1] = CONFIG_AIN0;
    if (port_number==1)
        config_data[1] = CONFIG_AIN1;
    if (port_number==2)
        config_data[1] = CONFIG_AIN2;
    if (port_number==3)
        config_data[1] = CONFIG_AIN3;

    ret = HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, config_data, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) return -10001;

    ADS1119_Start(hi2c);

    uint8_t read_cmd = ADS1119_CMD_READ;
    ret = HAL_I2C_Master_Transmit(hi2c, ADS1119_ADDR, &read_cmd, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return -10003;

    ret = HAL_I2C_Master_Receive(hi2c, ADS1119_ADDR, data, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) return -10004;

    int16_t adc_result = (data[0] << 8) | data[1];
    return adc_result;
}

void Analog_Read_ALL(){
    // MUX1
    HAL_GPIO_WritePin(GPIOB, MUX1_A1_B, 0);
    HAL_GPIO_WritePin(GPIOB, MUX1_A0_B, 0);
    // MUX2
    HAL_GPIO_WritePin(GPIOB, MUX2_A1_B, 0);
    HAL_GPIO_WritePin(GPIOB, MUX2_A0_B, 0);

    adc_analog_in_raw[0] = ADS1119_Read(&hi2c1,0);   //ADC1
    pt_analog_in_raw[1] = ADS1119_Read(&hi2c1,1);    //PT2
    pt_analog_in_raw[2] = ADS1119_Read(&hi2c1,2);    //PT3
    ADS1119_Read(&hi2c1,3);                          // Not connected

    // MUX1
    HAL_GPIO_WritePin(GPIOB, MUX1_A1_B, 0);
    HAL_GPIO_WritePin(GPIOB, MUX1_A0_B, 1);
    // MUX2
    HAL_GPIO_WritePin(GPIOB, MUX2_A1_B, 0);
    HAL_GPIO_WritePin(GPIOB, MUX2_A0_B, 1);

    ADS1119_Read(&hi2c1,0);                          //Not Connected
    pt_analog_in_raw[7] = ADS1119_Read(&hi2c1,1);    //PT8
    pt_analog_in_raw[4] = ADS1119_Read(&hi2c1,2);    //PT5
    height_left_analog_in_raw = ADS1119_Read(&hi2c1,3);//Height Left

    // MUX1
    HAL_GPIO_WritePin(GPIOB, MUX1_A1_B, 1);
    HAL_GPIO_WritePin(GPIOB, MUX1_A0_B, 0);
    // MUX2
    HAL_GPIO_WritePin(GPIOB, MUX2_A1_B, 1);
    HAL_GPIO_WritePin(GPIOB, MUX2_A0_B, 0);

    pt_analog_in_raw[0] = ADS1119_Read(&hi2c1,0);    //PT1
    pt_analog_in_raw[3] = ADS1119_Read(&hi2c1,1);    //PT4
    pt_analog_in_raw[6] = ADS1119_Read(&hi2c1,2);    //PT7
    ADS1119_Read(&hi2c1,3);                          // Not connected

    // MUX1
    HAL_GPIO_WritePin(GPIOB, MUX1_A1_B, 1);
    HAL_GPIO_WritePin(GPIOB, MUX1_A0_B, 1);
    // MUX2
    HAL_GPIO_WritePin(GPIOB, MUX2_A1_B, 1);
    HAL_GPIO_WritePin(GPIOB, MUX2_A0_B, 1);

    adc_analog_in_raw[1] = ADS1119_Read(&hi2c1,0);   //ADC2
    pt_analog_in_raw[5] = ADS1119_Read(&hi2c1,1);    //PT6
    height_right_analog_in_raw = ADS1119_Read(&hi2c1,2);//Height Right
    ADS1119_Read(&hi2c1,3);                          // Not connected
}

// Callback per la ricezione CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // Gestisci qui i messaggi CAN ricevuti
        // RxHeader contiene le informazioni sul messaggio
        // RxData contiene i dati ricevuti
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
