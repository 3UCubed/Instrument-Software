/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"

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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UART_WakeUpTypeDef WakeUpSelection;
uint8_t Rx_data[1];  //  creating a buffer of 1 bytes

static const uint8_t ADT7410_1 = 0x48 << 1; // Use 8-bit address
static const uint8_t ADT7410_2 = 0x49 << 1;
static const uint8_t REG_TEMP = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* TS_CAL1 & TS_CAL2 */
//#define TS_CAL1 *((uint16_t*) 0x1FFFF7B8)
//#define TS_CAL2 *((uint16_t*) 0x1FFFF7C2)

/* ADC DMA */
volatile uint16_t adcResultsDMA[16];
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);


/* DAC Variables */
uint32_t DAC_OUT[] = {0, 683, 1365, 2048, 2730, 3413};
uint8_t step;
int up = 1;

/* SPI Variables */
int raw;
const int WRITE = 0x1;

/* Erpa Data Struct */
typedef struct ErpaData {
	uint8_t sync;
	int spiData;
	uint32_t swpComm;
	uint32_t swpMon;
	uint16_t endMon;
	uint16_t tmp1;
	uint16_t tmp2;
} erpaData;

/* UART Variables */
uint8_t *erpa_buf;
const uint8_t erpa_sync = 0xAA;
uint16_t erpa_seq = 0;
uint8_t pmt_buf[6];
const uint8_t pmt_sync = 0xBB;
uint16_t pmt_seq = 0;
uint8_t hk_buf[26];
const uint8_t hk_sync = 0xCC;
uint16_t hk_seq = 0;
int hk_counter = 0;

int startupTimer = 0;


uint8_t *fillErpaBuffer(erpaData data) {

	uint8_t *erpa_ret = malloc(16 * sizeof(uint8_t));

	erpa_ret[0] = data.sync; // ERPA SYNC 0xAA MSB
	erpa_ret[1] = data.sync; // ERPA SYNC 0xAA LSB
	erpa_ret[2] = ((erpa_seq & 0xFF00) >> 8); // ERPA SEQ # MSB
	erpa_ret[3] = (erpa_seq & 0xFF); // ERPA SEQ # MSB
	erpa_ret[4] = ((data.spiData & 0xFF00) >> 8); // ERPA eADC MSB
	erpa_ret[5] = (data.spiData & 0xFF); // ERPA eADC LSB
	erpa_ret[6] = ((data.swpComm & 0xFF00) >> 8); //SWP Commanded MSB
	erpa_ret[7] = (data.swpComm & 0xFF); //SWP Commanded LSB
	erpa_ret[8] = ((data.swpMon & 0xFF00) >> 8); // SWP Monitored MSB
	erpa_ret[9] = (data.swpMon & 0xFF); // SWP Monitored LSB
	erpa_ret[10] = ((data.tmp1 & 0xFF00) >> 8); // TEMPURATURE 1 MSB
	erpa_ret[11] = (data.tmp1 & 0xFF); // TEMPURATURE 1 LSB
	erpa_ret[12] = ((data.tmp2 & 0xFF00) >> 8); // TEMPURATURE 2 MSB
	erpa_ret[13] = (data.tmp2 & 0xFF); // TEMPURATURE 2 LSB
	erpa_ret[14] = ((data.endMon & 0xFF00) >> 8); // ENDmon MSB
	erpa_ret[15] = (data.endMon & 0xFF); // ENDmon LSB

	erpa_seq++;

	return erpa_ret;
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
        if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))) { //check pin state

            /**
             * TIM1_CH1 Interrupt
             * Sets CNV and samples ERPA's ADC
             * Steps DAC
             * +/- 0.5v Every 100ms
             */

            /* Write to SPI (begin transfer?) */
            HAL_SPI_Transmit(&hspi1, (uint8_t * ) & WRITE, 1, 1);

            while (!(SPI1->SR));

            raw = SPI1->DR;

            DAC->DHR12R1 = DAC_OUT[step];

            HAL_ADC_Start_DMA(&hadc, (uint32_t *) adcResultsDMA, adcChannelCount);
            uint16_t PA0 = adcResultsDMA[0]; //ADC_IN0, END_mon: entrance/collimator monitor
            uint16_t PA7 = adcResultsDMA[4]; //ADC_IN7, SWP_mon: Sweep voltage monitor
            uint16_t PB0 = adcResultsDMA[5]; //ADC_IN8, TMP 1: Sweep temperature
            uint16_t PB1 = adcResultsDMA[6]; //ADC_IN9, TMP 2: feedbacks

//            erpaData data;
//            data.sync = 0xAA;
//            data.spiData = raw;
//            data.swpComm = DAC->DHR12R1;
//            data.swpMon = PA7;
//            data.endMon = PA0;
//            data.tmp1 = PB0;
//            data.tmp2 = PB1;
//
//            erpa_buf = fillErpaBuffer(data);

            HAL_UART_Transmit(&huart1, erpa_buf, sizeof(erpa_buf), 100);

            if (step == 5) {
                up = 0;
            } else if (step == 0) {
                up = 1;
            }

            up == 1 ? step++ : step--;

            if (hk_counter == 50) {
                HAL_ADC_Start_DMA(&hadc, (uint32_t *) adcResultsDMA, adcChannelCount);

                uint16_t PA1 = adcResultsDMA[1]; //ADC_IN1, BUS_Vmon: instrument bus voltage monitor
                uint16_t PA2 = adcResultsDMA[2]; //ADC_IN2, BUS_Imon: instrument bus current monitor
                uint16_t PA3 = adcResultsDMA[3]; //ADC_IN3, 5vref_mon: Accurate 5V for ADC monitor
                uint16_t PC0 = adcResultsDMA[7]; //ADC_IN10, 2v5_mon: power monitor
                uint16_t PC1 = adcResultsDMA[8]; //ADC_IN11, 3v3_mon: power monitor
                uint16_t PC2 = adcResultsDMA[9]; //ADC_IN12, 5v_mon: power monitor
                uint16_t PC3 = adcResultsDMA[10]; //ADC_IN13, n3v3_mon: power monitor
                uint16_t PC4 = adcResultsDMA[11]; //ADC_IN14, n5v_mon: power monitor
                uint16_t PC5 = adcResultsDMA[12]; //ADC_IN15, 15v_mon: power monitor
                uint16_t MCU_TEMP = adcResultsDMA[13]; //(internally connected) ADC_IN16, VSENSE
                uint16_t MCU_VREF = adcResultsDMA[14]; //(internally connected) ADC_IN17, VREFINT

                hk_buf[0] = hk_sync; // HK SYNC 0xCC MSB
                hk_buf[1] = hk_sync; // HK SYNC 0xCC LSB
                hk_buf[2] = ((hk_seq & 0xFF00) >> 8); // HK SEQ # MSB
                hk_buf[3] = (hk_seq & 0xFF); // HK SEQ # LSB
                hk_buf[4] = ((PA1 & 0xFF00) >> 8); // BUS_Vmon MSB
                hk_buf[5] = (PA1 & 0xFF); // BUS_Vmon LSB
                hk_buf[6] = ((PA2 & 0xFF00) >> 8); // BUS_Imon MSB
                hk_buf[7] = (PA2 & 0xFF); // BUS_Imon LSB
                hk_buf[8] = ((PC0 & 0xFF00) >> 8); // 2.5v_mon MSB
                hk_buf[9] = (PC0 & 0xFF); // 2.5v_mon LSB
                hk_buf[10] = ((PC1 & 0xFF00) >> 8); // 3v3_mon MSB
                hk_buf[11] = (PC1 & 0xFF); // 3v3_mon LSB
                hk_buf[12] = ((PC2 & 0xFF00) >> 8); // 5v_mon MSB
                hk_buf[13] = (PC2 & 0xFF); // 5v_mon LSB
                hk_buf[14] = ((PA3 & 0xFF00) >> 8); // 5vref_mon MSB
                hk_buf[15] = (PA3 & 0xFF); // 5vref_mon LSB
                hk_buf[16] = ((PC5 & 0xFF00) >> 8); // 15v_mon MSB
                hk_buf[17] = (PC5 & 0xFF); // 15v_mon LSB
                hk_buf[18] = ((PC3 & 0xFF00) >> 8); // n3v3_mon MSB
                hk_buf[19] = (PC3 & 0xFF); // n3v3_mon LSB
                hk_buf[20] = ((PC4 & 0xFF00) >> 8); // n5v_mon MSB
                hk_buf[21] = (PC4 & 0xFF); // n5v_mon LSB
                hk_buf[22] = ((MCU_TEMP & 0xFF00) >> 8); // VSENSE MSB
                hk_buf[23] = (MCU_TEMP & 0xFF); // VSENSE LSB
                hk_buf[24] = ((MCU_VREF & 0xFF00) >> 8); // VREFINT MSB
                hk_buf[25] = (MCU_VREF & 0xFF); // VREFINT LSB

                HAL_UART_Transmit(&huart1, hk_buf, sizeof(hk_buf), 100);

                hk_counter = 1;

                hk_seq++;

            } else {
                hk_counter++;
            }

        }
    } else if (htim == &htim2) {
        if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))) { //check pin state

            /**
             * TIM2_CH4 Interrupt
             * Sets CNV and samples UVPMT's ADC
             * Every 125ms
             */

            /* Write to SPI (begin transfer?) */
            HAL_SPI_Transmit(&hspi2, (uint8_t * ) & WRITE, 1, 1);

            while (!(SPI2->SR));

            raw = SPI2->DR;


            pmt_buf[0] = pmt_sync;
            pmt_buf[1] = pmt_sync;
            pmt_buf[2] = ((pmt_seq & 0xFF00) >> 8);
            pmt_buf[3] = (pmt_seq & 0xFF);;
            pmt_buf[4] = ((raw & 0xFF00) >> 8);
            pmt_buf[5] = (raw & 0xFF);

            pmt_seq++;

            HAL_UART_Transmit(&huart1, pmt_buf, sizeof(pmt_buf), 100);


        }
    }


    /* Timer 3 also called but doesn't need to do anything on IT */
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char sleepChar = Rx_data;
	if (Rx_data[0] == 's') { // should be "¶" in the future
		HAL_SuspendTick();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		NVIC_SystemReset();
	}
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);
}

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
  HAL_StatusTypeDef ret;
  uint8_t buf[2];
  int16_t val;
  float temp_c;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);


    /* Start Timers with OC & Interrupt */
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);


    while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_BUSY) == SET);
    while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_REACK) == RESET);

    WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_ADDRESS;
    WakeUpSelection.AddressLength = UART_ADDRESS_DETECT_7B;
    WakeUpSelection.Address = 0x23; // send "£"

    if (HAL_UARTEx_StopModeWakeUpSourceConfig(&huart1, WakeUpSelection) != HAL_OK) {
        Error_Handler();
    }
    /* Enable the LPUART Wake UP from stop mode Interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_WUF);
    /* enable MCU wake-up by LPUART */
    HAL_UARTEx_EnableStopMode(&huart1);
    HAL_UART_Receive_IT(&huart1, Rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {


    	/*
    	// Tell ADT7410_1 that we want to read from the temperature register
		buf[0] = REG_TEMP;
		ret = HAL_I2C_Master_Transmit(&hi2c1, ADT7410_1, buf, 1, 1000);
		//I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if ( ret != HAL_OK ) {
		  strcpy((char*)buf, "Error Tx\r\n");
		} else {

		  // Read 2 bytes from the temperature register
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADT7410_1, buf, 2, 1000);
		  if ( ret != HAL_OK ) {
			strcpy((char*)buf, "Error Rx\r\n");
		  } else {

			val = (int16_t)(buf[0] << 8);
			val = (val | buf[1]) >> 3;

			// Convert to 2's complement, since temperature can be negative
			if ( val > 0x7FF ) {
			  val |= 0xF000;
			}

			// Convert to float temperature value (Celsius)
			temp_c = val * 0.0625;

			// Convert temperature to decimal value
			temp_c *= 100;

			sprintf((char*)buf,
						  "ADT7410_1: %u.%u C\r\n",
						  ((unsigned int)temp_c / 100),
						  ((unsigned int)temp_c % 100));
		  }
	   }

		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

		// Tell ADT7410_2 that we want to read from the temperature register
		buf[0] = REG_TEMP;
		ret = HAL_I2C_Master_Transmit(&hi2c1, ADT7410_2, buf, 1, 1000);
		//I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if ( ret != HAL_OK ) {
		  strcpy((char*)buf, "Error Tx\r\n");
		} else {

		  // Read 2 bytes from the temperature register
		  ret = HAL_I2C_Master_Receive(&hi2c1, ADT7410_2, buf, 2, 1000);
		  if ( ret != HAL_OK ) {
			strcpy((char*)buf, "Error Rx\r\n");
		  } else {

			val = (int16_t)(buf[0] << 8);
			val = (val | buf[1]) >> 3;

			// Convert to 2's complement, since temperature can be negative
			if ( val > 0x7FF ) {
			  val |= 0xF000;
			}

			// Convert to float temperature value (Celsius)
			temp_c = val * 0.0625;

			// Convert temperature to decimal value
			temp_c *= 100;

			sprintf((char*)buf,
						  "ADT7410 2: %u.%u C\r\n",
						  ((unsigned int)temp_c / 100),
						  ((unsigned int)temp_c % 100));
		  }
	   }

	HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
	*/


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
    step = 0;
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 24000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 12000 - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 6000 - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
    erpa_seq = 0;
    pmt_seq = 0;
    hk_seq = 0;
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7|BLUE_LED_Pin
                          |GREEN_LED_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC6 PC7 BLUE_LED_Pin
                           GREEN_LED_Pin PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7|BLUE_LED_Pin
                          |GREEN_LED_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF6 PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
