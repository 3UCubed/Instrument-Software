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
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef *gpio;
	uint16_t pin;
} gpio_pins;

const gpio_pins gpios[] = { { GPIOB, GPIO_PIN_5 }, { GPIOB, GPIO_PIN_6 }, {
GPIOC, GPIO_PIN_10 }, { GPIOC, GPIO_PIN_13 }, { GPIOC, GPIO_PIN_7 }, {
GPIOC, GPIO_PIN_8 }, { GPIOC, GPIO_PIN_9 }, { GPIOC, GPIO_PIN_6 } };

const char *gpio_names[] = { "sys_on PB5", "800v_en PB6", "3v3_en PC10", "n150v_en PC13", "15v_en PC7", "n5v_en PC8", "5v_en PC9",
		"n3v3_en PC6" };

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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static const uint8_t ADT7410_1 = 0x48 << 1; // Use 8-bit address
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t REG_TEMP = 0x00;
HAL_StatusTypeDef ret;
uint8_t buf[2];
int16_t val;
float temp_c;
#define BUFFER_SIZE 100
uint8_t rx_buf[BUFFER_SIZE];
uint8_t rx_index;
//int gpio_count = 0;
int num_gpios = 8;
volatile uint32_t adcResultsDMA[17];
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);
int gpio_flags[17];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Receive_IT(&huart1, rx_buf, 1);
		char key = rx_buf[0];
		if (key == 'a' || key == 'b' || key == 'c' || key == 'd' || key == 'e' || key == 'f' || key == 'g' || key == 'h') {
			int charNum = -1;
			switch (key) {
			case 'a':
				charNum = 0;
				break;
			case 'b':
				charNum = 1;
				break;
			case 'c':
				charNum = 2;
				break;
			case 'd':
				charNum = 3;
				break;
			case 'e':
				charNum = 4;
				break;
			case 'f':
				charNum = 5;
				break;
			case 'g':
				charNum = 6;
				break;
			case 'h':
				charNum = 7;
				break;
			}

//			HAL_UART_Transmit(&huart1, "\x1B" , 1, 100);
//			HAL_UART_Transmit(&huart1, "\x5B" , 1, 100);
//			HAL_UART_Transmit(&huart1, "\x32" , 1, 100);
//			HAL_UART_Transmit(&huart1, "\x4A" , 1, 100);
			// Read all the ADCs (adcResultsDMA needs to be uint32_t!!!)
			HAL_ADC_Start_DMA(&hadc, (uint32_t*) adcResultsDMA,
					adcChannelCount);
			for (int i = 0; i < num_gpios; i++) {

				// Changing to the next GPIO
				if (i == charNum) {
					if (HAL_GPIO_ReadPin(gpios[i].gpio, gpios[i].pin)
							== GPIO_PIN_SET) {
						HAL_GPIO_WritePin(gpios[i].gpio, gpios[i].pin,
								GPIO_PIN_RESET);
					} else {

						//			gpio_count == num_gpios - 1 ? gpio_count = 0 : gpio_count++;
						HAL_GPIO_TogglePin(gpios[i].gpio, gpios[i].pin);
						if (gpio_flags[i] == 1) {
						}
					}
				}

				HAL_UART_Transmit(&huart1, gpio_names[i], strlen(gpio_names[i]), 100);
				if (HAL_GPIO_ReadPin(gpios[i].gpio, gpios[i].pin)
						== GPIO_PIN_SET) {
					HAL_UART_Transmit(&huart1, ": H", 3, 100);
				} else {
					HAL_UART_Transmit(&huart1, ": L", 3, 100);
				}
				HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
			}

			HAL_UART_Transmit(&huart1, "\r\n", 2, 100);

			// For each ADC get its voltage
			for (int i = 0; i < adcChannelCount; i++) {

				// Parsing ADCs value based on gpio_count
				uint16_t adc = adcResultsDMA[i];
				uint8_t adcval[2];
				adcval[0] = ((adc & 0xFF00) >> 8); // ADC reading MSB
				adcval[1] = (adc & 0xFF); // ADC reading LSB

				// Processing results for UART Transmission

				char value[8];
				if (i == 0) { // When i is < 8 you read from one of the ADC channels
					float voltage = adc * (3.3 / 4095) * 6;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "END_mon PA0", 11, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 1) {
					float voltage = adc * (3.3 / 4095);
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "BUS_Imon PA1", 12, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 2) {
					float voltage = adc * (3.3 / 4095) * 2;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "BUS_Vmon PA2", 12, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 3) {
					float voltage = adc * (3.3 / 4095) * 1.1 ;
					// Need to multiply by 2
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "3v3_mon PA3", 11, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 4) {
					float voltage = adc * (3.3 / 4095) * -62.5;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "n150v_mon PA5", 13, 100);
					if (voltage > -150.1 && voltage < -149.9) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 5) {
					float voltage = adc * (3.3 / 4095) * -320;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "n800v_mon PA6", 13, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 6) {
					float voltage = adc * (3.3 / 4095);
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "SWP_mon PA7", 11, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 7) {
					float voltage = adc * (3.3 / 4095);
//					 voltage *= -2;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "TMP_1 PB0", 9, 100);
					if (voltage > -5.1 && voltage < -4.9) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 8) {
					float voltage = adc * (3.3 / 4095);
//					 voltage *= 2;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "TMP_2 PB1", 9, 100);
					if (voltage > -5.1 && voltage < -4.9) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 9) {
					float voltage = adc * (3.3 / 4095);
//					 voltage *= 2;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "2v5_mon PC0", 11, 100);
					if (voltage < 5.1 && voltage > 4.9) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 10) {
					float voltage = adc * (3.3 / 4095) * -1.65;
//					 voltage *= -1;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "n5v_mon PC1", 11, 100);
					if (voltage > -3.4 && voltage < -3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}
				} else if (i == 11) {
					float voltage = adc * (3.3 / 4095) * 1.67;
//					 voltage *= -50;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "5v_mon PC2", 10, 100);
					if (voltage > -150.1 && voltage < -149.9) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 12) {
					float voltage = adc * (3.3 / 4095) * -1.05;
//					 voltage *= 5;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "n3v3_mon PC3", 12, 100);
					if (voltage < 15.1 && voltage > 14.9) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 13) {
					float voltage = adc * (3.3 / 4095) * 1.67;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "5vref_mon PC4", 13, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}
				} else if (i == 14) { // for i = 13 you read the internal temperature
					// Should be 1.5ish for our actual Signal Board
					float voltage = adc * (3.3 / 4095) * 5;
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "15v_mon PC5", 11, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 15) { // for i = 13 you read the internal temperature
					// Should be 1.5ish for our actual Signal Board
					float voltage = adc * (3.3 / 4095);
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "TMPSENSE", 8, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				} else if (i == 16) { // for i = 14 you read the internal voltage
					// Should be 3.3 for our actual Signal Board
					float voltage = adc * (3.3 / 4095);
					sprintf(value, "%f", voltage);
					HAL_UART_Transmit(&huart1, "VREFINT", 7, 100);
					if (voltage < 3.4 && voltage > 3.2) {
						gpio_flags[i] = 1;
					} else if (voltage != 0) {
						gpio_flags[i] = 0;
					}

				}

				HAL_UART_Transmit(&huart1, ": ", 2, 100);
				HAL_UART_Transmit(&huart1, value, 8, 100);
				HAL_UART_Transmit(&huart1, "\r\n", 2, 100);

			}

			HAL_UART_Transmit(&huart1, "\r\n", 2, 100);

			 // Tell ADT7410_1 that we want to read from the temperature register
			 buf[0] = REG_TEMP;
			 ret = HAL_I2C_Master_Transmit(&hi2c1, ADT7410_1, buf, 1, 1000);
			 //			I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
			 if (ret != HAL_OK) {
			 strcpy((char*) buf, "Error Tx\r\n");
			 } else {

			 //				 Read 2 bytes from the temperature register
			 ret = HAL_I2C_Master_Receive(&hi2c1, ADT7410_1, buf, 2, 1000);
			 if (ret != HAL_OK) {
			 strcpy((char*) buf, "Error Rx\r\n");
			 } else {

			 val = (int16_t) (buf[0] << 8);
			 val = (val | buf[1]) >> 3;

			 // Convert to 2's complement, since temperature can be negative
			 if (val > 0x7FF) {
			 val |= 0xF000;
			 }

			 // Convert to float temperature value (Celsius)
			 temp_c = val * 0.0625;

			 // Convert temperature to decimal value
			 temp_c *= 100;

			 sprintf((char*) buf, "ADT7410_1: %u.%u C\r\n",
			 ((unsigned int) temp_c / 100),
			 ((unsigned int) temp_c % 100));
			 }
			 }

			 HAL_UART_Transmit(&huart1, buf, strlen((char*) buf), HAL_MAX_DELAY);

			 // Tell ADT7410_2 that we want to read from the temperature register
			 buf[0] = REG_TEMP;
			 ret = HAL_I2C_Master_Transmit(&hi2c1, ADT7410_2, buf, 1, 1000);
			 //			I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
			 if (ret != HAL_OK) {
			 strcpy((char*) buf, "Error Tx\r\n");
			 } else {

			 //				 Read 2 bytes from the temperature register
			 ret = HAL_I2C_Master_Receive(&hi2c1, ADT7410_2, buf, 2, 1000);
			 if (ret != HAL_OK) {
			 strcpy((char*) buf, "Error Rx\r\n");
			 } else {

			 val = (int16_t) (buf[0] << 8);
			 val = (val | buf[1]) >> 3;

			 // Convert to 2's complement, since temperature can be negative
			 if (val > 0x7FF) {
			 val |= 0xF000;
			 }

			 // Convert to float temperature value (Celsius)
			 temp_c = val * 0.0625;

			 // Convert temperature to decimal value
			 temp_c *= 100;

			 sprintf((char*) buf, "ADT7410 2: %u.%u C\r\n",
			 ((unsigned int) temp_c / 100),
			 ((unsigned int) temp_c % 100));
			 }
			 }

			 HAL_UART_Transmit(&huart1, buf, strlen((char*) buf), 100);
			 HAL_UART_Transmit(&huart1, "\r\n", 2, 100);

		}
	}
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

	/* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	for (int i = 0; i < 15; i++) {
		gpio_flags[i] = 1;
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_SET);

	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_UART_Receive_IT(&huart1, rx_buf, 1);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
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

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC6 PC7 PC8
                           PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
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

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
