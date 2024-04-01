/* USER CODE BEGIN Header */
// ---------------------------- FIRMWARE -------------------------------
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
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct erpa_data erpa_data;
typedef struct pmt_data pmt_data;
typedef struct hk_data hk_data;
typedef struct
{
  GPIO_TypeDef *gpio;
  uint16_t pin;
} gpio_pins;
const int HK_CADENCE = 1; // Should be set at 5
const gpio_pins gpios[] = {{GPIOB, GPIO_PIN_5}, {GPIOB, GPIO_PIN_6}, {GPIOC, GPIO_PIN_10}, {GPIOC, GPIO_PIN_13}, {GPIOC, GPIO_PIN_7}, {GPIOC, GPIO_PIN_8}, {GPIOC, GPIO_PIN_9}, {GPIOC, GPIO_PIN_6}, {GPIOF, GPIO_PIN_6}, {GPIOF, GPIO_PIN_7}};

struct pmt_data
{
  int pmt_raw;
  int pmt_seq;
};

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
#define BUFFER_SIZE 100
unsigned char rx_buf[BUFFER_SIZE];
UART_WakeUpTypeDef WakeUpSelection;

/* Hexadecimal Addresses for I2C Temperature Sensors */
static const uint8_t ADT7410_1 = 0x48 << 1;
static const uint8_t ADT7410_2 = 0x4A << 1;
static const uint8_t ADT7410_3 = 0x49 << 1;
static const uint8_t ADT7410_4 = 0x4B << 1;

/* Internal ADC DMA variables */
#define ADCRESULTSDMA_LENGTH 17
volatile uint16_t adcResultsDMA[ADCRESULTSDMA_LENGTH];                        // array to store results of Internal ADC Poll (not sure if it has to be "volatile")
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]); // variable to store the number of ADCs in use

/* DAC Variables for SWP */
uint32_t DAC_OUT[8] = {0, 620, 1241, 1861, 2482, 3103, 3723, 4095}; // For 3.3 volts
uint8_t step = 0;
int auto_sweep = 0;
int is_increasing = 1;

/* SPI Variables */
int pmt_raw;           // Stores raw pmt SPI value
int erpa_raw;          // Stores raw epra SPI value
const int WRITE = 0x1; // Hex 0x1 that is sent to external ADC to trigger transfer of data

/* ERPA Packet Variables */
int SAMPLING_FACTOR = 1;
int FACTOR_COUNTER = 0;

/* UART Variables */
const uint8_t erpa_sync = 0xAA; // SYNC byte to let packet interpreter / OBC know which packet is which
uint16_t erpa_seq = 0;          // SEQ byte which keeps track of what # ERPA packet is being sent (0-65535)
const uint8_t pmt_sync = 0xBB;  // SYNC byte to let packet interpreter / OBC know which packet is which
uint16_t pmt_seq = 0;           // SEQ byte which keeps track of what # ERPA packet is being sent (0-65535)
const uint8_t hk_sync = 0xCC;   // SYNC byte to let packet interpreter / OBC know which packet is which
uint16_t hk_seq = 0;            // SEQ byte which keeps track of what # HK packet is being sent (0-65535)

int hk_counter = 0; // counter to know when to send HK packet (sent every 50 ERPA packets)
                    // we put them in the same routine and send HK when this count == 50
uint8_t PMT_ON = 0;
uint8_t ERPA_ON = 0;
uint8_t HK_ON = 0;

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

/**
 * @brief function to poll individual i2c sensor
 *
 * @param TEMP_ADDR address of individual i2c sensor
 * @return int16_t integer result from sensor
 */
int16_t poll_i2c_sensor(const uint8_t TEMP_ADDR)
{
  int16_t output;
  uint8_t buf[2];
  HAL_StatusTypeDef ret;
  buf[0] = REG_TEMP;
  ret = HAL_I2C_Master_Transmit(&hi2c1, TEMP_ADDR, buf, 1,
                                1000);
  if (ret != HAL_OK)
  {
    strcpy((char *)buf, "Error Tx\r\n");
  }
  else
  {
    /* Read 2 bytes from the temperature register */
    ret = HAL_I2C_Master_Receive(&hi2c1, TEMP_ADDR, buf, 2,
                                 1000);
    if (ret != HAL_OK)
    {
      strcpy((char *)buf, "Error Rx\r\n");
    }
    else
    {
      output = (int16_t)(buf[0] << 8);
      output = (output | buf[1]) >> 3;
    }
  }
  return output;
}

/**
 * @brief called in hk routine to poll each i2c sensor
 *
 * @return int16_t* size 4 buffer to return i2c values
 */
int16_t *i2c()
{
  int16_t output1 = poll_i2c_sensor(ADT7410_1);
  int16_t output2 = poll_i2c_sensor(ADT7410_2);
  int16_t output3 = poll_i2c_sensor(ADT7410_3);
  int16_t output4 = poll_i2c_sensor(ADT7410_4);

  int16_t *results = malloc(4 * sizeof(int16_t));
  results[0] = output1;
  results[1] = output2;
  results[2] = output3;
  results[3] = output4;
  return results;
}

/**
 * @brief polls hk adcs
 *
 * @param adc which adc to call (currently not being used)
 * @return uint16_t* results of hk adcs
 */
uint16_t *hk_adc(ADC_HandleTypeDef *adc)
{

  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adcResultsDMA,
                    adcChannelCount);

  uint16_t PA1 = adcResultsDMA[1];       // ADC_IN1, BUS_Vmon: instrument bus voltage monitor
  uint16_t PA2 = adcResultsDMA[2];       // ADC_IN2, BUS_Imon: instrument bus current monitor
  uint16_t PA3 = adcResultsDMA[3];       // ADC_IN3, 3v3_mon: Accurate 5V for ADC monitor
  uint16_t PA5 = adcResultsDMA[4];       // ADC_IN5, n150v_mon: n150 voltage monitor
  uint16_t PA6 = adcResultsDMA[5];       // ADC_IN6, n800v_mon: n800 voltage monitor
  uint16_t PC0 = adcResultsDMA[9];       // ADC_IN10, 2v5_mon: 2.5v voltage monitor
  uint16_t PC1 = adcResultsDMA[10];      // ADC_IN11, n5v_mon: n5v voltage monitor
  uint16_t PC2 = adcResultsDMA[11];      // ADC_IN12, 5v_mon: 5v voltage monitor
  uint16_t PC3 = adcResultsDMA[12];      // ADC_IN13, n3v3_mon: n3v3 voltage monitor
  uint16_t PC4 = adcResultsDMA[13];      // ADC_IN14, 5vref_mon: 5v reference voltage monitor
  uint16_t PC5 = adcResultsDMA[14];      // ADC_IN15, 15v_mon: 15v voltage monitor
  uint16_t MCU_TEMP = adcResultsDMA[15]; //(internally connected) ADC_IN16, VSENSE
  uint16_t MCU_VREF = adcResultsDMA[16]; //(internally connected) ADC_IN17, VREFINT

  uint16_t *results = malloc(13 * sizeof(uint16_t));
  results[0] = MCU_TEMP;
  results[1] = MCU_VREF;
  results[2] = PA1;
  results[3] = PA2;
  results[4] = PC0;
  results[5] = PA3;
  results[6] = PC2;
  results[7] = PC3;
  results[8] = PC1;
  results[9] = PC5;
  results[10] = PC4;
  results[11] = PA5;
  results[12] = PA6;

  return results;
}

/**
 * @brief polls erpa adcs
 *
 * @return uint16_t* results of erpa adcs
 */
uint16_t *erpa_adc()
{

  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adcResultsDMA,
                    adcChannelCount);
  uint16_t PA0 = adcResultsDMA[0]; // ADC_IN0, END_mon: entrance/collimator monitor
  uint16_t PA7 = adcResultsDMA[6]; // ADC_IN7, SWP_mon: Sweep voltage monitor
  uint16_t PB0 = adcResultsDMA[7]; // ADC_IN8, TMP 1: Sweep temperature
  uint16_t PB1 = adcResultsDMA[8]; // ADC_IN9, TMP 2: feedbacks

  uint16_t *results = malloc(4 * sizeof(uint16_t));

  results[0] = PA0;
  results[1] = PA7;
  results[2] = PB0;
  results[3] = PB1;

  return results;
}

/**
 * @brief Set the erpa sweep value
 *
 */
void set_erpa_sweep()
{
  DAC->DHR12R1 = DAC_OUT[step];
}

/**
 * @brief if auto sweep is enabled, will increase step
 *
 */
void do_auto_sweep()
{
  if (step == 7)
  {
    is_increasing = 0;
  }
  else if (step == 0)
  {
    is_increasing = 1;
  }

  if (is_increasing)
  {
    step++;
  }
  else
  {
    step--;
  }

  DAC->DHR12R1 = DAC_OUT[step];
}

/**
 * @brief polls respective spi sensor and returns uint16_t value
 *
 * @param spi given spi peripheral
 * @return uint16_t raw value from reading spi peripheral
 */
uint16_t spi(SPI_HandleTypeDef spi)
{
  /* Write to SPI (begin transfer?) */
  HAL_SPI_Transmit(&spi, (uint8_t *)&WRITE, 1, 1);
  spi.Instance->CR1 &= ~(1 << 10); // THIS IS NEEDED TO STOP SPI1_SCK FROM GENERATING CLOCK PULSES
  while (!(spi.Instance->SR))
    ;
  uint16_t raw = spi.Instance->DR;
  return raw;
}

/**
 * @brief sends erpa packet via UART
 *
 * @param erpa_spi erpa spi raw value
 * @param erpa_adc_results erpa adc results
 */
void send_erpa_packet(int erpa_spi, uint16_t *erpa_adc_results)
{

  uint8_t erpa_buf[14];
  erpa_buf[0] = erpa_sync;                              // ERPA SYNC 0xAA MSB
  erpa_buf[1] = erpa_sync;                              // ERPA SYNC 0xAA LSB
  erpa_buf[2] = ((erpa_seq & 0xFF00) >> 8);             // ERPA SEQ # MSB
  erpa_buf[3] = (erpa_seq & 0xFF);                      // ERPA SEQ # MSB
  erpa_buf[4] = ((erpa_adc_results[0]) >> 8);           // ENDmon MSB
  erpa_buf[5] = (erpa_adc_results[0] & 0xFF);           // ENDmon LSB
  erpa_buf[6] = ((erpa_adc_results[1] & 0xFF00) >> 8);  // SWP Monitored MSB
  erpa_buf[7] = (erpa_adc_results[1] & 0xFF);           // SWP Monitored LSB
  erpa_buf[8] = ((erpa_adc_results[2] & 0xFF00) >> 8);  // TEMPURATURE 1 MSB
  erpa_buf[9] = (erpa_adc_results[2] & 0xFF);           // TEMPURATURE 1 LSB
  erpa_buf[10] = ((erpa_adc_results[3] & 0xFF00) >> 8); // TEMPURATURE 2 MSB
  erpa_buf[11] = (erpa_adc_results[3] & 0xFF);          // TEMPURATURE 2 LSB
  erpa_buf[12] = ((erpa_spi & 0xFF00) >> 8);            // ERPA eADC MSB
  erpa_buf[13] = (erpa_spi & 0xFF);                     // ERPA eADC LSB

  HAL_UART_Transmit(&huart1, erpa_buf, sizeof(erpa_buf), 100);

  erpa_seq++;
}

/**
 * @brief sends hk packet via UART
 *
 * @param i2c_values values of all i2c sensors
 * @param hk_adc_results hk adc results
 */
void send_hk_packet(int16_t *i2c_values, uint16_t *hk_adc_results)
{

  uint8_t hk_buf[38];                              // buffer that is filled with HK packet info
  hk_buf[0] = hk_sync;                             // HK SYNC 0xCC MSB					0 SYNC
  hk_buf[1] = hk_sync;                             // HK SYNC 0xCC LSB
  hk_buf[2] = ((hk_seq & 0xFF00) >> 8);            // HK SEQ # MSB		1 SEQUENCE
  hk_buf[3] = (hk_seq & 0xFF);                     // HK SEQ # LSB
  hk_buf[4] = ((hk_adc_results[0] & 0xFF00) >> 8); // VSENSE MSB		13 VSENSE
  hk_buf[5] = (hk_adc_results[0] & 0xFF);          // VSENSE LSB
  hk_buf[6] = ((hk_adc_results[1] & 0xFF00) >> 8);
  hk_buf[7] = (hk_adc_results[1] & 0xFF);
  hk_buf[8] = ((i2c_values[0] & 0xFF00) >> 8);
  hk_buf[9] = (i2c_values[0] & 0xFF);
  hk_buf[10] = ((i2c_values[1] & 0xFF00) >> 8);
  hk_buf[11] = (i2c_values[1] & 0xFF);
  hk_buf[12] = ((i2c_values[2] & 0xFF00) >> 8);
  hk_buf[13] = (i2c_values[2] & 0xFF);
  hk_buf[14] = ((i2c_values[3] & 0xFF00) >> 8);
  hk_buf[15] = (i2c_values[3] & 0xFF);
  hk_buf[16] = ((hk_adc_results[2] & 0xFF00) >> 8);  // BUS_Vmon MSB			2 BUS_VMON PA1
  hk_buf[17] = (hk_adc_results[2] & 0xFF);           // BUS_Vmon LSB
  hk_buf[18] = ((hk_adc_results[3] & 0xFF00) >> 8);  // BUS_Imon MSB			3 BUS_IMON PA2
  hk_buf[19] = (hk_adc_results[3] & 0xFF);           // BUS_Imon LSB
  hk_buf[20] = ((hk_adc_results[4] & 0xFF00) >> 8);  // 2v5_mon MSB			7 2V5_MON PC0
  hk_buf[21] = (hk_adc_results[4] & 0xFF);           // 2v5_mon LSB
  hk_buf[22] = ((hk_adc_results[5] & 0xFF00) >> 8);  // 3v3_mon MSB			4 3v3_MON PA3
  hk_buf[23] = (hk_adc_results[5] & 0xFF);           // 3v3_mon LSB
  hk_buf[24] = ((hk_adc_results[6]) >> 8);           // 5v_mon MSB			9 5V_MON PC2
  hk_buf[25] = (hk_adc_results[6] & 0xFF);           // 5v_mon LSB
  hk_buf[26] = ((hk_adc_results[7] & 0xFF00) >> 8);  // n3v3_mon MSB			10 N3V3_MON PC3
  hk_buf[27] = (hk_adc_results[7] & 0xFF);           // n3v3_mon LSB
  hk_buf[28] = ((hk_adc_results[8] & 0xFF00) >> 8);  // n5v_mon MSB			8 N5V_MON PC1
  hk_buf[29] = (hk_adc_results[8] & 0xFF);           // n5v_mon LSB
  hk_buf[30] = ((hk_adc_results[9] & 0xFF00) >> 8);  // 15v_mon MSB			12 15V_MON PC5
  hk_buf[31] = (hk_adc_results[9] & 0xFF);           // 15v_mon LSB
  hk_buf[32] = ((hk_adc_results[10] & 0xFF00) >> 8); // 5vref_mon MSB		11 5VREF_MON PC4
  hk_buf[33] = (hk_adc_results[10] & 0xFF);          // 5vref_mon LSB
  hk_buf[34] = ((hk_adc_results[11] & 0xFF00) >> 8); // n150v_mon MSB		5 N150V_MON PA5
  hk_buf[35] = (hk_adc_results[11] & 0xFF);          // n150v_mon LSB
  hk_buf[36] = ((hk_adc_results[12] & 0xFF00) >> 8); // n800v_mon MSB		6 N800V_MON PA6
  hk_buf[37] = (hk_adc_results[12] & 0xFF);          // n800v_mon LSB

  HAL_UART_Transmit(&huart1, hk_buf, sizeof(hk_buf), 100);
  hk_counter = 1;
  hk_seq++;
}

/**
 * @brief sends pmt packet via UART
 *
 * @param pmt_spi raw pmt spi value
 */
void send_pmt_packet(uint16_t pmt_spi)
{

  uint8_t pmt_buf[6];
  pmt_buf[0] = pmt_sync;
  pmt_buf[1] = pmt_sync;
  pmt_buf[2] = ((pmt_seq & 0xFF00) >> 8);
  pmt_buf[3] = (pmt_seq & 0xFF);
  pmt_buf[4] = ((pmt_raw & 0xFF00) >> 8);
  pmt_buf[5] = (pmt_raw & 0xFF);

  HAL_UART_Transmit(&huart1, pmt_buf, sizeof(pmt_buf), 100);
  pmt_seq++;
}

/**
 * @brief timer interrupt function
 * will either be passed htim1 or htim2
 * performs necessary routines for given timer
 *
 * @param htim
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    FACTOR_COUNTER++;
    int fc = FACTOR_COUNTER;
    int sc = SAMPLING_FACTOR;

    if (FACTOR_COUNTER == SAMPLING_FACTOR)
    {
      FACTOR_COUNTER = 0;
      if (ERPA_ON)
      {
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))
          ;

        erpa_raw = spi(hspi2);
        uint16_t *erpa_adc_results = erpa_adc();

        if (auto_sweep)
        {
          do_auto_sweep();
        } else {
          set_erpa_sweep();
        }

        send_erpa_packet(erpa_raw, erpa_adc_results);

        free(erpa_adc_results);
      }
      if (HK_ON)
      {
        int16_t *i2c_values = i2c();
        uint16_t *hk_adc_results = hk_adc(&hadc);

        send_hk_packet(i2c_values, hk_adc_results);

        free(i2c_values);
        free(hk_adc_results);
      }
      else
      {
        hk_counter++;
      }
    }
  }
  if (htim == &htim1)
  {
    if (PMT_ON)
    {
      while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
        ;

      pmt_raw = spi(hspi1);

      send_pmt_packet(pmt_raw);
    }
  }
}

/**
 * @brief UART rx interrupt function
 *
 * @param huart UART peripheral to receive from
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  HAL_UART_Receive_IT(&huart1, rx_buf, 1);
  unsigned char key = rx_buf[0];

  switch (key)
  {
  case 0x0B:
  {
    HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_SET);
    break;
  }
  case 0x0A:
  {
    HAL_GPIO_WritePin(gpios[8].gpio, gpios[8].pin, GPIO_PIN_RESET);
    break;
  }

  case 0x08:
  {
    HAL_GPIO_WritePin(gpios[9].gpio, gpios[9].pin, GPIO_PIN_SET);
    break;
  }
  case 0x09:
  {
    HAL_GPIO_WritePin(gpios[9].gpio, gpios[9].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x1B:
  {
    if (step < 7)
    {
      step++;
    }
    break;
  }
  case 0x1C:
  {
    if (step > 0)
    {
      step--;
    }
    break;
  }
  case 0x1D:
  {
    if (!auto_sweep)
    {
      auto_sweep = 1;
      step = 0;
    }
    else
    {
      auto_sweep = 0;
      step = 0;
    }
    break;
  }
  case 0x24:
  {
    if (SAMPLING_FACTOR < 32)
    {
      SAMPLING_FACTOR *= 2;
      FACTOR_COUNTER = 0;
    }
    break;
  }
  case 0x25:
  {
    if (SAMPLING_FACTOR > 1)
    {
      SAMPLING_FACTOR /= 2;
      FACTOR_COUNTER = 0;
    }
    break;
  }
  case 0x00:
  {
    HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_SET);
    break;
  }
  case 0x13:
  {
    HAL_GPIO_WritePin(gpios[0].gpio, gpios[0].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x01:
  {
    HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_SET);
    break;
  }
  case 0x14:
  {
    HAL_GPIO_WritePin(gpios[1].gpio, gpios[1].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x02:
  {
    HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_SET);
    break;
  }
  case 0x15:
  {
    HAL_GPIO_WritePin(gpios[2].gpio, gpios[2].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x03:
  {
    HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_SET);
    break;
  }
  case 0x16:
  {
    HAL_GPIO_WritePin(gpios[3].gpio, gpios[3].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x04:
  {
    HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_SET);
    break;
  }
  case 0x17:
  {
    HAL_GPIO_WritePin(gpios[4].gpio, gpios[4].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x05:
  {
    HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_SET);
    break;
  }
  case 0x18:
  {
    HAL_GPIO_WritePin(gpios[5].gpio, gpios[5].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x06:
  {
    HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_SET);
    break;
  }
  case 0x19:
  {
    HAL_GPIO_WritePin(gpios[6].gpio, gpios[6].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x07:
  {
    HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_SET);
    break;
  }
  case 0x1A:
  {
    HAL_GPIO_WritePin(gpios[7].gpio, gpios[7].pin, GPIO_PIN_RESET);
    break;
  }
  case 0x0C:
  {
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    NVIC_SystemReset();
    break;
  }
  case 0x0D:
  {
    PMT_ON = 1;
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
    break;
  }
  case 0x10:
  {
    PMT_ON = 0;
    HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
    break;
  }
  case 0x0E:
  {
    ERPA_ON = 1;
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
    break;
  }
  case 0x11:
  {
    ERPA_ON = 0;
    HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
    break;
  }
  case 0x0F:
  {
    HK_ON = 1;
    break;
  }
  case 0x12:
  {
    HK_ON = 0;
    break;
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
  HAL_StatusTypeDef ret;
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

  while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_BUSY) == SET)
    ;
  while (__HAL_UART_GET_FLAG(&huart1, USART_ISR_REACK) == RESET)
    ;

  WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_ADDRESS;
  WakeUpSelection.AddressLength = UART_ADDRESS_DETECT_7B;
  WakeUpSelection.Address = 0x5B; // send "["

  if (HAL_UARTEx_StopModeWakeUpSourceConfig(&huart1, WakeUpSelection) != HAL_OK)
  {
    Error_Handler();
  }
  /* Enable the LPUART Wake UP from stop mode Interrupt */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_WUF);

  /* enable MCU wake-up by LPUART */
  HAL_UARTEx_EnableStopMode(&huart1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  htim1.Init.Period = 48000 - 1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 480 -1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  htim2.Init.Prescaler = 1000 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 150 - 1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30 - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  huart1.Init.BaudRate = 57600;
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
  while (1)
  {
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
