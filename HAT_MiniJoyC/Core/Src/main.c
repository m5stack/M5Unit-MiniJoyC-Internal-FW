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
#include "i2c_ex.h"
#include "flash.h"
#include "ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FIRMWARE_VERSION 1
#define I2C_ADDRESS 0x54
#define BUTTON_FILTER 500
#define BUTTON_FILTER_TIMEROUT BUTTON_FILTER*3
#define FLASH_DATA_SIZE 16

#define X_DEFAULT_MIN 60
#define X_DEFAULT_MAX 3900
#define Y_DEFAULT_MIN 260
#define Y_DEFAULT_MAX 3800
#define X_DEFAULT_MID 1986
#define Y_DEFAULT_MID 2065
#define GAP_DEFAULT 1

#define UINT_MAX 1023
#define UINT_MIN 0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct location_cal_s
{
  uint8_t x_min_flag;
  uint16_t x_min;
  uint8_t x_max_flag;
  uint16_t x_max;
  uint8_t y_min_flag;
  uint16_t y_min;
  uint8_t y_max_flag;
  uint16_t y_max;
  uint8_t x_mid_flag;
  uint16_t x_mid;
  uint8_t y_mid_flag;
  uint16_t y_mid;
  uint8_t busy;
  uint8_t mode;
  uint8_t gap;
} location_cal_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
uint8_t flash_data[FLASH_DATA_SIZE] = {0};
uint8_t i2c_address[1] = {0};
volatile uint8_t fm_version = FIRMWARE_VERSION;
// ADC DMA data in buffer
__IO uint32_t uiAdcValueBuf[40];
__IO uint16_t usAdcValue16[2];
__IO int32_t calAdcValue16[2];
__IO int32_t calMidValueX;
__IO int32_t calMidValueY;
volatile location_cal_t cal_data = {
  .x_min = X_DEFAULT_MIN,
  .x_max = X_DEFAULT_MAX,
  .y_min = Y_DEFAULT_MIN,
  .y_max = Y_DEFAULT_MAX,
  .x_mid = X_DEFAULT_MID,
  .y_mid = Y_DEFAULT_MID,
  .x_min_flag = 0,
  .x_max_flag = 0,
  .y_min_flag = 0,
  .y_max_flag = 0,
  .x_mid_flag = 0,
  .y_mid_flag = 0,
  .busy = 0,
  .mode = 2,
  .gap = GAP_DEFAULT,
};
volatile uint16_t cal_data_set[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  long result;

  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min)
    result = out_min;
  else if (result > out_max)
    result = out_max;
  
  return result;
}

void user_i2c_init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = i2c_address[0]<<1;
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

}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    flash_data[0] = I2C_ADDRESS;
    flash_data[1] = 0;
    flash_data[2] = cal_data.x_min & 0xff;
    flash_data[3] = (cal_data.x_min >> 8) & 0xff;
    flash_data[4] = cal_data.x_max & 0xff;
    flash_data[5] = (cal_data.x_max >> 8) & 0xff;
    flash_data[6] = cal_data.y_min & 0xff;
    flash_data[7] = (cal_data.y_min >> 8) & 0xff;
    flash_data[8] = cal_data.y_max & 0xff;
    flash_data[9] = (cal_data.y_max >> 8) & 0xff;
    flash_data[10] = cal_data.x_mid & 0xff;
    flash_data[11] = (cal_data.x_mid >> 8) & 0xff;
    flash_data[12] = cal_data.y_mid & 0xff;
    flash_data[13] = (cal_data.y_mid >> 8) & 0xff;
    flash_data[14] = cal_data.gap;
    flash_data[15] = 0;
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    cal_data.x_min = flash_data[2] | (flash_data[3] << 8);
    cal_data_set[0] = cal_data.x_min;
    cal_data.x_max = flash_data[4] | (flash_data[5] << 8);
    cal_data_set[1] = cal_data.x_max;
    if (cal_data.x_max == 0)
      cal_data.x_max = X_DEFAULT_MAX;
    cal_data.y_min = flash_data[6] | (flash_data[7] << 8);
    cal_data_set[2] = cal_data.y_min;
    cal_data.y_max = flash_data[8] | (flash_data[9] << 8);
    cal_data_set[3] = cal_data.y_max;
    if (cal_data.y_max == 0)
      cal_data.y_max = Y_DEFAULT_MAX;    
    cal_data.x_mid = flash_data[10] | (flash_data[11] << 8);
    cal_data_set[4] = cal_data.x_mid;
    cal_data.y_mid = flash_data[12] | (flash_data[13] << 8);
    cal_data_set[5] = cal_data.y_mid;
    cal_data.gap = GAP_DEFAULT;
  }
}

void cover_mid_data(void)
{
  calMidValueX = map(cal_data.x_mid,cal_data.x_min,cal_data.x_max,UINT_MIN,UINT_MAX);
  calMidValueY = map(cal_data.y_mid,cal_data.y_min,cal_data.y_max,UINT_MIN,UINT_MAX);
}

uint8_t read_button_status(void)
{
  uint8_t button_status = 0; 
  uint8_t last_button_status = 0;
  uint16_t counter = 0;

  last_button_status = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
  for (uint16_t i = 0; i < BUTTON_FILTER_TIMEROUT; i++) {  
    button_status = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
    if (button_status == last_button_status) {
      counter++;
    }
    else {
      last_button_status = button_status;
      counter = 0;
    }
    if (counter >= BUTTON_FILTER) {
      return button_status;
    }
  }
  //TODO: Just for debug
  return 1;  
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  uint64_t adcTotal[2]={0};
  
	HAL_ADC_Stop_DMA(hadc);
	for (uint8_t i = 0; i < 40; i++) {
    adcTotal[i%2] += uiAdcValueBuf[i];
  }
	for (uint8_t i = 0; i < 2; i++) {
    usAdcValue16[i] = (adcTotal[i%2] / 20);
  }

	calAdcValue16[1] = map(usAdcValue16[1],cal_data.x_min,cal_data.x_max,UINT_MIN,UINT_MAX);
	calAdcValue16[0] = map(usAdcValue16[0],cal_data.y_min,cal_data.y_max,UINT_MIN,UINT_MAX); 

	HAL_ADC_Start_DMA(hadc, (uint32_t *)uiAdcValueBuf, 40);
}

#pragma GCC push_options
#pragma GCC optimize ("O0") 
int16_t change_adc_gap(int16_t number)
{
  int16_t unit_other, unit_bit;
  int16_t tmp_other, tmp_bit, tmp;  

  tmp_other = number / 10;
  unit_other = tmp_other * 10;
  tmp = number % 10;
  tmp_bit = tmp / cal_data.gap;
  unit_bit = tmp_bit * cal_data.gap;
  return (unit_other + unit_bit); 
}
#pragma GCC pop_options 

void cal_data_write_back(void)
{
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[2] = cal_data.x_min & 0xff;
    flash_data[3] = (cal_data.x_min >> 8) & 0xff;
    flash_data[4] = cal_data.x_max & 0xff;
    flash_data[5] = (cal_data.x_max >> 8) & 0xff;   
    flash_data[6] = cal_data.y_min & 0xff;
    flash_data[7] = (cal_data.y_min >> 8) & 0xff;
    flash_data[8] = cal_data.y_max & 0xff;
    flash_data[9] = (cal_data.y_max >> 8) & 0xff;   
    flash_data[10] = cal_data.x_mid & 0xff;
    flash_data[11] = (cal_data.x_mid >> 8) & 0xff;
    flash_data[12] = cal_data.y_mid & 0xff;
    flash_data[13] = (cal_data.y_mid >> 8) & 0xff;                                                  
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  }     
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t buf[4];
  int16_t tmp;
  int16_t result = 0;

  if(len == 1 && (rx_data[0] == 0x00 || rx_data[0] == 0x01)) 
  {
    buf[0] = usAdcValue16[1] & 0xff;
    buf[1] = (usAdcValue16[1] >> 8) & 0xff;
    i2c1_set_send_data(buf, 2);
	}
  else if(len == 1 && (rx_data[0] == 0x02 || rx_data[0] == 0x03)) 
  {
    buf[0] = usAdcValue16[0] & 0xff;
    buf[1] = (usAdcValue16[0] >> 8) & 0xff;    
    i2c1_set_send_data(buf, 2);
	}                 
  if(len == 1 && (rx_data[0] == 0x10 || rx_data[0] == 0x11)) 
  {
    tmp = calAdcValue16[1] - calMidValueX;
    if (tmp > 0) {
      result = map(tmp,0,UINT_MAX-calMidValueX,0,511);
    } else if (tmp < 0) {
      result = map(tmp,UINT_MIN-calMidValueX,0,-512,0);
    }
    // result = change_adc_gap(result);
    buf[0] = result & 0xff;
    buf[1] = (result >> 8) & 0xff;
    i2c1_set_send_data(buf, 2);
	}
  else if(len == 1 && (rx_data[0] == 0x12 || rx_data[0] == 0x13)) 
  {
    tmp = calAdcValue16[0] - calMidValueY;
    if (tmp > 0) {
      result = map(tmp,0,UINT_MAX-calMidValueY,0,511);
    } else if (tmp < 0) {
      result = map(tmp,UINT_MIN-calMidValueY,0,-512,0);
    }
    // result = change_adc_gap(result);
    buf[0] = result & 0xff;
    buf[1] = (result >> 8) & 0xff;
    i2c1_set_send_data(buf, 2);  
	}                 
  if(len == 1 && (rx_data[0] == 0x20)) 
  {
    tmp = calAdcValue16[1] - calMidValueX;
    if (tmp > 0) {
      result = map(tmp,0,UINT_MAX-calMidValueX,0,127);
    } else if (tmp < 0) {
      result = map(tmp,UINT_MIN-calMidValueX,0,-128,0);
    }
    buf[0] = result & 0xff;
    i2c1_set_send_data(buf, 1);
	}
  else if(len == 1 && (rx_data[0] == 0x21)) 
  {
    tmp = calAdcValue16[0] - calMidValueY;
    if (tmp > 0) {
      result = map(tmp,0,UINT_MAX-calMidValueY,0,127);
    } else if (tmp < 0) {
      result = map(tmp,UINT_MIN-calMidValueY,0,-128,0);
    }
    buf[0] = result & 0xff;
    i2c1_set_send_data(buf, 1);  
	}                 
  else if(len == 1 && (rx_data[0] == 0x30)) 
  {
    buf[0] = read_button_status();
    i2c1_set_send_data(buf, 1);
	}  
	else if (len > 1 && ((rx_data[0] >= 0x40) & (rx_data[0] <= 0x42))) 
	{
    if (len == 4) {
      neopixel_set_color(0, (rx_data[1] << 24) | (rx_data[2] << 16) | (rx_data[3] << 8));
      neopixel_show(); 
    }
  }                  
	else if (len == 1 && ((rx_data[0] >= 0x40) & (rx_data[0] <= 0x42))) 
	{
    buf[0] = (color_buf[0] >> 8) & 0xff;
    buf[1] = (color_buf[0] >> 16) & 0xff;
    buf[2] = (color_buf[0]) & 0xff;     
    i2c1_set_send_data(buf, 3);
  }                  
	else if (len > 1 && ((rx_data[0] >= 0x50) & (rx_data[0] <= 0x5B))) 
	{

    for (int i = 0; i < (len - 1) / 2; i++) {    
      cal_data_set[(rx_data[0]+i*2 - 0x50) / 2] = (rx_data[2+i*2] << 8) | (rx_data[1+i*2]);
    }
    cal_data.x_min = cal_data_set[0];
    cal_data.x_max = cal_data_set[1];
    cal_data.y_min = cal_data_set[2];
    cal_data.y_max = cal_data_set[3];
    cal_data.x_mid = cal_data_set[4];
    cal_data.y_mid = cal_data_set[5]; 
    cal_data_write_back(); 
    cover_mid_data(); 
  }                  
	else if (len == 1 && ((rx_data[0] >= 0x50) & (rx_data[0] <= 0x5B))) 
	{
    uint8_t cmd = 0;

    if (rx_data[0] <= 0x5B)
      cmd = (rx_data[0] - 0x50) / 2;
    else
      cmd = rx_data[0];

    switch (cmd) {
    case 0:
      i2c1_set_send_data((uint8_t *)&cal_data.x_min, 2);
      break;
    case 1:
      i2c1_set_send_data((uint8_t *)&cal_data.x_max, 2);
      break;
    case 2:
      i2c1_set_send_data((uint8_t *)&cal_data.y_min, 2);
      break;                   
    case 3:
      i2c1_set_send_data((uint8_t *)&cal_data.y_max, 2);
      break;          
    case 4:
      i2c1_set_send_data((uint8_t *)&cal_data.x_mid, 2);
      break;  
    case 5:
      i2c1_set_send_data((uint8_t *)&cal_data.y_mid, 2);
      break;                        
    
    default:
      break;
    } 

  }                  
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c1_set_send_data((uint8_t *)&fm_version, 1);
  }
  else if (len > 1 && (rx_data[0] == 0xFF))
  {
    if (len == 2) {
      if ((rx_data[1] >= 0) && (rx_data[1] < 128)) {
        if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
          i2c_address[0] = rx_data[1];
          flash_data[0] = i2c_address[0];
          flash_data[1] = 0;
          writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
          user_i2c_init();
        }        
      }
    }    
  }
  else if (len == 1 && (rx_data[0] == 0xFF))
  {
    i2c1_set_send_data(i2c_address, 1);    
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  // MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init_flash_data();
  cover_mid_data();
  user_i2c_init(); 
  sk6812_init(TOTAL_RGB); 
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)uiAdcValueBuf, 40);
  // HAL_ADC_Start(&hadc);
  HAL_I2C_EnableListen_IT(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
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
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
  hi2c1.Init.OwnAddress1 = 0x54<<1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_Pin */
  GPIO_InitStruct.Pin = RGB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_GPIO_Port, &GPIO_InitStruct);

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

