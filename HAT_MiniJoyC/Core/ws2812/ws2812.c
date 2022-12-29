/**
  ******************************************************************************
  * File Name          : encoder.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include <stdlib.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/

uint8_t led_num;
uint32_t *color_buf = NULL;
void color_set_single(uint32_t color);
uint8_t rled[TOTAL_RGB] = {0};
uint8_t gled[TOTAL_RGB] = {0};
uint8_t bled[TOTAL_RGB] = {0};

void restart(void) {
  for (uint8_t i = 0; i < 113; i++) {
    delay_600ns();
  }
}

void GPIO_init(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void sk6812_init(uint8_t num) {
  color_buf = (uint32_t *)calloc(num, sizeof(uint32_t));
  led_num = num;
}

void neopixel_set_color(uint8_t num, uint32_t color) {
	uint8_t rled = (color >> 24) & 0xff;
	uint8_t gled = (color >> 16) & 0xff;
	uint8_t bled = (color >> 8)  & 0xff;
	color_buf[num] = gled << 16 | rled << 8 | bled;
}

void neopixel_set_single_color(uint8_t num, uint8_t color, uint8_t shiftnum) {
	if(shiftnum == 0)
		rled[num] = color;
	else if(shiftnum == 1)
		gled[num] = color;
	else if(shiftnum == 2)
		bled[num] = color;
	color_buf[num] = gled[num] << 16 | rled[num] << 8 | bled[num];
}

void neopixel_set_all_color(uint32_t *color) {
  for (uint8_t i = 0; i < led_num; i++) {
		uint8_t rled = (color[i] >> 24) & 0xff;
		uint8_t gled = (color[i] >> 16) & 0xff;
		uint8_t bled = (color[i] >> 8)  & 0xff;
		color_buf[i] = gled << 16 | rled << 8 | bled;
  }
}

void neopixel_show(void) {
  __disable_irq();
  color_set_single(color_buf[0]);
  __enable_irq();
  restart();
}

void color_set_single(uint32_t color) {
  for (uint8_t i = 0; i < 24; i++) {
    if (color & (1 << (23 - i))) {
      out_bit_high();
    }
    else {
      out_bit_low();
    }
  }
}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
