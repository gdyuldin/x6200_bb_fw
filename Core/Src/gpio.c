/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC9   ------> I2S_CKIN
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FPGA_DATA_4_Pin|FPGA_DATA_3_Pin|FPGA_DATA_0_Pin|FPGA_DATA_2_Pin
                          |FPGA_DATA_1_Pin|FPGA_DATA_5_Pin|FPGA_DATA_6_Pin|FPGA_STROBE_0_Pin
                          |FPGA_DATA_7_Pin|FPGA_STROBE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PA_BIAS_EN_Pin|T_R_RELAY_2_Pin|CODEC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, AUDIO_OUT_EN_Pin|AUDIO_OUT_ATT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, RF_PA_RELAY_2_Pin|RF_TX_RELAY_Pin|RF_RX_EN_Pin|ACC_TRX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STATUS_LED_GREEN_Pin|STATUS_LED_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_HOLD_GPIO_Port, POWER_HOLD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LINUX_POWER_GPIO_Port, LINUX_POWER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PB3 PB15 PB14 PB13
                           PB12 PB2 PB11 PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13
                          |GPIO_PIN_12|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG9 PG14 PG10
                           PG15 PG12 PG6 PG5
                           PG8 PG7 PG4 PG2
                           PG3 PG1 PG0 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_14|GPIO_PIN_10
                          |GPIO_PIN_15|GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD7 PD2 PD4
                           PD0 PD14 PD15 PD11
                           PD9 PD10 PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_11
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA14 PA13 PA8 PA12
                           PA10 PA11 PA9 PA7
                           PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_12
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_7
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FPGA_DATA_4_Pin FPGA_DATA_3_Pin FPGA_DATA_0_Pin FPGA_DATA_2_Pin
                           FPGA_DATA_1_Pin FPGA_DATA_5_Pin FPGA_DATA_6_Pin FPGA_STROBE_0_Pin
                           FPGA_DATA_7_Pin FPGA_STROBE_1_Pin */
  GPIO_InitStruct.Pin = FPGA_DATA_4_Pin|FPGA_DATA_3_Pin|FPGA_DATA_0_Pin|FPGA_DATA_2_Pin
                          |FPGA_DATA_1_Pin|FPGA_DATA_5_Pin|FPGA_DATA_6_Pin|FPGA_STROBE_0_Pin
                          |FPGA_DATA_7_Pin|FPGA_STROBE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA_BIAS_EN_Pin T_R_RELAY_2_Pin CODEC_RESET_Pin */
  GPIO_InitStruct.Pin = PA_BIAS_EN_Pin|T_R_RELAY_2_Pin|CODEC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC15 PC14 PC8
                           PC7 PC6 PC0 PC1
                           PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_8
                          |GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AUDIO_OUT_EN_Pin AUDIO_OUT_ATT_Pin POWER_HOLD_Pin */
  GPIO_InitStruct.Pin = AUDIO_OUT_EN_Pin|AUDIO_OUT_ATT_Pin|POWER_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_PA_RELAY_2_Pin RF_TX_RELAY_Pin RF_RX_EN_Pin LINUX_POWER_Pin */
  GPIO_InitStruct.Pin = RF_PA_RELAY_2_Pin|RF_TX_RELAY_Pin|RF_RX_EN_Pin|LINUX_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : BOARD_DETECT_0_Pin BOARD_DETECT_2_Pin BOARD_DETECT_1_Pin CW_HOST_Pin
                           POWER_BUTTON_Pin PF13 */
  GPIO_InitStruct.Pin = BOARD_DETECT_0_Pin|BOARD_DETECT_2_Pin|BOARD_DETECT_1_Pin|CW_HOST_Pin
                          |POWER_BUTTON_Pin|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_DC_POWER_Pin */
  GPIO_InitStruct.Pin = EXT_DC_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_DC_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_LED_GREEN_Pin STATUS_LED_RED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_GREEN_Pin|STATUS_LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_MIC_DETECT_Pin PH10 HEADPHONE_DETECT_Pin */
  GPIO_InitStruct.Pin = EXT_MIC_DETECT_Pin|GPIO_PIN_10|HEADPHONE_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CW_PADDLE_TIP_Pin EXT_MIC_PTT_Pin CW_PADDLE_RING_Pin */
  GPIO_InitStruct.Pin = CW_PADDLE_TIP_Pin|EXT_MIC_PTT_Pin|CW_PADDLE_RING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE13 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : FPGA_IRQ_Pin */
  GPIO_InitStruct.Pin = FPGA_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FPGA_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_TRX_Pin */
  GPIO_InitStruct.Pin = ACC_TRX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACC_TRX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
