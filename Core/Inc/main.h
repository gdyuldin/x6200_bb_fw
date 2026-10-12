/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define UART4_TX_Pin GPIO_PIN_1
#define UART4_TX_GPIO_Port GPIOD
#define I2S3_WS_Pin GPIO_PIN_15
#define I2S3_WS_GPIO_Port GPIOA
#define FPGA_DATA_4_Pin GPIO_PIN_4
#define FPGA_DATA_4_GPIO_Port GPIOE
#define FPGA_DATA_3_Pin GPIO_PIN_3
#define FPGA_DATA_3_GPIO_Port GPIOE
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define FPGA_DATA_0_Pin GPIO_PIN_0
#define FPGA_DATA_0_GPIO_Port GPIOE
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
#define PA_BIAS_EN_Pin GPIO_PIN_4
#define PA_BIAS_EN_GPIO_Port GPIOB
#define USART2_TX_Pin GPIO_PIN_5
#define USART2_TX_GPIO_Port GPIOD
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define UART4_RX_Pin GPIO_PIN_14
#define UART4_RX_GPIO_Port GPIOH
#define FPGA_DATA_2_Pin GPIO_PIN_2
#define FPGA_DATA_2_GPIO_Port GPIOE
#define FPGA_DATA_1_Pin GPIO_PIN_1
#define FPGA_DATA_1_GPIO_Port GPIOE
#define T_R_RELAY_2_Pin GPIO_PIN_5
#define T_R_RELAY_2_GPIO_Port GPIOB
#define I2S3_EXT_SD_Pin GPIO_PIN_11
#define I2S3_EXT_SD_GPIO_Port GPIOC
#define I2S3_CK_Pin GPIO_PIN_10
#define I2S3_CK_GPIO_Port GPIOC
#define AUDIO_OUT_EN_Pin GPIO_PIN_13
#define AUDIO_OUT_EN_GPIO_Port GPIOH
#define FPGA_DATA_5_Pin GPIO_PIN_5
#define FPGA_DATA_5_GPIO_Port GPIOE
#define USART2_RX_Pin GPIO_PIN_6
#define USART2_RX_GPIO_Port GPIOD
#define AUDIO_OUT_ATT_Pin GPIO_PIN_15
#define AUDIO_OUT_ATT_GPIO_Port GPIOH
#define FPGA_DATA_6_Pin GPIO_PIN_6
#define FPGA_DATA_6_GPIO_Port GPIOE
#define I2C2_SCL_Pin GPIO_PIN_1
#define I2C2_SCL_GPIO_Port GPIOF
#define I2C2_SDA_Pin GPIO_PIN_0
#define I2C2_SDA_GPIO_Port GPIOF
#define RF_PA_RELAY_2_Pin GPIO_PIN_2
#define RF_PA_RELAY_2_GPIO_Port GPIOF
#define BOARD_DETECT_0_Pin GPIO_PIN_6
#define BOARD_DETECT_0_GPIO_Port GPIOF
#define RF_TX_RELAY_Pin GPIO_PIN_4
#define RF_TX_RELAY_GPIO_Port GPIOF
#define RF_RX_EN_Pin GPIO_PIN_5
#define RF_RX_EN_GPIO_Port GPIOF
#define EXT_DC_POWER_Pin GPIO_PIN_3
#define EXT_DC_POWER_GPIO_Port GPIOF
#define I2C4_SDA_Pin GPIO_PIN_13
#define I2C4_SDA_GPIO_Port GPIOD
#define BOARD_DETECT_2_Pin GPIO_PIN_8
#define BOARD_DETECT_2_GPIO_Port GPIOF
#define BOARD_DETECT_1_Pin GPIO_PIN_7
#define BOARD_DETECT_1_GPIO_Port GPIOF
#define CW_HOST_Pin GPIO_PIN_9
#define CW_HOST_GPIO_Port GPIOF
#define TIM4_CH1_Pin GPIO_PIN_12
#define TIM4_CH1_GPIO_Port GPIOD
#define POWER_BUTTON_Pin GPIO_PIN_10
#define POWER_BUTTON_GPIO_Port GPIOF
#define ADC1_EXT_ALC_Pin GPIO_PIN_5
#define ADC1_EXT_ALC_GPIO_Port GPIOC
#define ADC1_TEMP_Pin GPIO_PIN_1
#define ADC1_TEMP_GPIO_Port GPIOB
#define I2C3_SCL_Pin GPIO_PIN_7
#define I2C3_SCL_GPIO_Port GPIOH
#define STATUS_LED_GREEN_Pin GPIO_PIN_14
#define STATUS_LED_GREEN_GPIO_Port GPIOE
#define I2C4_SCL_Pin GPIO_PIN_11
#define I2C4_SCL_GPIO_Port GPIOH
#define POWER_HOLD_Pin GPIO_PIN_9
#define POWER_HOLD_GPIO_Port GPIOH
#define EXT_MIC_DETECT_Pin GPIO_PIN_2
#define EXT_MIC_DETECT_GPIO_Port GPIOH
#define TIM2_CH4_Pin GPIO_PIN_3
#define TIM2_CH4_GPIO_Port GPIOA
#define ADC1_FWD_Pin GPIO_PIN_11
#define ADC1_FWD_GPIO_Port GPIOF
#define FPGA_STROBE_0_Pin GPIO_PIN_8
#define FPGA_STROBE_0_GPIO_Port GPIOE
#define CODEC_RESET_Pin GPIO_PIN_10
#define CODEC_RESET_GPIO_Port GPIOB
#define I2C3_SDA_Pin GPIO_PIN_8
#define I2C3_SDA_GPIO_Port GPIOH
#define HEADPHONE_DETECT_Pin GPIO_PIN_12
#define HEADPHONE_DETECT_GPIO_Port GPIOH
#define ADC1_VEXT_Pin GPIO_PIN_1
#define ADC1_VEXT_GPIO_Port GPIOA
#define CW_PADDLE_TIP_Pin GPIO_PIN_4
#define CW_PADDLE_TIP_GPIO_Port GPIOH
#define CW_PADDLE_TIP_EXTI_IRQn EXTI4_IRQn
#define DAC1_OUT1_Pin GPIO_PIN_4
#define DAC1_OUT1_GPIO_Port GPIOA
#define DAC1_OUT2_Pin GPIO_PIN_5
#define DAC1_OUT2_GPIO_Port GPIOA
#define FPGA_DATA_7_Pin GPIO_PIN_7
#define FPGA_DATA_7_GPIO_Port GPIOE
#define LINUX_POWER_Pin GPIO_PIN_12
#define LINUX_POWER_GPIO_Port GPIOF
#define STATUS_LED_RED_Pin GPIO_PIN_15
#define STATUS_LED_RED_GPIO_Port GPIOE
#define EXT_MIC_PTT_Pin GPIO_PIN_6
#define EXT_MIC_PTT_GPIO_Port GPIOH
#define EXT_MIC_PTT_EXTI_IRQn EXTI9_5_IRQn
#define CW_PADDLE_RING_Pin GPIO_PIN_3
#define CW_PADDLE_RING_GPIO_Port GPIOH
#define CW_PADDLE_RING_EXTI_IRQn EXTI3_IRQn
#define ADC1_SPARE_Pin GPIO_PIN_4
#define ADC1_SPARE_GPIO_Port GPIOC
#define ADC1_REFL_Pin GPIO_PIN_6
#define ADC1_REFL_GPIO_Port GPIOA
#define FPGA_IRQ_Pin GPIO_PIN_10
#define FPGA_IRQ_GPIO_Port GPIOE
#define FPGA_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define ACC_TRX_Pin GPIO_PIN_14
#define ACC_TRX_GPIO_Port GPIOF
#define FPGA_STROBE_1_Pin GPIO_PIN_9
#define FPGA_STROBE_1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
