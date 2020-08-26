/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RF_FILTERSEL_V3_Pin GPIO_PIN_2
#define RF_FILTERSEL_V3_GPIO_Port GPIOE
#define RF_FILTERSEL_V4_Pin GPIO_PIN_4
#define RF_FILTERSEL_V4_GPIO_Port GPIOE
#define RF_FILTERSEL_V5_Pin GPIO_PIN_5
#define RF_FILTERSEL_V5_GPIO_Port GPIOE
#define RF_FILTERSEL_V6_Pin GPIO_PIN_6
#define RF_FILTERSEL_V6_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define KEYBD_IRQ_Pin GPIO_PIN_0
#define KEYBD_IRQ_GPIO_Port GPIOC
#define TOUCH_IRQ_Pin GPIO_PIN_1
#define TOUCH_IRQ_GPIO_Port GPIOC
#define REF_PLL_LOCK_Pin GPIO_PIN_2
#define REF_PLL_LOCK_GPIO_Port GPIOC
#define TFT_DC_Pin GPIO_PIN_1
#define TFT_DC_GPIO_Port GPIOA
#define TFT_RST_Pin GPIO_PIN_2
#define TFT_RST_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_4
#define TFT_CS_GPIO_Port GPIOA
#define TFT_SCK_Pin GPIO_PIN_5
#define TFT_SCK_GPIO_Port GPIOA
#define TFT_MISO_Pin GPIO_PIN_6
#define TFT_MISO_GPIO_Port GPIOA
#define TFT_MOSI_Pin GPIO_PIN_7
#define TFT_MOSI_GPIO_Port GPIOA
#define FAN_PWM_Pin GPIO_PIN_0
#define FAN_PWM_GPIO_Port GPIOB
#define FAN_RPM_Pin GPIO_PIN_1
#define FAN_RPM_GPIO_Port GPIOB
#define AD9957_IOUP_Pin GPIO_PIN_8
#define AD9957_IOUP_GPIO_Port GPIOE
#define AD9957_CS_Pin GPIO_PIN_9
#define AD9957_CS_GPIO_Port GPIOE
#define AD9957_RESET_Pin GPIO_PIN_10
#define AD9957_RESET_GPIO_Port GPIOE
#define AD9957_IORESET_Pin GPIO_PIN_11
#define AD9957_IORESET_GPIO_Port GPIOE
#define AD9957_PLL_LOCK_Pin GPIO_PIN_12
#define AD9957_PLL_LOCK_GPIO_Port GPIOE
#define ADF4355_CS_Pin GPIO_PIN_13
#define ADF4355_CS_GPIO_Port GPIOE
#define ADF4355_LE_Pin GPIO_PIN_14
#define ADF4355_LE_GPIO_Port GPIOE
#define ADF4355_MUX_Pin GPIO_PIN_15
#define ADF4355_MUX_GPIO_Port GPIOE
#define SEL_PAATTEN_C_Pin GPIO_PIN_10
#define SEL_PAATTEN_C_GPIO_Port GPIOB
#define SEL_PAATTEN_D_Pin GPIO_PIN_11
#define SEL_PAATTEN_D_GPIO_Port GPIOB
#define MAX10_CS_Pin GPIO_PIN_12
#define MAX10_CS_GPIO_Port GPIOB
#define SEL_PAATTEN_A_Pin GPIO_PIN_8
#define SEL_PAATTEN_A_GPIO_Port GPIOD
#define SEL_PAATTEN_B_Pin GPIO_PIN_9
#define SEL_PAATTEN_B_GPIO_Port GPIOD
#define SEL_SIGSRC_B_Pin GPIO_PIN_6
#define SEL_SIGSRC_B_GPIO_Port GPIOC
#define SEL_SIGSRC_A_Pin GPIO_PIN_7
#define SEL_SIGSRC_A_GPIO_Port GPIOC
#define VUSB_SENSE_Pin GPIO_PIN_8
#define VUSB_SENSE_GPIO_Port GPIOA
#define ATTEN_0_Pin GPIO_PIN_0
#define ATTEN_0_GPIO_Port GPIOD
#define ATTEN_1_Pin GPIO_PIN_1
#define ATTEN_1_GPIO_Port GPIOD
#define ATTEN_2_Pin GPIO_PIN_2
#define ATTEN_2_GPIO_Port GPIOD
#define ATTEN_3_Pin GPIO_PIN_3
#define ATTEN_3_GPIO_Port GPIOD
#define ATTEN_4_Pin GPIO_PIN_4
#define ATTEN_4_GPIO_Port GPIOD
#define ATTEN_5_Pin GPIO_PIN_5
#define ATTEN_5_GPIO_Port GPIOD
#define ATTEN_6_Pin GPIO_PIN_6
#define ATTEN_6_GPIO_Port GPIOD
#define ATTEN_7_Pin GPIO_PIN_7
#define ATTEN_7_GPIO_Port GPIOD
#define RF_FILTERSEL_V1_Pin GPIO_PIN_0
#define RF_FILTERSEL_V1_GPIO_Port GPIOE
#define RF_FILTERSEL_V2_Pin GPIO_PIN_1
#define RF_FILTERSEL_V2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
