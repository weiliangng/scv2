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
#include "stm32g4xx_hal.h"

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
#define ADC1_IN1_Vcap_Pin GPIO_PIN_0
#define ADC1_IN1_Vcap_GPIO_Port GPIOA
#define ADC1_IN2_Vbus_Pin GPIO_PIN_1
#define ADC1_IN2_Vbus_GPIO_Port GPIOA
#define OPA1_DAC3_1_FBout_Pin GPIO_PIN_2
#define OPA1_DAC3_1_FBout_GPIO_Port GPIOA
#define DAC1_1_IMONINP_Pin GPIO_PIN_4
#define DAC1_1_IMONINP_GPIO_Port GPIOA
#define DAC1_2_IMONINN_Pin GPIO_PIN_5
#define DAC1_2_IMONINN_GPIO_Port GPIOA
#define COMP4_DIR_Pin GPIO_PIN_1
#define COMP4_DIR_GPIO_Port GPIOB
#define OPA3_IMONON_Pin GPIO_PIN_13
#define OPA3_IMONON_GPIO_Port GPIOB
#define OPA2_IMONOP_Pin GPIO_PIN_14
#define OPA2_IMONOP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
