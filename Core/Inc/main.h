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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOC
#define PHASE_A_CURRENT_ADC_Pin GPIO_PIN_0
#define PHASE_A_CURRENT_ADC_GPIO_Port GPIOA
#define PHASE_B_CURRENT_ADC_Pin GPIO_PIN_1
#define PHASE_B_CURRENT_ADC_GPIO_Port GPIOA
#define BATTERY_VOLTAGE_Pin GPIO_PIN_2
#define BATTERY_VOLTAGE_GPIO_Port GPIOA
#define HALL_B_Pin GPIO_PIN_4
#define HALL_B_GPIO_Port GPIOA
#define HALL_A_Pin GPIO_PIN_6
#define HALL_A_GPIO_Port GPIOA
#define HALL_C_Pin GPIO_PIN_0
#define HALL_C_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOB
#define MTR_PHASE_A_LOW_Pin GPIO_PIN_13
#define MTR_PHASE_A_LOW_GPIO_Port GPIOB
#define MTR_PHASE_B_LOW_Pin GPIO_PIN_14
#define MTR_PHASE_B_LOW_GPIO_Port GPIOB
#define MTR_PHASE_C_LOW_Pin GPIO_PIN_15
#define MTR_PHASE_C_LOW_GPIO_Port GPIOB
#define MTR_PHASE_A_HIGH_Pin GPIO_PIN_8
#define MTR_PHASE_A_HIGH_GPIO_Port GPIOA
#define MTR_PHASE_B_HIGH_Pin GPIO_PIN_9
#define MTR_PHASE_B_HIGH_GPIO_Port GPIOA
#define MTR_PHASE_C_HIGH_Pin GPIO_PIN_10
#define MTR_PHASE_C_HIGH_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
