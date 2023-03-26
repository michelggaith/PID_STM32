/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Entrada1_Pin GPIO_PIN_0
#define Entrada1_GPIO_Port GPIOA
#define Entrada3_Pin GPIO_PIN_7
#define Entrada3_GPIO_Port GPIOA
#define Entrada4_Pin GPIO_PIN_0
#define Entrada4_GPIO_Port GPIOB
#define Entrada5_Pin GPIO_PIN_1
#define Entrada5_GPIO_Port GPIOB
#define Entrada6_Pin GPIO_PIN_10
#define Entrada6_GPIO_Port GPIOB
#define Entrada7_Pin GPIO_PIN_11
#define Entrada7_GPIO_Port GPIOB
#define Salida8_Pin GPIO_PIN_12
#define Salida8_GPIO_Port GPIOB
#define Salida7_Pin GPIO_PIN_13
#define Salida7_GPIO_Port GPIOB
#define Salida6_Pin GPIO_PIN_14
#define Salida6_GPIO_Port GPIOB
#define Salida5_Pin GPIO_PIN_15
#define Salida5_GPIO_Port GPIOB
#define Salida4_Pin GPIO_PIN_8
#define Salida4_GPIO_Port GPIOA
#define Salida3_Pin GPIO_PIN_9
#define Salida3_GPIO_Port GPIOA
#define Salida2_Pin GPIO_PIN_10
#define Salida2_GPIO_Port GPIOA
#define Salida1_Pin GPIO_PIN_11
#define Salida1_GPIO_Port GPIOA
#define Entrada8_Pin GPIO_PIN_9
#define Entrada8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
