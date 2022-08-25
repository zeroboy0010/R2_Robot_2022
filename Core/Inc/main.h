/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define step1_Pin GPIO_PIN_2
#define step1_GPIO_Port GPIOB
#define dir1_Pin GPIO_PIN_7
#define dir1_GPIO_Port GPIOE
#define step2_Pin GPIO_PIN_8
#define step2_GPIO_Port GPIOE
#define dir2_Pin GPIO_PIN_9
#define dir2_GPIO_Port GPIOE
#define M4_CHB_Pin GPIO_PIN_12
#define M4_CHB_GPIO_Port GPIOB
#define M4_CHB_EXTI_IRQn EXTI15_10_IRQn
#define M4_CHA_Pin GPIO_PIN_13
#define M4_CHA_GPIO_Port GPIOB
#define M4_CHA_EXTI_IRQn EXTI15_10_IRQn
#define M3_CHB_Pin GPIO_PIN_14
#define M3_CHB_GPIO_Port GPIOB
#define M3_CHB_EXTI_IRQn EXTI15_10_IRQn
#define M3_CHA_Pin GPIO_PIN_15
#define M3_CHA_GPIO_Port GPIOB
#define M3_CHA_EXTI_IRQn EXTI15_10_IRQn
#define M2_CHB_Pin GPIO_PIN_8
#define M2_CHB_GPIO_Port GPIOD
#define M2_CHB_EXTI_IRQn EXTI9_5_IRQn
#define M2_CHA_Pin GPIO_PIN_9
#define M2_CHA_GPIO_Port GPIOD
#define M2_CHA_EXTI_IRQn EXTI9_5_IRQn
#define M1_CHB_Pin GPIO_PIN_10
#define M1_CHB_GPIO_Port GPIOD
#define M1_CHB_EXTI_IRQn EXTI15_10_IRQn
#define M1_CHA_Pin GPIO_PIN_11
#define M1_CHA_GPIO_Port GPIOD
#define M1_CHA_EXTI_IRQn EXTI15_10_IRQn
#define limit1_Pin GPIO_PIN_0
#define limit1_GPIO_Port GPIOD
#define limit2_Pin GPIO_PIN_1
#define limit2_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
