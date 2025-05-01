/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define PUL_Motor3_Pin GPIO_PIN_0
#define PUL_Motor3_GPIO_Port GPIOB
#define DIR_Motor3_Pin GPIO_PIN_1
#define DIR_Motor3_GPIO_Port GPIOB
#define ENA_Motor3_Pin GPIO_PIN_2
#define ENA_Motor3_GPIO_Port GPIOB
#define PUL_Motor4_Pin GPIO_PIN_7
#define PUL_Motor4_GPIO_Port GPIOE
#define DIR_Motor4_Pin GPIO_PIN_8
#define DIR_Motor4_GPIO_Port GPIOE
#define ENA_Motor4_Pin GPIO_PIN_9
#define ENA_Motor4_GPIO_Port GPIOE
#define PUL_Motor1_Pin GPIO_PIN_10
#define PUL_Motor1_GPIO_Port GPIOE
#define DIR_Motro1_Pin GPIO_PIN_11
#define DIR_Motro1_GPIO_Port GPIOE
#define ENA_Motor1_Pin GPIO_PIN_12
#define ENA_Motor1_GPIO_Port GPIOE
#define PUL_Motor5_Pin GPIO_PIN_10
#define PUL_Motor5_GPIO_Port GPIOD
#define DIR_Motor5_Pin GPIO_PIN_11
#define DIR_Motor5_GPIO_Port GPIOD
#define ENA_Motor5_Pin GPIO_PIN_12
#define ENA_Motor5_GPIO_Port GPIOD
#define PUL_Motor6_Pin GPIO_PIN_13
#define PUL_Motor6_GPIO_Port GPIOD
#define PUL_Motor6D14_Pin GPIO_PIN_14
#define PUL_Motor6D14_GPIO_Port GPIOD
#define PUL_Motor6D15_Pin GPIO_PIN_15
#define PUL_Motor6D15_GPIO_Port GPIOD
#define PUL_Motor2_Pin GPIO_PIN_6
#define PUL_Motor2_GPIO_Port GPIOC
#define DIR_Motor2_Pin GPIO_PIN_7
#define DIR_Motor2_GPIO_Port GPIOC
#define ENA_Motor2_Pin GPIO_PIN_8
#define ENA_Motor2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
