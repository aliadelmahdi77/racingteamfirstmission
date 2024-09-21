/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define pot_Pin GPIO_PIN_0
#define pot_GPIO_Port GPIOA
#define led_Pin GPIO_PIN_7
#define led_GPIO_Port GPIOA
#define C_L_Pin GPIO_PIN_0
#define C_L_GPIO_Port GPIOB
#define B_L_Pin GPIO_PIN_1
#define B_L_GPIO_Port GPIOB
#define A_L_Pin GPIO_PIN_2
#define A_L_GPIO_Port GPIOB
#define C_H_Pin GPIO_PIN_8
#define C_H_GPIO_Port GPIOA
#define B_H_Pin GPIO_PIN_9
#define B_H_GPIO_Port GPIOA
#define A_H_Pin GPIO_PIN_10
#define A_H_GPIO_Port GPIOA
#define hallC_Pin GPIO_PIN_3
#define hallC_GPIO_Port GPIOB
#define hallC_EXTI_IRQn EXTI3_IRQn
#define hallB_Pin GPIO_PIN_4
#define hallB_GPIO_Port GPIOB
#define hallB_EXTI_IRQn EXTI4_IRQn
#define hallA_Pin GPIO_PIN_5
#define hallA_GPIO_Port GPIOB
#define hallA_EXTI_IRQn EXTI9_5_IRQn
#define pushbutton_Pin GPIO_PIN_6
#define pushbutton_GPIO_Port GPIOB
#define pushbutton_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
