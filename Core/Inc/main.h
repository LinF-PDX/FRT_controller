/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define APPS1_Pin GPIO_PIN_0
#define APPS1_GPIO_Port GPIOC
#define APPS2_Pin GPIO_PIN_1
#define APPS2_GPIO_Port GPIOC
#define STEERING_ANGLE_SENSOR_Pin GPIO_PIN_2
#define STEERING_ANGLE_SENSOR_GPIO_Port GPIOC
#define BPPS_BUFFERED_Pin GPIO_PIN_3
#define BPPS_BUFFERED_GPIO_Port GPIOC
#define DEBUG_LED_Pin GPIO_PIN_3
#define DEBUG_LED_GPIO_Port GPIOA
#define START_BTN_LED_EN_Pin GPIO_PIN_5
#define START_BTN_LED_EN_GPIO_Port GPIOA
#define BRAKE_LIGHT_EN_Pin GPIO_PIN_0
#define BRAKE_LIGHT_EN_GPIO_Port GPIOG
#define RTDS_EN_Pin GPIO_PIN_3
#define RTDS_EN_GPIO_Port GPIOG
#define START_BTN_n_Pin GPIO_PIN_6
#define START_BTN_n_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
