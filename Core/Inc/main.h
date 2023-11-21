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
#define SPI_IMU1_CS_Pin GPIO_PIN_2
#define SPI_IMU1_CS_GPIO_Port GPIOA
#define IMU1_INT1_Pin GPIO_PIN_3
#define IMU1_INT1_GPIO_Port GPIOA
#define IMU1_INT2_Pin GPIO_PIN_4
#define IMU1_INT2_GPIO_Port GPIOA
#define SPI1_IMU2_INT3_Pin GPIO_PIN_0
#define SPI1_IMU2_INT3_GPIO_Port GPIOB
#define SP1_IM2_CS_Pin GPIO_PIN_1
#define SP1_IM2_CS_GPIO_Port GPIOB
#define SPI1_IMU2_INT1_Pin GPIO_PIN_2
#define SPI1_IMU2_INT1_GPIO_Port GPIOB
#define SPI1_IMU2_INT2_Pin GPIO_PIN_10
#define SPI1_IMU2_INT2_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
