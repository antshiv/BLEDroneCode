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
#define SPI_I2C_N_Pin GPIO_PIN_13
#define SPI_I2C_N_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define INT_C_Pin GPIO_PIN_4
#define INT_C_GPIO_Port GPIOA
#define INT_C_EXTI_IRQn EXTI4_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define PWR_EN_Pin GPIO_PIN_7
#define PWR_EN_GPIO_Port GPIOA
#define LPn_Pin GPIO_PIN_0
#define LPn_GPIO_Port GPIOB
#define AVDD_EN_Pin GPIO_PIN_1
#define AVDD_EN_GPIO_Port GPIOB
#define NCS_Pin GPIO_PIN_12
#define NCS_GPIO_Port GPIOB
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define VDDIO_EN_Pin GPIO_PIN_15
#define VDDIO_EN_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */


#define BUILD_FOR_PCB4128A

#ifdef BUILD_FOR_PCB4128A
	#define GPIO_FOR_PWR_EN
#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
