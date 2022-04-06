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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_util.h"
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
#define Buttom1_Pin GPIO_PIN_14
#define Buttom1_GPIO_Port GPIOC
#define Buttom2_Pin GPIO_PIN_15
#define Buttom2_GPIO_Port GPIOC
#define EncoderRight_A_Pin GPIO_PIN_0
#define EncoderRight_A_GPIO_Port GPIOA
#define EncoderRight_B_Pin GPIO_PIN_1
#define EncoderRight_B_GPIO_Port GPIOA
#define SNAIL_LEFT_Pin GPIO_PIN_5
#define SNAIL_LEFT_GPIO_Port GPIOA
#define SNAIL_RIGHT_Pin GPIO_PIN_6
#define SNAIL_RIGHT_GPIO_Port GPIOA
#define EncoderLeft_A_Pin GPIO_PIN_0
#define EncoderLeft_A_GPIO_Port GPIOB
#define EncoderLeft_B_Pin GPIO_PIN_1
#define EncoderLeft_B_GPIO_Port GPIOB
#define USB_ID_Pin GPIO_PIN_2
#define USB_ID_GPIO_Port GPIOB
#define LED_Green_Pin GPIO_PIN_5
#define LED_Green_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
