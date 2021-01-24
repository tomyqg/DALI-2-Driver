/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dali_application.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	false = 0,
	true
}bool_t;
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
#define AOUT_Pin GPIO_PIN_1
#define AOUT_GPIO_Port GPIOA
#define SENSOR_CONFIG_Pin GPIO_PIN_4
#define SENSOR_CONFIG_GPIO_Port GPIOA
#define SENSOR_Pin GPIO_PIN_1
#define SENSOR_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_9
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOA
#define RX_EXTI_IRQn EXTI4_15_IRQn
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define TP_Pin GPIO_PIN_0
#define TP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern volatile uint8_t adc_flag;
extern volatile uint16_t adc_time;
#ifdef DEBUG
extern volatile uint8_t power_down;
extern volatile uint16_t time1;
#endif
#ifdef CONTROLLER
extern volatile uint8_t power_down;
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
