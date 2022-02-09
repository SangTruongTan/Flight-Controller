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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOC
#define PWM_1_Pin GPIO_PIN_0
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_1
#define PWM_2_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_2
#define PWM_3_GPIO_Port GPIOA
#define PWM_4_Pin GPIO_PIN_3
#define PWM_4_GPIO_Port GPIOA
#define BAT_Pin GPIO_PIN_6
#define BAT_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_5
#define BTN2_GPIO_Port GPIOC
#define MAG_SCL_Pin GPIO_PIN_10
#define MAG_SCL_GPIO_Port GPIOB
#define MAG_SDA_Pin GPIO_PIN_11
#define MAG_SDA_GPIO_Port GPIOB
#define PC8_Pin GPIO_PIN_8
#define PC8_GPIO_Port GPIOC
#define PC9_Pin GPIO_PIN_9
#define PC9_GPIO_Port GPIOC
#define FTDI_TX_Pin GPIO_PIN_9
#define FTDI_TX_GPIO_Port GPIOA
#define FTDI_RX_Pin GPIO_PIN_10
#define FTDI_RX_GPIO_Port GPIOA
#define TEL_TX_Pin GPIO_PIN_10
#define TEL_TX_GPIO_Port GPIOC
#define TEL_RX_Pin GPIO_PIN_11
#define TEL_RX_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_12
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_2
#define GPS_RX_GPIO_Port GPIOD
#define MEMS_SCL_Pin GPIO_PIN_6
#define MEMS_SCL_GPIO_Port GPIOB
#define MEMS_SDA_Pin GPIO_PIN_7
#define MEMS_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
