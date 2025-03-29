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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LED_Curr_Pin GPIO_PIN_0
#define LED_Curr_GPIO_Port GPIOA
#define LED_Volt_Pin GPIO_PIN_1
#define LED_Volt_GPIO_Port GPIOA
#define BattVolt_Pin GPIO_PIN_2
#define BattVolt_GPIO_Port GPIOA
#define HeatTemp_Pin GPIO_PIN_3
#define HeatTemp_GPIO_Port GPIOA
#define NegPowerPWM_Pin GPIO_PIN_0
#define NegPowerPWM_GPIO_Port GPIOB
#define L_CurrComp_Pin GPIO_PIN_12
#define L_CurrComp_GPIO_Port GPIOB
#define L_CurrComp_EXTI_IRQn EXTI15_10_IRQn
#define LED_VoltComp_Pin GPIO_PIN_13
#define LED_VoltComp_GPIO_Port GPIOB
#define LED_VoltComp_EXTI_IRQn EXTI15_10_IRQn
#define SW_Pin GPIO_PIN_14
#define SW_GPIO_Port GPIOB
#define SelfPowerON_Pin GPIO_PIN_15
#define SelfPowerON_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_4
#define PWM1_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_5
#define FAN_PWM_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_6
#define PWM2_GPIO_Port GPIOB
#define L_CurrSetting_Pin GPIO_PIN_7
#define L_CurrSetting_GPIO_Port GPIOB
#define LED_VoltSetting_Pin GPIO_PIN_8
#define LED_VoltSetting_GPIO_Port GPIOB
#define ADC_Trig_Pin GPIO_PIN_9
#define ADC_Trig_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
