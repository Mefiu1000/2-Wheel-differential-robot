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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define RightMotor_PWM_Pin GPIO_PIN_6
#define RightMotor_PWM_GPIO_Port GPIOA
#define LeftMotor_PWM_Pin GPIO_PIN_7
#define LeftMotor_PWM_GPIO_Port GPIOA
#define RightMotor_FWD_Pin GPIO_PIN_13
#define RightMotor_FWD_GPIO_Port GPIOB
#define RightMotor_BWD_Pin GPIO_PIN_14
#define RightMotor_BWD_GPIO_Port GPIOB
#define HC_SR04p_ECHO_Pin GPIO_PIN_8
#define HC_SR04p_ECHO_GPIO_Port GPIOA
#define HC05_TX_Pin GPIO_PIN_9
#define HC05_TX_GPIO_Port GPIOA
#define HC_SR04p_TRIG_Pin GPIO_PIN_10
#define HC_SR04p_TRIG_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LeftMotor_FWD_Pin GPIO_PIN_4
#define LeftMotor_FWD_GPIO_Port GPIOB
#define LeftMotor_BWD_Pin GPIO_PIN_5
#define LeftMotor_BWD_GPIO_Port GPIOB
#define HC05_RX_Pin GPIO_PIN_7
#define HC05_RX_GPIO_Port GPIOB
#define MinIMU_9_SCL_Pin GPIO_PIN_8
#define MinIMU_9_SCL_GPIO_Port GPIOB
#define MinIMU_9_SDA_Pin GPIO_PIN_9
#define MinIMU_9_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
