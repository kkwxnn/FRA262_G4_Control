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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define QEI_PERIOD 65536
#define Proximity_1_Pin GPIO_PIN_1
#define Proximity_1_GPIO_Port GPIOC
#define Proximity_2_Pin GPIO_PIN_2
#define Proximity_2_GPIO_Port GPIOC
#define Proximity_2_EXTI_IRQn EXTI2_IRQn
#define Proximity_3_Pin GPIO_PIN_3
#define Proximity_3_GPIO_Port GPIOC
#define Proximity_3_EXTI_IRQn EXTI3_IRQn
#define Joystick_Analog_X_Pin GPIO_PIN_0
#define Joystick_Analog_X_GPIO_Port GPIOA
#define Joystick_Analog_Y_Pin GPIO_PIN_1
#define Joystick_Analog_Y_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define Joystick_Reset_Pin GPIO_PIN_5
#define Joystick_Reset_GPIO_Port GPIOA
#define QEI_B_Pin GPIO_PIN_6
#define QEI_B_GPIO_Port GPIOA
#define QEI_A_Pin GPIO_PIN_7
#define QEI_A_GPIO_Port GPIOA
#define Motor_Drive_L_EN_Pin GPIO_PIN_10
#define Motor_Drive_L_EN_GPIO_Port GPIOB
#define Emergency_Switch_Pin GPIO_PIN_15
#define Emergency_Switch_GPIO_Port GPIOB
#define PWM_CH1_Pin GPIO_PIN_8
#define PWM_CH1_GPIO_Port GPIOA
#define PWM_CH2_Pin GPIO_PIN_9
#define PWM_CH2_GPIO_Port GPIOA
#define Joystick_Fine_Pin GPIO_PIN_10
#define Joystick_Fine_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Motor_Drive_R_EN_Pin GPIO_PIN_4
#define Motor_Drive_R_EN_GPIO_Port GPIOB
#define Joystick_Rough_Pin GPIO_PIN_5
#define Joystick_Rough_GPIO_Port GPIOB
#define Joystick_Get_Position_Pin GPIO_PIN_6
#define Joystick_Get_Position_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
