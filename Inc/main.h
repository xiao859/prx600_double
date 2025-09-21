/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE BEGIN Private defines */
#define XRAY_LED_Pin GPIO_PIN_9
#define XRAY_LED_GPIO_Port GPIOD
#define FAULT_LED_Pin GPIO_PIN_11
#define FAULT_LED_GPIO_Port GPIOD
#define HEART_LED_Pin GPIO_PIN_13
#define HEART_LED_GPIO_Port GPIOD

#define HV_V_FAULT_Pin GPIO_PIN_1
#define HV_V_FAULT_GPIO_Port GPIOA
#define HV_V_FAULT_EXTI_IRQn EXTI3_IRQn
#define RESET_Pin GPIO_PIN_0
#define RESET_GPIO_Port GPIOA
#define HV_C_FAULT_Pin GPIO_PIN_2
#define HV_C_FAULT_GPIO_Port GPIOF
#define HV_EN_Pin GPIO_PIN_3
#define HV_EN_GPIO_Port GPIOC
#define HV_SW_A_PIN GPIO_PIN_0
#define HV_SW_A_GPIO_Port GPIOC
#define HV_SW_B_PIN GPIO_PIN_1
#define HV_SW_B_GPIO_Port GPIOC
#define HV_RE_PIN GPIO_PIN_2
#define HV_RE_GPIO_Port GPIOA
#define MCU_LOCK_Pin GPIO_PIN_2
#define MCU_LOCK_GPIO_Port GPIOC

#define HV_REF_Pin GPIO_PIN_4
#define HV_REF_GPIO_Port GPIOA
#define POWER_24V_Pin GPIO_PIN_7
#define POWER_24V_GPIO_Port GPIOA
#define T_SINK_Pin GPIO_PIN_5
#define T_SINK_GPIO_Port GPIOC
#define HV_N_Pin GPIO_PIN_7
#define HV_N_GPIO_Port GPIOE
#define T_OIL_Pin GPIO_PIN_8
#define T_OIL_GPIO_Port GPIOE
#define IA_Pin GPIO_PIN_9
#define IA_GPIO_Port GPIOE
#define HV_P_Pin GPIO_PIN_10
#define HV_P_GPIO_Port GPIOE

#define FILAMENT_A_EN_Pin GPIO_PIN_13
#define FILAMENT_A_EN_GPIO_Port GPIOB
#define FILAMENT_B_EN_Pin GPIO_PIN_14
#define FILAMENT_B_EN_GPIO_Port GPIOB
#define FILA_A_REF_Pin GPIO_PIN_5
#define FILA_A_REF_GPIO_Port GPIOA
#define FILA_B_REF_Pin GPIO_PIN_3
#define FILA_B_REF_GPIO_Port GPIOA
#define VS_Pin GPIO_PIN_2
#define VS_GPIO_Port GPIOB
#define IS_Pin GPIO_PIN_4
#define IS_GPIO_Port GPIOC

#define MCU_RX3_Pin GPIO_PIN_15
#define MCU_RX3_GPIO_Port GPIOE
#define MCU_TX3_Pin GPIO_PIN_10
#define MCU_TX3_GPIO_Port GPIOB
#define MCU_TX4_Pin GPIO_PIN_10
#define MCU_TX4_GPIO_Port GPIOC
#define MCU_RX4_Pin GPIO_PIN_11
#define MCU_RX4_GPIO_Port GPIOC
#define MCU_TX5_Pin GPIO_PIN_12
#define MCU_TX5_GPIO_Port GPIOC
#define MCU_RX5_Pin GPIO_PIN_2
#define MCU_RX5_GPIO_Port GPIOD
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOB

#define EXP_A_Pin GPIO_PIN_2
#define EXP_A_GPIO_Port GPIOE

#define EXP_B_Pin GPIO_PIN_0
#define EXP_B_GPIO_Port GPIOE

#define INTERLOCK_Pin GPIO_PIN_3
#define INTERLOCK_GPIO_Port GPIOE
#define INTERLOCK_EXTI_IRQn EXTI3_IRQn
#define READY_Pin GPIO_PIN_4
#define READY_GPIO_Port GPIOE
#define XRAY_ON_Pin GPIO_PIN_5
#define XRAY_ON_GPIO_Port GPIOE
#define FAULT_Pin GPIO_PIN_6
#define FAULT_GPIO_Port GPIOE

#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA

#define HOLD_Pin GPIO_PIN_6
#define HOLD_GPIO_Port GPIOD
#define WP_Pin GPIO_PIN_9
#define WP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DAC_HandleTypeDef hdac1;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
