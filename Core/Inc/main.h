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

void console_log(const char *message);
void light_led(GPIO_PinState state);

void emg_read_loop();
float normalize_emg(uint16_t *buffer);
float emg_to_angle(float emg);

uint16_t read_feedback();
void calibrate_actuator(Servo servo, uint16_t *min_calibration, uint16_t *max_calibration, uint16_t *mid_calibration);
void configure_actuator(Servo servo, uint16_t min_calibration, uint16_t max_calibration, uint16_t mid_calibration);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CHANGE_STATE_Pin GPIO_PIN_13
#define CHANGE_STATE_GPIO_Port GPIOC
#define CHANGE_STATE_EXTI_IRQn EXTI15_10_IRQn
#define MYOWARE_Pin GPIO_PIN_0
#define MYOWARE_GPIO_Port GPIOC
#define SERVO_FEEDBACK_Pin GPIO_PIN_1
#define SERVO_FEEDBACK_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BLUE_LED_Pin GPIO_PIN_6
#define BLUE_LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
