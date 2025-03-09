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
#include "stm32c0xx_hal.h"
#include "stm32c0xx_ll_adc.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_cortex.h"
#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_pwr.h"
#include "stm32c0xx_ll_gpio.h"
#include "stm32c0xx_ll_dma.h"

#include "stm32c0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum holding_register_e
{
	MODBUS_ID,
	MB_BAUD_RATE,
	MB_TRANSMIT_TIMEOUT,
	MB_TRANSMIT_RETRIES,
	MB_ERRORS,
	I2C_ERRORS,
	I2C_SHUTDOWN,
	AUTOPILOT,

	ADC_0,
	ADC_1,
	ADC_2,
	ADC_3,
	ADC_4,
	ADC_5,
	ADC_6,
	ADC_7,
	ADC_8,

	ACCELEROMETER_X,
	ACCELEROMETER_Y,
	ACCELEROMETER_Z,
	MAGNETOMETER_X,
	MAGNETOMETER_Y,
	MAGNETOMETER_Z,
	GYROSCOPE_X,
	GYROSCOPE_Y,
	GYROSCOPE_Z,
	EULER_HEADING,
	EULER_ROLL,
	EULER_PITCH,
	LINEAR_ACCELERATION_X,
	LINEAR_ACCELERATION_Y,
	LINEAR_ACCELERATION_Z,
	GRAVITY_X,
	GRAVITY_Y,
	GRAVITY_Z,
	QUARTERNION_W,
	QUARTERNION_X,
	QUARTERNION_Y,
	QUARTERNION_Z,

	REMOTE_ACCELEROMETER_X,
	REMOTE_ACCELEROMETER_Y,
	REMOTE_ACCELEROMETER_Z,
	REMOTE_MAGNETOMETER_X,
	REMOTE_MAGNETOMETER_Y,
	REMOTE_MAGNETOMETER_Z,
	REMOTE_GYROSCOPE_X,
	REMOTE_GYROSCOPE_Y,
	REMOTE_GYROSCOPE_Z,
	REMOTE_EULER_HEADING,
	REMOTE_EULER_ROLL,
	REMOTE_EULER_PITCH,
	REMOTE_LINEAR_ACCELERATION_X,
	REMOTE_LINEAR_ACCELERATION_Y,
	REMOTE_LINEAR_ACCELERATION_Z,
	REMOTE_GRAVITY_X,
	REMOTE_GRAVITY_Y,
	REMOTE_GRAVITY_Z,
	REMOTE_QUARTERNION_W,
	REMOTE_QUARTERNION_X,
	REMOTE_QUARTERNION_Y,
	REMOTE_QUARTERNION_Z,

	ACTUATOR_A_TARGET,
	ACTUATOR_B_TARGET,
	ACTUATOR_C_TARGET,
	PROPORTIONAL_GAIN_HI,
	PROPORTIONAL_GAIN_LO,
	INTEGRAL_GAIN_HI,
	INTEGRAL_GAIN_LO,
	DERIVATIVE_GAIN_HI,
	DERIVATIVE_GAIN_LO,
	D_FILTER_TIME_HI,
	D_FILTER_TIME_LO,
	TIME_STEP_HI,
	TIME_STEP_LO,
	MAX_RATE_OF_CHANGE_HI,
	MAX_RATE_OF_CHANGE_LO,
	NUM_HOLDING_REGISTERS
}holding_register_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define USE_TIMEOUT 1

#define NUM_ACTUATORS 3
#define ACTUATOR_TOLERANCE 15 // TODO: relate this adc value to mm with some equation
#define ACTUATOR_TRANSIENT_DELAY 150 // TODO: figure out a safe value for this
#define I2C_TIMEOUT_MS 100
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
#define Actuator_C_EN_Pin GPIO_PIN_0
#define Actuator_C_EN_GPIO_Port GPIOB
#define Encoder_Pulse_A_Pin GPIO_PIN_13
#define Encoder_Pulse_A_GPIO_Port GPIOB
#define Encoder_Pulse_B_Pin GPIO_PIN_14
#define Encoder_Pulse_B_GPIO_Port GPIOB
#define Actuator_A_EN_Pin GPIO_PIN_6
#define Actuator_A_EN_GPIO_Port GPIOC
#define Actuator_B_EN_Pin GPIO_PIN_7
#define Actuator_B_EN_GPIO_Port GPIOC
#define Actuator_PWM_Pin GPIO_PIN_15
#define Actuator_PWM_GPIO_Port GPIOA
#define Actuator_CS_Pin GPIO_PIN_0
#define Actuator_CS_GPIO_Port GPIOD
#define IMU_Reset_Pin GPIO_PIN_1
#define IMU_Reset_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
