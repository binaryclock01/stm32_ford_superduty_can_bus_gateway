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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Add any additional includes here
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Define any additional types here
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// #define LOOPBACK_MODE

#include "device_configs.h"
#include "ssd1306_conf.h"

/* ---| CAN TYPES CONSTANTS |------------------------------------------------ */

//#define CAN_TRUCK 0
//#define CAN_AUX   1
//#define CAN_MAX   2

/**
 * @brief CAN-related variables.
 */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Display-related variables.
 */

/**
 * @brief Time-related variables.
 */
extern uint32_t last_can_request_time; /**< Last CAN request timestamp. */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/**
 * @brief Check if a value is within a given range.
 * @param value The value to check.
 * @param min The minimum range value.
 * @param max The maximum range value.
 * @return True if value is in range, false otherwise.
 */
#define IN_RANGE(value, min, max) ((value) >= (min) && (value) <= (max))

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// Add any additional exported function prototypes here
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
// Add any additional private defines here
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
