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
#include "ssd1306.h"

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

/* ---| CAN TYPES CONSTANTS |------------------------------------------------ */
#define CAN_TRUCK 0
#define CAN_OTHER 1
#define CAN_MAX   2
#define DLC_MAX   8

/* ---| FONT CONSTANTS |----------------------------------------------------- */
#define SCREEN_FONT_HEIGHT 8   /**< Font height in pixels (Font_6x8). */
#define SCREEN_FONT_WIDTH  6   /**< Font width in pixels (Font_6x8). */

/* ---| DISPLAY CONSTANTS |-------------------------------------------------- */
#define SCREEN_MAX_CHAR_LINES (SSD1306_HEIGHT / SCREEN_FONT_HEIGHT) /**< Max lines on screen height. */
#define SCREEN_MAX_CHAR_WIDTH (SSD1306_WIDTH / SCREEN_FONT_WIDTH)   /**< Max chars on screen width. */
#define STATE_LINES 4           /**< Number of lines used for state info. */
#define MESSAGE_LINES (SCREEN_MAX_CHAR_LINES - STATE_LINES) /**< Remaining lines for messages. */

/* ---| TIME CONSTANTS |----------------------------------------------------- */
#define ONE_MILLISECOND 1       /**< One millisecond in timer ticks. */
#define ONE_SECOND (ONE_MILLISECOND * 1000) /**< One second in timer ticks. */
#define CAN_REQUEST_INTERVAL (50 * ONE_MILLISECOND) /**< Interval for CAN requests in ms. */

/**
 * @brief Define CAN instance indexes for clarity and efficiency.
 */
typedef uint8_t CANInstance;

/**
 * @brief CAN-related variables.
 */
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;

extern CAN_TxHeaderTypeDef TxHeader[CAN_MAX];  /**< Transmission headers for CAN1 and CAN2. */
extern CAN_RxHeaderTypeDef RxHeader[CAN_MAX];  /**< Reception headers for CAN1 and CAN2. */
extern uint32_t TxMailbox[CAN_MAX];            /**< Transmission mailboxes. */
extern uint8_t RxData[CAN_MAX][DLC_MAX];       /**< Reception data buffers (8 bytes each). */
extern uint8_t TxData[CAN_MAX][DLC_MAX];       /**< Transmission data buffers (8 bytes each). */
extern uint32_t tx_count[CAN_MAX];             /**< Transmission counts for CAN instances. */
extern uint32_t rx_count[CAN_MAX];             /**< Reception counts for CAN instances. */

/**
 * @brief Display-related variables.
 */
extern char screen_data[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH]; /**< Screen data buffer. */
extern char screen_data_states[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH]; /**< State info buffer. */
extern uint8_t screen_line; /**< Current screen line index. */

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
