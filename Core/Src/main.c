/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <device_configs.h>	// Include the configurations for devices
#include <string.h>  // For memcpy
#include <stdio.h> // for printf
#include <stdbool.h> // for boolean support in c
#include "ssd1306.h" // for OLED screen https://github.com/afiskon/stm32-ssd1306
#include "ssd1306_fonts.h"
#include "ui.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---| CAN TYPEDEFS |------------------------------------------------------------------------- */

/* ---| CAN VARIABLES |------------------------------------------------------------------------ */
CAN_TxHeaderTypeDef TxHeader[CAN_MAX]; // [0] for CAN1, [1] for CAN2
CAN_RxHeaderTypeDef RxHeader[CAN_MAX]; // [0] for CAN1, [1] for CAN2
uint32_t TxMailbox[CAN_MAX];           // Use 2 mailboxes
uint8_t RxData[CAN_MAX][DLC_MAX];      // [0] for CAN1, [1] for CAN2, each with an 8-byte data buffer
uint8_t TxData[CAN_MAX][DLC_MAX];      // [0] for CAN1, [1] for CAN2, each with an 8-byte data buffer
uint32_t tx_count[CAN_MAX] = {0, 0};
uint32_t rx_count[CAN_MAX] = {0, 0};
/* ---| DISPLAY VARIABLES |-------------------------------------------------------------------- */
char screen_data[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH];
char screen_data_states[SCREEN_MAX_CHAR_LINES][SCREEN_MAX_CHAR_WIDTH];
uint8_t screen_line = 0;
/* ---| TIME VARIABLES |-------------------------------------------------------------------- */
uint32_t last_can_request_time = 0;  // Store the last time CAN requests were sent

/*
 * FUNCTIONS
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//send_Console_Msg("Rx INT");
    // Ensure this is the correct CAN instance
    if (hcan->Instance != CAN1)
    {
        return;
    }
    // Retrieve the message from FIFO0
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader[CAN_TRUCK], RxData[CAN_TRUCK]) != HAL_OK)
    {
        send_Console_Msg("RX:ERR CAN_GetRxMessage");
    	// Log or handle error if message retrieval fails
        return;
    }
    // Increment the receive counter
    rx_count[CAN_TRUCK]++;
    // Format the received message for display
    char received_msg[255];
    int offset = snprintf(received_msg, sizeof(received_msg), "Rx %X/", (unsigned int)RxHeader[CAN_TRUCK].StdId);

    for (int i = 0; i < RxHeader[CAN_TRUCK].DLC; i++)
    {
        offset += snprintf(&received_msg[offset], sizeof(received_msg) - offset, "%02X", RxData[CAN_TRUCK][i]);
    }
    // Display the received message
    send_Console_Msg(received_msg);
    // Process the message to update the state

    parse_rx_CAN_message(RxHeader[CAN_TRUCK].StdId, RxData[CAN_TRUCK]);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init(); // init OLED screen
  init_OLED_Data(); // zero all string data

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  send_Console_Msg("Err Init TRUCK CAN");
	  ssd1306_SetCursor(0, 8);  // Move down by the font height
	  send_Console_Msg("Terminating.");
	  Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  send_Console_Msg("Err Init TRUCK Rx");
	  ssd1306_SetCursor(0, 8);  // Move down by the font height
	  send_Console_Msg("Terminating.");
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      draw_screen_data_states(CAN_TRUCK);  // Update the screen with state information

      // Check if 2 seconds have passed since the last CAN request
      uint32_t current_time = HAL_GetTick();  // Get the current system tick in milliseconds
      if ((current_time - last_can_request_time) >= CAN_REQUEST_INTERVAL)
      {
    	  send_all_can_requests();
          last_can_request_time = current_time;  // Update the last request time
      }

      // Additional tasks can be added here, and the loop will not block
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;

#ifdef LOOPBACK_MODE
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
#else
  hcan1.Init.Mode = CAN_MODE_NORMAL;
#endif

  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;                       // Use filter bank 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   // Use mask mode
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // Use 32-bit filter
  sFilterConfig.FilterIdHigh = 0x0000;                // Set ID high to 0 (don't filter on specific IDs)
  sFilterConfig.FilterIdLow = 0x0000;                 // No extended ID bits
  sFilterConfig.FilterMaskIdHigh = 0x0000;            // Mask set to 0 (allows all IDs to pass)
  sFilterConfig.FilterMaskIdLow = 0x0000;             // No extended mask bits
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Assign to FIFO 0
  sFilterConfig.FilterActivation = ENABLE;            // Enable the filter

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      send_Console_Msg("Er: Applying filter");
      Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Adjust pin as needed
	  HAL_Delay(500);  // Blink with a 500ms delay
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
