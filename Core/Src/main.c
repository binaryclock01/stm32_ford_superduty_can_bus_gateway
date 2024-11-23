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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "config.h"
#include <device_configs.h>	// Include the configurations for devices
#include <string.h>  // For memcpy
#include <stdio.h> // for printf
#include <stdbool.h> // for boolean support in c
#include "ssd1306.h" // for OLED screen https://github.com/afiskon/stm32-ssd1306
#include "ssd1306_fonts.h"
#include "log.h"
#include "ui.h"
#include "error.h"
#include "device_configs.h"
#include "rtos.h"
#include "can_core.h"
#include "utils.h"
#include "system.h"

#ifdef IS_SIMULATOR
#include "sim.h"
#endif // IS_SIMULATOR

int _write(int file, char *data, int len) {
    // Use HAL_UART_Transmit to send data to UART2
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
    return len;
}

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
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for CAN1_Rx_Task */
osThreadId_t CAN1_Rx_TaskHandle;
const osThreadAttr_t CAN1_Rx_Task_attributes = {
  .name = "CAN1_Rx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN2_Rx_Task */
osThreadId_t CAN2_Rx_TaskHandle;
const osThreadAttr_t CAN2_Rx_Task_attributes = {
  .name = "CAN2_Rx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN1_Tx_Task */
osThreadId_t CAN1_Tx_TaskHandle;
const osThreadAttr_t CAN1_Tx_Task_attributes = {
  .name = "CAN1_Tx_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Housekeeping_Ta */
osThreadId_t Housekeeping_TaHandle;
const osThreadAttr_t Housekeeping_Ta_attributes = {
  .name = "Housekeeping_Ta",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN1_Send_Reque */
osThreadId_t CAN1_Send_RequeHandle;
const osThreadAttr_t CAN1_Send_Reque_attributes = {
  .name = "CAN1_Send_Reque",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CAN2_Tx_Task */
osThreadId_t CAN2_Tx_TaskHandle;
const osThreadAttr_t CAN2_Tx_Task_attributes = {
  .name = "CAN2_Tx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

// Queue attributes are created in rtos.c

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN2_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
void StartCAN1_Rx_Task(void *argument);
void StartCAN2_Rx_Task(void *argument);
void StartCAN1_Tx_Task(void *argument);
void StartHousekeeping_Task(void *argument);
void StartCAN_Tx_Send_Requests(void *argument);
void StartCAN2_Tx_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Configure and apply a CAN filter dynamically.
 *
 * This function applies a CAN filter configuration dynamically for the specified CAN handle.
 * If the configuration fails, detailed error information is logged.
 *
 * @param hcan Pointer to the CAN hardware handle (e.g., CAN1 or CAN2).
 * @param filter_config Pointer to the CAN_FilterTypeDef containing filter settings.
 * @return bool True if the filter configuration succeeded, false otherwise.
 */
bool configure_can_filter(CAN_HandleTypeDef *hcan, const CAN_FilterTypeDef *filter_config) {
    if (hcan == NULL || filter_config == NULL) {
        user_error_handler(ERROR_INVALID_ARGUMENT, "NULL argument passed to configure_can_filter");
        return false;
    }

    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, (CAN_FilterTypeDef *)filter_config);
    if (status != HAL_OK) {
        // Dynamically construct error context
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Filter config failed on CAN instance %p, status=%d", hcan, status);
        user_error_handler(ERROR_CAN_FILTER_CONFIG_FAILED, error_msg);
        return false;
    }

    // Optional: Log success for debugging
    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg), "* CAN filter applied successfully on CAN instance %p", hcan);
    log_message(log_msg);

    return true;
}
/* ---| CAN TYPEDEFS |------------------------------------------------------------------------- */

/* ---| CAN VARIABLES |------------------------------------------------------------------------ */


/* ---| TIME VARIABLES |-------------------------------------------------------------------- */

/*
 * FUNCTIONS
 */

/**
 * @brief Interrupt callback for CAN Rx FIFO0 messages.
 *
 * This function is triggered by the HAL when a message is received in the
 * CAN Rx FIFO0. It delegates the handling of the received message to the
 * `process_can_rx_fifo_callback` function in `rtos.c`. This keeps the interrupt
 * service routine (ISR) lightweight and focused.
 *
 * The `process_can_rx_fifo_callback` function is responsible for:
 * - Determining the corresponding CAN instance and message queue.
 * - Allocating a CAN packet from the memory pool.
 * - Retrieving the message from the CAN peripheral.
 * - Enqueuing the packet in the RTOS message queue.
 *
 * The tasks consuming the queue are responsible for processing and freeing the packets.
 *
 * @param hcan Pointer to the CAN hardware instance (e.g., CAN1 or CAN2).
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // Delegate the handling of the CAN Rx FIFO0 message to `process_can_rx_fifo_callback`
    // This keeps the ISR clean and lightweight.
    process_can_rx_fifo_callback(hcan);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  init_log_system();

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
  MX_CAN2_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  send_clear_screen_ansi_code();

  flush_logs(); // now UART is ready, flush logs from init_log_system() above
  check_heap_usage();
  //check_heap_usage();

  EXECUTE_FUNCTION_AND_FLUSH_LOGS(display_welcome_message);

#ifdef IS_SIMULATOR
  __sim__initialize_states_mapping_array();
#endif // IS_SIMULATOR

#ifdef USE_SSD1306
  ssd1306_Init(); // init OLED screen
  init_oled_data(); // zero all string data
#endif

  // Initialize and log statuses of the HAL CAN buses
  EXECUTE_FUNCTION_AND_FLUSH_LOGS(start_hal_can_buses);

  EXECUTE_FUNCTION_AND_FLUSH_LOGS(activate_can_bus_fifo_callbacks);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  init_circular_buffers();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  init_rtos_queue_handles();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CAN1_Rx_Task */
  CAN1_Rx_TaskHandle = osThreadNew(StartCAN1_Rx_Task, NULL, &CAN1_Rx_Task_attributes);

  /* creation of CAN2_Rx_Task */
  CAN2_Rx_TaskHandle = osThreadNew(StartCAN2_Rx_Task, NULL, &CAN2_Rx_Task_attributes);

  /* creation of CAN1_Tx_Task */
  CAN1_Tx_TaskHandle = osThreadNew(StartCAN1_Tx_Task, NULL, &CAN1_Tx_Task_attributes);

  /* creation of Housekeeping_Ta */
  Housekeeping_TaHandle = osThreadNew(StartHousekeeping_Task, NULL, &Housekeeping_Ta_attributes);

  /* creation of CAN1_Send_Reque */
  CAN1_Send_RequeHandle = osThreadNew(StartCAN_Tx_Send_Requests, NULL, &CAN1_Send_Reque_attributes);

  /* creation of CAN2_Tx_Task */
  CAN2_Tx_TaskHandle = osThreadNew(StartCAN2_Tx_Task, NULL, &CAN2_Tx_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  log_message("* Initialization complete.");
  log_message("--------------------------------------------------------------------------");
  log_message("");
  flush_logs();

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hcan1.Init.Mode = CAN_MODE_NORMAL;
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

  if (!configure_can_filter(&hcan1, &sFilterConfig))
	 return;  // Exit gracefully

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 18;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

/* USER CODE BEGIN Header_StartCAN1_Rx_Task */
/**
 * @brief Task to handle incoming CAN1 messages.
 *
 * Steps:
 * 1. Wait for a CANPacket from the CAN1 queue (`osMessageQueueGet`).
 * 2. Process the packet using `process_can_packet`.
 * 3. Return the packet to the static pool using `_free_can_packet`.
 *
 * @param argument Unused FreeRTOS task argument placeholder.
 */
/* USER CODE END Header_StartCAN1_Rx_Task */
void StartCAN1_Rx_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  CAN_Packet *packet = NULL;
  for (;;) {
		if (osMessageQueueGet(can_circular_buffer[QUEUE_RX_CAN1].queue_handle, &packet, NULL, osWaitForever) == osOK)
			__rtos__StartCAN_Rx_Task(CAN_TRUCK, packet);
		osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCAN2_Rx_Task */
/**
 * @brief Task to handle incoming CAN2 messages.
 *
 * Steps:
 * 1. Wait for a CANPacket from the CAN2 queue (`osMessageQueueGet`).
 * 2. Process the packet using `process_can_packet`.
 * 3. Return the packet to the static pool using `_free_can_packet`.
 *
 * @param argument Unused FreeRTOS task argument placeholder.
 */
/* USER CODE END Header_StartCAN2_Rx_Task */
void StartCAN2_Rx_Task(void *argument)
{
  /* USER CODE BEGIN StartCAN2_Rx_Task */
	osThreadSuspend(osThreadGetId());
//	CAN_Packet *packet = NULL;
//	for (;;) {
//		if (osMessageQueueGet(can_circular_buffer[QUEUE_RX_CAN2].queue_handle, &packet, NULL, osWaitForever) == osOK)
//			__rtos__StartCAN_Rx_Task(CAN_AUX, packet);
//	}
  /* USER CODE END StartCAN2_Rx_Task */
}

/* USER CODE BEGIN Header_StartCAN1_Tx_Task */
/**
* @brief Function implementing the CAN1_Tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN1_Tx_Task */
void StartCAN1_Tx_Task(void *argument)
{
  /* USER CODE BEGIN StartCAN1_Tx_Task */
  /* Infinite loop */
  //osThreadSuspend(osThreadGetId());

  for(;;)
  {
     CAN_Packet *packet = NULL;
     if (osMessageQueueGet(can_circular_buffer[QUEUE_TX_CAN1].queue_handle, &packet, NULL, osWaitForever) == osOK)
    	 __rtos__StartCAN_Tx_Task(CAN_TRUCK, packet);
     osDelay(5);
  }
  /* USER CODE END StartCAN1_Tx_Task */
}

/* USER CODE BEGIN Header_StartHousekeeping_Task */
/**
 * @brief Task to handle periodic housekeeping actions.
 *
 * This task updates the OLED display for a specified CAN instance
 * and performs routine system health checks.
 *
 * @param argument Not used (FreeRTOS task argument placeholder).
 */
/* USER CODE END Header_StartHousekeeping_Task */
void StartHousekeeping_Task(void *argument)
{
  osThreadYield();
  /* USER CODE BEGIN StartHousekeeping_Task */
  for (;;) {
	__rtos__log_task();
	osDelay(100);
	// Yield to other threads without a fixed delay
	//osThreadYield();
	  //osThreadSuspend(osThreadGetId());
#ifdef USE_SSD1306
	  // Step 1: Update OLED display for the specified CAN instance
      //update_oled_status(DEFAULT_OLED_CAN_INSTANCE);
#endif
  }
  /* USER CODE END StartHousekeeping_Task */
}

/* USER CODE BEGIN Header_StartCAN_Tx_Send_Requests */
/**
* @brief Function implementing the CAN1_Send_Reque thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN_Tx_Send_Requests */
void StartCAN_Tx_Send_Requests(void *argument)
{
  /* USER CODE BEGIN StartCAN_Tx_Send_Requests */
  /* Infinite loop */
	for (;;)
	{
	  uint32_t queue_length = osMessageQueueGetCount(can_circular_buffer[QUEUE_TX_CAN1].queue_handle);
	  char msg[255];
	  if (queue_length >= 7)
	  {
		  sprintf(msg, "Cannot send Tx requests - Tx_QueueHandle has %lu elements and it would overflow the buffer.\r\n", queue_length);
		  printf(msg);
	  }
	  else
	  {
			// Step 1: Update OLED display for the specified CAN instance
			log_message("* Sending PID status read requests on Truck CAN");
			flush_logs();
			send_all_requests();
	  }

      osThreadYield();
      osDelay(15000);
	}
      //osThreadYield();
  /* USER CODE END StartCAN_Tx_Send_Requests */
}

/* USER CODE BEGIN Header_StartCAN2_Tx_Task */
/**
* @brief Function implementing the CAN2_Tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN2_Tx_Task */
void StartCAN2_Tx_Task(void *argument)
{
  /* USER CODE BEGIN StartCAN2_Tx_Task */
  /* Infinite loop */
  for(;;)
  {
		//__rtos__StartCAN_Tx_Task(CAN_TRUCK);
	    //osThreadYield();
	  	osThreadSuspend(osThreadGetId());
		//osDelay(1);
  }
  /* USER CODE END StartCAN2_Tx_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
	  osDelay(500);  // Blink with a 500ms delay
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
