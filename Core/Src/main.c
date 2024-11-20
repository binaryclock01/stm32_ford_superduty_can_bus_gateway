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
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include <device_configs.h>	// Include the configurations for devices
#include <string.h>  // For memcpy
#include <stdio.h> // for printf
#include <stdbool.h> // for boolean support in c
#include "ssd1306.h" // for OLED screen https://github.com/afiskon/stm32-ssd1306
#include "ssd1306_fonts.h"
#include "ui.h"
#include "device_configs.h"
#include "rtos.h"


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

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* Definitions for CAN1_Rx_Task */
osThreadId_t CAN1_Rx_TaskHandle;
const osThreadAttr_t CAN1_Rx_Task_attributes = {
  .name = "CAN1_Rx_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN2_Rx_Task */
osThreadId_t CAN2_Rx_TaskHandle;
const osThreadAttr_t CAN2_Rx_Task_attributes = {
  .name = "CAN2_Rx_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Tx_Task */
osThreadId_t Tx_TaskHandle;
const osThreadAttr_t Tx_Task_attributes = {
  .name = "Tx_Task",
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
/* USER CODE BEGIN PV */

const osMessageQueueAttr_t CAN1_Rx_Queue_attributes = {
  .name = "CAN1_Rx_Queue"
};

const osMessageQueueAttr_t CAN2_Rx_Queue_attributes = {
  .name = "CAN2_Rx_Queue"
};

const osMessageQueueAttr_t Tx_Queue_attributes = {
  .name = "Tx_Queue"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
void StartCAN1_Rx_Task(void *argument);
void StartCAN2_Rx_Task(void *argument);
void Start_Tx_Task(void *argument);
void StartHousekeeping_Task(void *argument);
void StartCAN_Tx_Send_Requests(void *argument);

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
    snprintf(log_msg, sizeof(log_msg), "CAN filter applied successfully on CAN instance %p", hcan);
    send_console_msg(log_msg);

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

void Delay(uint32_t delay)
{
	osDelay(delay);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TouchGFX_Init();
  /* Call PreOsInit function */
  MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */

  Displ_Init(Displ_Orientat_0);		// initialize the display and set the initial display orientation (here is orientaton: 0°) - THIS FUNCTION MUST PRECEED ANY OTHER DISPLAY FUNCTION CALL.
  touchgfxSignalVSync();		// asks TouchGFX to start initial display drawing
  Displ_BackLight('I');

  //ssd1306_Init(); // init OLED screen
  //init_oled_data(); // zero all string data


  // Initialize TRUCK CAN (hcan1)
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      user_error_handler(ERROR_CAN_INIT_FAILED, "Failed to start TRUCK CAN");
      return HAL_ERROR;  // Use HAL-defined error code for system failure
  }

  // Activate notification for TRUCK CAN
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      user_error_handler(ERROR_CAN_NOTIFICATION_FAILED, "Failed to activate TRUCK CAN notifications");
      return HAL_ERROR;  // Use HAL-defined error code for system failure
  }

  // Activate notification for AUX CAN (hcan2)
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      user_error_handler(ERROR_CAN_NOTIFICATION_FAILED, "Failed to activate AUX CAN notifications");
      return HAL_ERROR;  // Use HAL-defined error code for system failure
  }

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

  CAN1_Rx_QueueHandle = osMessageQueueNew(CAN_PACKET_POOL_SIZE, sizeof(CAN_Packet *), &CAN1_Rx_Queue_attributes);
  if (CAN1_Rx_QueueHandle == NULL) {
      user_error_handler(ERROR_RTOS_QUEUE_INIT_FAILED, "Failed to create CAN1 Rx queue");
  }
  CAN2_Rx_QueueHandle = osMessageQueueNew(CAN_PACKET_POOL_SIZE, sizeof(CAN_Packet *), &CAN2_Rx_Queue_attributes);
  if (CAN2_Rx_QueueHandle == NULL) {
      user_error_handler(ERROR_RTOS_QUEUE_INIT_FAILED, "Failed to create CAN2 Rx queue");
  }
  Tx_QueueHandle = osMessageQueueNew(CAN_PACKET_POOL_SIZE, sizeof(CAN_Packet *), &Tx_Queue_attributes);
  if (Tx_QueueHandle == NULL) {
	  user_error_handler(ERROR_RTOS_QUEUE_INIT_FAILED, "Failed to create Tx queue");
  }

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CAN1_Rx_Task */
  CAN1_Rx_TaskHandle = osThreadNew(StartCAN1_Rx_Task, NULL, &CAN1_Rx_Task_attributes);

  /* creation of CAN2_Rx_Task */
  CAN2_Rx_TaskHandle = osThreadNew(StartCAN2_Rx_Task, NULL, &CAN2_Rx_Task_attributes);

  /* creation of Tx_Task */
  Tx_TaskHandle = osThreadNew(Start_Tx_Task, NULL, &Tx_Task_attributes);

  /* creation of Housekeeping_Ta */
  Housekeeping_TaHandle = osThreadNew(StartHousekeeping_Task, NULL, &Housekeeping_Ta_attributes);

  /* creation of CAN1_Send_Reque */
  CAN1_Send_RequeHandle = osThreadNew(StartCAN_Tx_Send_Requests, NULL, &CAN1_Send_Reque_attributes);

  /* USER CODE BEGIN RTOS_THREADS */


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8399;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 166;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DISPL_CS_Pin|TOUCH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISPL_DC_Pin|DISPL_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPL_CS_Pin TOUCH_CS_Pin */
  GPIO_InitStruct.Pin = DISPL_CS_Pin|TOUCH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPL_DC_Pin DISPL_RST_Pin */
  GPIO_InitStruct.Pin = DISPL_DC_Pin|DISPL_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_INT_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  for (;;) {
	  CAN_Packet *packet = NULL; // Step 1: Declare a pointer to a CAN_Packet

	  // Step 2: Wait for a packet from the CAN2 queue
	  if (osMessageQueueGet(CAN1_Rx_QueueHandle, &packet, NULL, osWaitForever) == osOK) {
		  if (packet != NULL) {
			  process_can_rx_packet(packet); // Step 3: Process the received packet
			  _free_can_packet_from_circular_buffer(QUEUE_RX_CAN1, packet);    // Step 4: Return the packet to the pool
		  }
	  }
	  osThreadYield();
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
  for (;;) {
      CAN_Packet *packet = NULL; // Step 1: Declare a pointer to a CAN_Packet

      // Step 2: Wait for a packet from the CAN2 queue
      if (osMessageQueueGet(CAN2_Rx_QueueHandle, &packet, NULL, osWaitForever) == osOK) {
          if (packet != NULL) {
              process_can_rx_packet(packet); // Step 3: Process the received packet
              _free_can_packet_from_circular_buffer(QUEUE_RX_CAN2, packet);    // Step 4: Return the packet to the pool
          }
      }
      osThreadYield();
  }
  /* USER CODE END StartCAN2_Rx_Task */
}

/* USER CODE BEGIN Header_Start_Tx_Task */
/**
* @brief Function implementing the Tx_Task thread.
*        This task retrieves messages from the Tx queue and sends them to the CAN bus.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Tx_Task */
void Start_Tx_Task(void *argument)
{
  /* USER CODE BEGIN Start_Tx_Task */
  /* Infinite loop */
  for(;;)
  {
	  if (Touch_GotATouch(0))
		  touchgfxSignalVSync();		// asks TouchGFX to start initial display drawing
	  MX_TouchGFX_Process();

    osDelay(1);
  }
  /* USER CODE END Start_Tx_Task */
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
  /* USER CODE BEGIN StartHousekeeping_Task */
  for (;;) {
	  update_oled_status(DEFAULT_OLED_CAN_INSTANCE);
      // Step 1: Update OLED display for the specified CAN instance


      // Step 2: Perform system health checks
      // check_system_health(); // Uncomment if system health monitoring is implemented
      osThreadYield();
//      osDelay(ONE_SECOND); // Delay task execution to run periodically (every 1000ms)
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
	  uint32_t queue_length = osMessageQueueGetCount(Tx_QueueHandle);
	  char msg[100];
	  if (queue_length > 7)
		  sprintf(msg, "Cannot send Tx requests - Tx_QueueHandle has %lu elements and it would overflow the buffer.", queue_length);
	  else
	  {
		  // Step 1: Update OLED display for the specified CAN instance
		  sprintf(msg, "%s", "Sending requests...");
		  send_all_requests();
	  }

      send_console_msg(msg);

      osThreadYield();

      osDelay(ONE_SECOND);

  }
  /* USER CODE END StartCAN_Tx_Send_Requests */
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
  if (htim->Instance == TIM14)
  {
    touchgfxSignalVSync();
  }
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
