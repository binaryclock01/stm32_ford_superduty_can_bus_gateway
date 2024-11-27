/*
 * config.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Ryan
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

// if this is uncommented, the program acts as if it was the truck, simulating can responses to requests
#define IS_SIMULATOR

#define HID_DEVICE_UART 			USART1
#define HID_QUEUE_RX_QUEUE_MAX		15
#define HID_RXDATA_MAX_LENGTH 		5
#define UART_RX_BUFFER_SIZE 		128

#define ISR_BUFFER_SIZE		10

#define CAN_REQUEST_INTERVAL (50 * ONE_MILLISECOND) /**< Interval for CAN requests in ms. */
#define DLC_MAX 8                                   /**< Maximum data length for standard CAN frames */
#define MAX_RETRIES 3                               /**< Maximum retries for CAN message transmission */

#define LOG_BUFFER_SIZE 16   /**< Maximum number of messages in the log buffer */
#define LOG_MESSAGE_MAX_LENGTH 256  /**< Maximum length of each log message */


/** Used in can_core.c **/

#define USE_CAN_1
//#define USE_CAN_2
#define CHAR_BIT 8

/** Used in rtos.c **/

#define CAN_PACKET_POOL_SIZE 10 ///< Unified size for CAN packet pool and circular buffers
#define CAN_BUFFER_SIZE CAN_PACKET_POOL_SIZE ///< Circular buffer size matches packet pool size

#define RETRY_DELAY_MS 100   // Delay between retries in milliseconds


#endif /* INC_CONFIG_H_ */
