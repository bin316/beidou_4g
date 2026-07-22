/*
 * BufferedUart.h
 *
 *  Created on: Dec 28, 2024
 *      Author: IRIS
 */

#ifndef BUFFEREDUART_H_
#define BUFFEREDUART_H_

//******std Family Bucket******//
#include <os_Allocator.hpp>
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

//******FreeRTOS Family Bucket******//
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "queue.h"
#include "task.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "usart.h"



#define BUFFERED_UART_DEFAULT_TX_SIZE 		256
#define BUFFERED_UART_DEFAULT_RX_SIZE 		256

#define BUFFERED_UART_TX_MAX_DMA_SIZE		128
#define BUFFERED_UART_RX_MAX_DMA_SIZE		128
class BufferedUart: public osAllocator<BufferedUart> {
public:

	typedef enum : uint8_t {
		_non_block, _block,/*config non block mode: if operation failed retern intermediatly*/
		_baudrate, _parity, _stop_bits, _data_bits,
		/*config uart parameters: these four options will be taken after next current transmission complete*/
		_sem_timeout,
		/*config the semephore and buffer timeout*/
		_trigger_level,
		/*clear the rx buffer*/
		_flush_rx, _soft_rs485,
		/** 暂停 Idle-DMA 收（AGNSS/CASBIN 注入前），避免与阻塞发送打架 */
		_rx_pause,
		/** 恢复 Idle-DMA 收 */
		_rx_resume
	} io_opt_t;

	typedef struct {
		bool enabled;
		GPIO_TypeDef *port;
		uint16_t pin;
	} rs485_config_t;

	BufferedUart(UART_HandleTypeDef *huart, size_t tx_buffer_size =
	BUFFERED_UART_DEFAULT_TX_SIZE, size_t rx_buffer_size =
	BUFFERED_UART_DEFAULT_RX_SIZE, size_t tx_dma_size =
	BUFFERED_UART_TX_MAX_DMA_SIZE, size_t rx_dma_size =
	BUFFERED_UART_RX_MAX_DMA_SIZE);
	~BufferedUart();

	int read(void *data, size_t size, uint32_t timeout = portMAX_DELAY,bool readRest = false);
	int write(void *data, size_t size, uint32_t timeout = portMAX_DELAY);

//	two c-like useful functions for easy character string operation
	int print(const char *fmt, ...);
	int print(const char *fmt, va_list args);
//	[comment date--Dec 28, 2024] :scan is not implemented yet
	int scan(const char *fmt, ...);

	int ioctl(int cmd, void *arg);

private:

	UART_HandleTypeDef *phuart = NULL;

	rs485_config_t rs485_config = { .enabled = false, .port = NULL, .pin = 0 };

	uint8_t *rx_dma_buffer = NULL;
	uint8_t *tx_dma_buffer = NULL;

	size_t received_size = 0;
	size_t rx_dma_size = 0;
	size_t tx_dma_size = 0;

	bool tx_busy = false;
	volatile bool rx_paused_ = false; /**< true=注入中，停 Idle-DMA 收发 */
	uint32_t option_sem_timeout = portMAX_DELAY;

	StreamBufferHandle_t rx_stream_buffer = NULL;
	StreamBufferHandle_t tx_stream_buffer = NULL;

	SemaphoreHandle_t rx_sem = NULL;
	SemaphoreHandle_t tx_sem = NULL;

	TaskHandle_t uart_thread_handle = NULL;

	static void uart_thread(void *argument);

	static void isr_uart_rti(UART_HandleTypeDef *uart, uint16_t pos);
	static void isr_uart_error(UART_HandleTypeDef *uart);
	static void isr_uart_tx_complete(UART_HandleTypeDef *uart);

};

#endif /* BUFFEREDUART_H_ */
