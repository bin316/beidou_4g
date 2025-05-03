/*
 * Xuart.h
 *
 *  Created on: Apr 14, 2025
 *      Author: IRIS
 */

#ifndef XUART_H_
#define XUART_H_

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

//******HAL Family Bucket******//
#include "main.h"
#include "usart.h"

//******FreeRTOS Family Bucket******//
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "queue.h"
#include "task.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "os_Allocator.hpp"

class Xuart: public osAllocator<Xuart> {
public:
	Xuart(UART_HandleTypeDef *huart, size_t tx_buffer_size = 128,
			size_t rx_buffer_size = 128, size_t tx_dma_size = 128,
			size_t rx_dma_size = 128);
	~Xuart();

	typedef enum : uint8_t {
		baudrate, stop_bits, data_bits, parity, rx_flush, tx_flush
	} io_opt_t;

	int read(void *data, size_t size, uint32_t timeout = portMAX_DELAY);
	int write(void *data, size_t size, uint32_t timeout = portMAX_DELAY);
	int open(void);
	int close(void);

	int print(const char *fmt, ...);
	int print(const char *fmt, va_list args);

	int ioctl(int cmd, void *arg);

private:

	UART_HandleTypeDef *phuart = NULL;
	size_t tx_dma_size = 0;
	size_t rx_dma_size = 0;
	size_t tx_buffer_size = 0;
	size_t rx_buffer_size = 0;

	uint8_t *tx_dma_buffer = NULL;
	uint8_t *rx_dma_buffer = NULL;
	StreamBufferHandle_t tx_stream_buffer = NULL;
	StreamBufferHandle_t rx_stream_buffer = NULL;

	SemaphoreHandle_t tx_mutex = NULL;
	SemaphoreHandle_t rx_mutex = NULL;
	// 改为二值信号量
	SemaphoreHandle_t tx_send_binary = NULL;

	int allocate_resources(void);
	int release_resources(void);

	uint32_t flags = 0x00000000;

	/*helper functions*/
	static bool isOpened(Xuart *instance);

	static void isr_uart_rti(UART_HandleTypeDef *uart, uint16_t pos);
	static void isr_uart_error(UART_HandleTypeDef *uart);
	static void isr_uart_tx_complete(UART_HandleTypeDef *uart);
};

#endif /* XUART_H_ */
