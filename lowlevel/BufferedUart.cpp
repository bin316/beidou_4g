/*
 * BufferedUart.cpp
 *
 *  Created on: Dec 28, 2024
 *      Author: IRIS
 */

#include <BufferedUart.h>

#include "magic_enum.hpp"

#include "stdarg.h"
using namespace magic_enum;

/** 静态表代替 std::map：ISR 内禁止 map::operator[]（会 newlib malloc → HardFault） */
#define MAX_BUFFERED_UART_NUM 4
struct BufferedUartMapEntry {
	UART_HandleTypeDef *handle;
	BufferedUart *instance;
};
static BufferedUartMapEntry uart_map_array[MAX_BUFFERED_UART_NUM] = {};

static BufferedUart *find_buffered_uart(UART_HandleTypeDef *handle)
    {
    for (int i = 0; i < MAX_BUFFERED_UART_NUM; ++i)
	{
	if (uart_map_array[i].handle == handle)
	    {
	    return uart_map_array[i].instance;
	    }
	}
    return nullptr;
    }

static bool add_buffered_uart(UART_HandleTypeDef *handle, BufferedUart *instance)
    {
    for (int i = 0; i < MAX_BUFFERED_UART_NUM; ++i)
	{
	if (uart_map_array[i].handle == nullptr)
	    {
	    uart_map_array[i].handle = handle;
	    uart_map_array[i].instance = instance;
	    return true;
	    }
	}
    return false;
    }

static void remove_buffered_uart(UART_HandleTypeDef *handle)
    {
    for (int i = 0; i < MAX_BUFFERED_UART_NUM; ++i)
	{
	if (uart_map_array[i].handle == handle)
	    {
	    uart_map_array[i].handle = nullptr;
	    uart_map_array[i].instance = nullptr;
	    return;
	    }
	}
    }

BufferedUart::BufferedUart(UART_HandleTypeDef *huart, size_t tx_buffer_size,
		size_t rx_buffer_size, size_t tx_dma_size, size_t rx_dma_size) {
//	parameter check
	configASSERT(huart!=NULL);
	configASSERT(tx_buffer_size != 0 && rx_buffer_size != 0);

	this->phuart = huart;

//	buffer create and check
	rx_stream_buffer = xStreamBufferCreate(rx_buffer_size, 1);
	configASSERT(rx_stream_buffer != NULL);
	tx_stream_buffer = xStreamBufferCreate(tx_buffer_size, 1);
	configASSERT(tx_stream_buffer != NULL);

//	alloc dma buffer
	tx_dma_buffer = (uint8_t*) pvPortMalloc(tx_dma_size);
	configASSERT(tx_dma_buffer != NULL);
	rx_dma_buffer = (uint8_t*) pvPortMalloc(rx_dma_size);
	configASSERT(rx_dma_buffer != NULL);
	this->rx_dma_size = rx_dma_size;
	this->tx_dma_size = tx_dma_size;

//	create read and write semaphore
	tx_sem = xSemaphoreCreateBinary();
	configASSERT(tx_sem != NULL);
	rx_sem = xSemaphoreCreateBinary();
	configASSERT(rx_sem != NULL);

//	update linkage
	configASSERT(add_buffered_uart(huart, this));

//	callbacks register
	HAL_UART_RegisterCallback(phuart, HAL_UART_ERROR_CB_ID, isr_uart_error);
	HAL_UART_RegisterCallback(phuart, HAL_UART_TX_COMPLETE_CB_ID,
			isr_uart_tx_complete);
	HAL_UART_RegisterRxEventCallback(phuart, isr_uart_rti);
//	create daemon thread
	xTaskCreate(uart_thread, "uart_thread", 128, this, osPriorityISR,
			&uart_thread_handle);
	configASSERT(uart_thread_handle != NULL);

}

BufferedUart::~BufferedUart() {
//	release all freertos instances
	vStreamBufferDelete(tx_stream_buffer);
	vStreamBufferDelete(rx_stream_buffer);
	vPortFree(tx_dma_buffer);
	vPortFree(rx_dma_buffer);
	vSemaphoreDelete(tx_sem);
	vSemaphoreDelete(rx_sem);
	vTaskDelete(uart_thread_handle);
//	update linkage
	remove_buffered_uart(phuart);
}

#define UART_NOTIFY_TX_COMPLETE 0x00000001 << 0
#define UART_NOTIFY_RX_COMPLETE 0x00000001 << 1
#define UART_NOTIFY_RX_ERROR 0x00000001 << 2
#define UART_NOTIFY_TX_START 0x00000001 << 3

void BufferedUart::isr_uart_rti(UART_HandleTypeDef *uart, uint16_t pos) {
	BaseType_t mustYield = false;
	BufferedUart *inst = find_buffered_uart(uart);
//	record the receiced size
//	ignore event if task is not created
	if (inst == nullptr || inst->uart_thread_handle == NULL)
		return;
	if (inst->rx_paused_)
		return;
	inst->received_size = pos;
	xTaskNotifyFromISR(inst->uart_thread_handle,
			UART_NOTIFY_RX_COMPLETE, eSetBits, &mustYield);
	portYIELD_FROM_ISR(mustYield);
}

void BufferedUart::isr_uart_error(UART_HandleTypeDef *uart) {
//	errors are not transmitted to thread anymore, just clear the flags, and continue to receive here
	BufferedUart *inst = find_buffered_uart(uart);
	__HAL_UNLOCK(uart);
	__HAL_UART_CLEAR_IDLEFLAG(uart);
	__HAL_UART_CLEAR_PEFLAG(uart);
	__HAL_UART_CLEAR_FEFLAG(uart);
	__HAL_UART_CLEAR_NEFLAG(uart);
	__HAL_UART_CLEAR_OREFLAG(uart);
	if (inst == nullptr || inst->rx_paused_)
		return;
	HAL_UARTEx_ReceiveToIdle_DMA(uart, inst->rx_dma_buffer,
			inst->rx_dma_size);
}

static uint16_t debug_tx_cnt = 0;

void BufferedUart::isr_uart_tx_complete(UART_HandleTypeDef *uart) {
	BaseType_t mustYield = false;
	BufferedUart *inst = find_buffered_uart(uart);
	//	ignore event if task is not created
	if (inst == nullptr || inst->uart_thread_handle == NULL)
		return;
	xTaskNotifyFromISR(inst->uart_thread_handle,
			UART_NOTIFY_TX_COMPLETE, eSetBits, &mustYield);
	portYIELD_FROM_ISR(mustYield);
	debug_tx_cnt++;
}

//rx buffer has only one writter, so no need to use semaphore
//tx buffer has only one reader, so no need to use semaphore

//the receive operation of rx buffer may have multiple readers, protect it with semaphore
//the send operation of tx buffer may have multiple writers, protect it with semaphore

void BufferedUart::uart_thread(void *argument) {
	BufferedUart *pthis = (BufferedUart*) argument;
	uint32_t notify_value = 0;

//	start receive
	HAL_UARTEx_ReceiveToIdle_DMA(pthis->phuart, pthis->rx_dma_buffer,
			pthis->rx_dma_size);
//	sems initial give
	xSemaphoreGive(pthis->rx_sem);
	xSemaphoreGive(pthis->tx_sem);

	for (;;) {
		xTaskNotifyWait(0, 0xffffffff, &notify_value, portMAX_DELAY);
		if (notify_value & UART_NOTIFY_RX_COMPLETE) {
			if (!pthis->rx_paused_) {
				if (pthis->received_size > 0) {
					xStreamBufferSend(pthis->rx_stream_buffer,
							pthis->rx_dma_buffer, pthis->received_size, 0);
				}
				HAL_UARTEx_ReceiveToIdle_DMA(pthis->phuart, pthis->rx_dma_buffer,
						pthis->rx_dma_size);
			}
		}
		if (notify_value & UART_NOTIFY_RX_ERROR) {
			__HAL_UNLOCK(pthis->phuart);
			__HAL_UART_CLEAR_IDLEFLAG(pthis->phuart);
			__HAL_UART_CLEAR_PEFLAG(pthis->phuart);
			__HAL_UART_CLEAR_FEFLAG(pthis->phuart);
			__HAL_UART_CLEAR_NEFLAG(pthis->phuart);
			__HAL_UART_CLEAR_OREFLAG(pthis->phuart);
			if (!pthis->rx_paused_) {
				HAL_UARTEx_ReceiveToIdle_DMA(pthis->phuart, pthis->rx_dma_buffer,
						pthis->rx_dma_size);
			}
		}
		if (notify_value & UART_NOTIFY_TX_COMPLETE) {
			if (pthis->rx_paused_) {
				/* 注入期间不用流式 TX DMA */
				pthis->tx_busy = false;
			} else {
				size_t size = xStreamBufferBytesAvailable(
						pthis->tx_stream_buffer);
				if (size > 0) {
					size = size > pthis->tx_dma_size ?
							pthis->tx_dma_size : size;
					xStreamBufferReceive(pthis->tx_stream_buffer,
							pthis->tx_dma_buffer, size, 0);
					HAL_UART_Transmit_DMA(pthis->phuart, pthis->tx_dma_buffer,
							size);
				} else {
					pthis->tx_busy = false;
					if (pthis->rs485_config.enabled)
						HAL_GPIO_WritePin(pthis->rs485_config.port,
								pthis->rs485_config.pin, GPIO_PIN_RESET);
				}
			}
		}
		if (notify_value & UART_NOTIFY_TX_START) {
			if (!pthis->rx_paused_ && !pthis->tx_busy) {
				size_t size = xStreamBufferBytesAvailable(
						pthis->tx_stream_buffer);
				if (size > 0) {
					size = size > pthis->tx_dma_size ?
							pthis->tx_dma_size : size;
					xStreamBufferReceive(pthis->tx_stream_buffer,
							pthis->tx_dma_buffer, size, 0);
					if (pthis->rs485_config.enabled)
						HAL_GPIO_WritePin(pthis->rs485_config.port,
								pthis->rs485_config.pin, GPIO_PIN_SET);
					pthis->tx_busy = true;
					HAL_UART_Transmit_DMA(pthis->phuart, pthis->tx_dma_buffer,
							size);
				}
			}
		}
//		notify all cleard
		notify_value = 0;
	}
}

//use semaphore to protect
int BufferedUart::read(void *data, size_t size, uint32_t timeout,
		bool readRest) {
	if (size == 0)
		return 0;
	if (xSemaphoreTake(rx_sem, this->option_sem_timeout) == pdPASS) {
//		the caller now holds the semaphore
		uint32_t size_read = 0;
		size_read = xStreamBufferReceive(rx_stream_buffer, data, size, timeout);
		xSemaphoreGive(rx_sem);
//		return the actual size read
//		$notice the read size may not always be equal to the size parameter
//			due to the variety of the buffer timeout parameter
		return size_read;
	}
	return -1;
}

//use semaphore to protect
int BufferedUart::write(void *data, size_t size, uint32_t timeout) {
	if (size == 0)
		return 0;
//	try to claim the semaphore, if failed return -1
	if (xSemaphoreTake(tx_sem, this->option_sem_timeout) == pdPASS) {
		//		the caller now holds the semaphore
		uint32_t size_write = 0;
		size_write = xStreamBufferSend(tx_stream_buffer, data, size, timeout);
//		notify the daemon thread to start a transmission
		xTaskNotify(uart_thread_handle, UART_NOTIFY_TX_START, eSetBits);
		xSemaphoreGive(tx_sem);
//		return the actual size written
//		$notice the written size may not always be equal to the size parameter
//			due to the variety of the buffer timeout parameter
		return size_write;
	}

	return -1;
}

int BufferedUart::ioctl(int cmd, void *arg) {
	io_opt_t opt = (io_opt_t) cmd;
//	$notice that the cmd could not be combined
	if (!enum_contains<io_opt_t>(opt))
		return -1;
	switch (opt) {
	case _non_block:
		this->option_sem_timeout = 0;
		break;
	case _block:
		this->option_sem_timeout = portMAX_DELAY;
		break;
	case _baudrate:
//		$todo implement the uart paramaters on the fly config later
		break;
	case _parity:
		break;
	case _stop_bits:
		break;
	case _data_bits:
		break;
	case _sem_timeout:
		this->option_sem_timeout = *(uint32_t*) arg;
		break;
	case _trigger_level:
		xStreamBufferSetTriggerLevel(tx_stream_buffer, *(uint32_t*) arg);
		break;
	case _flush_rx:
		xStreamBufferReset(rx_stream_buffer);
		break;
	case _rx_pause:
		this->rx_paused_ = true;
		(void) HAL_UART_Abort(this->phuart); /* 收发一并停，清 BUSY */
		__HAL_UNLOCK(this->phuart);
		this->phuart->gState = HAL_UART_STATE_READY;
		this->phuart->RxState = HAL_UART_STATE_READY;
		this->tx_busy = false;
		xStreamBufferReset(this->tx_stream_buffer);
		xStreamBufferReset(this->rx_stream_buffer);
		break;
	case _rx_resume:
		this->tx_busy = false;
		this->rx_paused_ = false;
		(void) HAL_UARTEx_ReceiveToIdle_DMA(this->phuart, this->rx_dma_buffer,
				this->rx_dma_size);
		break;
	case _soft_rs485: {
		rs485_config_t *config = (rs485_config_t*) arg;
		this->rs485_config.enabled = config->enabled;
		this->rs485_config.port = config->port;
		this->rs485_config.pin = config->pin;
	}
		break;

	}
	return 0;
}

int BufferedUart::print(const char *fmt, ...) {
//	store the format string in a dynamically allocated buffer
	char *buffer = (char*) pvPortMalloc(256);
	if (buffer == NULL)
		return -1;
	memset(buffer, 0, 256);
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, 256, fmt, args);
	va_end(args);
	int result = 0;
	result = this->write(buffer, strlen(buffer));
	vPortFree(buffer);

	return result;
}

/*
 * $bugfix [comment date--Feb 6, 2025] :implemented a new print function with va_list
 * 	fixed the problem of the variable list could not pass to the print function
 *
 * 	this function is used for va list passing
 */
int BufferedUart::print(const char *fmt, va_list args) {
	//	store the format string in a dynamically allocated buffer
	char *buffer = (char*) pvPortMalloc(256);
	if (buffer == NULL)
		return -1;
	memset(buffer, 0, 256);
	vsnprintf(buffer, 256, fmt, args);
	int result = 0;
	result = this->write(buffer, strlen(buffer));
	vPortFree(buffer);

	return result;
}

int BufferedUart::scan(const char *fmt, ...) {
//    $todo implement the scan function later
//	print should be enough for now
	return 0;
}

