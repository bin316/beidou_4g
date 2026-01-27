/*
 * Xuart.cpp
 *
 *  Created on: Apr 14, 2025
 *      Author: IRIS
 */

#include <Xuart.h>

#include "errno.h"

#define MAX_UART_NUM 4
struct UartMapEntry {
    UART_HandleTypeDef* handle;
    Xuart* instance;
};
static UartMapEntry uart_map_array[MAX_UART_NUM] = {0};

static Xuart* find_uart_instance(UART_HandleTypeDef* handle) {
    for (int i = 0; i < MAX_UART_NUM; ++i) {
        if (uart_map_array[i].handle == handle) {
            return uart_map_array[i].instance;
        }
    }
    return nullptr;
}

static bool add_uart_instance(UART_HandleTypeDef* handle, Xuart* instance) {
    for (int i = 0; i < MAX_UART_NUM; ++i) {
        if (uart_map_array[i].handle == nullptr) {
            uart_map_array[i].handle = handle;
            uart_map_array[i].instance = instance;
            return true;
        }
    }
    return false;
}

static void remove_uart_instance(UART_HandleTypeDef* handle) {
    for (int i = 0; i < MAX_UART_NUM; ++i) {
        if (uart_map_array[i].handle == handle) {
            uart_map_array[i].handle = nullptr;
            uart_map_array[i].instance = nullptr;
            return;
        }
    }
}

Xuart::Xuart(UART_HandleTypeDef *huart, size_t tx_buffer_size,
		size_t rx_buffer_size, size_t tx_dma_size, size_t rx_dma_size) {
	// $todo Auto-generated constructor stub
	this->phuart = huart;
	this->tx_dma_size = tx_dma_size;
	this->rx_dma_size = rx_dma_size;
	this->tx_buffer_size = tx_buffer_size;
	this->rx_buffer_size = rx_buffer_size;
}

Xuart::~Xuart() {
	// $todo Auto-generated destructor stub
}

bool Xuart::isOpened(Xuart *instance) {
    if (find_uart_instance(instance->phuart) != nullptr) {
        return true;
    }
    return false;
}

int Xuart::open(mode_t mode) {
    /*do not open twice*/
    if (isOpened(this)) {
        return EBUSY;
    }
    this->current_mode = mode;
    /*try to allocate resources*/
    int allocate_result = allocate_resources();
    if (allocate_result != 0) {
        return allocate_result;
    }
    /*add the opened uart to the array*/
    if (!add_uart_instance(phuart, this)) {
        release_resources();
        return ENOMEM;
    }
    /*register callbacks*/
    // 错误回调
    HAL_UART_RegisterCallback(phuart, HAL_UART_ERROR_CB_ID, isr_uart_error);

    // 发送完成回调
    if (mode == Mode_FullDuplex || mode == Mode_TxOnly) {
        HAL_UART_RegisterCallback(phuart, HAL_UART_TX_COMPLETE_CB_ID, isr_uart_tx_complete);
    }

    // 接收回调
    if (mode == Mode_FullDuplex || mode == Mode_RxOnly) {
        HAL_UART_RegisterRxEventCallback(phuart, isr_uart_rti);
        HAL_UARTEx_ReceiveToIdle_DMA(phuart, this->rx_dma_buffer, this->rx_dma_size);
    } else {
        HAL_UART_UnRegisterRxEventCallback(phuart);
    }
    
    /*success*/
    return 0;
}

int Xuart::close(void) {
    /*do not close twice*/
    if (!isOpened(this)) {
        return EBUSY;
    }
    /*disable usart intterrupt*/
    HAL_UART_Abort(phuart);
    /*unregister all callbacks*/
    HAL_UART_UnRegisterRxEventCallback(phuart);
    HAL_UART_UnRegisterCallback(phuart, HAL_UART_ERROR_CB_ID);
    HAL_UART_UnRegisterCallback(phuart, HAL_UART_TX_COMPLETE_CB_ID);
    /*free all resources*/
    release_resources();
    /*remove item from array*/
    remove_uart_instance(phuart);
    /*success*/
    return 0;
}

int Xuart::read(void *data, size_t size, uint32_t timeout) {
	/*open check*/
	if (!isOpened(this)) {
		return EBUSY;
	}

	/*size check*/
	if (size == 0) {
		return 0;
	}

	/*特殊处理：如果 size 为 0xFFFFFFFF，读取缓冲区中所有内容*/
	if (size == 0xFFFFFFFF) {
		xSemaphoreTake(this->rx_mutex, timeout);

		/*获取缓冲区中可用数据大小*/
		size_t available_size = xStreamBufferBytesAvailable(
				this->rx_stream_buffer);
		if (available_size == 0) {
			/*如果没有数据，立即返回*/
			xSemaphoreGive(this->rx_mutex);
			return 0;
		}

		/*读取所有可用数据*/
		size_t size_read = xStreamBufferReceive(this->rx_stream_buffer, data,
				available_size, 0);
		xSemaphoreGive(this->rx_mutex);

		return size_read;
	}

	/*正常读取指定大小的数据*/
	xSemaphoreTake(this->rx_mutex, timeout);

	/*尝试从流缓冲区读取数据*/
	size_t size_read = xStreamBufferReceive(this->rx_stream_buffer, data, size,
			timeout);
	xSemaphoreGive(this->rx_mutex);

	return size_read;
}

int Xuart::write(void *data, size_t size, uint32_t timeout) {
    if (!isOpened(this)) {
        return -EBUSY;
    }
    if (size == 0) {
        return 0;
    }
    if (xSemaphoreTake(this->tx_mutex, pdMS_TO_TICKS(timeout)) != pdTRUE) {
        return -ETIMEDOUT;
    }
    BaseType_t send_idle = xSemaphoreTake(this->tx_send_binary, 0);
    int ret = 0;
    if (send_idle == pdTRUE) {
        // 发送空闲
        if (size <= this->tx_dma_size) {
            memcpy(this->tx_dma_buffer, data, size);
            HAL_UART_Transmit_DMA(this->phuart, this->tx_dma_buffer, size);
            ret = size;
        } else {
            // 先发第一包
            memcpy(this->tx_dma_buffer, data, this->tx_dma_size);
            HAL_UART_Transmit_DMA(this->phuart, this->tx_dma_buffer, this->tx_dma_size);
            // 剩余数据持续写入流缓冲区
            size_t total_written = this->tx_dma_size;
            uint8_t *data_ptr = static_cast<uint8_t *>(data) + this->tx_dma_size;
            size_t remaining_size = size - this->tx_dma_size;
            while (remaining_size > 0) {
                size_t batch = remaining_size > this->tx_dma_size ? this->tx_dma_size : remaining_size;
                size_t written = xStreamBufferSend(this->tx_stream_buffer, data_ptr, batch, pdMS_TO_TICKS(200));
                if (written == 0) {
                    ret = -EIO;
                    xSemaphoreGive(this->tx_mutex);
                    return ret;
                }
                total_written += written;
                data_ptr += written;
                remaining_size -= written;
            }
            ret = total_written;
        }
        // 不在此释放tx_send_binary，由isr释放
    } else {
        // 发送忙，允许写入缓冲区，分批写入
        size_t total_written = 0;
        uint8_t *data_ptr = static_cast<uint8_t *>(data);
        size_t remaining_size = size;
        while (remaining_size > 0) {
            size_t batch = remaining_size > this->tx_dma_size ? this->tx_dma_size : remaining_size;
            size_t written = xStreamBufferSend(this->tx_stream_buffer, data_ptr, batch, pdMS_TO_TICKS(200));
            if (written == 0) {
                ret = -EIO;
                xSemaphoreGive(this->tx_mutex);
                return ret;
            }
            total_written += written;
            data_ptr += written;
            remaining_size -= written;
        }
        if (ret == 0) {
            ret = total_written;
        }
    }
    xSemaphoreGive(this->tx_mutex);
    return ret;
}

int Xuart::ioctl(int cmd, void *arg) {
	/*open check*/
	if (!isOpened(this)) {
		return EBUSY;
	}
	return 0;
}

void Xuart::isr_uart_rti(UART_HandleTypeDef *uart, uint16_t pos) {
    /*$todo this is a stub function*/
    BaseType_t mustYield = false;
    /*check if the uart is opened*/
    Xuart *pthis = find_uart_instance(uart);
    if (pthis != nullptr) {
        /*continue receive*/
        HAL_UARTEx_ReceiveToIdle_DMA(uart, pthis->rx_dma_buffer,
                pthis->rx_dma_size);
        /*get the size of the data in the buffer*/
        size_t size = xStreamBufferSpacesAvailable(pthis->rx_stream_buffer);
        /*if the free space is not capable for holding all received data, ignore the content that exceeds the size of buffer*/
        if (size > pos)
            /*space is enough*/
            size = pos;
        if (size > 0) {
            xStreamBufferSendFromISR(pthis->rx_stream_buffer,
                    pthis->rx_dma_buffer, size, &mustYield);
            portYIELD_FROM_ISR(mustYield);
        }
    }
}

void Xuart::isr_uart_error(UART_HandleTypeDef *uart) {
    __HAL_UNLOCK(uart);
    __HAL_UART_CLEAR_IDLEFLAG(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_OREFLAG(uart);
    Xuart *pthis = find_uart_instance(uart);
    if (pthis != nullptr) {
        HAL_UARTEx_ReceiveToIdle_DMA(uart, pthis->rx_dma_buffer,
                pthis->rx_dma_size);
    }
}

int Xuart::print(const char *fmt, ...) {
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

int Xuart::print(const char *fmt, va_list args) {
	// 动态分配缓冲区
	char *buffer = (char*) pvPortMalloc(256);
	if (buffer == NULL)
		return -1;

	memset(buffer, 0, 256);

	// 格式化字符串
//    int needed_size = vsnprintf(buffer, 256, fmt, args);
	vsnprintf(buffer, 256, fmt, args);
//    if (needed_size >= 256) {
//        // 缓冲区不足，重新分配
//        vPortFree(buffer);
//        buffer = (char*) pvPortMalloc(needed_size + 1);
//        if (buffer == NULL)
//            return -1;
//        vsnprintf(buffer, needed_size + 1, fmt, args);
//    }

	// 写入 UART
	int result = this->write(buffer, strlen(buffer));

	// 释放缓冲区
	vPortFree(buffer);

	return result;
}

void Xuart::isr_uart_tx_complete(UART_HandleTypeDef *uart) {
    BaseType_t mustYield = false;
    Xuart *pthis = find_uart_instance(uart);
    if (pthis != nullptr) {
        size_t size = xStreamBufferBytesAvailable(pthis->tx_stream_buffer);
        if (size > 0) {
            size = size > pthis->tx_dma_size ? pthis->tx_dma_size : size;
            xStreamBufferReceiveFromISR(pthis->tx_stream_buffer,
                    pthis->tx_dma_buffer, size, &mustYield);
            HAL_UART_Transmit_DMA(pthis->phuart, pthis->tx_dma_buffer, size);
        } else {
            // 发送完成，释放发送状态二值信号量
            xSemaphoreGiveFromISR(pthis->tx_send_binary, &mustYield);
        }
    }
    portYIELD_FROM_ISR(mustYield);
}

int Xuart::allocate_resources(void) {
	/*try to allocate resources*/
	this->rx_dma_buffer = (uint8_t*) pvPortMalloc(this->rx_dma_size);
	if (this->rx_dma_buffer == NULL) {
		return ENOMEM;
	}
	this->tx_dma_buffer = (uint8_t*) pvPortMalloc(this->tx_dma_size);
	if (this->tx_dma_buffer == NULL) {
		vPortFree(this->rx_dma_buffer);
		return ENOMEM;
	}
	this->rx_stream_buffer = xStreamBufferCreate(this->rx_buffer_size, 1);
	if (this->rx_stream_buffer == NULL) {
		vPortFree(this->rx_dma_buffer);
		vPortFree(this->tx_dma_buffer);
		return ENOMEM;
	}
	this->tx_stream_buffer = xStreamBufferCreate(this->tx_buffer_size, 1);
	if (this->tx_stream_buffer == NULL) {
		vPortFree(this->rx_dma_buffer);
		vPortFree(this->tx_dma_buffer);
		vStreamBufferDelete(this->rx_stream_buffer);
		return ENOMEM;
	}
	this->tx_mutex = xSemaphoreCreateMutex();
	if (this->tx_mutex == NULL) {
		vPortFree(this->rx_dma_buffer);
		vPortFree(this->tx_dma_buffer);
		vStreamBufferDelete(this->rx_stream_buffer);
		vStreamBufferDelete(this->tx_stream_buffer);
		return ENOMEM;
	}
	this->rx_mutex = xSemaphoreCreateMutex();
	if (this->rx_mutex == NULL) {
		vPortFree(this->rx_dma_buffer);
		vPortFree(this->tx_dma_buffer);
		vStreamBufferDelete(this->rx_stream_buffer);
		vStreamBufferDelete(this->tx_stream_buffer);
		vSemaphoreDelete(this->tx_mutex);
		return ENOMEM;
	}
	// 新增：创建发送二值信号量
	this->tx_send_binary = xSemaphoreCreateBinary();
	if (this->tx_send_binary == NULL) {
		vPortFree(this->rx_dma_buffer);
		vPortFree(this->tx_dma_buffer);
		vStreamBufferDelete(this->rx_stream_buffer);
		vStreamBufferDelete(this->tx_stream_buffer);
		vSemaphoreDelete(this->tx_mutex);
		vSemaphoreDelete(this->rx_mutex);
		return ENOMEM;
	}
	xSemaphoreGive(this->tx_send_binary); // 初始为可用
	return 0;
}

int Xuart::release_resources(void) {
	/*release all allocated resource on heap*/
	vPortFree(this->rx_dma_buffer);
	vPortFree(this->tx_dma_buffer);
	vStreamBufferDelete(this->rx_stream_buffer);
	vStreamBufferDelete(this->tx_stream_buffer);
	vSemaphoreDelete(this->rx_mutex);
	vSemaphoreDelete(this->tx_mutex);
	vSemaphoreDelete(this->tx_send_binary);
	this->rx_dma_buffer = NULL;
	this->tx_dma_buffer = NULL;
	this->rx_stream_buffer = NULL;
	this->tx_stream_buffer = NULL;
	this->rx_mutex = NULL;
	this->tx_mutex = NULL;
	this->tx_send_binary = NULL;
	return 0;
}

