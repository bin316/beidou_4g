/*
 * utilusb.cpp
 *
 *  Created on: Feb 10, 2025
 *      Author: IRIS
 */

/*
 * $tip shell command example
 *
 * SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
 navi_launch, navi_launch, "launch navi task");
 * */

#include "utilties.h"

#include "main.h"

//#include "usbd_cdc_if.h"

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

#include "shell_cpp.h"

#include "Solution.h"

//******FreeRTOS Family Bucket******//
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "queue.h"
#include "task.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "Xuart.h"
#include "SEGGER_RTT.h"

Xuart *shellUart = NULL;
Shell shell;
Log shellLogger;
char *shellBuffer = NULL;

static void shell_connect(void) {
	shellBuffer = (char*) pvPortMalloc(512);
	configASSERT(shellBuffer != NULL);

	shellInit(&shell, shellBuffer, 512);
	logRegister(&shellLogger, &shell);
}

short shellWrite(char *data, unsigned short len) {
	configASSERT(shellUart!=NULL);
	return (short) shellUart->write(data, len);
}
short shellRead(char *data, unsigned short len) {
	configASSERT(shellUart!=NULL);
	return (short) shellUart->read(data, len);
}
extern "C" int __io_putchar(int ch) {
	uint8_t ch_ = (uint8_t) ch;
	HAL_UART_Transmit(&huart2, &ch_, 1, HAL_MAX_DELAY);
	return ch;
}

void __util_shell_init__(void) {
	auto logWritter = [](char *str, short size) {
		if (shellLogger.shell) {
			shellWriteEndLine(&shell, str, size);
		}
	};

	shellUart = new Xuart(&huart2, 512, 512, 512, 512);
	configASSERT(shellUart);

	shellUart->open(Xuart::Mode_TxOnly);

	// shell.read = shellRead;
	shell.read = NULL;
	shell.write = shellWrite;

	shellLogger.write = logWritter;
	shellLogger.active = true;
	shellLogger.shell = &shell;
	shellLogger.level = LOG_VERBOSE;

	shell_connect();
}

/**
 * @brief 通过 SEGGER RTT（SWD）输出 logInfo 等日志，不占用 UART2
 * @note 与 __util_shell_init__ 二选一；需 ST-Link + OpenOCD/RTT Viewer 查看
 */
void __util_rtt_log_init__(void) {
	SEGGER_RTT_Init();

	auto rttLogWriter = [](char *str, short size) {
		if (shellLogger.shell) {
			SEGGER_RTT_Write(0, str, size);
		}
	};

	shellLogger.write = rttLogWriter;
	shellLogger.active = true;
	shellLogger.shell = &shell;
	shellLogger.level = LOG_VERBOSE;

	shellBuffer = (char*) pvPortMalloc(512);
	configASSERT(shellBuffer != NULL);

	shellInit(&shell, shellBuffer, 512);
	logRegister(&shellLogger, &shell);
}
