/*
 * events.cpp
 *
 *  Created on: Oct 21, 2024
 *      Author: IRIS
 */

#include <utilties.h>
#include "main.h"

#include "stdint-gcc.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "semphr.h"
#include "event_groups.h"

#include "utilties.h"

static QueueHandle_t event_queue;

#define IDLE_EVENT_DELAY 100

void __util_events_init__(void) {
//	创建事件组
	event_queue = xQueueCreate(16, sizeof(util_event_code_t));
	configASSERT(event_queue);
	logInfo("event queue module initialized.");
}

void util_events_generate(util_event_code_t code) {
//	发送事件
	if (xPortIsInsideInterrupt()) {
		xQueueSendFromISR(event_queue, &code, NULL);
	} else {
		xQueueSend(event_queue, &code, portMAX_DELAY);
	}
}

bool util_events_poll(util_event_code_t *code, size_t timeout) {
	return xQueueReceive(event_queue, code, pdMS_TO_TICKS(timeout));
}

bool util_events_poll_nonblocked(util_event_code_t *code) {
	return xQueueReceive(event_queue, code, 0);
}

int util_events_flush(void) {
	if (xQueueReset(event_queue) == pdPASS)
		return 0;
	return -1;
}

