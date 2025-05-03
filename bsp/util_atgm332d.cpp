/*
 * util_atgm332d.cpp
 *
 *  Created on: Feb 10, 2025
 *      Author: IRIS
 */

#include "utilties.h"

//******FreeRTOS Family Bucket******//
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "queue.h"
#include "task.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "semphr.h"

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

#include "BufferedUart.h"
#include "NMEA0183.h"

#include "usart.h"
#include "NVM.h"

static const util_atgm332d_status_t default_status = { 114.350161f, 30.528721f,
		1739245514, false };

static util_atgm332d_status_t status;

static BufferedUart *uart = NULL;
static TaskHandle_t rxTaskHandle = NULL;

static NVM *nvm_atgm332d = NULL;

static int32_t deactivate_hysteresis = 0;
static bool updated_since_activate = false;

TimerHandle_t deactivate_timer = NULL;

static void atgm332d_UpdateLocalTime(void) {
	static bool Updated = false;

	if (Updated)
		return;
	//    $notice update time only once
	time_t t = status.time;
	util_lowpower_update_rtc(t);
	Updated = true;
}

static void atgm332d_rx_thread(void *argument) {
	NMEA0183 parser;
	char *buffer = (char*) pvPortMalloc(1000);
	parser.setMessageField(buffer, 1000);

	auto clear = [&buffer]() {
		memset(buffer, 0, 1000);
	};
	loop:

	clear();
	if (uart->read(buffer, 1000, 100) <= 0)
		goto loop;

	if (parser.parseRmcMessage() != false) {
		auto rmc = parser.getContent<NMEA0183::RMC_t>();
		status.latitude = rmc->latitude;
		status.longitude = rmc->longitude;
		status.time = rmc->unixTime;
		status.position_fixed = true;
		atgm332d_UpdateLocalTime();
		nvm_atgm332d->save();

		logInfo("updated pos: [lon%f, la%f]", status.longitude, status.latitude);

		/*激活后首次更新*/
		if (!updated_since_activate) {
			/*创建deact定时器*/
			if (deactivate_timer == NULL) {
				/*至少迟滞1秒*/
				deactivate_hysteresis =
						deactivate_hysteresis <= 0 ? 1 : deactivate_hysteresis;
				deactivate_timer = xTimerCreate("bd_deact",
						pdMS_TO_TICKS(deactivate_hysteresis * 1000), pdFALSE,
						NULL, [](TimerHandle_t xtimer) {
							util_atgm332d_deactivate();
							xTimerDelete(xtimer, portMAX_DELAY);
							deactivate_timer = NULL;
						});
			}
			/*拉起标志位，取消激活之前，后续更新都不会再进入这条分支*/
			updated_since_activate = true;
		}

	}

	goto loop;
}

void __util_atgm332d_init__(void) {
//	uart = new BufferedUart(&huart2, 32, 1200, 32, 1200);
//	configASSERT(uart != nullptr);

	nvm_atgm332d = new NVM(NVM::partition_bd, (void*) &status,
			(void*) &default_status, sizeof(util_atgm332d_status_t));
	configASSERT(nvm_atgm332d != nullptr);

	util_atgm332d_load();

//	xTaskCreate(atgm332d_rx_thread, "atgm332d rx", 512, NULL, osPriorityHigh,
//			&rxTaskHandle);
//	configASSERT(rxTaskHandle != NULL);
//	logInfo("bd positioning daemon thread started..");
}

/*
 * @func util_atgm332d_load
 * @param None
 * @return util_atgm332d_status_t
 * @brief
 * 	从nvm加载状态，可以在不初始化的情况下加载状态
 */
void util_atgm332d_load(void) {
	nvm_atgm332d->load();
	if (nvm_atgm332d->isFactoryDefault()) {
		nvm_atgm332d->restoreDefault();
		nvm_atgm332d->save();
	}
	status.position_fixed = false;
}

util_atgm332d_status_t util_atgm332d_get_status(void) {
	return status;
}

/*
 * @func util_atgm332d_activate
 * @param hysteresis uint32_t
 * @return void
 * @brief 激活北斗模块, hysteresis表示自动掉线迟滞，当北斗数据首次刷新后，延迟这个秒数后关闭北斗模块
 */
void util_atgm332d_activate(uint32_t hysteresis_sec) {
	/*不重复激活*/
	if (HAL_GPIO_ReadPin(BD_PWR_GPIO_Port, BD_PWR_Pin) == GPIO_PIN_SET)
		return;
	HAL_GPIO_WritePin(BD_PWR_GPIO_Port, BD_PWR_Pin, GPIO_PIN_SET);
	deactivate_hysteresis = hysteresis_sec;
	updated_since_activate = false;
}

/*
 * @func util_atgm332d_deactivate
 * @param void
 * @return void
 * @brief 立即关闭北斗模块
 */
void util_atgm332d_deactivate(void) {
	HAL_GPIO_WritePin(BD_PWR_GPIO_Port, BD_PWR_Pin, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(50));
	updated_since_activate = false;
	deactivate_hysteresis = 0;
}

