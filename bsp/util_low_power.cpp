/*
 * util_low_power.cpp
 *
 *  Created on: Oct 21, 2024
 *      Author: IRIS
 */

#include "PRODUCT_CONFIG.h"

#include "main.h"
#include "i2c.h"
#include "rtc.h"

#if PROD_CONFIG_FACTORY_ENABLE_IWDG
#include "iwdg.h"
#endif

#include "stdint-gcc.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "typeinfo"
#include "time.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "semphr.h"
#include "timers.h"

#include "utilties.h"

#include "magic_enum.hpp"
using namespace magic_enum;

static util_lowpower_wake_source_e wake_source =
		util_lowpower_wake_source_e::pin;

static util_lowpower_config_s factory_lowpower_config = { .wake_pin_enable =
		false, .requested_wakeup_period = 0, .wakeup_remain = 0, };

static util_lowpower_config_s rt_lowpower_config;

#include "NVM.h"

NVM *nvm_lopwr = NULL;

#if PROD_CONFIG_FACTORY_ENABLE_IWDG
static TimerHandle_t iwdg_feed_timer = NULL;
#endif
//this delay must be greater than 0.1s
#define LOWPOWER_STANDBY_DELAY	250

#if PROD_CONFIG_FACTORY_ENABLE_IWDG
static void util_lowpower_iwdg_refresh_timer_callback(TimerHandle_t xtimer) {
	util_lowpower_iwdg_feed();
	HAL_GPIO_TogglePin(LED_BOOT_GPIO_Port, LED_BOOT_Pin);
}
#endif
/*
 * 低功耗初始化流程：
 *	1. 读取配置，检查是否需要休眠
 *		1)如果处在休眠状态（剩余时长不为0），根据剩余的休眠时长继续休眠
 *		2)如果剩余时长为0，退出休眠状态，初始化完成，系统继续运行
 * */
extern int INT1_PIN_STATE;
void __util_lowpower_init__(void) {
	// 检查是否是由RTC唤醒
	nvm_lopwr = new NVM(NVM::partition_lowpower, &rt_lowpower_config,
			&factory_lowpower_config, sizeof(rt_lowpower_config));
	configASSERT(nvm_lopwr != NULL);
	nvm_lopwr->load();
	if (nvm_lopwr->isFactoryDefault()) {
		nvm_lopwr->restoreDefault();
		nvm_lopwr->save();
		logInfo("initially restored..");
	}

	/*timezone setup*/
	setenv("TZ", "UTC+1", 1); // 设置为 UTC
//	unsetenv("TZ"); // 设置为 UTC
	tzset();             // 更新时区设置

	// 检查IWDG复位标志
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		// 处理IWDG复位事件（如记录日志、点亮LED等）
		// 示例：通过串口输出复位信息
		logWarning("watchdog triggered!");
		// 清除复位标志
		__HAL_RCC_CLEAR_RESET_FLAGS();
	}

	/*
	 * 创建看门狗刷新定时器
	 * 创建后立即刷新一次看门狗，然后再启动定时器
	 * */
#if PROD_CONFIG_FACTORY_ENABLE_IWDG
	util_lowpower_iwdg_feed();
	iwdg_feed_timer = xTimerCreate("iwdg_refresh", pdMS_TO_TICKS(200), pdTRUE,
	NULL, util_lowpower_iwdg_refresh_timer_callback);
	configASSERT(iwdg_feed_timer!=NULL);
	xTimerStart(iwdg_feed_timer, portMAX_DELAY);
	logInfo("watchdog feed timer created..");
#endif
// 检查是否是由Wake Up引脚唤醒
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_WU) || INT1_PIN_STATE
			|| __HAL_PWR_GET_FLAG(PWR_FLAG_WUF1)
			|| (HAL_GPIO_ReadPin(SENSOR_INT1_GPIO_Port, SENSOR_INT1_Pin)
					== GPIO_PIN_SET)) {
		// 清除Wake Up引脚唤醒标志
		wake_source = util_lowpower_wake_source_e::pin;
	} else if (__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTF)) {
		// 清除RTC唤醒标志
		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
		wake_source = util_lowpower_wake_source_e::rtc;
	} else {
		wake_source = util_lowpower_wake_source_e::regular;
	}
	logInfo("wake source: %s..", enum_name(wake_source).data());

	if (rt_lowpower_config.wakeup_remain == 0) {
		/*如果剩余时间为0不继续休眠过程*/
		logInfo("standby exited..");
		return;
	}
	if (rt_lowpower_config.wake_pin_enable
			&& wake_source == util_lowpower_wake_source_e::pin) {
		/*如果唤醒源不是rtc
		 * 1.长期休眠可能被打断
		 * 2.正常上电启动，无需休眠
		 *
		 * 清空剩余休眠时间，保存，然后退出休眠状态
		 */
		rt_lowpower_config.wakeup_remain = 0;
		nvm_lopwr->save();
		logInfo("standby aborted by pin..");
		return;
	}

	/*持续休眠流程*/
	uint32_t seconds = 0;
	if (rt_lowpower_config.wakeup_remain > 64800) {
		/*如果剩余休眠时间大于18h...*/
		seconds = 64800;
		rt_lowpower_config.wakeup_remain -= 64800;
	} else {
		/*如果剩余休眠时间小于18h...*/
		seconds = rt_lowpower_config.wakeup_remain;
		rt_lowpower_config.wakeup_remain = 0;
	}

	logInfo("standby continued, %d seconds left..",
			rt_lowpower_config.wakeup_remain);

	nvm_lopwr->save();
	/*$notice 继续休眠时，写入一次flash*/
	__flash_sync();
	// 设置RTC唤醒定时器
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
			(seconds > 2) ? (seconds - 1) : (seconds),
			RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
	// 清除RTC和Wake Up标志
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTWF);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
	HAL_PWREx_EnableInternalWakeUpLine();

	// 进入待机模式
//	HAL_PWREx_EnterSHUTDOWNMode();
	HAL_PWR_EnterSTANDBYMode();
}

/*
 * 休眠流程：
 * 	1.读取（同步）低功耗配置
 * 	2.等待一段时间，确保所有设备完成操作
 * 	3.根据设定的总休眠时间计算本次休眠时间
 * 	4.根据唤醒源设置唤醒中断
 *
 * */
void util_lowpower_standby(void) {
	unsigned int seconds;
	bool wake_pin_enable = rt_lowpower_config.wake_pin_enable;
	logInfo("wakeup pin %s", wake_pin_enable ? "enabled" : "disabled");
	/*
	 * 根据requested_wakeup_period开启一次新的休眠
	 * */
	if (rt_lowpower_config.requested_wakeup_period < 3) {
		rt_lowpower_config.wakeup_remain = 0;
		rt_lowpower_config.requested_wakeup_period = 0;
		/*最低休眠3s*/
		seconds = 3;
	}
	if (rt_lowpower_config.requested_wakeup_period > 64800) {
		seconds = 64800;
		rt_lowpower_config.wakeup_remain =
				rt_lowpower_config.requested_wakeup_period - 64800;
	} else {
		seconds = rt_lowpower_config.requested_wakeup_period;
		rt_lowpower_config.wakeup_remain = 0;
	}

	logInfo("requested_wakeup_period %d secs",
			rt_lowpower_config.requested_wakeup_period);
	logInfo("wakeup countdown remain %d secs", rt_lowpower_config.wakeup_remain);
	logInfo("sleep countdown for this cycle %d secs", seconds);

	nvm_lopwr->save();

	__flash_sync();
	//	等待其余设备执行完成

	vTaskDelay(pdMS_TO_TICKS(50));

	taskENTER_CRITICAL();
//	进入关键区后，所有依赖freertos的组件都将失效（包括调试串口）
// 使能唤醒中断
	if (wake_pin_enable) {
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	} else {
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	}
// 设置RTC唤醒定时器
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
			(seconds > 2) ? (seconds - 1) : (seconds),
			RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
	HAL_PWREx_EnableInternalWakeUpLine();
// 清除RTC和Wake Up标志

	/*$notice 请求休眠时，写入一次flash*/
// 进入待机模式
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTWF);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF4);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF5);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	HAL_PWR_EnterSTANDBYMode();

	/*should never reach here*/
	taskEXIT_CRITICAL();
}

util_lowpower_wake_source_e util_lowpower_get_wake_source(void) {
	return wake_source;
}

util_lowpower_config_s util_lowpower_get_config(void) {
	return rt_lowpower_config;
}
void util_lowpower_set_config(util_lowpower_config_s config) {
	rt_lowpower_config.requested_wakeup_period = config.requested_wakeup_period;
	rt_lowpower_config.wake_pin_enable = config.wake_pin_enable;
	rt_lowpower_config.wakeup_remain = config.wakeup_remain;
	nvm_lopwr->save();
}

void util_lowpower_iwdg_feed(void) {
#if PROD_CONFIG_FACTORY_ENABLE_IWDG
	HAL_IWDG_Refresh(&hiwdg);
#endif
}

/*
 * @func util_lowpower_set_rtc
 * @brief update rtc time
 * @param time: the time to set
 * @return void
 * @note always use beijing time
 */
void util_lowpower_update_rtc(time_t time) {
	struct tm *local = localtime(&time);
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	sTime.Hours = local->tm_hour;
	sTime.Minutes = local->tm_min;
	sTime.Seconds = local->tm_sec;

	sDate.WeekDay = local->tm_wday;
	sDate.Month = local->tm_mon + 1;
	sDate.Date = local->tm_mday;
	sDate.Year = local->tm_year - 100;

	logInfo("rtc set: %02d:%02d:%02d %02d/%02d/%02d", sTime.Hours,
			sTime.Minutes, sTime.Seconds, sDate.Year + 2000, sDate.Month,
			sDate.Date);

	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}

time_t util_lowpower_get_rtc(void) {
	time_t time;
	struct tm local;

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	local.tm_hour = sTime.Hours;
	local.tm_min = sTime.Minutes;
	local.tm_sec = sTime.Seconds;
	local.tm_wday = sDate.WeekDay;
	local.tm_mon = sDate.Month - 1;
	local.tm_mday = sDate.Date;
	local.tm_year = sDate.Year + 100;

	time = mktime(&local);

	return time;
}

