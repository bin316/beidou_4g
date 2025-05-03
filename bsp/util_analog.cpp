/*
 * util_analog.cpp
 *
 *  Created on: Oct 23, 2024
 *      Author: IRIS
 */

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "adc.h"

#include "stdint-gcc.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "typeinfo"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "semphr.h"
#include "timers.h"

#include "utilties.h"

#include "NVM.h"

NVM *analog_nvm = NULL;

typedef struct {
	util_analog_config_s config;
	util_analog_status_s status;
} analog_handle;

analog_handle factory = { .config =
		{ . analog_performance = perf_balanced, . vbat_take_from_dedicated_pin =
				true, . alert_on_battery = thres_low, . low_battery_threshold =
				2.0f, . high_battery_threshold = 3.4f, . alert_on_temperature =
				thres_high, . high_temperture_threshold = 80.0f,
				. low_temperture_threshold = -10.0f, }, .status = { . vbat =
		0.0f, . temperture = 0.0f, . vdda = 0.0f, . mode = analog_stop } };

static analog_handle analog_rt;

static uint16_t analog_dma_buffer[4] = { 0 };

TimerHandle_t analog_timer;
const char *analog_timer_name = "analog_timer";
int analog_timer_id = 0;
TickType_t analog_timer_period = 100 / portTICK_PERIOD_MS;

static float DEBUG_TEMP = 0.0f;
static float DEBUG_VBAT = 0.0f;
static float DEBUG_VREF = 0.0f;

void util_analog_load(void) {
	analog_nvm->load();
}

void util_analog_save(void) {
	analog_nvm->save();
}

void util_analog_timer_callback(ADC_HandleTypeDef *adc) {
	// Implementation for the timer callback
	int vref;
	static float vbat_internal, vbat_dedicate;
	if (analog_dma_buffer[3] == 0) {
		return;
	}

	vref = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(analog_dma_buffer[3],
			ADC_RESOLUTION_12B);
	analog_rt.status.vdda = (float) vref / 1000.0f;

	if (analog_dma_buffer[0] != 0) {
		vbat_dedicate = ((float) analog_dma_buffer[0] / 4095.0f)
				* analog_rt.status.vdda * 2.0f;
	}
	if (analog_dma_buffer[2] != 0) {
		vbat_internal = ((float) analog_dma_buffer[2] / 4095.0f)
				* analog_rt.status.vdda * 3.0f;
	}
	if (analog_rt.config.vbat_take_from_dedicated_pin) {
		analog_rt.status.vbat = vbat_dedicate;
	} else {
		analog_rt.status.vbat = vbat_internal;
	}
	if (analog_dma_buffer[1] != 0) {
		analog_rt.status.temperture = __HAL_ADC_CALC_TEMPERATURE(vref,
				analog_dma_buffer[1], ADC_RESOLUTION_12B);
	}

	DEBUG_TEMP = analog_rt.status.temperture;
	DEBUG_VBAT = analog_rt.status.vbat;
	DEBUG_VREF = analog_rt.status.vdda;

}

/**
 * @brief Initialize the analog utilities.
 */
void __util_analog_init__(void) {

	analog_nvm = new NVM(NVM::partition_analog, &analog_rt, &factory,
			sizeof(analog_rt));

	analog_nvm->load();
	if (analog_nvm->isFactoryDefault()) {
		analog_nvm->restoreDefault();
		analog_nvm->save();
		logInfo("initially restored..");
	}

	HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID,
			util_analog_timer_callback);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analog_dma_buffer, 4);

	analog_rt.status.mode = analog_run;
	logInfo("adc started..");
}

/**
 * @brief Get the current status of the analog utilities.
 *
 * @return util_analog_status_s Current status of the analog utilities.
 */
util_analog_status_s util_analog_get_status(void) {
	// Implementation for getting the current status
	return analog_rt.status;
}

/**
 * @brief Set the configuration for the analog utilities.
 *
 * @param config Configuration structure to set.
 */
void util_analog_set_config(util_analog_config_s config) {
	// Implementation for setting the configuration
	analog_rt.config = config;
}

/**
 * @brief Get the current configuration of the analog utilities.
 *
 * @return util_analog_config_s Current configuration of the analog utilities.
 */
util_analog_config_s util_analog_get_config(void) {
// Implementation for getting the current configuration
	return analog_rt.config;
}
