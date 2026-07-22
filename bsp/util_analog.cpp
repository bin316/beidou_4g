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

#include "arm_math.h"

// Filter configuration macros
#define ANALOG_FILTER_SIZE            256      // Size of the filter buffer (must be power of 2 for efficiency)
#define ANALOG_FILTER_INIT_THRESHOLD  (ANALOG_FILTER_SIZE/4)  // Number of samples required before initialization

NVM *analog_nvm = NULL;

typedef struct
    {
    util_analog_config_s config;
    util_analog_status_s status;
    } analog_handle;

analog_handle factory =
    {
    .config =
	{
		. analog_performance = perf_balanced,
		. vbat_take_from_dedicated_pin = true,
		. alert_on_battery = thres_low,
		. low_battery_threshold = 2.0f,
		. high_battery_threshold = 3.4f,
		. alert_on_temperature = thres_high,
		. high_temperture_threshold = 80.0f,
		. low_temperture_threshold = -10.0f,
	}, .status =
	{
	. vbat = 0.0f, . temperture = 0.0f, . vdda = 0.0f, . mode = analog_stop
	}
    };

static analog_handle analog_rt;

static uint16_t analog_dma_buffer[4] =
    {
    0
    };
static q15_t adc_filt_buffer[4][ANALOG_FILTER_SIZE] =
    {
    0
    };
static q15_t mean_values[4] =
    {
    0
    };

static q15_t adc_max_buffer[4][ANALOG_FILTER_SIZE] =
    {
    0
    };
static q15_t adc_max_means[4] =
    {
    0
    };

TimerHandle_t analog_timer;
const char *analog_timer_name = "analog_timer";
int analog_timer_id = 0;
TickType_t analog_timer_period = 100 / portTICK_PERIOD_MS;

static float DEBUG_TEMP = 0.0f;
static float DEBUG_VBAT = 0.0f;
static float DEBUG_VREF = 0.0f;

void util_analog_load(void)
    {
    analog_nvm->load();
    }

void util_analog_save(void)
    {
    analog_nvm->save();
    }

static bool process_adc_buffer(int channel)
    {
    static bool buffer_initialized[4] =
	{
	false, false, false, false
	};

    // Shift buffer elements correctly using Q15 function
    arm_copy_q15(&adc_filt_buffer[channel][1], &adc_filt_buffer[channel][0],
    ANALOG_FILTER_SIZE - 1);

    // Store the raw ADC value at the beginning of the buffer as Q15 format
    adc_filt_buffer[channel][ANALOG_FILTER_SIZE - 1] =
	    (q15_t) analog_dma_buffer[channel];

    // Check if the buffer is sufficiently filled
    if (!buffer_initialized[channel])
	{
	int filled_count = 0;
	for (int j = 0; j < ANALOG_FILTER_SIZE; j++)
	    {
	    if (adc_filt_buffer[channel][j] != 0)
		{
		filled_count++;
		}
	    }

	if (filled_count >= ANALOG_FILTER_INIT_THRESHOLD) // First quarter filled
	    {
	    buffer_initialized[channel] = true;
	    //buffer is already initialized, fill the buffer with current mean
	    q15_t filled_mean = 0;

	    // Calculate mean from the LAST quarter of the buffer (newest values)
	    // where the actual filled data is (ANALOG_FILTER_SIZE - ANALOG_FILTER_INIT_THRESHOLD to ANALOG_FILTER_SIZE)
	    arm_mean_q15(
		    &adc_filt_buffer[channel][ANALOG_FILTER_SIZE
			    - ANALOG_FILTER_INIT_THRESHOLD],
		    ANALOG_FILTER_INIT_THRESHOLD, &filled_mean);

	    // Fill the all of the buffer with the mean value
	    arm_fill_q15(filled_mean, &adc_filt_buffer[channel][0],
	    ANALOG_FILTER_SIZE);
	    //fill the max buffer with the mean value
	    arm_fill_q15(filled_mean, &adc_max_buffer[channel][0],
	    ANALOG_FILTER_SIZE);

	    // Initialize means with the calculated value
	    mean_values[channel] = filled_mean;
	    adc_max_means[channel] = filled_mean;
	    }

	// Don't calculate mean yet until buffer is initialized
	mean_values[channel] = (q15_t) analog_dma_buffer[channel];
	return false;
	}

    // Calculate mean using Q15 function

    arm_mean_q15(&adc_filt_buffer[channel][0], ANALOG_FILTER_SIZE,
	    &mean_values[channel]);

    // Calculate max using Q15 function and push it into the max buffer

//    buffer shifting
    arm_copy_q15(&adc_max_buffer[channel][1], &adc_max_buffer[channel][0],
    ANALOG_FILTER_SIZE - 1);
    uint32_t max_index = 0; //max index is not used in this case, but required by the function
    arm_max_q15(&adc_filt_buffer[channel][0], ANALOG_FILTER_SIZE,
	    &adc_max_buffer[channel][ANALOG_FILTER_SIZE - 1], &max_index);

    arm_mean_q15(&adc_max_buffer[channel][0], ANALOG_FILTER_SIZE,
	    &adc_max_means[channel]);

    return true;
    }

void util_analog_timer_callback(ADC_HandleTypeDef *adc)
    {
    int vref;
    static float vbat_internal, vbat_dedicate;
    bool inited = false;

    // Process all channels
    for (int i = 0; i < 4; i++)
	{
	inited = process_adc_buffer(i);
	}

    // Basic validation
    if (mean_values[3] == 0)
	return;

    if (adc_max_means[3] == 0)
	return;

    if (!inited)
	return;

//     Convert Q15 mean values back to uint16_t for hardware calculations
    uint16_t adc_vref = (uint16_t) adc_max_means[3];
    uint16_t adc_vbat_dedicate = (uint16_t) adc_max_means[0];
    uint16_t adc_vbat_internal = (uint16_t) adc_max_means[2];
    uint16_t adc_temp = (uint16_t) adc_max_means[1];
//    uint16_t adc_vref = (uint16_t) mean_values[3];
//    uint16_t adc_vbat_dedicate = (uint16_t) mean_values[0];
//    uint16_t adc_vbat_internal = (uint16_t) mean_values[2];
//    uint16_t adc_temp = (uint16_t) mean_values[1];

    // Calculate reference voltage
    vref = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_vref, ADC_RESOLUTION_12B);
    analog_rt.status.vdda = (float) vref / 1000.0f;

    // Calculate battery voltage (dedicated pin)
    if (adc_vbat_dedicate != 0)
	{
	vbat_dedicate = ((float) adc_vbat_dedicate / 4095.0f)
		* analog_rt.status.vdda * 2.0f;
	if (vbat_dedicate < 0.0f || vbat_dedicate > 5.0f)
	    {
	    logError("电池电压(专用通道): 读数超范围");
	    vbat_dedicate = 0.0f;
	    }
	}

    // Calculate battery voltage (internal)
    if (adc_vbat_internal != 0)
	{
	vbat_internal = ((float) adc_vbat_internal / 4095.0f)
		* analog_rt.status.vdda * 3.0f;
	if (vbat_internal < 0.0f || vbat_internal > 5.0f)
	    {
	    logError("电池电压(内部通道): 读数超范围");
	    vbat_internal = 0.0f;
	    }
	}

    // Select the battery voltage source
    if (analog_rt.config.vbat_take_from_dedicated_pin)
	{
	analog_rt.status.vbat = vbat_dedicate;
	}
    else
	{
	analog_rt.status.vbat = vbat_internal;
	}

    // Calculate temperature
    if (adc_temp != 0)
	{
	analog_rt.status.temperture = __HAL_ADC_CALC_TEMPERATURE(vref, adc_temp,
		ADC_RESOLUTION_12B);
	}

    // Apply fixed offset as in the original code
    analog_rt.status.vbat += 0.3;
    analog_rt.status.temperture += 5.0f;

    // Update debug values
    DEBUG_TEMP = analog_rt.status.temperture;
    DEBUG_VBAT = analog_rt.status.vbat;
    DEBUG_VREF = analog_rt.status.vdda;
    }

/**
 * @brief Initialize the analog utilities.
 */
void __util_analog_init__(void)
    {

    analog_nvm = new NVM(NVM::partition_analog, &analog_rt, &factory,
	    sizeof(analog_rt));

    analog_nvm->load();
    if (analog_nvm->isFactoryDefault())
	{
	analog_nvm->restoreDefault();
	analog_nvm->save();
	logInfo("NVM: 已恢复出厂默认");
	}

    HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID,
	    util_analog_timer_callback);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analog_dma_buffer, 4);

    analog_rt.status.mode = analog_run;
    logInfo("ADC: 已启动");
    }

/**
 * @brief Get the current status of the analog utilities.
 *
 * @return util_analog_status_s Current status of the analog utilities.
 */
util_analog_status_s util_analog_get_status(void)
    {
    // Implementation for getting the current status
    return analog_rt.status;
    }

/**
 * @brief Set the configuration for the analog utilities.
 *
 * @param config Configuration structure to set.
 */
void util_analog_set_config(util_analog_config_s config)
    {
    // Implementation for setting the configuration
    analog_rt.config = config;
    }

/**
 * @brief Get the current configuration of the analog utilities.
 *
 * @return util_analog_config_s Current configuration of the analog utilities.
 */
util_analog_config_s util_analog_get_config(void)
    {
// Implementation for getting the current configuration
    return analog_rt.config;
    }

void util_analog_suspend(void)
    {
    //calculation and filter process are linked to the ADC DMA callback,
    //so we can stop the ADC to suspend the analog utilities
    HAL_ADC_Stop_DMA(&hadc1);
    analog_rt.status.mode = analog_stop;
    logInfo("ADC: 已停止");
    }

void util_analog_resume(void)
    {
    // Resume the ADC to continue processing
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analog_dma_buffer, 4);
    analog_rt.status.mode = analog_run;
    logInfo("ADC: 已恢复");
    }
