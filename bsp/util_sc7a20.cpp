/*
 * peri_sc7a20.cpp
 *
 *  Created on: Oct 21, 2024
 *      Author: IRIS
 */

#include "main.h"
#include "i2c.h"

#include "stdint-gcc.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "typeinfo"
#include "math.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "semphr.h"
#include "timers.h"

#include "utilties.h"
#include "PRODUCT_CONFIG.h"
#include "log.h"
#include "NVM.h"
NVM *sc7a20_nvm = NULL;

typedef struct {
	util_sc7a20_config_s config;
	util_sc7a20_status_s status;
	const uint32_t magic = MAGIC_FLASH_NUMBER;
} sc7a20_handle;

/* cool_down_timeout / update_period 单位均为毫秒 */
static volatile sc7a20_handle factory = { .config = { .leanDetectEnabled = true,
		.leanDetectOffset = 0, .range = 15, .acc_thres16mg_lsb = 1,
		.cool_down_timeout =
				(uint32_t) PROD_CONFIG_FACTORY_VIBRATION_DEBOUNCE_SEC
						* 1000u,
		.update_period = 200 }, .status = { .pitch = 0, .roll = 0,
		.lean_angle = 0, .x = 0, .y = 0, .z = 0, .leaned = false,
		.vibration_occured = false, .vibration_cooling = false, } };

#define SC7A20_CONFIG_IIC	hi2c1
#define SC7A20_CONFIG_ADDR	0x33
#define SC7A20_CONFIG_TIMER	htim21

#define SC7A20_REG_OUT_TEMP_L         (uint8_t)0x0C
#define SC7A20_REG_OUT_TEMP_H         (uint8_t)0x0D
#define SC7A20_REG_WHO_AM_I           (uint8_t)0x0F
#define SC7A20_REG_USER_CAL           (uint8_t)0x13
#define SC7A20_REG_USER_CAL_LEN       (uint8_t)0x08
#define SC7A20_REG_NVM_WR             (uint8_t)0x1E
#define SC7A20_REG_TEMP_CFG           (uint8_t)0x1F
#define SC7A20_REG_CTRL_REG1          (uint8_t)0x20
#define SC7A20_REG_CTRL_REG2          (uint8_t)0x21
#define SC7A20_REG_CTRL_REG3          (uint8_t)0x22
#define SC7A20_REG_CTRL_REG4          (uint8_t)0x23
#define SC7A20_REG_CTRL_REG5          (uint8_t)0x24
#define SC7A20_REG_CTRL_REG6          (uint8_t)0x25
#define SC7A20_REG_REFERENCE          (uint8_t)0x26
#define SC7A20_REG_STATUS_REG 	      (uint8_t)0x27
#define SC7A20_REG_OUT_X_L            (uint8_t)0x28
#define SC7A20_REG_OUT_X_H            (uint8_t)0x29
#define SC7A20_REG_OUT_Y_L            (uint8_t)0x2A
#define SC7A20_REG_OUT_Y_H            (uint8_t)0x2B
#define SC7A20_REG_OUT_Z_L            (uint8_t)0x2C
#define SC7A20_REG_OUT_Z_H            (uint8_t)0x2D
#define SC7A20_REG_FIFO_CTRL_REG      (uint8_t)0x2E
#define SC7A20_REG_FIFO_SRC_REG       (uint8_t)0x2F
#define SC7A20_REG_INT1_CFG           (uint8_t)0x30
#define SC7A20_REG_INT1_SOURCE        (uint8_t)0x31
#define SC7A20_REG_INT1_THS           (uint8_t)0x32
#define SC7A20_REG_INT1_DURATION      (uint8_t)0x33
#define SC7A20_REG_INT2_CFG           (uint8_t)0x34
#define SC7A20_REG_INT2_SOURCE        (uint8_t)0x35
#define SC7A20_REG_INT2_THS           (uint8_t)0x36
#define SC7A20_REG_INT2_DURATION      (uint8_t)0x37
#define SC7A20_REG_CLICK_CFG          (uint8_t)0x38
#define SC7A20_REG_CLICK_SRC          (uint8_t)0x39
#define SC7A20_REG_CLICK_THS          (uint8_t)0x3A
#define SC7A20_REG_TIME_LIMIT         (uint8_t)0x3B
#define SC7A20_REG_TIME_LATENCY       (uint8_t)0x3C
#define SC7A20_REG_TIME_WINDOW        (uint8_t)0x3D
#define SC7A20_REG_ACT_THS            (uint8_t)0x3E
#define SC7A20_REG_ACT_DURATION       (uint8_t)0x3F

#define CTRL_REG1_ODR_pos	0x04
#define CTRL_REG1_LPen_pos	0x03
#define CTRL_REG1_Zen_pos	0x02
#define CTRL_REG1_Yen_pos	0x01
#define CTRL_REG1_Xen_pos	0x00

#define CTRL_REG4_BDU_pos	0x07
#define CTRL_REG4_BLE_pos	0x06
#define CTRL_REG4_FS_pos	0x04
#define CTRL_REG4_HR_pos	0x03
#define CTRL_REG4_ST_pos	0x01
#define CTRL_REG4_SIM_pos	0x00

static sc7a20_handle rt_sc7a20;

#define SL_SC7A20H_FIFO_CTRL_REG   (uint8_t)0x2E
#define SL_SC7A20H_FIFO_SRC_REG    (uint8_t)0x2F
#define SL_SC7A20H_SPI_OUT_X_L     (uint8_t)0x27
#define SL_SC7A20H_IIC_OUT_X_L     (uint8_t)0xA8

TimerHandle_t sc7a20_update_timer;
int sc7a20_update_timer_id = 0;
const char *sc7a20_update_timer_name = "sc7a20_update_timer";

TimerHandle_t sc7a20_cool_down_timer;
int sc7a20_cool_down_timer_id = 0;
const char *sc7a20_cool_down_timer_name = "sc7a20_cool_down_timer";

extern "C" {
EXTI_HandleTypeDef sc7a20_int1;
}

TaskHandle_t sc7a20_daemon_thread_handle = NULL;

static bool util_sc7a20_angle_is_in_range(int currentAngle, int targetAngle,
		int range) {
	// 计算目标角度的范围
	int minAngle = targetAngle - range;
	int maxAngle = targetAngle + range;

	// 处理角度超出-180到180度范围的情况
	if (minAngle < -180) {
		minAngle += 360;
	}
	if (maxAngle > 180) {
		maxAngle -= 360;
	}

	// 判断当前角度是否在范围内
	if (minAngle <= maxAngle) {
		return currentAngle >= minAngle && currentAngle <= maxAngle;
	} else {
		// 处理跨越-180到180度的情况
		return currentAngle >= minAngle || currentAngle <= maxAngle;
	}
}

static void util_sc7a20_disable_interrupts(void) {
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
}

static void util_sc7a20_enable_interrupts(void) {
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void util_sc7a20_reset_chip(void) {
	HAL_GPIO_WritePin(SENSOR_NPWR_GPIO_Port, SENSOR_NPWR_Pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(100));
	HAL_GPIO_WritePin(SENSOR_NPWR_GPIO_Port, SENSOR_NPWR_Pin, GPIO_PIN_RESET);
}

static void util_sc7a20_iic_read(uint8_t addr, size_t addrLen, uint8_t *dest,
		size_t len) {
	HAL_I2C_Mem_Read(&hi2c1, SC7A20_CONFIG_ADDR, addr, I2C_MEMADD_SIZE_8BIT,
			dest, len, HAL_MAX_DELAY);
}

static void util_sc7a20_iic_write(uint8_t addr, size_t addrLen, uint8_t *data,
		size_t len) {
	HAL_I2C_Mem_Write(&hi2c1, SC7A20_CONFIG_ADDR, addr, I2C_MEMADD_SIZE_8BIT,
			data, len, HAL_MAX_DELAY);
}

static int16_t util_sc7a20_12bit_complement(uint8_t msb, uint8_t lsb) {
	int16_t temp;
	temp = msb << 8 | lsb;
	temp = temp >> 4;
	temp = temp & 0x0fff;
	if (temp & 0x0800) {
		temp = temp & 0x07ff;
		temp = ~temp;
		temp = temp + 1;
		temp = temp & 0x07ff;
		temp = -temp;
	}
	return temp;
}

static void util_sc7a20_read_acc(void) {

	uint8_t buffer[6] = { 0 };

	util_sc7a20_iic_read(SC7A20_REG_OUT_X_L | 0x80, 1, buffer, 6);

// Convert raw data to 16-bit signed integers
	rt_sc7a20.status.x = util_sc7a20_12bit_complement(buffer[1], buffer[0])
			* (-1); // X-axis
	rt_sc7a20.status.y = util_sc7a20_12bit_complement(buffer[3], buffer[2])
			* (-1); // Y-axis
	rt_sc7a20.status.z = util_sc7a20_12bit_complement(buffer[5], buffer[4])
			* (-1); // Z-axis
}

float util_calculate_angle_relative_to_z(int16_t ax, int16_t ay, int16_t az) {
	// Convert raw data to float
	float fax = ax / 1000.0f;
	float fay = ay / 1000.0f;
	float faz = az / 1000.0f;

	// Calculate the magnitude of the acceleration vector
	float magnitude = sqrtf(fax * fax + fay * fay + faz * faz);

	// Calculate the angle relative to the Z-axis
	float angle = acosf(faz / magnitude) * (180.0f / 3.1415926f);

	return angle;
}

volatile float acc_mod = 0;

static void util_sc7a20_update_timer_callback(TimerHandle_t xTimer) {
#define latest 4
	static int16_t bufferd_acc_x[5] = { 0 };
	static int16_t bufferd_acc_y[5] = { 0 };
	static int16_t bufferd_acc_z[5] = { 0 };
	static float weight[5] = { 0.4, 0.2, 0.2, 0.1, 0.1 };

	float weighted_acc_x = 0, weighted_acc_y = 0, weighted_acc_z = 0;
	util_sc7a20_read_acc();

	acc_mod = util_sc7a20_get_acc_mod();

	memmove(bufferd_acc_x, bufferd_acc_x + 1, 4 * sizeof(int16_t));
	memmove(bufferd_acc_y, bufferd_acc_y + 1, 4 * sizeof(int16_t));
	memmove(bufferd_acc_z, bufferd_acc_z + 1, 4 * sizeof(int16_t));

	bufferd_acc_x[latest] = rt_sc7a20.status.x;
	bufferd_acc_y[latest] = rt_sc7a20.status.y;
	bufferd_acc_z[latest] = rt_sc7a20.status.z;

	for (int i = 0; i < 5; i++) {
		weighted_acc_x += bufferd_acc_x[i] * weight[i];
		weighted_acc_y += bufferd_acc_y[i] * weight[i];
		weighted_acc_z += bufferd_acc_z[i] * weight[i];
	}

	rt_sc7a20.status.lean_angle = util_calculate_angle_relative_to_z(
			weighted_acc_x, weighted_acc_y, weighted_acc_z);

	if (rt_sc7a20.config.leanDetectEnabled
			&& (!util_sc7a20_angle_is_in_range(rt_sc7a20.status.lean_angle,
					rt_sc7a20.config.leanDetectOffset, rt_sc7a20.config.range))) {
		rt_sc7a20.status.leaned = true;
	} else {
		rt_sc7a20.status.leaned = false;
	}
}

#define SC7A20_NOTIFY_INT1	(0x00000001<<0)
#define SC7A20_NOTIFY_INT2	(0x00000001<<1)
#define SC7A20_NOTIFY_COOLDOWN	(0x00000001<<2)

static void util_sc7a20_cool_down_timer_callback(TimerHandle_t xTimer) {
	(void) xTimer;
	util_sc7a20_enable_interrupts();
	/* 定时器回调在任务上下文，勿用 FromISR */
	xTaskNotify(sc7a20_daemon_thread_handle, SC7A20_NOTIFY_COOLDOWN, eSetBits);
}

static void util_sc7a20_int1_isr_callback(void) {

	if (sc7a20_daemon_thread_handle == NULL
			|| rt_sc7a20.status.vibration_cooling)
		return;
	rt_sc7a20.status.vibration_cooling = true;
	util_sc7a20_disable_interrupts();
	xTaskNotifyFromISR(sc7a20_daemon_thread_handle, SC7A20_NOTIFY_INT1,
			eSetBits, NULL);
}

static void util_sc7a20_daemon_thread(void *args) {

	uint32_t notify_value = 0;

	xTimerStart(sc7a20_update_timer, portMAX_DELAY);

	loop:

	xTaskNotifyWait(0, 0xffffffff, &notify_value, portMAX_DELAY);

	if (notify_value & SC7A20_NOTIFY_INT1) {
		rt_sc7a20.status.vibration_occured = true;
		xTimerStart(sc7a20_cool_down_timer, portMAX_DELAY);
		logInfo("加计: INT1震动, 投递事件");
		util_events_generate(util_event_code_t::vibrate);
	}
	if (notify_value & SC7A20_NOTIFY_COOLDOWN) {
		rt_sc7a20.status.vibration_cooling = false;
	}
	goto loop;
}

/**
 * @brief Initialize the SC7A20 sensor.
 *
 * This function initializes the SC7A20 sensor with the specified parameters.
 * It configures the sensor registers, sets up interrupts, and initializes
 * timers for periodic updates and vibration cooldown.
 *
 * @param leanDetectEnabled Enable or disable lean detection.
 * @param calibAngle Calibration angle for lean detection.
 * @param range Range for lean detection.
 * @param accThres16mgLsb Acceleration threshold in 16mg LSB.
 * @return The device ID of the SC7A20 sensor.
 */

void __util_sc7a20_init__(void) {

	sc7a20_nvm = new NVM(NVM::partition_sc7a20, &rt_sc7a20, (void*) &factory,
			sizeof(rt_sc7a20));

	sc7a20_nvm->load();
	if (sc7a20_nvm->isFactoryDefault()) {
		sc7a20_nvm->restoreDefault();
		sc7a20_nvm->save();
		logInfo("NVM: 已恢复出厂默认");
	}
//	初始化后，清空现场状态
	rt_sc7a20.status.clear();

	uint8_t id = 0;
	uint8_t SC7A20_REG_CTRL_REG1_data = 0x47;
	uint8_t SC7A20_REG_CTRL_REG2_data = 0x03;
	uint8_t SC7A20_REG_CTRL_REG3_data = 0x40;
	uint8_t SC7A20_REG_CTRL_REG4_data = 0x08;
	uint8_t SC7A20_REG_CTRL_REG6_data = 0x40;
	uint8_t SC7A20_REG_INT1_CFG_data = 0x2a;
	uint8_t SC7A20_REG_INT1_THS_data =
			(rt_sc7a20.config.acc_thres16mg_lsb > 127) ?
					127 : rt_sc7a20.config.acc_thres16mg_lsb; // Set threshold

	EXTI_ConfigTypeDef exti;

	exti.GPIOSel = EXTI_GPIOA;
	exti.Line = EXTI_LINE_0;
	exti.Mode = EXTI_MODE_INTERRUPT;
	exti.Trigger = EXTI_TRIGGER_RISING;
	HAL_EXTI_SetConfigLine(&sc7a20_int1, &exti);

	HAL_EXTI_RegisterCallback(&sc7a20_int1,
			EXTI_CallbackIDTypeDef::HAL_EXTI_COMMON_CB_ID,
			util_sc7a20_int1_isr_callback);

	util_sc7a20_iic_read(SC7A20_REG_WHO_AM_I, 1, &id, 1);

	util_sc7a20_iic_write(SC7A20_REG_CTRL_REG1, 1, &SC7A20_REG_CTRL_REG1_data,
			1);
	util_sc7a20_iic_write(SC7A20_REG_CTRL_REG2, 1, &SC7A20_REG_CTRL_REG2_data,
			1);
	util_sc7a20_iic_write(SC7A20_REG_CTRL_REG3, 1, &SC7A20_REG_CTRL_REG3_data,
			1);
	util_sc7a20_iic_write(SC7A20_REG_CTRL_REG4, 1, &SC7A20_REG_CTRL_REG4_data,
			1);
	util_sc7a20_iic_write(SC7A20_REG_CTRL_REG6, 1, &SC7A20_REG_CTRL_REG6_data,
			1);
	util_sc7a20_iic_write(SC7A20_REG_INT1_CFG, 1, &SC7A20_REG_INT1_CFG_data, 1);
	util_sc7a20_iic_write(SC7A20_REG_INT1_THS, 1, &SC7A20_REG_INT1_THS_data, 1);
	util_sc7a20_iic_write(SC7A20_REG_INT2_CFG, 1, &SC7A20_REG_INT1_CFG_data, 1);
	util_sc7a20_iic_write(SC7A20_REG_INT2_THS, 1, &SC7A20_REG_INT1_THS_data, 1);

	// Initialize vibration cooldown timer and acceleration update timer
	sc7a20_cool_down_timer = xTimerCreate(sc7a20_cool_down_timer_name,
			pdMS_TO_TICKS(rt_sc7a20.config.cool_down_timeout), false,
			&sc7a20_cool_down_timer_id, util_sc7a20_cool_down_timer_callback);
	sc7a20_update_timer = xTimerCreate(sc7a20_update_timer_name,
			pdMS_TO_TICKS(rt_sc7a20.config.update_period), pdTRUE,
			&sc7a20_update_timer_id, util_sc7a20_update_timer_callback);

	xTimerStart(sc7a20_update_timer, portMAX_DELAY);

	xTaskCreate(util_sc7a20_daemon_thread, "sc7a20_daemon", 256,
	NULL, osPriorityHigh, &sc7a20_daemon_thread_handle);
	configASSERT(sc7a20_daemon_thread_handle != NULL);

	configASSERT(sc7a20_cool_down_timer != NULL);
	configASSERT(sc7a20_update_timer != NULL);

	vTaskDelay(pdMS_TO_TICKS(100));
	// Delay to allow angle to stabilize
	logInfo("加计: 已启动");
}

util_sc7a20_status_s util_sc7a20_get_status(void) {
	return rt_sc7a20.status;
}

util_sc7a20_config_s util_sc7a20_get_config(void) {
	return rt_sc7a20.config;
}

void util_sc7a20_clear_vibration_flag(void) {
	rt_sc7a20.status.vibration_occured = false;
}

void util_sc7a20_mark_vibration(void) {
	rt_sc7a20.status.vibration_occured = true;
	logInfo("加计: 标记震动");
}

void util_sc7a20_set_config(util_sc7a20_config_s config) {

//	COPY_STRUCTURE(rt_sc7a20.config, config);
	memcpy(&rt_sc7a20.config, &config, sizeof(util_sc7a20_config_s));

	/* 周期字段单位为毫秒（与 init / PRODUCT_CONFIG 消抖一致） */
	xTimerChangePeriod(sc7a20_update_timer,
			pdMS_TO_TICKS(rt_sc7a20.config.update_period),
			portMAX_DELAY);
	xTimerChangePeriod(sc7a20_cool_down_timer,
			pdMS_TO_TICKS(rt_sc7a20.config.cool_down_timeout),
			portMAX_DELAY);
	xTimerReset(sc7a20_update_timer, portMAX_DELAY);
	xTimerReset(sc7a20_cool_down_timer, portMAX_DELAY);
//	修改震动阈值
	util_sc7a20_iic_write(SC7A20_REG_INT1_THS, 1,
			&rt_sc7a20.config.acc_thres16mg_lsb, 1);
	util_sc7a20_iic_write(SC7A20_REG_INT2_THS, 1,
			&rt_sc7a20.config.acc_thres16mg_lsb, 1);

//	设置参数后保存
	sc7a20_nvm->save();
}

float util_sc7a20_sample_angle(uint32_t average_count, uint32_t delay) {
	float sum = 0;
	float sampled_angle;

	configASSERT(delay > 0 && average_count > 0);

//	log_i("sampling angle...");

	for (uint32_t i = 0; i < average_count; i++) {
		vTaskDelay(pdMS_TO_TICKS(delay));
		sum += rt_sc7a20.status.lean_angle;
	}
	sampled_angle = sum / average_count;
	return sampled_angle;
}

float util_sc7a20_get_acc_mod(void) {
	return sqrtf(
			rt_sc7a20.status.x * rt_sc7a20.status.x
					+ rt_sc7a20.status.y * rt_sc7a20.status.y
					+ rt_sc7a20.status.z * rt_sc7a20.status.z);
}

