/*
 * utils.h
 *
 *  Created on: Oct 21, 2024
 *      Author: IRIS
 */

#ifndef UTILTIES_H_
#define UTILTIES_H_

#include "stdint-gcc.h"
#include "rtc.h"
#include "time.h"
#include "string.h"

#include "magic_enum.hpp"
using namespace magic_enum;

/****************shell include****************/
#include "shell_cpp.h"
#include "log.h"
/*********************************************/

//用于辨识存储状态的魔数
#define MAGIC_FLASH_NUMBER 0x5a5a5a5a

typedef enum : uint8_t {
	perf_ultra_efficient, /**< Ultra efficient performance */
	perf_efficient, /**< Efficient performance */
	perf_balanced, /**< Balanced performance */
	perf_performance, /**< Performance update speed */
	perf_ultra_performance /**< Ultra performance update speed */
} util_common_performance_e;

typedef enum : uint8_t {
	thres_high, thres_low, thres_all, thres_none
} util_common_threshold_e;

/*					event queue				*/

/**
 * @brief Enumeration for event codes used in the system.
 */
typedef enum : uint32_t {
	idle, /**< Idle event */
	vibrate, /**< Vibrate event */
	message, /**< Message event */
	network_confirmed, /**< Network confirmed event */
	gnss_update, /**< GNSS update event */
} util_event_code_t;

/**
 * @brief Initialize the event system with the given arguments.
 *
 * @param args Pointer to the initialization arguments.
 */
void __util_events_init__(void);

/**
 * @brief Generate an event with the specified event code.
 *
 * @param code Event code to generate.
 */
void util_events_generate(util_event_code_t code);

/**
 * @brief Poll for events and retrieve the event code.
 *
 * @param code Pointer to store the retrieved event code.
 */
bool util_events_poll(util_event_code_t *code, size_t timeout = 100);

/**
 * @brief Poll for events in a non-blocking manner and retrieve the event code.
 *
 * @param code Pointer to store the retrieved event code.
 */
bool util_events_poll_nonblocked(util_event_code_t *code);

int util_events_flush(void);
/*					low power				*/
typedef enum : uint8_t {
	rtc, pin, regular
} util_lowpower_wake_source_e;

typedef struct {
	bool wake_pin_enable;
	uint32_t requested_wakeup_period;
	uint32_t wakeup_remain;
} util_lowpower_config_s;

void __util_lowpower_init__(void);
void util_lowpower_standby(void);
util_lowpower_wake_source_e util_lowpower_get_wake_source(void);
util_lowpower_config_s util_lowpower_get_config(void);
void util_lowpower_set_config(util_lowpower_config_s config);
void util_lowpower_generate_fault_reboot(uint8_t fault_code);
void util_lowpower_iwdg_feed(void);
time_t util_lowpower_get_rtc(void);
void util_lowpower_update_rtc(time_t time);

/*					misc					*/
int util_misc_string_to_float(const char *str, float *out);
int util_misc_string_to_int(const char **str, int *result);
int util_misc_string_to_rtc(const char *datetimeStr, RTC_TimeTypeDef *rtcTime,
		RTC_DateTypeDef *rtcDate);
void util_misc_ip_to_string(uint8_t ip[4], uint16_t port, char *result);
bool util_misc_find_message(uint8_t *buffer, uint16_t totalLen, uint8_t *dest,
		uint8_t major, uint32_t word);
char* util_misc_itoa(int value, char *buffer, int base);

/*					sc7a20					*/

#define ID_AXIS_X		0
#define ID_AXIS_Y		1
#define ID_AXIS_Z		2

/**
 * @brief Configuration structure for the SC7A20 sensor.
 */
typedef struct {
	bool leanDetectEnabled; /**< Enable or disable lean detection */
	float leanDetectOffset; /**< Calibration angle for lean detection */
	uint8_t range; /**< Range for lean detection */
	uint8_t acc_thres16mg_lsb; /**< Acceleration threshold in 16mg LSB */
	uint32_t cool_down_timeout; /**< Timeout for lean detection */
	uint32_t update_period; /**< Update period for the sensor */
} util_sc7a20_config_s;

/**
 * @brief Status structure for the SC7A20 sensor.
 */
typedef struct {
	float pitch; /**< Pitch angle */
	float roll; /**< Roll angle */
	float lean_angle; /**< Yaw angle */
	int16_t x; /**< X-axis acceleration */
	int16_t y; /**< Y-axis acceleration */
	int16_t z; /**< Z-axis acceleration */
	bool leaned; /**< Lean detection status */
	bool vibration_occured; /**< Vibration status */
	bool vibration_cooling; /**< Cooling status */

	inline void clear(void) {
		pitch = 0;
		roll = 0;
		lean_angle = 0;
		x = 0;
		y = 0;
		z = 0;
		leaned = false;
		vibration_occured = false;
		vibration_cooling = false;
	}
} util_sc7a20_status_s;

/**
 * @brief Initialize the SC7A20 sensor with the given configuration parameters.
 *
 * @param params Pointer to the configuration structure.
 * @return int8_t Status of the initialization (0 for success, negative for error).
 */
void __util_sc7a20_init__(void);

/**
 * @brief Get the current status of the SC7A20 sensor.
 *
 * @return util_sc7a20_status_s Current status of the sensor.
 */
util_sc7a20_status_s util_sc7a20_get_status(void);

/**
 * @brief Set the configuration for the SC7A20 sensor.
 *
 * @param config Configuration structure to set.
 */
void util_sc7a20_set_config(util_sc7a20_config_s config);

/**
 * @brief Get the current configuration of the SC7A20 sensor.
 *
 * @return util_sc7a20_config_s Current configuration of the sensor.
 */
util_sc7a20_config_s util_sc7a20_get_config(void);

void util_sc7a20_clear_vibration_flag(void);

/**
 * @brief Sample the angle of the SC7A20 sensor.
 *
 * @param average_count Number of samples to average.
 * @return float Averaged angle.
 */
float util_sc7a20_sample_angle(uint32_t average_count, uint32_t delay);

float util_sc7a20_get_acc_mod(void);

///*					air780					*/
//
#define ID_LONGITUDE	0
#define ID_LATITUDE		1
///**
// * @brief Configuration structure for the AIR780 module.
// */
//typedef struct {
//	uint8_t main_ip[4]; /**< IP address of the AIR780 module */
//	uint16_t main_port; /**< Port number for communication */
//	uint8_t backup_ip[4]; /**< IP address of the AIR780 module */
//	uint16_t backup_port; /**< Port number for communication */
//	bool connect_to_backup; /**< Connect to backup server */
//	uint32_t position_update_period;/**< Timer period for positioning update */
//} util_air780_config_s;
//
///**
// * @brief Structure to hold the status information of the AIR780 module.
// */
//typedef struct {
//	float latitude; /**< Latitude coordinate */
//	float longitude; /**< Longitude coordinate */
//	int carrier_to_signal_quality; /**< Carrier to signal quality ratio */
//	bool time_updated_since_boot; /**< Time since the last update from boot */
//	bool position_updated_since_boot; /**< Time since the last update from boot */
//	time_t time_up_to_date; /**< Time up to date */
//	uint32_t statistic_data_sent; /**< Amount of data sent in bytes */
//	uint32_t statistic_data_received; /**< Amount of data received in bytes */
//	bool network_ready; /**< Network status */
//	struct {
//		bool baud_updated; /**< Baud rate updated */
//		bool boot_fine; /**< Boot status*/
//		bool net_configured; /**< Network configured status */
//		bool net_connected; /**< Network connected status */
//		bool net_saved; /**< Network saved status */
//		bool csq_updated; /**< Carrier to signal quality updated status */
//		bool time_updated; /**< Time updated status */
//	} net_connect_process;
//
//	inline void clear_net_process(void) {
//		net_connect_process.baud_updated = false;
//		net_connect_process.boot_fine = false;
//		net_connect_process.net_configured = false;
//		net_connect_process.net_connected = false;
//		net_connect_process.net_saved = false;
//		net_connect_process.csq_updated = false;
//		net_connect_process.time_updated = false;
//	}
//
//	inline void clear(void) {
//		latitude = 0;
//		longitude = 0;
//		carrier_to_signal_quality = 0;
//		time_updated_since_boot = false;
//		position_updated_since_boot = false;
//		time_up_to_date = 0;
//		statistic_data_sent = 0;
//		statistic_data_received = 0;
//		network_ready = false;
//		clear_net_process();
//	}
//
//	inline void clear_except_postion(void) {
//		carrier_to_signal_quality = 0;
//		time_updated_since_boot = false;
//		position_updated_since_boot = false;
//		time_up_to_date = 0;
//		statistic_data_sent = 0;
//		statistic_data_received = 0;
//		network_ready = false;
//		clear_net_process();
//	}
//
//} util_air780_status_s;
//
//void __util_air780_init__(void);
//void __util_air780_deinit__(void);

///**
// * @brief Get the current status of the AIR780 module.
// *
// * @return util_air780_status_s Current status of the AIR780 module.
// */
//util_air780_status_s util_air780_get_status(void);
//
///**
// * @brief Get the current configuration of the AIR780 module.
// *
// * @return util_air780_config_s Current configuration of the AIR780 module.
// */
//util_air780_config_s util_air780_get_config(void);
//
///**
// * @brief Set the configuration for the AIR780 module.
// *
// * @param config Configuration structure to set.
// */
//void util_air780_set_config(util_air780_config_s config);
//
///**
// * @brief Write data to the AIR780 network.
// *
// * @param data Pointer to the data to be written.
// * @param size Size of the data to be written in bytes.
// * @return size_t Number of bytes written.
// */
//size_t util_air780_net_write(void *data, size_t size);
//
///**
// * @brief Read data from the AIR780 network.
// *
// * @param data Pointer to the buffer where the read data will be stored.
// * @param size Size of the buffer in bytes.
// * @return size_t Number of bytes read.
// */
//size_t util_air780_net_read(void *data, size_t size);
//
///**
// * @brief Restart the AIR780 module.
// */
//void util_air780_restart(void);
//
//void util_air780_load(void);
//void util_air780_save(void);

/*					analog					*/
/**
 * @brief Enumeration for analog utility modes.
 */
typedef enum {
	analog_run, /**< Analog utility is running */
	analog_suspend, /**< Analog utility is suspended */
	analog_stop, /**< Analog utility is stopped */
	analog_error /**< Analog utility encountered an error */
} util_analog_mode_e;

/**
 * @brief Structure to hold the status information of the analog utilities.
 */
typedef struct {
	float vbat; /**< Battery voltage */
	float temperture; /**< Temperature */
	float vdda; /**< Internal reference voltage */
	util_analog_mode_e mode; /**< Current mode of the analog utility */

	inline void clear(void) {
		vbat = 0;
		temperture = 0;
		vdda = 0;
		mode = analog_stop;
	}

} util_analog_status_s;

/**
 * @brief Configuration structure for the analog utilities.
 */
typedef struct {
	util_common_performance_e analog_performance; /**< Performance level */
	bool vbat_take_from_dedicated_pin; /**< Take battery voltage from dedicated pin */
	util_common_threshold_e alert_on_battery; /**< Alert on low battery */
	float low_battery_threshold; /**< Low battery threshold */
	float high_battery_threshold; /**< High battery threshold */
	util_common_threshold_e alert_on_temperature; /**< Alert on high temperature */
	float high_temperture_threshold; /**< Temperature threshold */
	float low_temperture_threshold; /**< Temperature threshold */
} util_analog_config_s;

/**
 * @brief Initialize the analog utilities.
 */
void __util_analog_init__(void);

/**
 * @brief Get the current status of the analog utilities.
 *
 * @return util_analog_status_s Current status of the analog utilities.
 */
util_analog_status_s util_analog_get_status(void);

/**
 * @brief Set the configuration for the analog utilities.
 *
 * @param config Configuration structure to set.
 */
void util_analog_set_config(util_analog_config_s config);

/**
 * @brief Get the current configuration of the analog utilities.
 *
 * @return util_analog_config_s Current configuration of the analog utilities.
 */
util_analog_config_s util_analog_get_config(void);

void __util_shell_init__(void);

typedef struct {
	float longitude;
	float latitude;
	time_t time;
	int position_fixed;
} util_atgm332d_status_t;

void __util_atgm332d_init__(void);
void util_atgm332d_load(void);
void util_atgm332d_activate(uint32_t hysteresis_sec);
void util_atgm332d_deactivate(void);
util_atgm332d_status_t util_atgm332d_get_status(void);

#endif /* UTILTIES_H_ */

