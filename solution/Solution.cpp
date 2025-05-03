/*
 * Solution.cpp
 *
 *  Created on: Jan 22, 2025
 *      Author: IRIS
 */

#include <Solution.h>
#include "utilties.h"
#include "crc.h"

#include "helper_and_reload.hpp"

#include "magic_enum.hpp"
using namespace magic_enum;

#include "PRODUCT_CONFIG.h"

TaskHandle_t solution_thread_handle = NULL;

solution_handle fc_solution = { .runningConfig = { .t0_netGood_wakeup_min =
PROD_CONFIG_FACTORY_NET_GOOD_WAKEUP_TIMEOUT_MIN, //5(单位[分钟])
		.t1_netBad_wakeup_min = PROD_CONFIG_FACTORY_NET_BAD_WAKEUP_TIMEOUT_MIN, //3(单位[分钟])
		.t2_serverRsps_timeout_sec = PROD_CONFIG_FACTORY_SERVER_RSP_TIMEOUT_SEC, //3(单位[秒])
		.t3_gnssSearch_sleep_sec = PROD_CONFIG_FACTORY_GNSS_SEARCH_TIMEOUT, //180(单位[秒])
		.t4_gnssGood_sleep_sec =
		PROD_CONFIG_FACTORY_GNSS_SEARCH_COMPLETE_TIMEOUT, //60(单位[秒])
		.t6_rtcEvent_sleep_sec = 30, //60(单位[秒])$notice 这个参数不再使用了
		.t5_motionDetect_delay_sec =
		PROD_CONFIG_FACTORY_VIBRATION_DEBOUNCE_SEC, //5(单位[秒])
		.n0_gnssUpdate_wakeup_count = 60, //60(单位[次])/*$notice parameter has no effect*/
		.n1_vbatAlarm_threshold_volt =
		PROD_CONFIG_FACTORY_VBAT_LOW
//22(单位[0.1V])
		}, .systemConfig = { .code = { .major =
PROD_CONFIG_FACTORY_DEFAULT_CODE_MAJOR, .minor =
PROD_CONFIG_FACTORY_DEFAULT_CODE_MINOR, .index =
PROD_CONFIG_FACTORY_DEFAULT_CODE_INDEX, }, . sensorReverseRange = 15,
		. sensorVibrationThreshold = 1, . backupServerIP =
		PROD_CONFIG_FACTORY_DEFAULT_AUX_IP, . backupServerPort =
		PROD_CONFIG_FACTORY_DEFAULT_AUX_PORT, . runServerIP =
		PROD_CONFIG_FACTORY_DEFAULT_MAIN_IP, . runServerPort =
		PROD_CONFIG_FACTORY_DEFAULT_MAIN_PORT }, .pwd = { .item =
PROD_CONFIG_FACTORY_DEFAULT_PASSWORD }, .mode =
PROD_CONFIG_FACTORY_DEFAULT_WORK_MODE, .configEditting = 0,
		.passwordConfirm = 0, .newPassword = { 0 }, .wakeSource =
				util_lowpower_wake_source_e::regular, .connect_to_main_server =
				false, .updatePositionOnStart = false };

Solution::Solution() {
	/* create NVM: parameters save and load
	 *
	 * 加载参数，如果加载失败，使用默认参数
	 * */

	logInfo("solution init..");

	nvm = new NVM(NVM::partition_solution, (uint8_t*) &this->rt_solution,
			&fc_solution, sizeof(this->rt_solution));
	configASSERT(nvm != NULL);

	logInfo("solution nvm created..");
	nvm->load();
	if (nvm->isFactoryDefault()) {
		logInfo("solution nvm factory default..");
		nvm->restoreDefault();
		if (PROD_CONFIG_FACTORY_GENERATE_UNIQ_INDEX == 1) {
			/*根据芯片的Unique ID生成唯一的index */
			/*unique index使用96位UID的每两个字节累加得出16位index */
			/*无视溢出 */
			/*获取uid */
			uint32_t UID[3] = { { 0 } };
			UID[0] = HAL_GetUIDw0();
			UID[1] = HAL_GetUIDw1();
			UID[2] = HAL_GetUIDw2();
			this->rt_solution.systemConfig.code.index = 0;
			for (int i = 0; i < 3; i++) {
				this->rt_solution.systemConfig.code.index += UID[i] & 0xFFFF;
				this->rt_solution.systemConfig.code.index += (UID[i] >> 16)
						& 0xFFFF;
			}
		}
		nvm->save();
	}

	/*
	 * 解决方案任务启动
	 * 模式启动不同任务
	 */

	switch (rt_solution.mode) {
	case solution_mode_e::wm_work:
		xTaskCreate(Solution::solution_work_routine_thread, "solution_work",
				512, this, osPriorityNormal, &solution_thread_handle);
		break;
	case solution_mode_e::wm_idle:
		xTaskCreate(Solution::solution_idle_routine_thread, "solution_idle",
				512, this, osPriorityNormal, &solution_thread_handle);
		break;
	default:
		xTaskCreate(Solution::solution_fact_routine_thread, "solution_fact",
				512, this, osPriorityNormal, &solution_thread_handle);
	}
	configASSERT(solution_thread_handle != NULL);

	logInfo("routine started: %s..", enum_name(rt_solution.mode).data());

}

void Solution::report(void) {

	pb_packReport rsps;
	util_sc7a20_status_s sensor = util_sc7a20_get_status();
	util_analog_status_s analog = util_analog_get_status();

	util_sc7a20_clear_vibration_flag();

	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.dataLen = sizeof(pb_report);
	rsps.body.header.function = e_pb_func::up_report;
	rsps.body.report.acc[ID_AXIS_X] = sensor.x;
	rsps.body.report.acc[ID_AXIS_Y] = sensor.y;
	rsps.body.report.acc[ID_AXIS_Z] = sensor.z;
	rsps.body.report.angle = (int16_t) sensor.lean_angle;
	rsps.body.report.geo[ID_LONGITUDE] = util_atgm332d_get_status().longitude; //测试地理位置
	rsps.body.report.geo[ID_LATITUDE] = util_atgm332d_get_status().latitude; //测试地理位置;
	rsps.body.report.status = helper_status_byte_maker(this->rt_solution.mode,
			util_lowpower_get_wake_source(), sensor.vibration_occured,
			sensor.leaned, util_atgm332d_get_status().position_fixed);
	rsps.body.report.temp = (int8_t) analog.temperture;
	rsps.body.report.time = (uint32_t) util_lowpower_get_rtc();
	rsps.body.report.vbat = (uint8_t) (analog.vbat * 10.0f);
	rsps.body.report.csq = air->status.csq;
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));

	/*log some critical information: accel data, angle, status byte, temp, time, csq*/
	logInfo("angle: %d, temp: %d, time: %ld", rsps.body.report.angle,
			rsps.body.report.temp, rsps.body.report.time);
	logInfo("geo: [%f,%f], vbat: %d ", rsps.body.report.geo[ID_LONGITUDE],
			rsps.body.report.geo[ID_LATITUDE], rsps.body.report.vbat);

	send_message((uint8_t*) &rsps, sizeof(pb_packReport));
}

void Solution::restart(void) {
//	$todo simply reset chip
	logInfo("chip restart..");
	vTaskDelay(pdMS_TO_TICKS(50));
	/*$notice 重启时，写入一次flash*/
	__flash_sync();
	HAL_NVIC_SystemReset();
}

void Solution::refresh_server(void) {
	xTimerReset(server_timeout_timer, portMAX_DELAY);
}

void Solution::refresh_device(void) {
	xTimerReset(device_timeout_timer, portMAX_DELAY);
}

void Solution::event_action_vibration(void) {
	/*震动发生后，刷新服务器响应超时定时器*/
	logInfo("vibration detected..");
	xTimerReset(server_timeout_timer, portMAX_DELAY);
	this->report();
	/*
	 * activate bd system if the running mode is "work"
	 * 		if the mode is idle or factory, position
	 * 		update will not work for saving power
	 */
	if (rt_solution.mode == solution_mode_e::wm_work) {
		logInfo("positioning..");
		util_atgm332d_activate(10);
	}
}

void Solution::event_action_message(void) {
	uint8_t *message = (uint8_t*) pvPortMalloc(64);
	uint16_t read_size = 0;
	uint16_t target_crc = 0;
	uint8_t *crc_ptr = NULL;
	uint8_t *protocol_ptr = NULL;
	pb_header *possible_header = NULL;
	void *param_ptr = NULL;

	configASSERT(message != nullptr);

	read_size = read_message(message, 64);

	if (read_size < sizeof(pb_header)) {
		vPortFree(message);
		return;
	}

	protocol_ptr = message;
	while (protocol_ptr < message + read_size - 2) {
		if (*protocol_ptr == 0xFB && *(protocol_ptr + 1) == 0xFB
				&& *(protocol_ptr + 2) == 0xFB) {
			protocol_ptr += 3;
			possible_header = (pb_header*) protocol_ptr;
			if (possible_header->code == this->rt_solution.systemConfig.code) {
				crc_ptr = protocol_ptr + sizeof(pb_header)
						+ possible_header->dataLen;
				target_crc = (*(crc_ptr + 1) << 8) | *crc_ptr;
				if (HAL_CRC_Calculate(&hcrc, (uint32_t*) possible_header,
						sizeof(pb_header) + possible_header->dataLen)
						== target_crc) {
					param_ptr = ((uint8_t*) possible_header)
							+ sizeof(pb_header);
					logInfo("message confirmed..");
					this->refresh_server();
					this->refresh_device();
					logInfo("message received: %s",
							enum_name((e_pb_func )(possible_header->function)).data());
					switch (possible_header->function) {
					case e_pb_func::down_uploadReport:
						this->message_report();
						break;
					case e_pb_func::down_uploadRunningConfig:
						this->message_upload_run_parameters();
						break;
					case e_pb_func::down_configRunning:
						this->message_change_run_parameters(
								(pb_runningConfig*) param_ptr);
						break;
					case e_pb_func::down_sleep:
						this->message_sleep();
						break;
					case e_pb_func::down_configMode:
						this->message_enter_config_mode((password*) param_ptr);
						break;
					case e_pb_func::down_uploadSystemConfig:
						this->message_upload_sys_parameters();
						break;
					case e_pb_func::down_configSystem:
						this->message_change_sys_parameters(
								(pb_systemConfig*) param_ptr);
						break;
					case e_pb_func::down_configWorkMode:
						this->message_change_execute_mode(
								(solution_mode_e*) param_ptr);
						break;
					}
					vPortFree(message);
					return;
				}
			}
		}
		protocol_ptr++;
	}
	vPortFree(message);
	logWarning("bad message..");
	return;
}

void Solution::message_report(void) {
	this->report();
}

void Solution::message_upload_run_parameters(void) {
	pb_packRunningConfig rsps;
	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.dataLen = sizeof(pb_runningConfig);
	rsps.body.header.function = e_pb_func::up_runningConfigUpload;
	rsps.body.runningConfig = this->rt_solution.runningConfig;

	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));

	send_message((uint8_t*) &rsps, sizeof(pb_packRunningConfig));
}

void Solution::message_change_run_parameters(pb_runningConfig *params) {
	pb_packCmdletOrResponse rsps;

	util_sc7a20_config_s sensor_config = util_sc7a20_get_config();
	util_analog_config_s analog_config = util_analog_get_config();

	rt_solution.runningConfig = *params;
//	修改Solution相关配置
	xTimerChangePeriod(server_timeout_timer,
			pdMS_TO_TICKS(rt_solution.runningConfig.t2_serverRsps_timeout_sec * 1000),
			portMAX_DELAY); //    修改服务器响应超时定时器,单位为秒,立即生效
	xTimerChangePeriod(device_timeout_timer,
			pdMS_TO_TICKS( util_atgm332d_get_status().position_fixed?rt_solution.runningConfig.t4_gnssGood_sleep_sec * 1000:rt_solution.runningConfig.t3_gnssSearch_sleep_sec * 1000),
			portMAX_DELAY); //    修改设备超时定时器,单位为分钟,立即生效

//	修改传感器配置:震动消抖
	sensor_config.cool_down_timeout = params->t5_motionDetect_delay_sec;
	util_sc7a20_set_config(sensor_config);
//	修改模拟信号配置:电池报警阈值
	analog_config.low_battery_threshold = params->n1_vbatAlarm_threshold_volt
			/ 10.0f;
	util_analog_set_config(analog_config);

	nvm->save();

	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.function = e_pb_func::up_runningConfigResult;
	rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

	rsps.body.cmdletOrResponse =
			this->rt_solution.configEditting ?
					(this->rt_solution.runningConfig = *params) : 0xFF;
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));

//    util_air780_net_write((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
	send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
}

void Solution::message_sleep(void) {
	pb_packCmdletOrResponseSimple rsps;

//	log_i("requested sleep.");

	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.function = e_pb_func::up_responseSleep;
	rsps.body.header.dataLen = 0;
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));

//    util_air780_net_write((uint8_t*) &rsps,
//	    sizeof(pb_packCmdletOrResponseSimple));
	send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponseSimple));
//	save all configurations before sleep
	nvm->save();
//	sleep
	auto lpconfig = util_lowpower_get_config();
	lpconfig.requested_wakeup_period =
			this->rt_solution.runningConfig.t0_netGood_wakeup_min * 60;
	lpconfig.wakeup_remain = lpconfig.requested_wakeup_period;
	lpconfig.wake_pin_enable = true;
	util_lowpower_set_config(lpconfig);
	util_lowpower_standby();
}

void Solution::message_enter_config_mode(password *pwd) {
	pb_packCmdletOrResponse rsps;
	password exitWord = { .item = { 0 } };
	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.function = e_pb_func::up_responseConfigMode;
	rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

//	非编辑模式，判断密码并进入编辑模式
	if (!this->rt_solution.configEditting) {
		if (this->rt_solution.pwd == *pwd) {
			this->rt_solution.configEditting = true;
			rsps.body.cmdletOrResponse = edit_enabled;
		} else {
			rsps.body.cmdletOrResponse = edit_notEnabled;
		}
	}
//	编辑模式，识别退出密码，或更改密码
	else {
//	 编辑模式优先识别退出功能
		if (*pwd == exitWord) {
//	 	退出编辑模式也会清空newpassword
			this->rt_solution.configEditting = false;
			this->rt_solution.newPassword = exitWord;
			this->rt_solution.passwordConfirm = 0;
			rsps.body.cmdletOrResponse = edit_exit;
		}
//		若已经开始编辑密码则不进入下一分支（开始编辑密码）
		else if (this->rt_solution.passwordConfirm) {
//	 	若密码确认成功则返回密码修改成功
			if (*pwd == this->rt_solution.newPassword) {
				rsps.body.cmdletOrResponse = edit_pwdChangeSuccess;
			} else {
//		 若密码确认失败则清空记录的newpassword并返回失败
				this->rt_solution.newPassword = exitWord;
				rsps.body.cmdletOrResponse = edit_pwdChangeFailed;
			}
			this->rt_solution.passwordConfirm = 0;
		}
//		若未编辑密码则开始编辑密码
		else {
//	    返回f0表示再输一次密码
			this->rt_solution.newPassword = *pwd;
			this->rt_solution.passwordConfirm = 1;
			rsps.body.cmdletOrResponse = edit_pwdConfirm;
		}
	}
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));
	send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
}

void Solution::message_upload_sys_parameters(void) {
	pb_packSystemConfig rsps;

	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.function = e_pb_func::up_systemConfigUpload;
	rsps.body.header.dataLen = sizeof(rsps.body.systemConfig);

	rsps.body.systemConfig = this->rt_solution.systemConfig;

	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));

	send_message((uint8_t*) &rsps, sizeof(pb_packSystemConfig));
}

void Solution::message_change_sys_parameters(pb_systemConfig *params) {
	pb_packCmdletOrResponse rsps;

	rsps.body.cmdletOrResponse =
			this->rt_solution.configEditting ? (this->rt_solution.systemConfig =
														*params) :
												0xFF;

	air->setServer(params->runServerIP, params->runServerPort,
			AIR780EP::air780_server_t::server_main);
	air->setServer(params->backupServerIP, params->backupServerPort,
			AIR780EP::air780_server_t::server_aux);

//	修改传感器配置:倒置角度范围，震动检测阈值
	util_sc7a20_config_s sensor_config = util_sc7a20_get_config();
	sensor_config.range = this->rt_solution.systemConfig.sensorReverseRange;
	sensor_config.acc_thres16mg_lsb = params->sensorVibrationThreshold;
	util_sc7a20_set_config(sensor_config);

	nvm->save();

	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.function = e_pb_func::up_systemConfigResult;
	rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));
//    util_air780_net_write((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
	send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
}

void Solution::message_change_execute_mode(solution_mode_e *mode) {
	bool modeValid = false;

	pb_packCmdletOrResponse rsps;

	rsps.body.header.code = this->rt_solution.systemConfig.code;
	rsps.body.header.function = e_pb_func::up_responseWorkMode;
	rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

	if (enum_contains<solution_mode_e>(*mode)) {
		modeValid = true;
	}

//	@notice 只要设置模式打开，界桩应尽力切换到目标模式，而不应存在切换失败的问题
//	且输入的模式应合法
	rsps.body.cmdletOrResponse =
			(this->rt_solution.configEditting && modeValid) ? true : false;

//	如果当前模式和将要设置的模式相同则不进行任何操作并返回成功，
//	如果相同则变更系统模式，并生成一个系统模式切换事件（因为这个模式切换可能引发重启或休眠）
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
			CFL(rsps.body));
	send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
	if (!modeValid)
		return;
	if (this->rt_solution.mode == *mode)
		return;

	this->rt_solution.mode = *mode;
	util_sc7a20_config_s sensor_config;

	switch (this->rt_solution.mode) {
	case solution_mode_e::wm_work: {
		/*
		 * 切换到工作模式，标定工作角度
		 * 校正角度，持续3s
		 */
		float sampled_angle = util_sc7a20_sample_angle(30, 100);
		sensor_config = util_sc7a20_get_config();
		sensor_config.leanDetectOffset = sampled_angle;
		sensor_config.leanDetectEnabled = true;
		util_sc7a20_set_config(sensor_config);
		/*切换到主服务器*/
		this->rt_solution.connect_to_main_server = true;
		/*启动时更新一次定位*/
		this->rt_solution.updatePositionOnStart = true;
	}
		break;
	case solution_mode_e::wm_idle: {
		sensor_config = util_sc7a20_get_config();
		sensor_config.leanDetectEnabled = false;
		util_sc7a20_set_config(sensor_config);
		/*切换到主服务器*/
		this->rt_solution.connect_to_main_server = true;
	}
		break;
	case solution_mode_e::wm_factory: {
		sensor_config = util_sc7a20_get_config();
		sensor_config.leanDetectEnabled = false;
		util_sc7a20_set_config(sensor_config);
		/*切换到主服务器*/
		this->rt_solution.connect_to_main_server = false;
	}
		break;
	default:
		;
	}
	nvm->save();
	this->restart();
//	    模式变更，保存系统然后重启

}

#pragma GCC diagnostic pop

int Solution::send_message(void *msg, uint16_t len) {
	auto connServer =
			this->rt_solution.connect_to_main_server ?
					AIR780EP::air780_server_t::server_main :
					AIR780EP::air780_server_t::server_aux;
	return air->write((char*) msg, len, portMAX_DELAY, connServer);
}

int Solution::read_message(void *dest, uint16_t len) {
	auto connServer =
			this->rt_solution.connect_to_main_server ?
					AIR780EP::air780_server_t::server_main :
					AIR780EP::air780_server_t::server_aux;
	/*$notice 当poll到message事件时，在0等待时间时，至少应该能获取到1条消息*/
	return air->read((char*) dest, len, 0, connServer);
}

void Solution::server_timeout_timer_callback(TimerHandle_t xTimer) {
	auto *pthis = (Solution*) pvTimerGetTimerID(xTimer);
	pthis->report();
}

void Solution::device_timeout_timer_callback(TimerHandle_t xTimer) {
	auto *pthis = (Solution*) pvTimerGetTimerID(xTimer);
	/*
	 * $todo 休眠时间一定是netgood，因为这个定时器是在连接网络之后创建的
	 * 		    如果网络连接失败，在创建这个定时器之前就会进入休眠
	 */
	uint32_t sleep_time = pthis->rt_solution.runningConfig.t0_netGood_wakeup_min
			* 60;
	auto lpconfig = util_lowpower_get_config();
	lpconfig.wake_pin_enable = true;
	lpconfig.requested_wakeup_period = sleep_time;
	lpconfig.wakeup_remain = sleep_time;
	util_lowpower_set_config(lpconfig);
	util_lowpower_standby();
}

void Solution::event_process(void) {
	util_event_code_t polled_event;
	/*等待事件发生并处理*/
	/*
	 * $notice 实际上有意义的事件只有连个：vibrate和message
	 *         [comment date--Feb 18, 2025] :其他事件都没有实际用途，可以删除
	 *         休眠或服务器超时定时器不需要在连接到服务器之前就创建，因为就算发生了
	 *         震动事件，因为没有连接到服务器，界桩也是哑巴，所以不会有任何反应，所以
	 *         这两个定时器应该在连接之后创建
	 *
	 * */
	if (util_events_poll(&polled_event)) {
		switch (polled_event) {
		case util_event_code_t::vibrate:
			this->event_action_vibration();
			break;
		case util_event_code_t::message:
			this->event_action_message();
			break;
			/*
			 * $notice 网络连接确认事件已经删除，当网络初始化（连接）后，立即report
			 *    	然后创建超时定时器，超时则再次report
			 * */

			/*
			 * $notice GNSS更新事件已经删除，当振动发生过后，北斗会立即激活，然后开始刷新
			 *     	定位信息，所以不需要GNSS更新事件，服务器如果需要定位信息，只需要等待
			 *     	振动事件发生即可，或设计一条命令，手动激活北斗上电，然后等待北斗定位
			 *     	刷新完成即可
			 */
		default:
			break;
		}
	}
}

void Solution::setup_network(void) {
	logInfo("Setting up network.");
	auto lpconfig = util_lowpower_get_config();
	this->air->poweron();
	if (!this->air->waitEutran(10000)) {
		/*等待LTE附着，LTE附着超过10秒说明附近没有基站或信号微弱*/
		logWarning("LTE not attached!");
		lpconfig.wake_pin_enable = true;
		lpconfig.requested_wakeup_period =
				this->rt_solution.runningConfig.t1_netBad_wakeup_min * 60;
		lpconfig.wakeup_remain = lpconfig.requested_wakeup_period;
		util_lowpower_set_config(lpconfig);
		util_lowpower_standby();/*LTE附着失败，休眠*/
	}
	logInfo("LTE attached, module configuring..");
	this->air->setup();/*模块初始配置*/
	this->air->setServer(this->rt_solution.systemConfig.runServerIP,
			this->rt_solution.systemConfig.runServerPort,
			AIR780EP::air780_server_t::server_main);/*配置主服务器链接参数*/
	this->air->setServer(this->rt_solution.systemConfig.backupServerIP,
			this->rt_solution.systemConfig.backupServerPort,
			AIR780EP::air780_server_t::server_aux);/*配置备用服务器链接参数*/
	logInfo("main server: %d.%d.%d.%d:%d, aux server: %d.%d.%d.%d:%d",
			this->rt_solution.systemConfig.runServerIP[0],
			this->rt_solution.systemConfig.runServerIP[1],
			this->rt_solution.systemConfig.runServerIP[2],
			this->rt_solution.systemConfig.runServerIP[3],
			this->rt_solution.systemConfig.runServerPort,
			this->rt_solution.systemConfig.backupServerIP[0],
			this->rt_solution.systemConfig.backupServerIP[1],
			this->rt_solution.systemConfig.backupServerIP[2],
			this->rt_solution.systemConfig.backupServerIP[3],
			this->rt_solution.systemConfig.backupServerPort);
	/*连接到主服务器或备用服务器*/
	bool connectReady;
	if (this->rt_solution.connect_to_main_server)/*选择要连接的通信服务器*/
		connectReady = this->air->connect(
				AIR780EP::air780_server_t::server_main);
	else
		connectReady = this->air->connect(
				AIR780EP::air780_server_t::server_aux);

	logInfo("connect to %s server: %s",
			this->rt_solution.connect_to_main_server ? "main" : "aux",
			connectReady ? "success" : "failed");

	if (!connectReady) {
		/*等待LTE附着，LTE附着超过10秒说明附近没有基站或信号微弱*/
		lpconfig.wake_pin_enable = true;
		lpconfig.requested_wakeup_period =
				this->rt_solution.runningConfig.t1_netBad_wakeup_min * 60;
		lpconfig.wakeup_remain = lpconfig.requested_wakeup_period;
		util_lowpower_set_config(lpconfig);
		util_lowpower_standby();/*LTE附着失败，休眠*/
	}

}

void Solution::timers_create(void) {
	/*----	定时器创建流程开始	----*/
	this->server_timeout_timer =
			xTimerCreate(this->server_timeout_timer_name,
					pdMS_TO_TICKS(
							1000
									* this->rt_solution.runningConfig.t2_serverRsps_timeout_sec),
					pdTRUE, (void*) this, server_timeout_timer_callback);
	this->device_timeout_timer = xTimerCreate(this->device_timeout_timer_name,
			pdMS_TO_TICKS(
					this->rt_solution.runningConfig.t3_gnssSearch_sleep_sec
							* 1000),
			pdTRUE, (void*) this, device_timeout_timer_callback);
	logInfo("server response timeout countdown: %d secs",
			this->rt_solution.runningConfig.t2_serverRsps_timeout_sec);
	logInfo("device sleep countdown: %d secs",
			this->rt_solution.runningConfig.t3_gnssSearch_sleep_sec);
	xTimerStart(this->server_timeout_timer, portMAX_DELAY);
	xTimerStart(this->device_timeout_timer, portMAX_DELAY);
	/*----	定时器创建流程结束	----*/
}

void Solution::solution_work_routine_thread(void *argument) {
	Solution *pthis = (Solution*) argument;

	/*----	4G启动		----*/
	pthis->air = new AIR780EP(&huart1);
	configASSERT(pthis->air != NULL);
	pthis->setup_network();
	/*清空等待网络期间发生的所有事件*/
	util_events_flush();
	pthis->report(); //连接后立即上传一次数据

	/*----	定时器启动	----*/
	pthis->timers_create();

	/*----	定位刷新 		----*/
	if (pthis->rt_solution.updatePositionOnStart) {
		util_atgm332d_activate(3);
		pthis->rt_solution.updatePositionOnStart = false;
		pthis->nvm->save();
	}

	loop: pthis->event_process();
	goto loop;
}

void Solution::solution_idle_routine_thread(void *argument) {
	auto *pthis = (Solution*) argument;
	/*
	 * $outline 空闲模式运行例程
	 * 		1. 不使用北斗模块，尚未激活工作模式时，不需要定位信息，全部上传默认定位即可
	 * 			这是为了降低非工作模式期间的电能消耗，但是需要加载北斗状态，
	 * 			以便服务器能够收到一个特定的定位用以区分空闲模式
	 * 		2. 创建4G通信模组实例，连接到备用服务器
	 * 		3. 开始处理事件
	 * 			除了北斗之外，其他部分和工作模式一致
	 */

	/*创建通信模组实例*/
	pthis->air = new AIR780EP(&huart1);
	configASSERT(pthis->air != NULL);
	pthis->setup_network();
	util_events_flush();
	pthis->report(); //连接后立即上传一次数据

	/*----	定时器创建 	----*/
	pthis->timers_create();

	loop: pthis->event_process();
	goto loop;

}

void Solution::solution_fact_routine_thread(void *argument) {
	/*
	 * $outline 工厂模式运行例程
	 *      	1. 读取加速度传感器信息
	 *      		1.1 一段时间内判断机身是否垂直放置（在一定误差范围内）
	 *      		如果机身垂直，则系统激活，修改系统模式并保存，然后重启
	 *      		如果机身不垂直，则系统继续休眠，保持所有参数不变更
	 * */

	/*
	 * $outline 脱离工厂模式检测方法
	 * 		1. 连续十秒持续检测，十秒内以10hz对倾斜标志位进行采样
	 *  	2. 若十秒后检测通过，则认为机身垂直正置防止，激活系统
	 *  	3. 要求共100次采样全部为假，以达到严格判定的目的
	 */

	auto *pthis = (Solution*) argument;

	uint8_t sample_count = 0;

	vTaskDelay(pdMS_TO_TICKS(100));

	for (unsigned int i = 0; i < 100; i++) {
		if (util_sc7a20_get_status().leaned)
			break;
		sample_count++;
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	if (sample_count < 98) {
		auto lpconfig = util_lowpower_get_config();
		/*
		 * $notice 工厂模式下，每过两小时唤醒自身重新检测
		 * 		期间禁止震动唤醒
		 * */
		lpconfig.requested_wakeup_period = PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC;
		lpconfig.wakeup_remain = PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC;
		lpconfig.wake_pin_enable = false;
		util_lowpower_set_config(lpconfig);
		util_lowpower_standby();
	}
	/*系统激活*/
	pthis->rt_solution.mode = wm_work;			//进入空闲模式
	pthis->rt_solution.connect_to_main_server = false; //连接到备用服务器（默认参数中备用服务器和主服务器相同）
	pthis->rt_solution.configEditting = false;		//默认非编辑模式
	pthis->rt_solution.passwordConfirm = 0;		//默认密码未确认
	/*运行参数和系统参数都保持默认*/

	pthis->nvm->save();
	pthis->restart();
}
