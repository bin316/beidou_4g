/*
 * util_atgm332d.cpp
 *
 *  Created on: Feb 10, 2025
 *      Author: IRIS
 *
 * ------------------------------------------------------------------------------
 * 卫星数量 sats 字段说明
 * ------------------------------------------------------------------------------
 * 解析 GGA 语句获取 numSv，更新 status.sats，供上报数据包表示定位质量。
 * 在 atgm332d_rx_thread 中先 parseGgaMessage() 再 parseRmcMessage()。
 *
 * ------------------------------------------------------------------------------
 * 定位状态 position_fixed 与“上报后置否”说明
 * ------------------------------------------------------------------------------
 * position_fixed 表示当前缓存的经纬度是否为新的定位数据（协议 status 的 bit0）。
 * 为 true 时表示“本帧地理信息为新定位数据”，为 false 表示“非新/旧定位数据”。
 *
 * 每次成功上报数据后，Solution::report() 会调用 util_atgm332d_clear_fix_flag() 将
 * position_fixed 置为 false。下一帧仅在有新 RMC 时才会再为 1，避免多帧共用同一次
 * 定位却均标为“新”，提升服务器对“是否为新定位数据”判断的准确性。
 * 详见 solution/Solution.cpp、solution/Protocol.h、bsp/utilties.h。
 *
 * ------------------------------------------------------------------------------
 * 经纬度加权平均滤波
 * ------------------------------------------------------------------------------
 * 5点加权平均：窗口N=5，权重1~5（新点权大）。首次逐步填充，满窗后滚动。
 * 偏差剔除：新点到当前均值的平面距离>D_max时丢弃。
 * 卫星数门限：每个点均要求本帧解析到 GGA 且卫星数>=4 才参与滤波，避免差定位混入。
 * 每次激活时清空缓冲，仅在同次唤醒内滤波。
 */

#include "utilties.h"
#include "PRODUCT_CONFIG.h"

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

#define GNSS_FILTER_N              5        //加权平均滤波窗口大小
#define GNSS_FILTER_DMAX_M         10.0f    //偏差剔除阈值(米)
#define GNSS_FILTER_MIN_SATS       4        //参与滤波的最低卫星数

// static const util_atgm332d_status_t default_status = { 116.397477f, 39.908692f,
static const util_atgm332d_status_t default_status = { 0.10, 10.0,
		1739245514, false, 0, 0u };

static util_atgm332d_status_t status;

static BufferedUart *uart = NULL;
static TaskHandle_t rxTaskHandle = NULL;

static NVM *nvm_atgm332d = NULL;

static int32_t deactivate_hysteresis = 0;
static bool updated_since_activate = false;
/** 本唤醒 LBS 至少成功一次（geoStat；与 GNSS position_fixed 分开，对齐 Slope） */
static bool lbs_ok_this_wake_ = false;
static bool nvm_dirty_ = false;

TimerHandle_t deactivate_timer = NULL;

/* 加权平均滤波：lat_buf[0]最旧、[count-1]最新，权重1~count */
static float lat_buf[GNSS_FILTER_N];
static float lon_buf[GNSS_FILTER_N];
static uint8_t filter_count = 0;

/* 新点到参考点的平面距离（米）*/
static float gnss_distance_m(float lat_new, float lon_new, float lat_ref, float lon_ref) {
    // 1. 使用更精确的 PI
    float lat_rad = lat_ref * 3.1415926535f / 180.0f;
    
    // 2. 采用 WGS84 椭球体的平均纬度一度跨度 (约 111132m)
    // 3. 采用赤道经度一度跨度 (约 111319m)
    float dx = (lon_new - lon_ref) * cosf(lat_rad) * 111319.0f;
    float dy = (lat_new - lat_ref) * 111132.0f;
    
    return sqrtf(dx * dx + dy * dy);
}

/* 对 buf[0..k-1] 做加权平均，权重 1..k，结果写入 *lat_out, *lon_out */
static void gnss_weighted_avg(float *lat_buf, float *lon_buf, uint8_t k,
	float *lat_out, float *lon_out) {
	float sum_w = 0, sum_lat = 0, sum_lon = 0;
	for (uint8_t i = 0; i < k; i++) {
		float w = (float)(i + 1);
		sum_w += w;
		sum_lat += w * lat_buf[i];
		sum_lon += w * lon_buf[i];
	}
	*lat_out = sum_lat / sum_w;
	*lon_out = sum_lon / sum_w;
}

static void atgm332d_rx_thread(void *argument) {
	NMEA0183 parser;
	/* 与 BufferedUart RX 深度一致；单帧 NMEA 远小于此 */
	constexpr size_t kNmeaRxBuf = 768;
	char *buffer = (char*) pvPortMalloc(kNmeaRxBuf);
	configASSERT(buffer != nullptr);
	parser.setMessageField(buffer, kNmeaRxBuf);

	auto clear = [&buffer]() {
		memset(buffer, 0, kNmeaRxBuf);
	};
	loop:

	clear();
	uint16_t read_len = uart->read(buffer, kNmeaRxBuf, 100);
	if (read_len <= 0)
		goto loop;

	//将原始NMEA数据通过串口1回显
	//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, read_len, 100);

	/* 解析 GGA 获取卫星数量，本帧解析成功则 sats 与 RMC 同源 */
	bool gga_parsed_this_frame = false;
	if (parser.parseGgaMessage()) {
		auto gga = parser.getContent<NMEA0183::GGA_t>();
		status.sats = gga->satelliteCount;
		gga_parsed_this_frame = true;
	}

	/* 解析 RMC 获取位置、时间、定位状态 */
	if (parser.parseRmcMessage() != false) {
		auto rmc = parser.getContent<NMEA0183::RMC_t>();
		float lat_new = rmc->latitude;
		float lon_new = rmc->longitude;

		bool accepted = false;
		if (filter_count == 0) {
			/* 首点质量门限：仅当本帧同时解析到 GGA 且卫星数>=4 才接受，避免用错帧 sats */
			if (gga_parsed_this_frame && status.sats >= GNSS_FILTER_MIN_SATS) {
				lat_buf[0] = lat_new;
				lon_buf[0] = lon_new;
				filter_count = 1;
				status.latitude = lat_new;
				status.longitude = lon_new;
				accepted = true;
			}
		} else {
			/* 后续点也要求本帧解析到 GGA 且卫星数>=4 */
			if (!gga_parsed_this_frame || status.sats < GNSS_FILTER_MIN_SATS) {
				accepted = false;
			} else {
			float lat_avg, lon_avg;
			gnss_weighted_avg(lat_buf, lon_buf, filter_count, &lat_avg, &lon_avg);
			float d = gnss_distance_m(lat_new, lon_new, lat_avg, lon_avg);
			if (d > GNSS_FILTER_DMAX_M) {
				/* 偏差过大，剔除 */
				accepted = false;
			} else {
				if (filter_count < GNSS_FILTER_N) {//窗口未满，直接添加
					lat_buf[filter_count] = lat_new;
					lon_buf[filter_count] = lon_new;
					filter_count++;
				} else {//窗口满后，滚动窗口丢弃最旧点，把新点写入末尾
					for (uint8_t i = 0; i < GNSS_FILTER_N - 1; i++) {
						lat_buf[i] = lat_buf[i + 1];
						lon_buf[i] = lon_buf[i + 1];
					}
					lat_buf[GNSS_FILTER_N - 1] = lat_new;
					lon_buf[GNSS_FILTER_N - 1] = lon_new;
				}
				gnss_weighted_avg(lat_buf, lon_buf, filter_count, &lat_avg, &lon_avg);
				status.latitude = lat_avg;
				status.longitude = lon_avg;
				accepted = true;
			}
			}
		}

		if (accepted) {
			status.time = rmc->unixTime;
			status.position_fixed = true;
			nvm_atgm332d->save();
			/* 不用 %f：newlib 浮点格式化栈大，RX 任务易溢出；单位=1e-4度 */
			logInfo("北斗定位已更新: lon_e4=%ld lat_e4=%ld 时间戳=%ld",
				(long) (status.longitude * 10000.0f),
				(long) (status.latitude * 10000.0f),
				(long) status.time);

			/*激活后首次更新*/
			if (!updated_since_activate) {
				if (deactivate_timer == NULL) {
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
				updated_since_activate = true;
			}
		}
	}

	goto loop;
}

void __util_atgm332d_init__(void) {
	uart = new BufferedUart(&huart2, 32, 768, 32, 256);
	configASSERT(uart != nullptr);

	nvm_atgm332d = new NVM(NVM::partition_bd, (void*) &status,
			(void*) &default_status, sizeof(util_atgm332d_status_t));
	configASSERT(nvm_atgm332d != nullptr);

	util_atgm332d_load();

	/* 对齐 Slope gnss_task=1024：NMEA 解析 + 偶发日志，512 易 HardFault */
	xTaskCreate(atgm332d_rx_thread, "atgm332d rx", 1024, NULL, osPriorityHigh,
			&rxTaskHandle);
	configASSERT(rxTaskHandle != NULL);
	logInfo("北斗: 定位守护线程已启动");
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
	lbs_ok_this_wake_ = false;
	nvm_dirty_ = false;
}

util_atgm332d_status_t util_atgm332d_get_status(void) {
	return status;
}

void util_atgm332d_wake_session_begin(void) {
	lbs_ok_this_wake_ = false;
	nvm_dirty_ = false;
}

bool util_atgm332d_lbs_interval_elapsed(void) {
	if (status.lbs_last_query_unix == 0u) {
		return true;
	}
	const time_t now = util_lowpower_get_rtc();
	if (now <= 0) {
		return true;
	}
	const time_t last = (time_t) status.lbs_last_query_unix;
	if (now < last) {
		return true; /* RTC 回拨则放行 */
	}
	const uint32_t elapsed = (uint32_t) (now - last);
	if (elapsed < PROD_CFG_LBS_MIN_INTERVAL_SEC) {
		logInfo("LBS跳过: 距上次查询 %us < %us (合宙免费单基站限频)",
			(unsigned) elapsed,
			(unsigned) PROD_CFG_LBS_MIN_INTERVAL_SEC);
		return false;
	}
	return true;
}

void util_atgm332d_lbs_note_query_sent(void) {
	const time_t now = util_lowpower_get_rtc();
	if (now <= 0) {
		return;
	}
	status.lbs_last_query_unix = (uint32_t) now;
	nvm_dirty_ = true;
}

bool util_atgm332d_lbs_ok_this_wake(void) {
	return lbs_ok_this_wake_;
}

bool util_atgm332d_geo_valid_for_report(void) {
	return status.position_fixed || lbs_ok_this_wake_;
}

void util_atgm332d_nvm_flush(void) {
	if (!nvm_dirty_ || nvm_atgm332d == nullptr) {
		return;
	}
	nvm_atgm332d->save();
	nvm_dirty_ = false;
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
	filter_count = 0; /* 清空滤波缓冲，仅在同次唤醒内滤波 */
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

/*
 * @func util_atgm332d_clear_fix_flag
 * @brief 将 position_fixed 置为 false，用于“上报数据后清除定位状态”策略。
 *        调用后，下次上报的 status 定位位为 0，直到再次解析到新 RMC 才置 1。
 */
void util_atgm332d_clear_fix_flag(void) {
	status.position_fixed = false;
}

/**
 * @brief 服务器写入经纬度：更新缓存并落 NVM（对齐 Slope sys_gnss_set_manual_geo）
 * @return 成功返回 true
 */
bool util_atgm332d_set_manual_geo(float lon, float lat) {
	status.longitude = lon;
	status.latitude = lat;
	status.sats = 0;
	/* 与 Slope 一致：写入后不标为 GNSS 新定位，坐标仍会被 report 读出 */
	status.position_fixed = false;
	status.time = util_lowpower_get_rtc();
	if (nvm_atgm332d != NULL) {
		nvm_atgm332d->save();
	}
	logInfo("北斗: 写入服务器坐标 lon_e4=%ld lat_e4=%ld",
		(long) (lon * 10000.0f), (long) (lat * 10000.0f));
	return true;
}

/**
 * @brief LBS 写经纬度：本周期 lbs_ok（geoStat），不置 GNSS position_fixed；延迟落盘
 */
bool util_atgm332d_apply_lbs_geo(float lon, float lat) {
	/* 查询返回时若 GNSS 已 fix，丢弃 LBS 坐标（对齐 Slope） */
	if (status.position_fixed) {
		logInfo("北斗: LBS坐标丢弃(北斗已定位)");
		return false;
	}
	status.longitude = lon;
	status.latitude = lat;
	status.sats = 0;
	status.time = util_lowpower_get_rtc();
	lbs_ok_this_wake_ = true;
	nvm_dirty_ = true;
	logInfo("北斗: 应用LBS坐标 lon_e4=%ld lat_e4=%ld",
		(long) (lon * 10000.0f), (long) (lat * 10000.0f));
	return true;
}

/** 流式注入前暂停 Idle-DMA，避免与 HAL 阻塞发送打架（对齐 Slope） */
bool util_atgm332d_casbin_stream_begin(void) {
	if (uart == nullptr) {
		return false;
	}
	(void) uart->ioctl(BufferedUart::_rx_pause, nullptr);
	uint32_t flush_arg = 0;
	(void) uart->ioctl(BufferedUart::_flush_rx, &flush_arg);
	vTaskDelay(pdMS_TO_TICKS(20));
	return true;
}

/**
 * @brief 流式写入一段 CASBIN（按 256B 切块，块间可延时）
 * @note  阻塞 HAL 发送；每块喂狗，避免 IWDG(~4s) 在长注入中复位
 */
bool util_atgm332d_casbin_stream_write(const uint8_t *data, size_t len,
		uint32_t inter_chunk_ms) {
	if (uart == nullptr || data == nullptr || len == 0) {
		return false;
	}

	const uint32_t gap_ms = (inter_chunk_ms > 0) ? inter_chunk_ms : 20u;
	constexpr size_t chunk = 256;
	for (size_t off = 0; off < len; off += chunk) {
		const size_t todo = (len - off > chunk) ? chunk : (len - off);
		/* HAL 超时单位是 ms，勿传 pdMS_TO_TICKS */
		const HAL_StatusTypeDef st = HAL_UART_Transmit(&huart2,
				const_cast<uint8_t*>(data + off), (uint16_t) todo, 2000u);
		if (st != HAL_OK) {
			logWarning("CASBIN: 流式写入失败 偏移=%u 状态=%d",
					(unsigned) off, (int) st);
			return false;
		}
		util_lowpower_iwdg_feed();
		if (gap_ms > 0 && off + todo < len) {
			vTaskDelay(pdMS_TO_TICKS(gap_ms));
		}
	}
	return true;
}

/**
 * @brief 向北斗串口注入完整 CASBIN（AGNSS）
 */
bool util_atgm332d_inject_casbin(const uint8_t *data, size_t len,
		uint32_t inter_chunk_ms) {
	if (!util_atgm332d_casbin_stream_begin()) {
		return false;
	}
	const bool ok = util_atgm332d_casbin_stream_write(data, len, inter_chunk_ms);
	(void) uart->ioctl(BufferedUart::_rx_resume, nullptr);
	if (!ok) {
		return false;
	}
	logInfo("CASBIN: 注入成功, %u字节", (unsigned) len);
	return true;
}
