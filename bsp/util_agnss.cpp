/**
 * @file util_agnss.cpp
 * @brief 中科微 AGNSS：AIR780 link2 拉 CASBIN，整包缓存于 SRAM2 后注入北斗
 * @note  是否拉取由 locate_switch bit1 决定；大缓冲在 ._ram2_area，不挤主 RAM
 */
#include "util_agnss.h"
#include "AIR780EP.h"
#include "PRODUCT_CONFIG.h"
#include "utilties.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"

/* 整包 CASBIN 落地在 SRAM2，与 NVM 页缓冲共用 RAM2，不占 48KB 主 RAM */
static uint8_t s_agnss_recv_buf[PROD_CONFIG_AGNSS_RECV_BUF_SIZE]
	__attribute__((section("._ram2_area")));
static volatile bool s_agnss_rx_active = false;
static volatile size_t s_agnss_rx_total = 0;
static volatile int s_agnss_rx_pending = 0;
static bool s_agnss_done_this_wake = false;

namespace {

constexpr const char *kDataLengthTag = "DataLength: ";
constexpr uint8_t kCasbinMagic0 = 0xBA;
constexpr uint8_t kCasbinMagic1 = 0xCE;

/** 组装中科微 ASCII 请求（单北斗 gnss=bds；不带粗略位置） */
int agnss_build_request(char *buf, size_t cap) {
	return snprintf(buf, cap,
			"user=%s;pwd=%s;cmd=full;gnss=bds;\r\n",
			PROD_CONFIG_AGNSS_USER, PROD_CONFIG_AGNSS_PWD);
}

/** 在缓冲中定位 CASBIN 魔数 0xBA 0xCE */
bool agnss_find_casbin(const uint8_t *raw, size_t total, size_t *out_off) {
	if (raw == nullptr || out_off == nullptr || total < 2) {
		return false;
	}
	for (size_t i = 0; i + 1 < total; ++i) {
		if (raw[i] == kCasbinMagic0 && raw[i + 1] == kCasbinMagic1) {
			*out_off = i;
			return true;
		}
	}
	return false;
}

/**
 * 读 DataLength 声明的载荷长度；未出现则返回 0。
 * 实网响应形如：AGNSS data from CASIC.\\nDataLength: N.\\nLimitation: x/y.\\n + CASBIN
 */
size_t agnss_peek_declared_len(const uint8_t *raw, size_t total) {
	if (raw == nullptr || total < strlen(kDataLengthTag) + 1) {
		return 0;
	}
	const size_t tag_len = strlen(kDataLengthTag);
	for (size_t i = 0; i + tag_len < total && i < 160; ++i) {
		if (memcmp(raw + i, kDataLengthTag, tag_len) != 0) {
			continue;
		}
		char *end = nullptr;
		const unsigned long n =
			strtoul(reinterpret_cast<const char *>(raw + i + tag_len), &end, 10);
		if (n > 0 && n <= PROD_CONFIG_AGNSS_RECV_BUF_SIZE) {
			return static_cast<size_t>(n);
		}
		return 0;
	}
	return 0;
}

/**
 * 从 TCP 响应提取 CASBIN。
 * @note DataLength 行之后可能还有 Limitation 等 ASCII 行，必须以 0xBA 0xCE 定位载荷。
 */
bool agnss_parse_payload(const uint8_t *raw, size_t total, const uint8_t **out_data,
		size_t *out_len) {
	if (raw == nullptr || total == 0 || out_data == nullptr || out_len == nullptr) {
		return false;
	}

	size_t magic_off = 0;
	if (!agnss_find_casbin(raw, total, &magic_off)) {
		return false;
	}

	const size_t remain = total - magic_off;
	const size_t declared = agnss_peek_declared_len(raw, total);
	if (declared > 0) {
		if (remain < declared) {
			return false;
		}
		*out_data = raw + magic_off;
		*out_len = declared;
		return true;
	}

	*out_data = raw + magic_off;
	*out_len = remain;
	return remain >= 2;
}

/** 解析失败时打出可读头，便于区分鉴权错误与截断 */
void agnss_log_recv_preview(const uint8_t *raw, size_t total) {
	char preview[81] = {};
	const size_t n = (total < 80) ? total : 80;
	for (size_t i = 0; i < n; ++i) {
		const uint8_t c = raw[i];
		preview[i] = (c >= 32 && c < 127) ? static_cast<char>(c) : '.';
	}
	logWarning("AGNSS: 响应预览[%u]: %s", static_cast<unsigned>(total), preview);
}

/** link2 短连：发请求 → 等 RX 收齐 DataLength / 排空 → 断开 */
bool air_link2_fetch(AIR780EP *air, const char *request, int req_len, uint32_t recv_timeout_ms,
		size_t cap, size_t *out_len) {
	util_agnss_rx_begin();

	/* 短连超时：对齐 Slope NetLte::connect(agnss, CONNECT_TIMEOUT) */
	if (!air->connect(AIR780EP::server_debug, PROD_CONFIG_AGNSS_CONNECT_TIMEOUT_MS)) {
		logWarning("AGNSS: 链路2 TCP连接失败或超时");
		util_agnss_rx_end();
		return false;
	}

	if (req_len > 0) {
		if (air->write(const_cast<char *>(request), static_cast<uint16_t>(req_len), 3000,
				AIR780EP::server_debug) != req_len) {
			logWarning("AGNSS: 链路2 发送失败");
			air->disconnect(AIR780EP::server_debug);
			util_agnss_rx_end();
			return false;
		}
	}

	const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(recv_timeout_ms);
	bool link_closed = false;
	TickType_t closed_at = 0;
	size_t last_total = 0;
	TickType_t last_progress = xTaskGetTickCount();

	while (xTaskGetTickCount() < deadline) {
		const size_t total = util_agnss_rx_total();
		if (total != last_total) {
			last_total = total;
			last_progress = xTaskGetTickCount();
		}

		const size_t declared = agnss_peek_declared_len(s_agnss_recv_buf, total);
		size_t magic_off = 0;
		if (declared > 0 && agnss_find_casbin(s_agnss_recv_buf, total, &magic_off)
			&& (total - magic_off) >= declared) {
			logInfo("AGNSS: 已收齐声明长度 %u (缓冲=%u)", static_cast<unsigned>(declared),
				static_cast<unsigned>(total));
			break;
		}

		if (total >= cap) {
			logWarning("AGNSS: 接收缓冲区已满且未收齐, 缓冲=%u", static_cast<unsigned>(total));
			air->disconnect(AIR780EP::server_debug);
			util_agnss_rx_end();
			return false;
		}

		if (!air->getConnectionsStatus(AIR780EP::server_debug)) {
			if (!link_closed) {
				link_closed = true;
				closed_at = xTaskGetTickCount();
				logInfo("AGNSS: 对端关闭, 已收 %u 字节, 继续排空",
						static_cast<unsigned>(total));
			}
			const bool pending_done = (util_agnss_rx_pending() <= 0);
			const bool idle =
				(xTaskGetTickCount() - last_progress) >= pdMS_TO_TICKS(400);
			const bool grace_done =
				(xTaskGetTickCount() - closed_at) >= pdMS_TO_TICKS(400);
			if (pending_done && idle && grace_done) {
				break;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
		util_lowpower_iwdg_feed();
	}

	logInfo("AGNSS: 断开链路2 ...");
	const size_t total = util_agnss_rx_total();
	util_agnss_rx_end();
	air->disconnect(AIR780EP::server_debug);
	logInfo("AGNSS: 链路2已断开, 收到 %u 字节", static_cast<unsigned>(total));

	if (total == 0) {
		logWarning("AGNSS: 未收到数据");
		return false;
	}
	if (total > cap) {
		logWarning("AGNSS: 数据超出缓冲");
		return false;
	}
	*out_len = total;
	return true;
}

} // namespace

bool util_agnss_done_this_wake(void) {
	return s_agnss_done_this_wake;
}

bool util_agnss_rx_begin(void) {
	s_agnss_rx_total = 0;
	s_agnss_rx_pending = 0;
	s_agnss_rx_active = true;
	return true;
}

void util_agnss_rx_end(void) {
	s_agnss_rx_active = false;
	s_agnss_rx_pending = 0;
}

bool util_agnss_rx_is_active(void) {
	return s_agnss_rx_active;
}

size_t util_agnss_rx_total(void) {
	return s_agnss_rx_total;
}

bool util_agnss_rx_append(const void *data, size_t len) {
	if (!s_agnss_rx_active || data == nullptr || len == 0) {
		return false;
	}
	{
		const size_t have = s_agnss_rx_total;
		const size_t declared = agnss_peek_declared_len(s_agnss_recv_buf, have);
		size_t magic_off = 0;
		if (declared > 0 && agnss_find_casbin(s_agnss_recv_buf, have, &magic_off)
			&& (have - magic_off) >= declared) {
			return true;
		}
	}
	taskENTER_CRITICAL();
	const size_t space = PROD_CONFIG_AGNSS_RECV_BUF_SIZE - s_agnss_rx_total;
	const size_t n = (len < space) ? len : space;
	if (n > 0) {
		memcpy(s_agnss_recv_buf + s_agnss_rx_total, data, n);
		s_agnss_rx_total += n;
	}
	taskEXIT_CRITICAL();
	return n > 0;
}

int util_agnss_rx_pending(void) {
	return s_agnss_rx_pending;
}

void util_agnss_rx_set_pending(int pending) {
	s_agnss_rx_pending = pending > 0 ? pending : 0;
}

void util_agnss_rx_consume_pending(size_t n) {
	if (n >= static_cast<size_t>(s_agnss_rx_pending)) {
		s_agnss_rx_pending = 0;
	} else {
		s_agnss_rx_pending -= static_cast<int>(n);
	}
}

bool util_agnss_rx_append_continuation(const void *data, size_t len) {
	if (!s_agnss_rx_active || data == nullptr || len == 0 || s_agnss_rx_pending <= 0) {
		return false;
	}
	const size_t copy = (len < static_cast<size_t>(s_agnss_rx_pending))
				? len
				: static_cast<size_t>(s_agnss_rx_pending);
	if (!util_agnss_rx_append(data, copy)) {
		return false;
	}
	util_agnss_rx_consume_pending(copy);
	return true;
}

bool util_agnss_fetch_and_inject_once(AIR780EP *air) {
	if (air == nullptr || s_agnss_done_this_wake) {
		return false;
	}
	s_agnss_done_this_wake = true;

	char request[96] = {};
	const int req_size = agnss_build_request(request, sizeof(request));
	if (req_size <= 0 || static_cast<size_t>(req_size) >= sizeof(request)) {
		logWarning("AGNSS: 请求组装失败");
		return false;
	}

	uint8_t ip[4] = PROD_CONFIG_AGNSS_SERVER_IP;
	air->setServer(ip, PROD_CONFIG_AGNSS_SERVER_PORT, AIR780EP::server_debug);
	logInfo("AGNSS: 连接 %u.%u.%u.%u:%u ...",
			ip[0], ip[1], ip[2], ip[3], static_cast<unsigned>(PROD_CONFIG_AGNSS_SERVER_PORT));

	size_t recv_len = 0;
	if (!air_link2_fetch(air, request, req_size, PROD_CONFIG_AGNSS_TCP_TIMEOUT_MS,
			PROD_CONFIG_AGNSS_RECV_BUF_SIZE, &recv_len)) {
		logWarning("AGNSS: 拉取失败，继续冷启动定位");
		return false;
	}

	const uint8_t *payload = nullptr;
	size_t payload_len = 0;
	if (!agnss_parse_payload(s_agnss_recv_buf, recv_len, &payload, &payload_len)) {
		size_t magic_off = 0;
		const bool has_magic = agnss_find_casbin(s_agnss_recv_buf, recv_len, &magic_off);
		const size_t decl = agnss_peek_declared_len(s_agnss_recv_buf, recv_len);
		logWarning("AGNSS: 解析失败 长度=%u 魔数=%u 声明=%u",
				static_cast<unsigned>(recv_len), has_magic ? 1u : 0u,
				static_cast<unsigned>(decl));
		agnss_log_recv_preview(s_agnss_recv_buf, recv_len);
		return false;
	}

	logInfo("AGNSS: 获取辅助数据 %u 字节", static_cast<unsigned>(payload_len));
	if (!util_atgm332d_inject_casbin(payload, payload_len, PROD_CONFIG_AGNSS_INJECT_DELAY_MS)) {
		logWarning("AGNSS: 注入失败");
		return false;
	}
	logInfo("AGNSS: 完成");
	return true;
}
