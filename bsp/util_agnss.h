/**
 * @file util_agnss.h
 * @brief 中科微 AGNSS：经 AIR780 link2 拉取 CASBIN 并注入北斗
 * @note  接收大缓冲位于 SRAM2（._ram2_area），不占主 RAM
 */
#pragma once

#include <cstddef>
#include <cstdint>

class AIR780EP;

/** 本次唤醒是否已尝试过 AGNSS（成败均只一次） */
bool util_agnss_done_this_wake(void);

/**
 * @brief 经 link2(server_debug) 拉取 CASBIN 并注入；每唤醒最多一次
 * @return true=拉取并注入成功
 */
bool util_agnss_fetch_and_inject_once(AIR780EP *air);

/* AIR780 RX 在 AGNSS 期间将 link2 +RECEIVE 写入 SRAM2 专用缓冲 */
bool util_agnss_rx_begin(void);
void util_agnss_rx_end(void);
bool util_agnss_rx_is_active(void);
size_t util_agnss_rx_total(void);
bool util_agnss_rx_append(const void *data, size_t len);
int util_agnss_rx_pending(void);
void util_agnss_rx_set_pending(int pending);
void util_agnss_rx_consume_pending(size_t n);
bool util_agnss_rx_append_continuation(const void *data, size_t len);
