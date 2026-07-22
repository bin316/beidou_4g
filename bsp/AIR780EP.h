/*
 * AIR780EP.h
 *
 *  Created on: Jan 23, 2025
 *      Author: IRIS
 */

#ifndef AIR780EP_H_
#define AIR780EP_H_

#include "BufferedUart.h"

//******HAL Family Bucket******//
#include "main.h"
#include "usart.h"

#include "utilties.h"

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

#include "time.h"

#include "magic_enum.hpp"
using namespace magic_enum;

#include "os_Allocator.hpp"

#include "NVM.h"

#include "functional"

#include "Xuart.h"

using namespace std;

class AIR780EP: public osAllocator<AIR780EP> {
public:

	typedef enum : uint8_t {
		server_main, server_aux, server_debug
	} air780_server_t;
	typedef struct {
		struct {
			uint8_t ip[4];
			uint16_t port;
			bool valid;
		} server[enum_count<air780_server_t>()];
	} params_t;

	const params_t defaultParams = { { { { 192, 168, 1, 1 }, 65535, false }, { {
			192, 168, 1, 1 }, 65535, false },
			{ { 192, 168, 1, 1 }, 65535, false } }

	};

	AIR780EP(UART_HandleTypeDef *huart = &huart1);

	~AIR780EP();

	/**
	 * @brief 连接 TCP；timeout_ms>0 且 server_debug 时走 AGNSS 短连（对齐 Slope）
	 */
	bool connect(air780_server_t server = server_main, uint32_t timeout_ms = 0);
	bool disconnect(air780_server_t server = server_main);

	/*
	 * @brief sleep the module
	 * 		expect lower power consumption
	 * @notice network keep connected
	 */
	void sleep();
	/*
	 * @brief shutdown the module
	 * 	  	expect lowerest power consumption
	 * $notice all network would be disconnected
	 */
	void poweroff();

	/*
	 * @brief power on the module
	 * 		expect normal operation, but need some time to boot
	 * 		module would be in the default state
	 */
	void poweron();

	/*
	 * @brief reset the module to the default state
	 * */
	void reset();

	/*
	 * @brief setup the module
	 * */
	void setup();

	/*
	 * @brief wait until the eutran is connected
	 */
	bool waitEutran(size_t timeout = portMAX_DELAY);

	/*
	 * @brief read data from specified server
	 * @param buffer buffer to store data
	 * @param len length of the buffer
	 * @param timeout time for waiting
	 * @param server server to read from
	 * $notice it would return -1 if the server is not connected, else return the length of the data read
	 */
	int read(char *buffer, uint16_t len, uint32_t timeout =
	portMAX_DELAY, air780_server_t server = server_main);

	/*
	 * @brief write data to the server
	 * @param buffer buffer to write
	 * @param len length of the buffer
	 * @param server server to write to
	 * $notice it would return -1 if the server is not connected, else return the length of the data written
	 */
	int write(char *buffer, uint16_t len, uint32_t timeout = portMAX_DELAY,
			air780_server_t server = server_main);

	/*
	 * @brief set the network parameters
	 * @param ip ip address
	 * @param port port number
	 * @param server server to set
	 * @return 0 for success, -1 for error
	 * $notice the server is set to server
	 */
	int setServer(uint8_t ip[4], uint16_t port, air780_server_t server =
			server_main);

	const bool getConnectionsStatus(air780_server_t server) const {
		configASSERT(enum_contains<air780_server_t>(server));
		return connectionsStatus[static_cast<unsigned int>(server)];
	}

	struct {
		struct {
			bool powerState = false;
			bool eutran = false;
			bool gprsAttached = false;
			bool multimode = false;
			bool fastSend = false;
			bool taskStarted = false;
			bool sceneActivated = false;
		} setup;
		bool sendPrompt = false;
		bool dataAccept = false;

        uint8_t ber = 0;
        uint8_t csq = 0;
        time_t time = 0;
	} status;
	params_t params;

	typedef enum : uint8_t {
		ATE,
		CREG,
		UPGRADE,
		CGATT,
		CIPMUX,
		CIPQSEND,
		CSTT,
		CIICR,
		CIPSEND,
		CSQ,
		RTIME,
		CIPSHUT,
		CSCLK,
		CIPCLOSE,
		CIPSTART,
		CIPSTATUS,
		CIPGSMLOC,
		__invalid,
	} rx_content_t;

	/** AT+CIPGSMLOC 解析结果 */
	struct LbsResult {
		bool ok = false;
		uint16_t code = 0;
		float latitude = 0.f;
		float longitude = 0.f;
	};

	bool sendCmd(size_t optime, rx_content_t rx, const char *fmt, ...);

	int getCsq();

	/**
	 * @brief 基站定位 AT+CIPGSMLOC（简单一次查询）
	 * @param out 结果；ok=true 表示拿到 lat/lon
	 */
	bool query_lbs(LbsResult *out, uint32_t timeout_ms = 35000u);

public:

private:
	Xuart *uart;

	NVM *nvm = NULL;

	MessageBufferHandle_t msgBuffer[3] = {
	NULL, NULL, NULL };

	int setAutoSleepTimeout(uint32_t time);

	TaskHandle_t rxTaskHandle = NULL;
	static void rxThread(void *argument);


	/*
	 * test lambda function
	 */
	bool testif(function<bool()> f, bool testPositive = true, size_t timeout =
	portMAX_DELAY, size_t interval = 10);

//    bool cmdPending = false;

	void updateLocalTime(void);

	bool connectionsStatus[enum_count<air780_server_t>()] = { false, false,
			false };

	SemaphoreHandle_t cmdSem = NULL;
	/** 互斥 CIPSTART/CIPSEND/CIPCLOSE/CSQ/CIPGSMLOC，避免与 AGNSS link2 交错 AT */
	SemaphoreHandle_t at_mutex_ = NULL;
	rx_content_t listeningRx;

	bool lbs_result_ready_ = false;
	LbsResult lbs_result_{};
};

#endif /* AIR780EP_H_ */
