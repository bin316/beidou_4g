/*
 * AIR780EP.cpp
 *
 *  Created on: Jan 23, 2025
 *      Author: IRIS
 */

#include <AIR780EP.h>
#include "string"
#include "string_view"
#include "stlAllocator.hpp"

#include "magic_enum.hpp"
#include "magic_enum_utility.hpp"
using namespace magic_enum;

using namespace std;

#include "utilties.h"

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

#include "Xuart.h"

/*
 * @notice the command send process is described
 * 		as below
 * 1. wait the cmd sem
 * 2. send command, set cmd pending flag
 * 3. rx thread keep waiting the coming response
 * 4. response received, clear cmd pending flag(if already set)
 *
 * @notice the process described above shows
 * 		there is something need attention
 * 1. continuous command send would block the thread
 * 		each block will keep until the response received,
 * 		or the timeout reached;
 *
 *
 *
 */

AIR780EP::AIR780EP(UART_HandleTypeDef *huart) {
	this->uart = new Xuart(huart, 600, 600, 600, 600);
	configASSERT(this->uart != nullptr);
	this->uart->open();

	cmdSem = xSemaphoreCreateBinary();
	configASSERT(cmdSem != NULL);
	/*initial sem give*/
	xSemaphoreGive(cmdSem);

	nvm = new NVM(NVM::partition_air780, (uint8_t*) &params,
			(void*) &defaultParams, sizeof(params));

	nvm->load();
	if (nvm->isFactoryDefault()) {
		nvm->restoreDefault();
		nvm->save();
	}

	xTaskCreate(rxThread, "air780 rx", 512, this, osPriorityHigh,
			&this->rxTaskHandle);
	configASSERT(this->rxTaskHandle != NULL);
}

AIR780EP::~AIR780EP() {
//    $todo 实现析构
//    	不过好像也不需要析构
}

bool AIR780EP::connect(air780_server_t server) {
	configASSERT(enum_contains<air780_server_t>(server));
	/*already connected*/
	if (connectionsStatus[enum_integer<air780_server_t>(server)])
		return true;

	do {
		/*connect*/
		sendCmd(100, rx_content_t::CIPSTART,
				"AT+CIPSTART=%d,\"TCP\",\"%d.%d.%d.%d\",%d\r\n", (int) (server),
				(int) params.server[server].ip[0],
				(int) params.server[server].ip[1],
				(int) params.server[server].ip[2],
				(int) params.server[server].ip[3],
				(int) params.server[server].port);
		/*query connection status*/
		sendCmd(100, rx_content_t::CIPSTATUS, "AT+CIPSTATUS\r\n");
	} while (!testif([this, server]() {
		return connectionsStatus[enum_integer<air780_server_t>(server)];
	}, true, 3000));
	/*create correspond msgbuffer*/
	if (msgBuffer[enum_integer<air780_server_t>(server)] == NULL)
		msgBuffer[enum_integer<air780_server_t>(server)] = xMessageBufferCreate(
				256);

	return connectionsStatus[enum_integer<air780_server_t>(server)];
}

bool AIR780EP::disconnect(air780_server_t server) {
	configASSERT(enum_contains<air780_server_t>(server));
	/*already disconnected*/
	if (!connectionsStatus[enum_integer<air780_server_t>(server)])
		return true;

	do {
		/*disconnect*/
		sendCmd(100, rx_content_t::CIPCLOSE, "AT+CIPCLOSE=%d\r\n",
				enum_integer<air780_server_t>(server));
		/*query connection status*/
		sendCmd(100, rx_content_t::CIPSTATUS, "AT+CIPSTATUS\r\n");
	} while (testif([this, server]() {
		return connectionsStatus[enum_integer<air780_server_t>(server)];
	}, false, 3000));
	/*delete correspond msgbuffer*/
	if (msgBuffer[enum_integer<air780_server_t>(server)] != NULL)
		vMessageBufferDelete(msgBuffer[enum_integer<air780_server_t>(server)]);

	return !connectionsStatus[enum_integer<air780_server_t>(server)];
}

void AIR780EP::sleep() {
	sendCmd(100, rx_content_t::CSCLK, "AT+CSCLK=1\r\n");
}

void AIR780EP::poweron() {
//	$todo operate the hardware power state
	if (status.setup.powerState)
		return;
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
	status.setup.powerState = true;
	logInfo("4g module poweron..");
}

void AIR780EP::poweroff() {
//	$todo operate the hardware power state
	if (!status.setup.powerState)
		return;
	/*deactivate scene*/
	sendCmd(100, rx_content_t::CIPSHUT, "AT+CIPSHUT\r\n");
	vTaskDelay(pdMS_TO_TICKS(50));
	status.setup.powerState = false;
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_RESET);
	nvm->save();
}

void AIR780EP::reset() {
	poweroff();
	vTaskDelay(pdMS_TO_TICKS(100));
	poweron();
}

int AIR780EP::read(char *buffer, uint16_t len, uint32_t timeout,
		air780_server_t server) {
	/*return -1 if server not connected*/
	if (!connectionsStatus[enum_integer<air780_server_t>(server)])
		return -1;
	/*read from message buffer*/
	/*$notice only one thread could read the buffer at one time*/
	return (int) xMessageBufferReceive(
			msgBuffer[enum_integer<air780_server_t>(server)], buffer, len,
			timeout);
}

int AIR780EP::write(char *buffer, uint16_t len, uint32_t timeout,
		air780_server_t server) {
	/*return -1 if not powered*/
	if (!status.setup.powerState)
		return -1;
	/*return -1 if server not connected*/
	/*$notice 如果目标网络没有链接，将要发送的消息就会被忽略*/
	if (!connectionsStatus[enum_integer<air780_server_t>(server)])
		return -1;
	status.sendPrompt = false;
	status.dataAccept = false;
	/*send command*/
	sendCmd(100, rx_content_t::CIPSEND, "AT+CIPSEND=%d,%d\r\n",
			enum_integer<air780_server_t>(server), len);
	/*wait for send prompt*/
	testif([this, &server]() {
		if (!this->connectionsStatus[server]) {
			return true;
		}
		return status.sendPrompt;
	}, true, 200, 5);
	/*if the connection lost ignore the rest part of data send*/
	if (!this->connectionsStatus[server])
		return -1;
	/*send data*/
	int sendRet = this->uart->write(buffer, len);
	/*wait for data accept*/
	return 0;
	/*$notice data accept is now ignored*/
//	return testif([this]() {
//		bool __dataAccept = status.dataAccept;
//		if (status.dataAccept) {
//			status.dataAccept = false;
//		}
//		return __dataAccept;
//	}, true, timeout, 2) == true ? sendRet : -1;
}

int AIR780EP::setServer(uint8_t ip[4], uint16_t port, air780_server_t server) {
	configASSERT(enum_contains<air780_server_t>(server));
	params.server[server].ip[0] = ip[0];
	params.server[server].ip[1] = ip[1];
	params.server[server].ip[2] = ip[2];
	params.server[server].ip[3] = ip[3];
	params.server[server].port = port;
	params.server[server].valid = true;
	return 0;
}

int AIR780EP::getCsq() {
	/*return -1 if not powered*/
	if (!status.setup.powerState)
		return -1;
	/*send command*/
	sendCmd(100, rx_content_t::CSQ, "AT+CSQ\r\n");
	return 0;
}

int AIR780EP::setAutoSleepTimeout(uint32_t time) {
	/*return -1 if not powered*/
	if (!status.setup.powerState)
		return -1;
	/*send command*/
	sendCmd(100, rx_content_t::RTIME, "AT*RTIME=%d\r\n", time);
	return 0;
}

#include "map"

void AIR780EP::setup() {
	/*enable echo*/
	sendCmd(200, rx_content_t::ATE, "ATE1\r\n");
	/*$notice always disable auto upgrade*/
	sendCmd(200, rx_content_t::UPGRADE, "AT+UPGRADE=\"AUTO\",0,1\r\n");
	/*wait until gprs attached*/
	sendCmd(200, rx_content_t::CGATT, "AT+CGATT?\r\n");
	/*set multi link mode*/
	sendCmd(200, rx_content_t::CIPMUX, "AT+CIPMUX=1\r\n");
	/*set fast send*/
	sendCmd(200, rx_content_t::CIPQSEND, "AT+CIPQSEND=1\r\n");
	/*launch task*/
	sendCmd(200, rx_content_t::CSTT, "AT+CSTT\r\n");
	/*activate scene*/
	sendCmd(200, rx_content_t::CIICR, "AT+CIICR\r\n");
	/*get csq*/
	sendCmd(200, rx_content_t::CSQ, "AT+CSQ\r\n");
}

uint8_t debug_pend_clear_count = 0;

void AIR780EP::rxThread(void *argument) {
	AIR780EP *pthis = (AIR780EP*) argument;

	char *buffer = (char*) pvPortMalloc(1024);
	configASSERT(buffer != NULL);
	string_view sv(buffer, 1024);
	auto clear = [&buffer]() {
		memset(buffer, 0, 1024);
	};
	static const map<rx_content_t, function<int(void)> > rx_ops_map = { {
			rx_content_t::CREG, [pthis, sv]() -> int {
				/*parse network registration*/
				if (sv.find("+CREG: 0,1") != string_view::npos) {
					pthis->status.setup.gprsAttached = true;
					return 0;
				}
				return -1;
			} }, { rx_content_t::CGATT, [pthis, sv]() -> int {
		/*parse gprs attached*/
		if (sv.find("+CGATT: 1") != string_view::npos) {
			pthis->status.setup.gprsAttached = true;
			return 0;
		}
		return -1;
	} }, { rx_content_t::CIPMUX, [pthis, sv]() -> int {
		/*parse multi link mode*/
		if (sv.find("OK") != string_view::npos) {
			pthis->status.setup.multimode = true;
			return 0;
		}
		return -1;
	} }, { rx_content_t::CIPQSEND, [pthis, sv]() -> int {
		/*parse fast send*/
		if (sv.find("OK") != string_view::npos) {
			pthis->status.setup.fastSend = true;
			return 0;
		}
		return -1;
	} }, { rx_content_t::CSTT, [pthis, sv]() -> int {
		/*parse start task*/
		if (sv.find("OK") != string_view::npos) {
			pthis->status.setup.taskStarted = true;
			return 0;
		}
		return -1;
	} }, { rx_content_t::CIICR, [pthis, sv]() -> int {
		/*parse activate scene*/
		if (sv.find("OK") != string_view::npos) {
			pthis->status.setup.sceneActivated = true;
			return 0;
		}
		return -1;
	} }, { rx_content_t::CIPSEND, [pthis, sv]() -> int {
		if (sv.find(">")) {
			pthis->status.sendPrompt = true;
			return 0;
		}
		return -1;
	} }, { rx_content_t::CSQ, [pthis, sv]() -> int {
		if (sv.find("+CSQ:") != string_view::npos) {
			static int __USED csqpos = sv.find("+CSQ:");

			sscanf(&sv[sv.find("+CSQ:")], "+CSQ: %d,%d", &pthis->status.csq,
					&pthis->status.ber);
			return 0;
		}
		return -1;
	} } };

	loop:

	clear();
	/*tempt to read one byte to block the thread*/
	pthis->uart->read(buffer, 1, portMAX_DELAY);
	pthis->uart->read(buffer + 1, 0xffffffff, portMAX_DELAY);
	sv = string_view(buffer);

	/*listening command receive operation*/
	/*存在已经定义的命令返回对应操作*/
	if (rx_ops_map.find(pthis->listeningRx) != rx_ops_map.end())
		/*操作成功，释放同步信号量，标记invalid*/
		rx_ops_map.at(pthis->listeningRx)();

	xSemaphoreGive(pthis->cmdSem);
	pthis->listeningRx = rx_content_t::__invalid;

	/*passively received message parse*/
	/*network access*/
	if (sv.find("+E_UTRAN") != string_view::npos) {
		pthis->status.setup.eutran = true;
	}
	/*time updated*/
	if (sv.find("+NITZ") != string_view::npos) {
		/*
		 * time update passive message format is shown below, convert to time_t
		 * "+NITZ: 25/02/05,08:33:52+32,0"
		 */
		struct tm timeinfo;
		int timezone = 0;
		sscanf(&sv[sv.find("+NITZ")], "+NITZ: %d/%d/%d,%d:%d:%d+%d",
				&timeinfo.tm_year, &timeinfo.tm_mon, &timeinfo.tm_mday,
				&timeinfo.tm_hour, &timeinfo.tm_min, &timeinfo.tm_sec,
				&timezone);
		timeinfo.tm_isdst = 0;
		timeinfo.tm_year += 100;
		timeinfo.tm_mon -= 1;
		pthis->status.time = mktime(&timeinfo);
		pthis->status.time -= 3600;
		pthis->updateLocalTime();
	}
	if (sv.find("C: ") != string_view::npos) {
		static const char *ipStatePattern[3] = { "C: 0,", "C: 1,", "C: 2," };
		for (uint8_t i = 0; i < 3; i++) {
			// 查找每条连接记录的状态
			size_t pos = sv.find(ipStatePattern[i]);
			if (pos != string_view::npos) {
				// 截取从匹配到的记录开始到换行符之前的内容
				size_t endPos = sv.find('\n', pos);
				if (endPos == string_view::npos) {
					endPos = sv.size();
				}
				string_view connectionState = sv.substr(pos, endPos - pos);
				pthis->connectionsStatus[i] =
						connectionState.find("CONNECTED") != string_view::npos ?
								true : false;
			}
		}
	}

	/*data accept*/
	if (sv.find("DATA ACCEPT") != string_view::npos) {
		pthis->status.dataAccept = true;
	}
	/*data received*/
	if (sv.find("+RECEIVE") != string_view::npos) {
		/*
		 * data received passive message format is shown below
		 * "+RECEIVE,0,5:<CR><LF>hello"
		 */
		int linkNum = 0xff;
		int len = 0;
		sscanf(&sv[sv.find("+RECEIVE")], "+RECEIVE,%d,%d:", &linkNum, &len);
		if (linkNum <= 2 && linkNum >= 0) {
			xMessageBufferSend(pthis->msgBuffer[linkNum],
					&sv[sv.find(":\r\n") + 3], len, portMAX_DELAY);
			util_events_generate(util_event_code_t::message);
		} else {
			/*$debug break here, the linkNum should not be overranged*/

		}
	}
	/*connection closed*/
	if (sv.find("CLOSED") != string_view::npos) {
		/*
		 * connection closed passive message format is shown below
		 * "1, CLOSED"
		 */
		int linkNum = 0xff;
		sscanf(&sv[sv.find("CLOSED") - 3], "%d, CLOSED", &linkNum);
		if (linkNum <= 2 && linkNum >= 0) {
			pthis->connectionsStatus[linkNum] = false;
			//delete the message buffer
			if (pthis->msgBuffer[linkNum] != NULL) {
				vMessageBufferDelete(pthis->msgBuffer[linkNum]);
				pthis->msgBuffer[linkNum] = NULL;
			}
		} else {
			/*$debug break here, the linkNum should not be overranged*/

		}
	}
	goto loop;
}

bool AIR780EP::testif(function<bool()> f, bool testPositive, size_t timeout,
		size_t interval) {
	int times = timeout / interval;
	while (--times && times > 0) {
		vTaskDelay(pdMS_TO_TICKS(interval));
		if (f() == testPositive) {
			return true;
		}
	}
	return false;
}

#include "stdarg.h"

bool AIR780EP::waitEutran(size_t timeout) {
	return this->testif([this]() {
		return this->status.setup.eutran;
	}, true, timeout);
}

bool AIR780EP::sendCmd(size_t optime, rx_content_t rx, const char *fmt, ...) {
	/*need few delay*/
//	vTaskDelay(pdMS_TO_TICKS(50));
	/*wait until last operation is done*/
//	BaseType_t semTake = xSemaphoreTake(this->cmdSem, optime);
//	if (semTake == pdFALSE)
//		return false;
	/*pass va list into print*/
	va_list args;
	va_start(args, fmt);
	this->uart->print(fmt, args);
	va_end(args);
	listeningRx = rx;
	vTaskDelay(pdMS_TO_TICKS(optime));
	/*if last rx is tagged invalid, rx thread need no extra operation also*/
//    if (rx != rx_content_t::__invalid)
//	cmdPending = true;
	/*wait for semephore*/
	return (bool) pdTRUE;
}

void AIR780EP::updateLocalTime(void) {
	static bool updated = false;

	if (updated)
		return;
//    $notice update time only once
	time_t t = status.time;
	util_lowpower_update_rtc(t);
	updated = true;
}
