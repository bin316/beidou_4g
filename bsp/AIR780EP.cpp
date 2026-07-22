/*
 * AIR780EP.cpp
 *
 *  Created on: Jan 23, 2025
 *      Author: IRIS
 */

/*
 * 2026-03 CSQ 信号强度相关修改说明
 *
 * 1. CSQ 解析修正
 *    - 原先使用 sscanf 直接将 "%d" 写入 uint8_t（status.csq / status.ber），存在类型不匹配风险，
 *      可能导致解析失败或值始终为 0。
 *    - 现在先用 int 临时变量接收，再强制转换为 uint8_t 赋给 status.csq 和 status.ber。
 *
 * 2. CSQ 被动解析增强
 *    - 在 rxThread 中，每次从串口读取完成后，无论当前 listeningRx 是什么，
 *      只要接收缓冲中包含字符串 "+CSQ:"，就立即解析并更新 status.csq / status.ber。
 *    - 这样即使 AT 命令状态机不同步，只要模块回了 "+CSQ: x,y"，全局 CSQ 状态都会被及时刷新。
 *
 * 3. getCsq 获取方式调整
 *    - getCsq() 发送 "AT+CSQ\r\n" 后增加适当延时，让 rxThread 有时间接收并解析返回值，
 *      然后直接返回当前解析得到的 status.csq（失败时返回 -1）。
 *    - 上报函数在调用 air->getCsq() 后，使用返回值填充报文中的 csq 字段，确保上报的 csq 与
 *      模块实际返回的信号强度一致。
 */

#include <AIR780EP.h>
#include "PRODUCT_CONFIG.h"
#include "util_agnss.h"
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

namespace {

/** 解析 link2 +RECEIVE；兼容 :\\r\\n / :\\n（对齐 Slope） */
bool agnss_parse_receive(string_view sv, int *data_len, const char **payload,
	size_t *payload_avail)
    {
    int link_num = -1;
    const size_t hdr = sv.find("+RECEIVE");
    if (hdr == string_view::npos || data_len == nullptr || payload == nullptr
	    || payload_avail == nullptr)
	{
	return false;
	}
    if (sscanf(sv.data() + hdr, "+RECEIVE,%d,%d:", &link_num, data_len) < 2)
	{
	return false;
	}
    if (link_num != (int) AIR780EP::server_debug || *data_len <= 0)
	{
	return false;
	}
    const size_t colon = sv.find(':', hdr);
    if (colon == string_view::npos)
	{
	return false;
	}
    size_t off = colon + 1;
    if (off + 1 < sv.size() && sv[off] == '\r' && sv[off + 1] == '\n')
	{
	off += 2;
	}
    else if (off < sv.size() && sv[off] == '\n')
	{
	off += 1;
	}
    *payload = sv.data() + off;
    *payload_avail = (sv.size() > off) ? (sv.size() - off) : 0U;
    return true;
    }

} // namespace

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

AIR780EP::AIR780EP(UART_HandleTypeDef *huart)
    {
    /* RX 放大：AGNSS +RECEIVE/CASBIN 分片需更大串口缓冲（对齐 Slope） */
    this->uart = new Xuart(huart, 600, 1536, 600, 600);
    configASSERT(this->uart != nullptr);
    this->uart->open();

    cmdSem = xSemaphoreCreateBinary();
    configASSERT(cmdSem != NULL);
    /*initial sem give*/
    xSemaphoreGive(cmdSem);

    at_mutex_ = xSemaphoreCreateMutex();
    configASSERT(at_mutex_ != NULL);

    nvm = new NVM(NVM::partition_air780, (uint8_t*) &params,
	    (void*) &defaultParams, sizeof(params));

    nvm->load();
    if (nvm->isFactoryDefault())
	{
	nvm->restoreDefault();
	nvm->save();
	}

    xTaskCreate(rxThread, "air780 rx", 512, this, osPriorityHigh,
	    &this->rxTaskHandle);
    configASSERT(this->rxTaskHandle != NULL);
    }

AIR780EP::~AIR780EP()
    {
//    $todo 实现析构
//    	不过好像也不需要析构
    }

bool AIR780EP::connect(air780_server_t server, uint32_t timeout_ms)
    {
    xSemaphoreTake(at_mutex_, portMAX_DELAY);
    configASSERT(enum_contains<air780_server_t>(server));

    if (connectionsStatus[enum_integer<air780_server_t>(server)])
	{
	xSemaphoreGive(at_mutex_);
	return true;
	}

    if (!params.server[server].valid)
	{
	logWarning("4G: link%d 服务器未配置", (int) server);
	xSemaphoreGive(at_mutex_);
	return false;
	}

    bool ok = false;

    if (server == server_debug && timeout_ms > 0)
	{
	/* AGNSS link2 短连：调用方超时控制，不建 MessageBuffer */
	sendCmd(100, rx_content_t::CIPSTART,
		"AT+CIPSTART=%d,\"TCP\",\"%d.%d.%d.%d\",%d\r\n", (int) server,
		(int) params.server[server].ip[0],
		(int) params.server[server].ip[1],
		(int) params.server[server].ip[2],
		(int) params.server[server].ip[3],
		(int) params.server[server].port);

	const TickType_t deadline = xTaskGetTickCount()
		+ pdMS_TO_TICKS(timeout_ms);
	while (xTaskGetTickCount() < deadline)
	    {
	    if (connectionsStatus[enum_integer<air780_server_t>(server)])
		{
		break;
		}
	    vTaskDelay(pdMS_TO_TICKS(200));
	    util_lowpower_iwdg_feed();
	    }
	ok = connectionsStatus[enum_integer<air780_server_t>(server)];
	if (!ok)
	    {
	    connectionsStatus[enum_integer<air780_server_t>(server)] = false;
	    }
	}
    else
	{
	uint8_t try_times = 3;
	while (try_times--)
	    {
	    sendCmd(100, rx_content_t::CIPSTART,
		    "AT+CIPSTART=%d,\"TCP\",\"%d.%d.%d.%d\",%d\r\n",
		    (int) server, (int) params.server[server].ip[0],
		    (int) params.server[server].ip[1],
		    (int) params.server[server].ip[2],
		    (int) params.server[server].ip[3],
		    (int) params.server[server].port);
	    sendCmd(100, rx_content_t::CIPSTATUS, "AT+CIPSTATUS\r\n");
	    if (testif([this, server]()
		{
		return connectionsStatus[enum_integer<air780_server_t>(server)];
		}, true, 2000))
		{
		break;
		}
	    }
	ok = connectionsStatus[enum_integer<air780_server_t>(server)];
	}

    if (ok && server != server_debug
	    && msgBuffer[enum_integer<air780_server_t>(server)] == NULL)
	{
	msgBuffer[enum_integer<air780_server_t>(server)] = xMessageBufferCreate(
		256);
	configASSERT(
		msgBuffer[enum_integer<air780_server_t>(server)] != NULL);
	}

    if (ok)
	{
	logInfo("4G: link%d TCP已连接", (int) server);
	}

    xSemaphoreGive(at_mutex_);
    return ok;
    }

bool AIR780EP::disconnect(air780_server_t server)
    {
    xSemaphoreTake(at_mutex_, portMAX_DELAY);
    configASSERT(enum_contains<air780_server_t>(server));
    if (!connectionsStatus[enum_integer<air780_server_t>(server)])
	{
	xSemaphoreGive(at_mutex_);
	return true;
	}

    sendCmd(100, rx_content_t::CIPCLOSE, "AT+CIPCLOSE=%d\r\n",
	    enum_integer<air780_server_t>(server));
    sendCmd(100, rx_content_t::CIPSTATUS, "AT+CIPSTATUS\r\n");
    /* 等断开一次即可。勿 while(testif(...,false))：断开成功时会空转死循环 */
    (void) testif([this, server]()
	{
	return connectionsStatus[enum_integer<air780_server_t>(server)];
	}, false, 3000);
    if (connectionsStatus[enum_integer<air780_server_t>(server)])
	{
	connectionsStatus[enum_integer<air780_server_t>(server)] = false;
	}

    if (msgBuffer[enum_integer<air780_server_t>(server)] != NULL)
	{
	vMessageBufferDelete(msgBuffer[enum_integer<air780_server_t>(server)]);
	msgBuffer[enum_integer<air780_server_t>(server)] = NULL;
	}

    xSemaphoreGive(at_mutex_);
    return true;
    }

void AIR780EP::sleep()
    {
    sendCmd(100, rx_content_t::CSCLK, "AT+CSCLK=1\r\n");
    }

void AIR780EP::poweron()
    {
//	$todo operate the hardware power state
    if (status.setup.powerState)
	return;
    HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
    status.setup.powerState = true;
    logInfo("4G模组: 上电");
    }

void AIR780EP::poweroff()
    {
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

void AIR780EP::reset()
    {
    poweroff();
    vTaskDelay(pdMS_TO_TICKS(100));
    poweron();
    }

int AIR780EP::read(char *buffer, uint16_t len, uint32_t timeout,
	air780_server_t server)
    {
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
	air780_server_t server)
    {
    xSemaphoreTake(at_mutex_, portMAX_DELAY);

    if (!status.setup.powerState
	    || !connectionsStatus[enum_integer<air780_server_t>(server)])
	{
	xSemaphoreGive(at_mutex_);
	return -1;
	}

    status.sendPrompt = false;
    status.dataAccept = false;
    sendCmd(100, rx_content_t::CIPSEND, "AT+CIPSEND=%d,%d\r\n",
	    enum_integer<air780_server_t>(server), len);
    const uint32_t wait_ms =
	    (timeout == portMAX_DELAY) ? 3000u :
		    ((timeout < 2000u) ? 2000u : timeout);
    testif([this]()
	{
	return status.sendPrompt || status.dataAccept;
	}, true, wait_ms, 10);

    int sendRet = -1;
    if (connectionsStatus[enum_integer<air780_server_t>(server)])
	{
	sendRet = this->uart->write(buffer, len);
	}

    xSemaphoreGive(at_mutex_);
    return sendRet;
    }

int AIR780EP::setServer(uint8_t ip[4], uint16_t port, air780_server_t server)
    {
    configASSERT(enum_contains<air780_server_t>(server));
    params.server[server].ip[0] = ip[0];
    params.server[server].ip[1] = ip[1];
    params.server[server].ip[2] = ip[2];
    params.server[server].ip[3] = ip[3];
    params.server[server].port = port;
    params.server[server].valid = true;
    return 0;
    }

int AIR780EP::getCsq()
    {
    if (!status.setup.powerState)
	{
	return -1;
	}

    xSemaphoreTake(at_mutex_, portMAX_DELAY);
    sendCmd(300, rx_content_t::CSQ, "AT+CSQ\r\n");
    const int csq = (int) status.csq;
    xSemaphoreGive(at_mutex_);
    return csq;
    }

/**
 * @brief 合宙 AT+CIPGSMLOC 基站定位（每唤醒简单查一次）
 */
bool AIR780EP::query_lbs(LbsResult *out, uint32_t timeout_ms)
    {
    logInfo("LBS: 开始查询");
    if (out == nullptr || !status.setup.powerState)
	{
	return false;
	}
    if (util_agnss_rx_is_active())
	{
	logWarning("LBS: 跳过(AGNSS收包中)");
	return false;
	}

    const int csq = getCsq();
    if (csq < 0 || csq == 99 || csq < (int) PROD_CFG_LBS_MIN_CSQ)
	{
	logWarning("LBS: 跳过 CSQ=%d", csq);
	return false;
	}

    xSemaphoreTake(at_mutex_, portMAX_DELAY);
    lbs_result_ready_ = false;
    lbs_result_ = {};

    sendCmd(200, rx_content_t::CIPGSMLOC, "AT+CIPGSMLOC=1,%u\r\n",
	    (unsigned) PROD_CFG_LBS_PDP_CID);

    uint32_t elapsed = 0;
    constexpr uint32_t kPollMs = 100u;
    while (elapsed < timeout_ms)
	{
	if (util_agnss_rx_is_active())
	    {
	    logWarning("LBS: 中止(AGNSS又激活)");
	    break;
	    }
	if (lbs_result_ready_)
	    {
	    break;
	    }
	vTaskDelay(pdMS_TO_TICKS(kPollMs));
	elapsed += kPollMs;
	util_lowpower_iwdg_feed();
	}

    const bool ready = lbs_result_ready_;
    if (ready)
	{
	*out = lbs_result_;
	if (!out->ok)
	    {
	    logWarning("LBS: 失败 错误码=%u", (unsigned) out->code);
	    }
	else
	    {
	    /* 不用 %f，避免 newlib 浮点格式化撑爆工作线程栈 */
	    logInfo("LBS: 成功 lon_e4=%ld lat_e4=%ld",
		    (long) (out->longitude * 10000.0f),
		    (long) (out->latitude * 10000.0f));
	    }
	}
    else
	{
	logWarning("LBS: 超时 %ums", (unsigned) timeout_ms);
	}
    xSemaphoreGive(at_mutex_);
    return ready && out->ok;
    }

int AIR780EP::setAutoSleepTimeout(uint32_t time)
    {
    /*return -1 if not powered*/
    if (!status.setup.powerState)
	return -1;
    /*send command*/
    sendCmd(100, rx_content_t::RTIME, "AT*RTIME=%d\r\n", time);
    return 0;
    }

#include "map"

void AIR780EP::setup()
    {
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
    /* 接收不加 CRLF，避免污染 AGNSS CASBIN 二进制（对齐 Slope） */
    sendCmd(200, rx_content_t::__invalid, "AT+CIPRXF=1\r\n");
    /*launch task*/
    sendCmd(200, rx_content_t::CSTT, "AT+CSTT\r\n");
    /*activate scene*/
    sendCmd(200, rx_content_t::CIICR, "AT+CIICR\r\n");
    /*get csq*/
    sendCmd(200, rx_content_t::CSQ, "AT+CSQ\r\n");
    }

uint8_t debug_pend_clear_count = 0;

void AIR780EP::rxThread(void *argument)
    {
    AIR780EP *pthis = (AIR780EP*) argument;

    /* 与 Slope 一致：帧缓冲 1536；必须用实际读长建 string_view（CASBIN 含 0x00） */
    constexpr size_t kRxFrameCap = 1536;
    char *buffer = (char*) pvPortMalloc(kRxFrameCap);
    configASSERT(buffer != NULL);
    string_view sv(buffer, kRxFrameCap);
    auto clear = [buffer]()
	{
	memset(buffer, 0, kRxFrameCap);
	};
    static const map<rx_content_t, function<int(void)> > rx_ops_map =
	{
	    {
	    rx_content_t::CREG, [pthis, sv]() -> int
		{
		/*parse network registration*/
		if (sv.find("+CREG: 0,1") != string_view::npos)
		    {
		    pthis->status.setup.gprsAttached = true;
		    return 0;
		    }
		return -1;
		}
	    },
	    {
	    rx_content_t::CGATT, [pthis, sv]() -> int
		{
		/*parse gprs attached*/
		if (sv.find("+CGATT: 1") != string_view::npos)
		    {
		    pthis->status.setup.gprsAttached = true;
		    return 0;
		    }
		return -1;
		}
	    },
	    {
	    rx_content_t::CIPMUX, [pthis, sv]() -> int
		{
		/*parse multi link mode*/
		if (sv.find("OK") != string_view::npos)
		    {
		    pthis->status.setup.multimode = true;
		    return 0;
		    }
		return -1;
		}
	    },
	    {
	    rx_content_t::CIPQSEND, [pthis, sv]() -> int
		{
		/*parse fast send*/
		if (sv.find("OK") != string_view::npos)
		    {
		    pthis->status.setup.fastSend = true;
		    return 0;
		    }
		return -1;
		}
	    },
	    {
	    rx_content_t::CSTT, [pthis, sv]() -> int
		{
		/*parse start task*/
		if (sv.find("OK") != string_view::npos)
		    {
		    pthis->status.setup.taskStarted = true;
		    return 0;
		    }
		return -1;
		}
	    },
	    {
	    rx_content_t::CIICR, [pthis, sv]() -> int
		{
		/*parse activate scene*/
		if (sv.find("OK") != string_view::npos)
		    {
		    pthis->status.setup.sceneActivated = true;
		    return 0;
		    }
		return -1;
		}
	    },
	    {
	    rx_content_t::CIPSEND, [pthis, sv]() -> int
		{
		if (sv.find(">"))
		    {
		    pthis->status.sendPrompt = true;
		    return 0;
		    }
		return -1;
		}
	    },
	    {
		rx_content_t::CSQ, [pthis, sv]() -> int
		{
			if (sv.find("+CSQ:") != string_view::npos)
			{
				int csq_val = 0;
				int ber_val = 0;
				sscanf(&sv[sv.find("+CSQ:")], "+CSQ: %d,%d",
					&csq_val, &ber_val);
				pthis->status.csq = (uint8_t) csq_val;
				pthis->status.ber = (uint8_t) ber_val;
				return 0;
			}
			return -1;
		}
	    },
	    {
		rx_content_t::CIPGSMLOC, [pthis, sv]() -> int
		{
		/* URC 已在下方统一解析；此处仅释放 sendCmd 等待 */
		(void) pthis;
		(void) sv;
		return 0;
		}
	    }
	};

    loop:

    clear();
    /* 先堵 1 字节；第二段无限等，避免 CASBIN/+RECEIVE 分片间隔>50ms 被截断 */
    const int n1 = pthis->uart->read(buffer, 1, portMAX_DELAY);
    if (n1 <= 0)
	{
	goto loop;
	}
    const int n2 = pthis->uart->read(buffer + n1,
	    kRxFrameCap - static_cast<size_t>(n1), portMAX_DELAY);
    const size_t rx_len = static_cast<size_t>(n1)
	    + ((n2 > 0) ? static_cast<size_t>(n2) : 0U);
    sv = string_view(buffer, rx_len);

    /*无论是否是当前等待的命令，只要收到 +CSQ: 就解析一次，保证 csq 能被更新*/
    if (sv.find("+CSQ:") != string_view::npos)
	{
	int csq_val = 0;
	int ber_val = 0;
	sscanf(&sv[sv.find("+CSQ:")], "+CSQ: %d,%d", &csq_val, &ber_val);
	pthis->status.csq = (uint8_t) csq_val;
	pthis->status.ber = (uint8_t) ber_val;
	}

    /* +CIPGSMLOC: 成功为 code,lat,lon；失败常为单 code */
    if (sv.find("+CIPGSMLOC:") != string_view::npos)
	{
	unsigned code = 65535u;
	float lat = 0.f;
	float lon = 0.f;
	const int n = sscanf(&sv[sv.find("+CIPGSMLOC:")],
		"+CIPGSMLOC: %u,%f,%f", &code, &lat, &lon);
	if (n >= 3)
	    {
	    pthis->lbs_result_.code = (uint16_t) code;
	    pthis->lbs_result_.latitude = lat;
	    pthis->lbs_result_.longitude = lon;
	    pthis->lbs_result_.ok = (code == 0u);
	    pthis->lbs_result_ready_ = true;
	    }
	else if (n >= 1)
	    {
	    pthis->lbs_result_.code = (uint16_t) code;
	    pthis->lbs_result_.ok = false;
	    pthis->lbs_result_ready_ = true;
	    }
	}

    /*listening command receive operation*/
    /*存在已经定义的命令返回对应操作*/
    if (rx_ops_map.find(pthis->listeningRx) != rx_ops_map.end())
	/*操作成功，释放同步信号量，标记invalid*/
	rx_ops_map.at(pthis->listeningRx)();

    xSemaphoreGive(pthis->cmdSem);
    pthis->listeningRx = rx_content_t::__invalid;

    /*passively received message parse*/
    /*network access*/
    if (sv.find("+E_UTRAN") != string_view::npos)
	{
	pthis->status.setup.eutran = true;
	}
    /*time updated*/
    if (sv.find("+NITZ") != string_view::npos)
	{
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
    if (sv.find("C: ") != string_view::npos)
	{
	/* AGNSS 二进制续传可能碰巧含 "C: n,"，勿据此改连接态 */
	const bool agnss_bin = util_agnss_rx_is_active()
		&& util_agnss_rx_pending() > 0;
	if (!agnss_bin)
	    {
	    static const char *ipStatePattern[3] =
		{
		"C: 0,", "C: 1,", "C: 2,"
		};
	    for (uint8_t i = 0; i < 3; i++)
		{
		size_t pos = sv.find(ipStatePattern[i]);
		if (pos != string_view::npos)
		    {
		    size_t endPos = sv.find('\n', pos);
		    if (endPos == string_view::npos)
			{
			endPos = sv.size();
			}
		    string_view connectionState = sv.substr(pos, endPos - pos);
		    pthis->connectionsStatus[i] =
			    connectionState.find("CONNECTED")
				    != string_view::npos;
		    }
		}
	    }
	}

    if (sv.find("DATA ACCEPT") != string_view::npos)
	{
	pthis->status.dataAccept = true;
	}
    if (sv.find('>') != string_view::npos)
	{
	pthis->status.sendPrompt = true;
	}

    for (uint8_t i = 0; i < 3; i++)
	{
	char ok_pat[20];
	snprintf(ok_pat, sizeof(ok_pat), "%u, CONNECT OK", i);
	if (sv.find(ok_pat) != string_view::npos)
	    {
	    pthis->connectionsStatus[i] = true;
	    }
	}
    if (sv.find("+CIPSTART:") != string_view::npos)
	{
	int linkNum = 0xff;
	int result = 0xff;
	const size_t pos = sv.find("+CIPSTART:");
	sscanf(sv.data() + pos, "+CIPSTART: %d,%d", &linkNum, &result);
	if (linkNum >= 0 && linkNum <= 2)
	    {
	    pthis->connectionsStatus[linkNum] = (result == 0);
	    }
	}

    if (sv.find("+RECEIVE") != string_view::npos)
	{
	int linkNum = 0xff;
	int len = 0;
	const size_t recv_pos = sv.find("+RECEIVE");
	if (sscanf(sv.data() + recv_pos, "+RECEIVE,%d,%d:", &linkNum, &len)
		>= 2 && linkNum >= 0 && linkNum <= 2 && len > 0)
	    {
	    if (linkNum == (int) AIR780EP::server_debug
		    && util_agnss_rx_is_active())
		{
		const char *payload = nullptr;
		size_t payload_avail = 0;
		if (agnss_parse_receive(sv, &len, &payload, &payload_avail))
		    {
		    const size_t copy_len =
			    ((size_t) len < payload_avail) ?
				    (size_t) len : payload_avail;
		    if (copy_len > 0)
			{
			util_agnss_rx_append(payload, copy_len);
			}
		    util_agnss_rx_set_pending(len - (int) copy_len);
		    }
		}
	    else if (pthis->msgBuffer[linkNum] != NULL)
		{
		const size_t hdr = sv.find(":\r\n");
		const char *payload =
			(hdr != string_view::npos) ? (&sv[hdr] + 3) : nullptr;
		if (payload != nullptr)
		    {
		    xMessageBufferSend(pthis->msgBuffer[linkNum], payload, len,
			    portMAX_DELAY);
		    util_events_generate(util_event_code_t::message);
		    }
		}
	    }
	}
    else if (util_agnss_rx_is_active() && util_agnss_rx_pending() > 0
	    && sv.find("+RECEIVE") == string_view::npos
	    && sv.find("AT+") == string_view::npos)
	{
	/* AGNSS 二进制续传帧（无 URC 头） */
	util_agnss_rx_append_continuation(sv.data(), sv.size());
	}

    if (sv.find("CLOSED") != string_view::npos)
	{
	/* 仅认 "n, CLOSED" 短 URC；AGNSS 二进制中的 CLOSED 子串忽略 */
	const size_t closed_pos = sv.find("CLOSED");
	const bool looks_urc = (closed_pos >= 3) && (sv.size() < 64)
		&& (sv[closed_pos - 2] == ',') && (sv[closed_pos - 1] == ' ');
	if (looks_urc)
	    {
	    int linkNum = 0xff;
	    sscanf(&sv[closed_pos - 3], "%d, CLOSED", &linkNum);
	    if (linkNum <= 2 && linkNum >= 0)
		{
		pthis->connectionsStatus[linkNum] = false;
		if (pthis->msgBuffer[linkNum] != NULL)
		    {
		    vMessageBufferDelete(pthis->msgBuffer[linkNum]);
		    pthis->msgBuffer[linkNum] = NULL;
		    }
		}
	    }
	}
    goto loop;
    }

bool AIR780EP::testif(function<bool()> f, bool testPositive, size_t timeout,
	size_t interval)
    {
    int times = timeout / interval;
    while (--times && times > 0)
	{
	vTaskDelay(pdMS_TO_TICKS(interval));
	if (f() == testPositive)
	    {
	    return true;
	    }
	}
    return false;
    }

#include "stdarg.h"

bool AIR780EP::waitEutran(size_t timeout)
    {
    return this->testif([this]()
	{
	return this->status.setup.eutran;
	}, true, timeout);
    }

bool AIR780EP::sendCmd(size_t optime, rx_content_t rx, const char *fmt, ...)
    {
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

void AIR780EP::updateLocalTime(void)
    {
    static bool updated = false;

    if (updated)
	return;
//    $notice update time only once
    time_t t = status.time;
    util_lowpower_update_rtc(t);
    updated = true;
    }
