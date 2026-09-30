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

/* 协议最大帧不足 266B；留出连续下行突发的排队空间，且 RX 任务绝不等待消费者。 */
constexpr size_t kBusinessRxStreamSize = 1024u;

/** 解析 link2 +RECEIVE；兼容 :\\r\\n / :\\n（对齐 Slope） */
/** 解析任意链路 +RECEIVE 头；调用方保证 sv 由 NUL 结尾的暂存区构造。 */
bool parse_receive_urc(string_view sv, int *link_num, int *data_len,
	const char **payload, size_t *payload_avail)
    {
    if (link_num == nullptr || data_len == nullptr || payload == nullptr
	    || payload_avail == nullptr
	    || sscanf(sv.data(), "+RECEIVE,%d,%d:", link_num, data_len) < 2
	    || *link_num < 0 || *link_num >= 3 || *data_len <= 0)
	{
	return false;
	}
    const size_t colon = sv.find(':');
    if (colon == string_view::npos)
	{
	return false;
	}
    size_t offset = colon + 1u;
    if (offset + 1u < sv.size() && sv[offset] == '\r'
	    && sv[offset + 1u] == '\n')
	{
	offset += 2u;
	}
    else if (offset < sv.size() && sv[offset] == '\n')
	{
	++offset;
	}
    *payload = sv.data() + offset;
    *payload_avail = sv.size() - offset;
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

    xTaskCreate(rxThread, "air780 rx", 1024, this, osPriorityHigh,
	    &this->rxTaskHandle);
    configASSERT(this->rxTaskHandle != NULL);
    }

AIR780EP::~AIR780EP()
    {
//    $todo 实现析构
//    	不过好像也不需要析构
    }

bool AIR780EP::connect(air780_server_t server, uint32_t timeout_ms,
	uint8_t attempts, uint32_t retry_interval_ms)
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

    /* 先建接收流；CONNECT OK 后服务器可立即下发，不能等确认后才创建。 */
    if (server != server_debug
	    && msgBuffer[enum_integer<air780_server_t>(server)] == NULL)
	{
	msgBuffer[enum_integer<air780_server_t>(server)] = xStreamBufferCreate(
		kBusinessRxStreamSize, 1u);
	configASSERT(msgBuffer[enum_integer<air780_server_t>(server)] != NULL);
	}

    if (server == server_debug && timeout_ms > 0)
	{
	/* AGNSS link2 短连：调用方超时控制，不建业务接收流。 */
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
	const uint8_t try_times = (attempts == 0u) ? 1u : attempts;
	const uint32_t attempt_timeout_ms =
		(timeout_ms == 0u) ? 2000u : timeout_ms;
	for (uint8_t try_index = 0; try_index < try_times; ++try_index)
	    {
	    connectionsStatus[enum_integer<air780_server_t>(server)] = false;
	    connectionTerminal[enum_integer<air780_server_t>(server)] = false;
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
		const size_t index = enum_integer<air780_server_t>(server);
		return connectionTerminal[index] || connectionsStatus[index];
		}, true, attempt_timeout_ms)
		&& connectionsStatus[enum_integer<air780_server_t>(server)])
		{
		break;
		}
	    /* 超时后仍可能迟到 CONNECT OK：下一次 CIPSTART 前先关闭并等状态回落。 */
	    connectionCloseObserved[enum_integer<air780_server_t>(server)] = false;
	    sendCmd(100, rx_content_t::CIPCLOSE, "AT+CIPCLOSE=%d\r\n",
		    enum_integer<air780_server_t>(server));
	    sendCmd(100, rx_content_t::CIPSTATUS, "AT+CIPSTATUS\r\n");
	    (void) testif([this, server]()
		{
		const size_t index = enum_integer<air780_server_t>(server);
		return connectionCloseObserved[index] && !connectionsStatus[index];
		}, true, 500);
	    connectionsStatus[enum_integer<air780_server_t>(server)] = false;
	    util_lowpower_iwdg_feed();
	    if (try_index + 1u < try_times && retry_interval_ms > 0u)
		{
		vTaskDelay(pdMS_TO_TICKS(retry_interval_ms));
		util_lowpower_iwdg_feed();
		}
	    }
	ok = connectionsStatus[enum_integer<air780_server_t>(server)];
	}

    if (ok)
	{
	logInfo("4G: link%d TCP已连接", (int) server);
	}

    xSemaphoreGive(at_mutex_);
    return ok;
    }

bool AIR780EP::disconnect(air780_server_t server, bool force)
    {
    xSemaphoreTake(at_mutex_, portMAX_DELAY);
    configASSERT(enum_contains<air780_server_t>(server));
    if (!connectionsStatus[enum_integer<air780_server_t>(server)] && !force)
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

    /* RX 与 Solution 分属两个任务，断链不删除句柄以免 read() 与 delete 并发。
     * 下一次会话复用同一流并清掉旧端点残留字节。 */
    if (server != server_debug
	    && msgBuffer[enum_integer<air780_server_t>(server)] != NULL)
	{
	xStreamBufferReset(msgBuffer[enum_integer<air780_server_t>(server)]);
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
    /* CLOSED 可与已入队的下行相邻；断链后仍允许 Solution 取完现有字节。 */
    if (msgBuffer[enum_integer<air780_server_t>(server)] == NULL)
	return -1;
    /* 业务 TCP 本质是字节流；Solution 负责跨 read() 的完整帧组装。 */
    return (int) xStreamBufferReceive(
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
    /* 东八区：NITZ 上报本地时，与 TZ=UTC-8 + mktime 一致（对齐 Inclination） */
    sendCmd(200, rx_content_t::__invalid, "AT+NITZDISSET=\"e\",8\r\n");
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
    /* +RECEIVE 的头和载荷都可能跨 UART 批次；静态区避免挤占 RX 任务栈。 */
    static char receive_staging[kRxFrameCap * 2u + 1u] = {};
    size_t receive_staging_len = 0u;
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
    /*time updated：必须用 "+NITZ:"，勿用 "+NITZ"（会误匹配 +NITZDISSET 应答） */
    if (sv.find("+NITZ:") != string_view::npos)
	{
	/*
	 * "+NITZ: 25/02/05,08:33:52+32,0"（±tz 为 1/4 小时；末尾 %d 兼容 +32/-32）
	 */
	struct tm timeinfo =
	    {
	    };
	int timezone = 0;
	const size_t nitz_pos = sv.find("+NITZ:");
	const int n = sscanf(sv.data() + nitz_pos,
		"+NITZ: %d/%d/%d,%d:%d:%d%d", &timeinfo.tm_year,
		&timeinfo.tm_mon, &timeinfo.tm_mday, &timeinfo.tm_hour,
		&timeinfo.tm_min, &timeinfo.tm_sec, &timezone);
	if (n >= 6)
	    {
	    timeinfo.tm_isdst = 0;
	    timeinfo.tm_year += 100;
	    timeinfo.tm_mon -= 1;
	    /* NITZDISSET 东八区本地时 + TZ=UTC-8 → mktime 得 unix */
	    const time_t unix_ts = mktime(&timeinfo);
	    if (unix_ts != (time_t) -1)
		{
		pthis->status.time = unix_ts;
		pthis->updateLocalTime();
		}
	    else
		{
		logWarning("4G: NITZ mktime失败 yy=%d mon=%d",
			timeinfo.tm_year, timeinfo.tm_mon);
		}
	    }
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
	    if (connectionState.find("CLOSED") != string_view::npos
		    || connectionState.find("INITIAL") != string_view::npos)
		{
		pthis->connectionCloseObserved[i] = true;
		}
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
	char ok_compact_pat[20];
	char fail_pat[24];
	char fail_compact_pat[24];
	snprintf(ok_pat, sizeof(ok_pat), "%u, CONNECT OK", i);
	snprintf(ok_compact_pat, sizeof(ok_compact_pat), "%u,CONNECT OK", i);
	snprintf(fail_pat, sizeof(fail_pat), "%u, CONNECT FAIL", i);
	snprintf(fail_compact_pat, sizeof(fail_compact_pat), "%u,CONNECT FAIL", i);
	if (sv.find(ok_pat) != string_view::npos
		|| sv.find(ok_compact_pat) != string_view::npos)
	    {
	    pthis->connectionsStatus[i] = true;
	    pthis->connectionTerminal[i] = true;
	    }
	if (sv.find(fail_pat) != string_view::npos
		|| sv.find(fail_compact_pat) != string_view::npos)
	    {
	    pthis->connectionsStatus[i] = false;
	    pthis->connectionTerminal[i] = true;
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
	    pthis->connectionTerminal[linkNum] = true;
	    }
	}


    bool saw_receive = false;
    if (rx_len > sizeof(receive_staging) - 1u - receive_staging_len)
	{
	logWarning("4G_RX: 接收暂存区溢出，丢弃未完成 +RECEIVE");
	receive_staging_len = 0u;
	}
    memcpy(receive_staging + receive_staging_len, sv.data(), rx_len);
    receive_staging_len += rx_len;
    receive_staging[receive_staging_len] = '\0';

    while (receive_staging_len > 0u)
	{
	string_view receive_view(receive_staging, receive_staging_len);
	const size_t recv_pos = receive_view.find("+RECEIVE");
	if (recv_pos == string_view::npos)
	    {
	    /* 仅留下可能构成下一批 +RECEIVE 前缀的尾巴。 */
	    const size_t keep = (receive_staging_len < sizeof("+RECEIVE") - 1u) ?
		    receive_staging_len : sizeof("+RECEIVE") - 2u;
	    if (keep > 0u)
		{
		memmove(receive_staging,
			receive_staging + receive_staging_len - keep, keep);
		}
	    receive_staging_len = keep;
	    receive_staging[receive_staging_len] = '\0';
	    break;
	    }
	saw_receive = true;
	if (recv_pos > 0u)
	    {
	    memmove(receive_staging, receive_staging + recv_pos,
		    receive_staging_len - recv_pos);
	    receive_staging_len -= recv_pos;
	    receive_staging[receive_staging_len] = '\0';
	    continue;
	    }

	int linkNum = -1;
	int len = 0;
	const char *payload = nullptr;
	size_t payload_avail = 0u;
	if (!parse_receive_urc(receive_view, &linkNum, &len, &payload,
		&payload_avail))
	    {
	    /* 头不完整时等待下一批；格式损坏才丢 1 字节重新同步。 */
	    if (receive_view.find(':') == string_view::npos)
		{
		break;
		}
	    memmove(receive_staging, receive_staging + 1u,
		    receive_staging_len - 1u);
	    --receive_staging_len;
	    receive_staging[receive_staging_len] = '\0';
	    continue;
	    }

	const size_t declared_len = static_cast<size_t>(len);
	if (linkNum == static_cast<int>(AIR780EP::server_debug)
		&& util_agnss_rx_is_active())
	    {
	    const size_t copy_len = (payload_avail < declared_len) ?
		    payload_avail : declared_len;
	    if (copy_len > 0u)
		{
		util_agnss_rx_append(payload, copy_len);
		}
	    util_agnss_rx_set_pending(len - static_cast<int>(copy_len));
	    const size_t consumed = static_cast<size_t>(payload - receive_staging)
		    + copy_len;
	    memmove(receive_staging, receive_staging + consumed,
		    receive_staging_len - consumed);
	    receive_staging_len -= consumed;
	    receive_staging[receive_staging_len] = '\0';
	    continue;
	    }
	if (payload_avail < declared_len)
	    {
	    break;
	    }
	if (pthis->msgBuffer[linkNum] != NULL)
	    {
	    const size_t space = xStreamBufferSpacesAvailable(
		    pthis->msgBuffer[linkNum]);
	    if (space < declared_len)
		{
		logWarning("4G_RX: link%d 业务流不足，丢弃 %u 字节", linkNum,
			(unsigned) declared_len);
		}
	    else if (xStreamBufferSend(pthis->msgBuffer[linkNum], payload,
		    declared_len, 0u) == declared_len)
		{
		/* 事件仅唤醒 Solution；队列满时字节仍留在 StreamBuffer 等轮询。 */
		(void) util_events_generate_noblock(util_event_code_t::message);
		}
	    else
		{
		logWarning("4G_RX: link%d 业务流写入失败", linkNum);
		}
	    }
	const size_t consumed = static_cast<size_t>(payload - receive_staging)
	    + declared_len;
	memmove(receive_staging, receive_staging + consumed,
	    receive_staging_len - consumed);
	receive_staging_len -= consumed;
	receive_staging[receive_staging_len] = '\0';
	}
    if (util_agnss_rx_is_active() && util_agnss_rx_pending() > 0
	    && !saw_receive && sv.find("AT+") == string_view::npos)
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
		pthis->connectionCloseObserved[linkNum] = true;
		/* 接收流在 AIR780EP 生命周期内保持，避免与 Solution::read 并发 delete。 */
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
    /* 仅成功写入 RTC 后置位，避免 NITZ 解析失败后永久跳过 */
    if (util_lowpower_update_rtc(status.time) == 0)
	{
	updated = true;
	}
    }
