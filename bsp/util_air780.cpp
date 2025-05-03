/*
 * util_air780.cpp
 *
 *  Created on: Feb 10, 2025
 *      Author: IRIS
 */

#include "utilties.h"

//******HAL Family Bucket******//
#include "main.h"

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

#include "time.h"

#include "magic_enum.hpp"
using namespace magic_enum;

#include "BufferedUart.h"

typedef enum : uint8_t
    {
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
    __invalid,
    } rx_content_t;

static struct
    {
    struct
	{
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

    } ATstatus;

typedef struct
    {
    struct
	{
	uint8_t ip[4];
	uint16_t port;
	} conn[3];
    } connInfo_t;

typedef struct
    {
    uint8_t rssi = 0;
    uint8_t ber = 0;
    uint8_t csq = 0;
    time_t time = 0;
    bool connStatus[3] =
	{
	false, false, false
	};
    } status_t;

static bufferedUart *uart = NULL;
static MessageBufferHandle_t portRxBuffer[3] =
    {
    NULL, NULL, NULL
    };
static SemaphoreHandle_t cmdSem = NULL;
static rx_content_t listeningRx;
static status_t status;
static connInfo_t connInfo;

static int __getCsq();
static int __setAutoSleepTimeout(uint32_t time);
static bool __testif(function<bool()> f, bool testPositive = true,
	size_t timeout = portMAX_DELAY, size_t interval = 10);
static bool __sendCmd(size_t optime, rx_content_t rx, const char *fmt, ...);

void util_a780_sleep();
void util_a780_poweron();
void util_a780_poweroff();
void util_a780_reset();
void util_a780_setup();
bool util_a780_waitEutran(size_t timeout = portMAX_DELAY);
int util_a780_connect(util_a780_server_t server = server_main);
int util_a780_disconnect(util_a780_server_t server = server_main);
int util_a780_read(char *buffer, uint16_t len, uint32_t timeout = portMAX_DELAY,
	util_a780_server_t server = server_main);
int util_a780_write(char *buffer, uint16_t len,
	uint32_t timeout = portMAX_DELAY, util_a780_server_t server =
		server_main);
int util_a780_setPort(uint8_t ip[4], uint16_t port, util_a780_server_t server =
	server_main);
bool util_a780_ifConnected(util_a780_server_t server = server_main);

int __getCsq()
    {
    /*return -1 if not powered*/
    if (!ATstatus.setup.powerState)
	return -1;
    /*send command*/
    __sendCmd(100, rx_content_t::CSQ, "AT+CSQ\r\n");
    return 0;
    }

int __setAutoSleepTimeout(uint32_t time)
    {
    /*return -1 if not powered*/
    if (!ATstatus.setup.powerState)
	return -1;
    /*send command*/
    __sendCmd(100, rx_content_t::RTIME, "AT*RTIME=%d\r\n", time);
    return 0;
    }

bool __testif(function<bool()> f, bool testPositive, size_t timeout,
	size_t interval)
    {
    }

bool __sendCmd(size_t optime, rx_content_t rx, const char *fmt, ...)
    {
    }

void util_a780_sleep()
    {
    }

void util_a780_poweron()
    {
    }

void util_a780_poweroff()
    {
    }

void util_a780_reset()
    {
    }

void util_a780_setup()
    {
    /*enable echo*/
    __sendCmd(200, rx_content_t::ATE, "ATE1\r\n");
    /*$notice always disable auto upgrade*/
    __sendCmd(200, rx_content_t::UPGRADE, "AT+UPGRADE=\"AUTO\",0,1\r\n");
    /*wait until gprs attached*/
    __sendCmd(200, rx_content_t::CGATT, "AT+CGATT?\r\n");
    /*set multi link mode*/
    __sendCmd(200, rx_content_t::CIPMUX, "AT+CIPMUX=1\r\n");
    /*set fast send*/
    __sendCmd(200, rx_content_t::CIPQSEND, "AT+CIPQSEND=1\r\n");
    /*launch task*/
    __sendCmd(200, rx_content_t::CSTT, "AT+CSTT\r\n");
    /*activate scene*/
    __sendCmd(200, rx_content_t::CIICR, "AT+CIICR\r\n");

    }

bool util_a780_waitEutran(size_t timeout)
    {
    }

int util_a780_connect(util_a780_server_t server)
    {
    configASSERT(enum_contains<util_a780_server_t>(server));
    /*already connected*/
    if (connectionsStatus[enum_integer<util_a780_server_t>(server)])
	return 0;

    do
	{
	/*connect*/
	__sendCmd(100, rx_content_t::CIPSTART,
		"AT+CIPSTART=%d,\"TCP\",\"%d.%d.%d.%d\",%d\r\n", (int) (server),
		(int) params.server[server].ip[0],
		(int) params.server[server].ip[1],
		(int) params.server[server].ip[2],
		(int) params.server[server].ip[3],
		(int) params.server[server].port);
	/*query connection status*/
	__sendCmd(100, rx_content_t::CIPSTATUS, "AT+CIPSTATUS\r\n");
	}
    while (!__testif([this, server]()
	{
	return status.connStatus[enum_integer<util_a780_server_t>(server)];
	}, true, 3000));
    /*create correspond msgbuffer*/
    if (msgBuffer[enum_integer<util_a780_server_t>(server)] == NULL)
	msgBuffer[enum_integer<util_a780_server_t>(server)] =
		xMessageBufferCreate(256);

    return status.connStatus[enum_integer<util_a780_server_t>(server)] == true ?
	    0 : -1;
    }

int util_a780_disconnect(util_a780_server_t server)
    {
    }

int util_a780_read(char *buffer, uint16_t len, uint32_t timeout,
	util_a780_server_t server)
    {
    }

int util_a780_write(char *buffer, uint16_t len, uint32_t timeout,
	util_a780_server_t server)
    {
    }

int util_a780_setPort(uint8_t ip[4], uint16_t port, util_a780_server_t server)
    {
    }

bool util_a780_ifConnected(util_a780_server_t server)
    {
    }
