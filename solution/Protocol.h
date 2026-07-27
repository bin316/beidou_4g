/*
 * Protocol.h
 *
 *  Created on: Jan 18, 2025
 *      Author: IRIS
 *
 * ------------------------------------------------------------------------------
 * 卫星数量 sats 字段说明
 * ------------------------------------------------------------------------------
 * pb_report 中添加第25字节 sats 字段表示用于定位的卫星数量，来源于 GGA 语句的 numSv，
 * 范围 0~24，0 表示未定位或无效，用于上报数据包中体现定位质量。
 *
 * ------------------------------------------------------------------------------
 * status 定位状态位（bit0）与“上报后置否”
 * ------------------------------------------------------------------------------
 * status 的 bit0 为定位状态位：1 表示本帧 geo[] 为新定位数据，0 表示非新/旧定位数据。
 * 每次成功上报后设备会将该位对应的内部标志置否，下一帧仅在有新 RMC 时才再为 1，
 * 以提升服务器对“是否为新定位数据”判断的准确性。详见 solution/Solution.cpp、
 * bsp/util_atgm332d.cpp、bsp/utilties.h。
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <os_Allocator.hpp>
#include "string_view"

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

typedef enum : uint8_t
    {
    up_report = 0,
    up_runningConfigUpload = 2,
    up_runningConfigResult = 4,
    up_responseSleep = 6,
    up_responseConfigMode = 8,
    up_systemConfigUpload = 10,
    up_systemConfigResult = 12,
    up_responseWorkMode = 14,
    up_firmwareVersionUpload = 16,
    up_locateGeoResult = 18,
    up_locateSwitchResult = 20,
    up_locateSwitchUpload = 22,

    down_uploadReport = 1,
    down_uploadRunningConfig = 3,
    down_configRunning = 5,
    down_sleep = 7,
    down_configMode = 9,
    down_uploadSystemConfig = 11,
    down_configSystem = 13,
    down_configWorkMode = 15,
    down_uploadFirmwareVersion = 17,
    down_configLocateGeo = 19,
    down_configLocateSwitch = 21,
    down_uploadLocateSwitch = 23,

    preserved
    } e_pb_func;

typedef enum : uint8_t
    {
    edit_notEnabled = 0x00,
    edit_enabled = 0x01,

    edit_pwdChangeFailed = 0x00,
    edit_pwdChangeSuccess = 0x01,
    edit_pwdConfirm = 0xf0,

    edit_exit = 0x02

    } e_editMode_result;

//校验域长度 帧长-校验长度
#define CFL(x)	(sizeof(x)-sizeof(uint16_t))

#pragma pack(push, 1)

typedef struct DeviceCode
    {
    //	界桩主代号
    uint8_t major;                 //'T'([0xFF]和[0x00]不可使用)
    //	界桩副代号
    uint8_t minor;                 //'T'([0xFF]和[0x00]不可使用)
    //	界桩序号
    uint16_t index;                //序列号CRC值([0x0000]和[0xFFFF]不可使用)

    /*
     * @func <pb_code = reload>
     *
     * $notice code copy will be successful unless the given value is not legal
     */
    bool operator =(DeviceCode &source)
	{
	/*all 0x00 or all 0xff are invalid device code*/
	if (!(source.index == 0x0000 && source.major == 0x00
		&& source.minor == 0x00)
		&& !(source.index == 0xFFFF && source.major == 0xFF
			&& source.minor == 0xFF))
	    {
	    memcpy(this, &source, sizeof(DeviceCode));
	    return true;
	    }
	return false;
	}

    } pb_code;

typedef struct
    {
    pb_code code;
    uint8_t function; 		//命令功能码
    uint8_t dataLen; 		//数据域长度
    } pb_header;

typedef struct
    {
    uint32_t time; 		//unix时间
    float geo[2]; 		//地理信息，0经度，1纬度
    uint8_t status; 	//工作模式(2)+唤醒源(2)+震动状态(1)+保留位(1)+倾斜状态(1)+定位状态(1)，对应bit7~0
    int16_t acc[3]; 	//x、y、z三周加速度，单位mg
    int16_t angle; 	    //倾角
    uint8_t vbat; 		//电池电压
    int8_t temp; 		//主机温度
    uint8_t csq;        //4G模块信号强度
    uint8_t sats;       //用于定位的卫星数量
    } pb_report; 		//sizeof(report) = 25

typedef struct
    {
    uint8_t item[8];
    } password;

/** 功能码 9 下行：密码 + 意图功能码（dataLen=9） */
typedef struct
    {
    password pwd;
    uint8_t intentFunc; /**< 进模式：5/13/15/19/21；改密：9；退出：必须 0 */
    } pb_configModeReq;

/** 功能码 8 上行：结果 + 意图回显（失败 intent 恒 0；dataLen=2） */
typedef struct
    {
    uint8_t result;
    uint8_t intentFunc;
    } pb_configModeRsp;

typedef struct RunningConfig
    {
//	网络连接成功时的唤醒周期	solution
    uint16_t t0_netGood_wakeup_min;      //5(单位[分钟])
//	网络连接失败时的唤醒周期	solution
    uint16_t t1_netBad_wakeup_min;       //3(单位[分钟])
//	服务器无响应重传超时	solution
    uint8_t t2_serverRsps_timeout_sec;   //3(单位[秒])
//	唤醒后硬计时休眠倒计时（收包不续命）	solution
    uint8_t t3_gnssSearch_sleep_sec;     //100(单位[秒])
//	定位成功休眠倒计时	solution
    uint8_t t4_gnssGood_sleep_sec;       //60(单位[秒])
//	例行唤醒事件休眠倒计时	solution
    uint8_t t6_rtcEvent_sleep_sec;       //60(单位[秒])
//	运动检测消抖延迟  	sc7a20
    uint8_t t5_motionDetect_delay_sec;   //5(单位[秒])
//	定位刷新周期计数		air780
    uint8_t n0_gnssUpdate_wakeup_count;  //60(单位[次])
//	低电压报警低阈值 		analog
    uint8_t n1_vbatAlarm_threshold_volt; //22(单位[0.1V])

    /*
     * @func <pb_runningConfig = reload>
     *
     * $notice running config copy will always be successful
     */
    bool operator =(RunningConfig &source)
	{

	t0_netGood_wakeup_min = source.t0_netGood_wakeup_min;
	t1_netBad_wakeup_min = source.t1_netBad_wakeup_min;
	t2_serverRsps_timeout_sec = source.t2_serverRsps_timeout_sec;
	t3_gnssSearch_sleep_sec = source.t3_gnssSearch_sleep_sec;
	t4_gnssGood_sleep_sec = source.t4_gnssGood_sleep_sec;
	t6_rtcEvent_sleep_sec = source.t6_rtcEvent_sleep_sec;
	t5_motionDetect_delay_sec = source.t5_motionDetect_delay_sec;
	n0_gnssUpdate_wakeup_count = source.n0_gnssUpdate_wakeup_count;
	n1_vbatAlarm_threshold_volt = source.n1_vbatAlarm_threshold_volt;

	return true;
	}
    } pb_runningConfig;
//11

typedef struct SystemConfig
    {
    pb_code code;
    //	传感器检测倒置范围	sc7a20
    uint8_t sensorReverseRange;        //25(单位[度])
//	传感器检测震动阈值	sc7a20
    uint8_t sensorVibrationThreshold;  //2(单位[100mG])
//	备用服务器ip	air780
    uint8_t backupServerIP[4];         //159,132,40,110(注意小端序)
//	备用服务器端口	air780
    uint16_t backupServerPort;         //2000
//	作业服务器ip 	    air780
    uint8_t runServerIP[4];            //159,132,40,110(注意小端序)
//	作业服务器端口	air780
    uint16_t runServerPort;            //2000

    /*
     * @func <pb_systemConfig = reload>
     *
     * $notice device code would not be modified by protocol, it is designed to be set only once
     *          or by the manufacturer
     */
    bool operator =(SystemConfig &source)
	{
	/*device code keep unchanged*/
	//    dest.code = source.code;
	sensorReverseRange = source.sensorReverseRange;
	sensorVibrationThreshold = source.sensorVibrationThreshold;
	backupServerIP[0] = source.backupServerIP[0];
	backupServerIP[1] = source.backupServerIP[1];
	backupServerIP[2] = source.backupServerIP[2];
	backupServerIP[3] = source.backupServerIP[3];
	backupServerPort = source.backupServerPort;
	runServerIP[0] = source.runServerIP[0];
	runServerIP[1] = source.runServerIP[1];
	runServerIP[2] = source.runServerIP[2];
	runServerIP[3] = source.runServerIP[3];
	runServerPort = source.runServerPort;
	//  $notice 倒置角度范围建议最小设置为±8度，小于这个范围，倒置倾角可能无法检测成功
	//    		[comment date--Feb 12, 2025] :这个配置暂时先不做
	return true;
	}
    } pb_systemConfig;

/** 固件版本：年(后两位)/月/当月修订号，均为二进制整数 */
typedef struct
    {
    uint8_t year;
    uint8_t month;
    uint8_t revision;
    } pb_firmwareVersion;

/** 下行 19：写入经纬度 */
typedef struct
    {
    float geo[2]; // [0]经度 [1]纬度
    } pb_locateGeo;

typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	}; //sizeof(prefix) = 3
    struct
	{
	pb_header header; //sizeof(header) = 6
	pb_report report; //sizeof(report) = 25
	uint16_t crc; //sizeof(crc) = 2
	} body;
    } pb_packReport;

typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	};
    struct
	{
	pb_header header;
	pb_systemConfig systemConfig;
	uint16_t crc;
	} body;
    } pb_packSystemConfig;

typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	};
    struct
	{
	pb_header header;
	pb_runningConfig runningConfig;
	uint16_t crc;
	} body;
    } pb_packRunningConfig;

typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	};
    struct
	{
	pb_header header;
	password pwd;
	uint16_t crc;
	} body;
    } pb_packPassword;

typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	};
    struct
	{
	pb_header header;
	uint8_t cmdletOrResponse;
	uint16_t crc;
	} body;
    } pb_packCmdletOrResponse;

/** 功能码 8：配置模式应答（result + intentFunc） */
typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	};
    struct
	{
	pb_header header;
	pb_configModeRsp configMode;
	uint16_t crc;
	} body;
    } pb_packConfigModeRsp;

typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	};
    struct
	{
	pb_header header;
	uint16_t crc;
	} body;
    } pb_packCmdletOrResponseSimple;

/** 上行固件版本回复包：FA×3 + header + year/month/revision + CRC */
typedef struct
    {
    const uint8_t prefix[3] =
	{
	0xFA, 0xFA, 0xFA
	};
    struct
	{
	pb_header header;
	pb_firmwareVersion firmwareVersion;
	uint16_t crc;
	} body;
    } pb_packFirmwareVersion;

#pragma pack(pop)

#endif /* PROTOCOL_H_ */
