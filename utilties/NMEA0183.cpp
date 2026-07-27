/*
 * NMEA0183.cpp
 *
 *  Created on: Jan 22, 2025
 *      Author: IRIS
 *
 * ------------------------------------------------------------------------------
 * 卫星数量解析说明
 * ------------------------------------------------------------------------------
 * parseGgaMessage() / parseGGA() 解析 GGA 语句第 8 字段 numSv，写入 GGA_t::satelliteCount，
 * 供 util_atgm332d 更新 status.sats，用于上报数据包。
 */

#include <NMEA0183.h>

#include "NMEA0183.h"
#include "magic_enum.hpp"
#include "magic_enum_utility.hpp"
using namespace magic_enum;

#include "cstring"
#include "stdio.h"

NMEA0183::NMEA0183()
    {
    }

NMEA0183::~NMEA0183()
    {
    }

int NMEA0183::setMessageField(const char *msgField, size_t length)
    {
    if (msgField == NULL || length <= 5)
	return -1;;
    sv = string_view(msgField, length);
    return 0;
    }

int NMEA0183::parseSysType(void)
    {
    if (this->msgView.find("BD") != string_view::npos)
	this->message.systemType = system_type_t::sys_type_bds;
    else if (this->msgView.find("GP") != string_view::npos)
	this->message.systemType = system_type_t::sys_type_gps;
    else if (this->msgView.find("GL") != string_view::npos)
	this->message.systemType = system_type_t::sys_type_glonass;
    else if (this->msgView.find("GN") != string_view::npos)
	this->message.systemType = system_type_t::sys_type_gnss;
    else
	this->message.systemType = system_type_t::sys_type_unknown;

    if (this->message.systemType == system_type_t::sys_type_unknown)
	return -1;
    return 0;
    }

bool NMEA0183::parseRmcMessage()
    {
    /* 必须先确认存在 RMC：find("RMC")==npos 时 rfind("$",npos) 会误匹配其它 $ */
    size_t rmcPos = sv.find("RMC");
    if (rmcPos == string_view::npos)
	return false;
    size_t rmcSentenceBegin = sv.rfind("$", rmcPos);
    if (rmcSentenceBegin == string_view::npos)
	return false;
    size_t rmcSentenceEnd = sv.find("\r\n", rmcSentenceBegin);
    if (rmcSentenceEnd == string_view::npos)
	return false;

    string_view sentence = sv.substr(rmcSentenceBegin,
	    rmcSentenceEnd - rmcSentenceBegin);
    if (!checkSum(sentence))
	return false;
    this->msgView = sentence;
    parseSysType();
    this->message.content = RMC_t();
    size_t comma = msgView.find(",");
    size_t star = msgView.find("*");
    if (comma == string_view::npos || star == string_view::npos || star <= comma + 1)
	return false;
    this->paramView = string_view(msgView.data() + comma + 1, star - comma - 1);
    return parseRMC();
    }

bool NMEA0183::parseGgaMessage()
    {
    /* 查找 GGA 句子起始位置 */
    size_t ggaPos = sv.find("GGA");
    if (ggaPos == string_view::npos)
	return false;
    /* 向前找到该句子的 $ 起始符 */
    size_t ggaSentenceBegin = sv.rfind("$", ggaPos);
    if (ggaSentenceBegin == string_view::npos)
	return false;
    size_t ggaSentenceEnd = sv.find("\r\n", ggaSentenceBegin);
    if (ggaSentenceEnd == string_view::npos)
	ggaSentenceEnd = sv.size();
    string_view sentence = sv.substr(ggaSentenceBegin,
	    ggaSentenceEnd - ggaSentenceBegin);
    if (!checkSum(sentence))
	return false;
    this->msgView = sentence;
    parseSysType();
    this->message.content = GGA_t();
    size_t comma = msgView.find(",");
    size_t star = msgView.find("*");
    if (comma == string_view::npos || star == string_view::npos || star <= comma + 1)
	return false;
    this->paramView = string_view(msgView.data() + comma + 1, star - comma - 1);
    return parseGGA();
    }

bool NMEA0183::parseGGA(void)
    {
    if (this->paramView.data() == NULL)
	return false;
    /* GGA 格式：UTCtime,lat,uLat,lon,uLon,FS,numSv,HDOP,...
     * paramView 不含消息头，numSv 为第 7 个字段，需跳过前 6 个逗号 */
    string_view pv = this->paramView;
    size_t pos = 0;
    for (int i = 0; i < 6; i++)
	{
	pos = pv.find(",", pos);
	if (pos == string_view::npos)
	    return false;
	pos++;
	}
    int numSv = 0;
    if (sscanf(pv.data() + pos, "%d", &numSv) < 1)
	return false;
    if (numSv < 0)
	numSv = 0;
    if (numSv > 24)
	numSv = 24;
    auto gga = getContent<GGA_t>();
    if (gga == nullptr)
	return false;
    gga->satelliteCount = (uint8_t) numSv;
    return true;
    }

/**
 * @brief NMEA 异或校验；缺起始符/星号或越界一律失败
 * @note AGNSS 注入后 RX 常夹杂二进制，不可对 string_view::npos 做下标访问
 */
bool NMEA0183::checkSum(string_view sentence)
    {
    if (sentence.data() == NULL || sentence.empty())
	return false;

    size_t dollar = sentence.find("$");
    size_t star = sentence.find('*');
    if (dollar == string_view::npos || star == string_view::npos)
	return false;
    if (star + 2 >= sentence.size() || star <= dollar + 1)
	return false;

    int chksum_original = 0;
    if (sscanf(sentence.data() + star + 1, "%x", &chksum_original) < 1)
	return false;

    uint8_t chksum_actual = 0;
    for (size_t i = dollar + 1; i < star; i++)
	chksum_actual ^= static_cast<uint8_t>(sentence[i]);

    return chksum_actual == static_cast<uint8_t>(chksum_original);
    }

bool NMEA0183::parseRMC(void)
    {
    if (this->sv.data() == NULL || this->paramView.data() == NULL)
	return false;
    std::string_view pv = this->paramView;
    const char *ff_ = "%f";
    const char *if_ = "%d";
    const char *cf_ = "%c";
    auto rmc = getContent<RMC_t>();
    if (rmc == nullptr)
	return false;

    using nmea_param = variant<float, char, int>;
    struct
	{
	nmea_param param;
	const char *format;
	} params[9] =
	{
	    {
	    0, ff_
	    },
	    {
	    0, cf_
	    },
	    {
	    0, ff_
	    },
	    {
	    0, cf_
	    },
	    {
	    0, ff_
	    },
	    {
	    0, cf_
	    },
	    {
	    0, ff_
	    },
	    {
	    0, ff_
	    },
	    {
	    0, if_
	    }
	};
    /*要从格式中提取的数据*/
    int result = 0;
    for (int i = 0; i < 9; i++)
	{
	if (pv.data() == nullptr || pv.empty())
	    return false;
	if (params[i].format == ff_)
	    {
	    float temp = 0.f;
	    result += sscanf(pv.data(), params[i].format, &temp);
	    params[i].param = temp;
	    }
	else if (params[i].format == cf_)
	    {
	    char temp = 0;
	    result += sscanf(pv.data(), params[i].format, &temp);
	    params[i].param = temp;
	    }
	else if (params[i].format == if_)
	    {
	    int temp = 0;
	    result += sscanf(pv.data(), params[i].format, &temp);
	    params[i].param = temp;
	    }
	/* 末字段后可能无逗号；缺逗号时禁止 pv[npos]（AGNSS 残留/残帧会 HardFault） */
	if (i < 8)
	    {
	    size_t comma = pv.find(",");
	    if (comma == string_view::npos || comma + 1 > pv.size())
		return false;
	    pv = string_view(pv.data() + comma + 1, pv.size() - comma - 1);
	    }
	}
    char valid = get<char>(params[1].param);
    if (valid != 'A')
	return false;

    rmc->dataValid = (get<char>(params[1].param) == 'A');
    rmc->latitude = degreeFormat(get<float>(params[2].param),
	    get<char>(params[3].param));
    rmc->longitude = degreeFormat(get<float>(params[4].param),
	    get<char>(params[5].param));
    rmc->speed = get<float>(params[6].param);
    rmc->course = get<float>(params[7].param);
    rmc->unixTime = timeDateFormat(get<float>(params[0].param),
	    get<int>(params[8].param));

    return true;
    }

time_t NMEA0183::timeDateFormat(float utctime, int date)
    {
    struct tm timeinfo =
	{
	0
	};

    // 提取时间
    int hours = static_cast<int>(utctime / 10000);
    int minutes = static_cast<int>((utctime - hours * 10000) / 100);
    int seconds = static_cast<int>(utctime - hours * 10000 - minutes * 100);

    // 提取日期
    int day = date / 10000;
    int month = (date / 100) % 100;
    int year = date % 100 + 2000; // 假设年份是 2000 年之后

    // 设置时间信息
    timeinfo.tm_year = year - 1900;
    timeinfo.tm_mon = month - 1;
    timeinfo.tm_mday = day;
    timeinfo.tm_hour = hours;
    timeinfo.tm_min = minutes;
    timeinfo.tm_sec = seconds;

    time_t unixTime = mktime(&timeinfo);

    return unixTime;
    }

float NMEA0183::degreeFormat(float raw, char dir)
    {
    float sign = ((dir == 'N') || (dir == 'E')) ? 1.0f : -1.0f;
    int degrees = static_cast<int>(raw / 100);
    float minutes = raw - (degrees * 100);
    return (float) ((degrees + (minutes / 60.0)) * sign);
    }
