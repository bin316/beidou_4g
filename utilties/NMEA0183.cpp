/*
 * NMEA0183.cpp
 *
 *  Created on: Jan 22, 2025
 *      Author: IRIS
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
    /*将sv的范围限定到rmc句子内，如果不存在，返回-1*/

    size_t rmcSentenceBegin = sv.rfind("$", sv.find("RMC"));
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
    this->paramView = string_view(&msgView[msgView.find(",") + 1],
	    msgView.find("*") - msgView.find(",") - 1);
    return parseRMC();
    }

bool NMEA0183::checkSum(string_view sentence)
    {
    /*assuming that sentence has been limited after $ and before<CR><LF>*/
    if (sentence.data() == NULL)
	return false;

    int chksum_original = 0;
    uint8_t chksum_actual = 0;

    /*find position*/
    const char *chksumPos = (&sentence[sentence.find('*')]);
    if (chksumPos == NULL)
	return false;
    chksumPos++;
    /*parse check sum*/
    sscanf(chksumPos, "%x", &chksum_original);

    /*calculate check sum*/
    for (size_t i = sentence.find("$") + 1; i < sentence.find("*"); i++)
	chksum_actual ^= sentence[i];

    if (chksum_actual == chksum_original)
	return true;
    return false;
    }

bool NMEA0183::parseRMC(void)
    {
    if (this->sv.data() == NULL)
	return false;
    std::string_view pv = this->paramView;
    const char *ff_ = "%f";
    const char *if_ = "%d";
    const char *cf_ = "%c";
    auto rmc = getContent<RMC_t>();

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
	if (params[i].format == ff_)
	    {
	    float temp;
	    result += sscanf(pv.data(), params[i].format, &temp);
	    params[i].param = temp;
	    }
	else if (params[i].format == cf_)
	    {
	    char temp;
	    result += sscanf(pv.data(), params[i].format, &temp);
	    params[i].param = temp;
	    }
	else if (params[i].format == if_)
	    {
	    int temp;
	    result += sscanf(pv.data(), params[i].format, &temp);
	    params[i].param = temp;
	    }
	pv = string_view(&pv[pv.find(",") + 1], pv.size() - pv.find(",") - 1);
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
