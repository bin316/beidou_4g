#pragma once
#include <stdint.h>

#include "variant"
#include "string_view"
#include "map"

#include "functional"
#include "ctime"

using namespace std;

class NMEA0183 {
public:
	typedef enum : uint8_t {
		parse_fine,
		parse_fail,
		parse_indicate_invalid
	} parse_state_t;

	typedef enum : uint8_t {
		sys_type_bds,
		sys_type_gps,
		sys_type_glonass,
		sys_type_gnss,
		sys_type_custom,
		sys_type_unknown
	} system_type_t;

	typedef enum : uint8_t {
		msg_type_gga,
		msg_type_gll,
		msg_type_gsa,
		msg_type_gsv,
		msg_type_rmc,
		msg_type_vtg,
		msg_type_gst,
		msg_type_zda,
		msg_type_ant,
		msg_type_lps,
		msg_type_dhv,
		msg_type_utc,
		msg_type_txt
	} message_type_t;

	typedef enum : uint8_t {
		N, S
	} orien_lat_t;

	typedef enum : uint8_t {
		E, W
	} orien_lon_t;

	/*------NEMA0183 MESSAGE CONTENT STRUCTURES------*/

	/*
	 GGA：接收机时间、位置及定位相关的数据
	 TYPE：output
	 FORMAT：$--GGA,UTCtime,lat,uLat,lon,uLon,FS,numSv,HDOP,msl,uMsl,sep,uSep,diffAge,diffSta*CS<CR><LF>
	 EXAMPLE：$GPGGA,235316.000,2959.9925,S,12000.0090,E,1,06,1.21,62.77,M,0.00,M,,*7B
	 DETAIL：
	 1	 $--GGA				字符串							消息ID，GGA语句头，’--‘为系统标识															存储格式
	 2	 UTCtime			hhmmss.sss				当前定位的UTC时间																							uint32_t
	 3   lat          ddmm.mmmm					纬度，前2字符表示度，后面的字符表示分															float
	 4   uLat         字符								纬度方向：N-北，S-南																						enum:uint8_t
	 5   lon          dddmm.mmm					经度，前3字符表示度，后面的字符表示分															float
	 6   uLon         字符								经度方向：E-东，W-西																						enum:uint8_t
	 7   FS           数值								指示当前定位质量（备注[1]），该字段不应为空												uint8_t
	 8   numSv        数值								用于定位的卫星数目，00~24																				uint8_t
	 9   HDOP         数值								水平精度因子（HDOP）																						float
	 10  msl          数值								海拔高度，即接收机天线相对于大地水准面的高度												float
	 11  uMsl         字符								高度单位，米，固定字符M																					float
	 12  sep          数值								参考椭球面与大地水准面之间的距离，“-”表示大地水准面低于参考椭球面			float
	 13  uSep         字符								高度单位，米，固定字符M																					char
	 14  diffAge      数值								差分修正的数据龄期，未使用DGPS时该域为空														float
	 15  diffSta      数值								差分参考站的ID																									uint8_t
	 16  CS           16进制数值					校验和，$和*之间（不包括$和*）所有字符的异或结果										uint8_t
	 17  <CR><LF>     字符								回车与换行符
	 */
	typedef enum : uint8_t {
		fix_invalid = 0, fix_SPS = 1, fix_approximate = 6,
	} fixStatus_t;
	typedef struct {
		uint32_t utcTime;        // 当前定位的UTC时间
		float latitude;         // 纬度
		orien_lat_t latitudeDirection; // 纬度方向
		float longitude;        // 经度
		orien_lon_t longitudeDirection; // 经度方向
		fixStatus_t fixStatus;      // 定位质量
		uint8_t satelliteCount;  // 用于定位的卫星数目
		float horizontalDilution; // 水平精度因子
		float altitude;         // 海拔高度
		char altitudeUnit;     // 高度单位
		float separation;       // 参考椭球面与大地水准面之间的距离
		char separationUnit;    // 高度单位
		float differentialAge;  // 差分修正的数据龄期
		uint8_t differentialStationID; // 差分参考站的ID
	} GGA_t;

	/*
	 // 信息 GLL
	 // 描述 纬度、经度、定位时间与定位状态等信息。
	 // 类型 输出
	 // 格式 $--GLL, lat, uLat, lon, uLon, UTCtime, valid, mode* CS<CR><LF>
	 // 示例 $GPGLL,2959.9925,S,12000.0090,E,235316.000,A,A*4E
	 // 参数说明
	 // 字段 名称 格式 参数说明
	 // 1 $--GLL 字符串 消息ID，GLL语句头，’--‘为系统标识
	 // 2 lat ddmm.mmmm 纬度，前2字符表示度，后面的字符表示分
	 // 3 uLat 字符 纬度方向：N - 北，S - 南
	 // 4 lon dddmm.mmm 经度，前3字符表示度，后面的字符表示分
	 // 5 uLon 字符 经度方向：E - 东，W - 西
	 // 6 UTCtime hhmmss.sss 当前定位的UTC时间
	 // 7 valid 字符 数据有效性（备注[1]）
	 // 8 mode 字符 定位模式（备注[2]），仅NMEA2.3及以上版本有效
	 // 9 CS 16进制数值 校验和，$和 * 之间（不包括$和 * ）所有字符的异或结果
	 // 10 <CR><LF> 字符 回车与换行符
	 */
	typedef enum : uint8_t {
		/*
		 1- Autonomous mode
		 2- Approximate mode
		 3- Invalid
		 4- Differential mode
		 5- External mode
		 */
		Autonomous = 'A',
		Approximate = 'E',
		Invalid = 'N',
		Differential = 'D',
		External = 'M',
	} position_mode_t;

	typedef struct {
		float latitude;               // 纬度
		orien_lat_t latitudeDirection; // 纬度方向
		float longitude;              // 经度
		orien_lon_t longitudeDirection; // 经度方向
		uint32_t utcTime;             // 当前定位的UTC时间
		bool isValid;                 // 数据有效性
		position_mode_t mode;                    // 定位模式
	} GLL_t;

	/*
	 // 信息 GSA
	 // 描述 GNSS DOP和活跃卫星信息。
	 // 类型 输出
	 // 格式 $--GSA, mode, mode1, SV1, SV2, SV3, SV4, SV5, SV6, SV7, SV8, SV9, SV10, SV11, SV12, PDOP, HDOP,systemID, VDOP* CS<CR><LF>
	 // 示例 $GPGSA, A, 3, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13, 14, 1.21, 0.9, 0.8*1A
	 // 参数说明
	 字段 名称			格式					参数说明
	 1	$--GSA		字符串				消息ID，GSA语句头，’--‘为系统标识
	 2	smode			字符					定位模式（备注[1]）
	 3	fs				数值					定位模式（备注[2]）
	 4	SV1				数值					PRN号码（备注[3]）
	 5	SV2				数值					PRN号码（备注[3]）
	 6	SV3				数值					PRN号码（备注[3]）
	 7	SV4				数值					PRN号码（备注[3]）
	 8	SV5				数值					PRN号码（备注[3]）
	 9	SV6				数值					PRN号码（备注[3]）
	 10	SV7				数值					PRN号码（备注[3]）
	 11	SV8				数值					PRN号码（备注[3]）
	 12	SV9				数值					PRN号码（备注[3]）
	 13	SV10			数值					PRN号码（备注[3]）
	 14	SV11			数值					PRN号码（备注[3]）
	 15	SV12			数值					PRN号码（备注[3]）
	 16	PDOP			数值					位置精度因子（PDOP）
	 17	HDOP			数值					水平精度
	 18	VDOP			数值					垂直精度
	 19 systemID	数值					系统ID
	 20	CS				16进制数值	  校验和，$和 * 之间（不包括$和 * ）所有字符的异或结果
	 21	<CR><LF>	字符					回车与换行符
	 */
	typedef enum : uint8_t {
		sw_mannual = 'M', sw_automatic = 'A'
	} switch_mode_t;

	typedef enum : uint8_t {
		pos_invalid = 1, pos_2D = 2, pos_3D = 3
	} position_status_t;

	typedef enum : uint8_t {
		id_ALL = 0, id_GPS = 1, id_GLONASS = 2, id_BDS = 4,
	} gnss_systemid_t;

	typedef struct {
		switch_mode_t switchMode;        // 定位切换模式
		position_status_t positionStatus;      // 定位状态标志
		uint8_t satelliteView[12];  // 卫星视图
		float positionDilutionOfPrecision; // 位置精度因子
		float horizontalDilutionOfPrecision; // 水平精度因子
		float verticalDilutionOfPrecision; // 垂直精度因子
		gnss_systemid_t systemID;   // 系统标识符
	} GSA_t;

	/*
	 // 信息 GSV
	 // 描述 可见卫星信息。
	 // 类型 输出
	 // 格式 $--GSV, numMsg, msgNum, numSV, SV1, elv1, az1, SNR1, SV2, elv2, az2, SNR2, SV3, elv3, az3, SNR3, SV4, elv4, az4, SNR4, systemID* CS<CR><LF>
	 // 示例 $GPGSV, 3, 1, 12, 04, 15, 051, 44, 05, 15, 325, 44, 06, 15, 051, 44, 07, 15, 051, 44, 08, 15, 051, 44, 09, 15, 051, 44, 10, 15, 051, 44, 11, 15, 051, 44, 12, 15, 051, 44, 1*1A
	 // 参数说明
	 字段		名称				格式					参数说明
	 1		  $--GSV		字符串				消息ID，GSV语句头，’--‘为系统标识
	 2		  numMsg		数值					总消息数
	 3		  msgNum		数值					当前消息编号
	 4		  numSV			数值					可见卫星数
	 5		  SV1				数值					PRN号码（备注[1]）
	 6		  elv1			数值					卫星仰角
	 7		  az1				数值					卫星方位角
	 8		  SNR1			数值					信噪比
	 9		  signalID	数值					NMEA所定义的GNSS信号ID（0代表全部信号）
	 10		CS				16进制数值	  校验和，$和 * 之间（不包括$和 * ）所有字符的异或结果
	 11		<CR><LF>	字符					回车与换行符
	 */
	typedef struct {
		uint8_t messageCount;          // 消息数量
		uint8_t messageNumber;         // 消息编号
		uint8_t satelliteCount;        // 卫星数量
		struct {
			uint8_t satelliteID;      // 卫星ID
			uint8_t elevation;        // 卫星仰角
			uint16_t azimuth;        // 卫星方位角
			uint8_t signalToNoiseRatio; // 信噪比
		} satelliteData[4];              // 卫星数据数组
		uint8_t signalID;        // 信号标识符
	} GSV_t;

	/*
	 // 信息 RMC
	 // 描述 推荐定位信息。
	 // 类型 输出
	 // 格式 $--RMC, UTCtime, valid, lat, uLat, lon, uLon, speed, course, date, magVar, uMagVar, mode* CS<CR><LF>
	 // 示例 $GPRMC, 235316.000, A, 2959.9925, S, 12000.0090, E, 0.0, 0.0, 010203, 0.0, E, A*1A
	 // 参数说明
	 字段		名称						格式					参数说明
	 1		  $--RMC				字符串				消息ID，RMC语句头，’--‘为系统标识
	 2		  UTCtime				hhmmss.sss	当前定位的UTC时间
	 3		  valid					字符					数据有效性（备注[1]）
	 4		  lat						ddmm.mmmm		纬度，前2字符表示度，后面的字符表示分
	 5		  uLat					字符					纬度方向：N - 北，S - 南
	 6		  lon						dddmm.mmm		经度，前3字符表示度，后面的字符表示分
	 7		  uLon					字符					经度方向：E - 东，W - 西
	 8		  speed					数值					地面速率
	 9		  course				数值					地面航向
	 10		date					日期					UTC日期
	 11		magVar				数值					磁偏角
	 12		uMagVar				字符					磁偏角方向
	 13		mode					字符					定位模式（备注[2]）
	 14		navStatus			字符					导航状态标示符（V表示系统不输出导航状态信息）
	 15		CS						16进制数值		校验和，$和 * 之间（不包括$和 * ）所有字符的异或结果
	 16		<CR><LF>
	 */
	typedef enum : uint8_t {
		alarm_invalid, data_valid
	} data_validity_t;

	typedef struct {
		bool dataValid;         // 数据有效性
		float latitude;            // 纬度
		float longitude;           // 经度
		float speed;               // 速度
		float course;              // 航向
		time_t unixTime;  // 磁偏角
	} RMC_t;

	/*
	 // 信息 VTG
	 // 描述：对地速度与对地航向信息。
	 // 类型：输出
	 // 格式：$--VTG,cogt,T,cogm,M,sog,N,kph,K,mode*CS<CR><LF>
	 // 示例：$GPVTG,75.20,T,,M,0.009,N,0.017,K,A*02
	 // 参数说明：
	 // 字段			名称					格式					参数说明
	 // 1				$--VTG			字符串				消息ID，VTG语句头，’--‘为系统标识
	 // 2				cogt				数值					对地真北航向，单位为度
	 // 3				T						字符					真北指示，固定为T
	 // 4				cogm				数值					对地磁北航向，单位为度
	 // 5				M						字符					磁北指示，固定为M
	 // 6				sog					数值					对地速度，单位为节
	 // 7				N						字符					速度单位节，固定为N
	 // 8				kph					数值					对地速度，单位为千米每小时
	 // 9				K						字符					速度单位，千米每小时，固定为K
	 // 10				mode				字符					定位模式标志（备注[1]）
	 // 11				CS					16进制数值		校验和，$和*之间（不包括$和*）所有字符的异或结果
	 // 12				<CR><LF>		字符					回车与换行符
	 // 备注[1]：定位模式标志
	 // A		自主模式
	 // E		估算模式（航位推算）
	 // N		数据无效
	 // D		差分模式
	 // M		未定位，但存在外部输入或历史保存的位置
	 */
	typedef struct {
		float courseOverGroundTrue; // 真北航向
		char trueHeadingIndicator;   // 真北航向指示符
		float courseOverGroundMagnetic; // 磁北航向
		char magneticHeadingIndicator; // 磁北航向指示符
		float speedOverGround;        // 地面速度
		char speedUnit;              // 速度单位
		float speedInKph;            // 以公里每小时为单位的速度
		char kphUnit;                // 公里每小时单位
		char navigationMode;         // 导航模式
	} VTG_t;

	/*
	 // 信息 ZDA
	 // 描述：时间与日期信息。
	 // 类型：输出
	 // 格式：$--ZDA,UTCtime,day,month,year,ltzh,ltzn*CS<CR><LF>
	 // 示例：$GPZDA,235316.000,02,07,2011,00,00*51
	 // 参数说明：
	 // 字段	名称				格式						参数说明
	 // 1		$--ZDA		字符串					消息ID，ZDA语句头，’--‘为系统标识
	 // 2		UTCtime		hhmmss.sss		定位时的UTC时间
	 // 3		day				数值						日，固定两位数字，取值范围01~31
	 // 4		month			数值						月，固定两位数字，取值范围01~12
	 // 5		year			数值						年，固定四位数字
	 // 6		ltzh			数值						本时区小时，不支持，固定为00
	 // 7		ltzn			数值						本时区分钟，不支持，固定为00
	 // 8		CS				16进制数值			校验和，$和*之间（不包括$和*）所有字符的异或结果
	 // 9		<CR><LF>	字符						CASIC多模卫星导航接收机协议规范回车与换行符
	 */
	typedef struct {
		uint32_t utcTime;        // 当前定位的UTC时间
		uint8_t day;            // 当前日期的日
		uint8_t month;          // 当前日期的月
		uint16_t year;          // 当前日期的年
		uint8_t localTimeZoneHour; // 本地时区小时
		uint8_t localTimeZoneMinute; // 本地时区分钟
	} ZDA_t;

	/*
	 // 信息 TXT
	 // 描述：产品信息
	 // 类型：输出，开机时输出一次
	 // 格式：$GPTXT,xx,yy,zz,info*hh<CR><LF>
	 // 示例：$GPTXT,01,01,02,MA=CASIC*27
	 // 表示生产厂家名称（CASIC）
	 // 示例：$GPTXT,01,01,02,IC=ATGB03+ATGR201*71
	 // 表示芯片或者芯片组的型号（基带芯片型号ATGB03，射频芯片型号ATGR201）
	 // 示例：$GPTXT,01,01,02,SW=URANUS2,V2.2.1.0*1D
	 // 表示软件名称及版本号（软件名称URANUS2，版本号V2.2.1.0）
	 // 示例：$GPTXT,01,01,02,TB=2013-06-20,13:02:49*43
	 // 表示代码编译时间（2013年6月20日，13时02分49秒）
	 // 示例：$GPTXT,01,01,02,MO=GB*77
	 // 表示接收机本次启动的工作模式（GB表示GPS+BDS的双模模式）
	 // 示例：$GPTXT,01,01,02,CI=00000000*7A
	 // 表示客户编号（客户编号为00000000）
	 // 参数说明：
	 // 字段			名称				格式						参数说明
	 // 1				$GPTXT		字符串					消息ID，TXT语句头
	 // 2				xx				数值						当前消息的语句总数01~99
	 // 3				yy				数值						语句编号01~99
	 // 4				zz				数值						文本识别符。00=错误信息；01=警告信息；02=通知信息；07=用户信息。
	 // 5				info										文本信息（文本信息无需单独保存，仅需一个表示为表示是否有信息即可）
	 // 6				CS				16进制数值			校验和，$和*之间（不包括$和*）所有字符的异或结果
	 // 7				<CR><LF>	字符						回车与换行符
	 */
	typedef enum : uint8_t {
		msg_error = 0, msg_warning = 1, msg_notification = 2, msg_user = 7
	} msg_identifier_t;
	typedef struct {
		uint8_t messageTotal;
		uint8_t messageCount;
		msg_identifier_t messageIdentifier;
		bool hasInfo;
	} TXT_t;
	/*

	 // 信息 ANT
	 // 描述：天线状态
	 // 类型：输出
	 // 格式：$GPTXT, xx, yy, zz, info*hh<CR><LF>
	 // 示例：$GPTXT, 01, 01, 01, ANTENNAOPEN*25  // 表示天线状态（开路）
	 // 示例：$GPTXT, 01, 01, 01, ANTENNA OK*35  // 表示天线状态（良好）
	 // 示例：$GPTXT, 01, 01, 01, ANTENNA SHORT*63  // 表示天线状态（短路）
	 // 参数说明：
	 // 字段	名称					格式					参数说明
	 // 1		$GPTXT			字符串				消息ID，TXT语句头
	 // 2		xx					数值					当前消息的语句总数01~99，如果某个消息过长，需分为多条信息显示，固定为01。
	 // 3		yy					数值					语句编号01~99，固定为01。
	 // 4		zz					数值					文本识别符。固定为01。
	 // 5		info				文本信息			ANTENNAOPEN = 天线开路，ANTENNAOK = 天线良好，ANTENNASHORT = 天线短路
	 // 6		CS					16进制数值		校验和，$和*之间（不包括$和*）所有字符的异或结果
	 // 7		<CR><LF>		字符					回车与换行符
	 */
	typedef enum : uint8_t {
		ANTENNAOPEN = 1, ANTENNAOK = 2, ANTENNASHORT = 3
	} ant_status_t;

	typedef struct {
		uint8_t messageTotal;
		uint8_t messageCount;
		uint8_t messageIdentifier;
		ant_status_t antStatus;
	} ANT_t;

	/*
	 // 信息 DHV
	 // 描述 接收机速度的详细信息
	 // 类型 输出
	 // 格式 $--DHV,UTCtime,speed3D,spdX,spdY,spdZ,gdspd*CS<CR><LF>
	 // 示例 $GNDHV,021150.000,0.03,0.006,-0.042,-0.026,0.06*65
	 // 参数说明
	 // 字段 名称 格式 参数说明
	 // 1 $--DHV 字符串 消息ID，DHV语句头，’--‘为系统标识
	 // 2 UTCtime hhmmss.sss 当前时刻的UTC时间
	 // 3 speed3D 数值 接收机三维速度，单位为m/s
	 // 4 spdX 数值 接收机ECEF-X轴方向速度，单位为m/s
	 // 5 spdY 数值 接收机ECEF-Y轴方向速度，单位为m/s
	 // 6 spdZ 数值 接收机ECEF-Z轴方向速度，单位为m/s
	 // 7 gdspd 数值 接收机水平地面方向速度，单位为m/s
	 // 8 CS 16进制数值 校验和，$和*之间（不包括$和*）所有字符的异或结果
	 // 9 <CR><LF> 字符 回车与换行符
	 */
	typedef struct {
		uint32_t utcTime;          // 当前定位的UTC时间
		float _3DSpeed;        // 三维速度
		float speedX;             // X方向速度
		float speedY;             // Y方向速度
		float speedZ;             // Z方向速度
		float groundSpeed;         // 地面速度
	} DHV_t;

	/*
	 信息 GST
	 描述 接收机伪距的测量精度详细信息
	 类型 输出
	 格式 $--GST,UTCtime,RMS,stdDevMaj,stdfDevMin,orientation,stdLat,stdLon,stdAlt* CS<CR><LF>
	 示例 $BDGST,081409.000,0.5,,,,0.2,0.1,0.4*5E
	 参数说明
	 字段		名称								格式					参数说明
	 1			$--GST						字符串				消息ID，DHV语句头，’--‘为系统标识
	 2			UTCtime						hhmmss.sss	当前时刻的UTC时间
	 3			RMS								数值					定位过程中接收机伪距误差标准差的RMS值，单位米
	 4			stdDevMaj					数值					接收机椭圆半长轴方向的位置标准差，不支持
	 5			stdfDevMin				数值					接收机椭圆半短轴方向的位置标准差，不支持
	 6			orientation				数值					接收机椭圆半长轴方向的朝向，不支持
	 7			stdLat						数值					接收机纬度向误差的标准差，单位米
	 8			stdLon						数值					接收机经度向误差的标准差，单位米
	 9			stdAlt						数值					接收机高度向误差的标准差，单位米
	 10			CS								16进制数值   校验和，$和*之间（不包括$和*）所有字符的异或结果
	 11			<CR><LF>					字符					回车与换行符
	 */
	typedef struct {
		uint32_t utcTime;          // 当前定位的UTC时间
		float rootMeanSquare;      // 均方根
		float standardDeviationMajor; // 主标准差
		float standardDeviationMinor; // 次标准差
		float orientationAngle;     // 方向角
		float standardDeviationLatitude; // 纬度标准差
		float standardDeviationLongitude; // 经度标准差
		float standardDeviationAltitude; // 高度标准差
	} GST_t;

	using MessageContent = variant<GGA_t, GLL_t, GSA_t, GSV_t, RMC_t, VTG_t, ZDA_t, ANT_t, DHV_t, GST_t, TXT_t>;

	struct {
		system_type_t systemType = sys_type_custom; // 初始化为默认值
		message_type_t messageType = msg_type_gga;   // 初始化为默认值
		MessageContent content;
	} message;

	NMEA0183();
	~NMEA0183();

	/*set the message entry*/
	int setMessageField(const char *msgField, size_t length);
	int parseSysType(void);
	/*find and parse the next NMEA message in the message field*/
	bool parseRmcMessage();

	template<typename T>
	T* getContent() {
		return std::get_if<T>(&message.content);
	}

private:
	string_view sv;
	string_view msgView;
	string_view paramView;

	time_t timeDateFormat(float utctime, int date);
	float degreeFormat(float raw, char dir);
	/*
	 check sum validation for current sentence iterator
	 */
	bool checkSum(string_view sentence);

	/*
	 handle functions
	 handle function receive string view from sentenceIterator
	 */

	bool parseRMC(void);

};
