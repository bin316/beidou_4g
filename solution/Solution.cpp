/*
 * Solution.cpp
 *
 *  Created on: Jan 22, 2025
 *      Author: IRIS
 *      AI生成注释: 此文件实现了北斗界桩设备的核心解决方案类，包含设备的工作模式、通信协议、参数配置等主要功能
 */

/*
 * ================================================================================================
 * 北斗界桩设备运行流程图 (AI生成)
 * ================================================================================================
 *
 *                                    ┌─────────────────┐
 *                                    │    系统启动     │
 *                                    │  Solution()     │
 *                                    └─────────┬───────┘
 *                                              │
 *                                    ┌─────────▼───────┐
 *                                    │  初始化NVM存储  │
 *                                    │  加载/生成配置  │
 *                                    └─────────┬───────┘
 *                                              │
 *                              ┌───────────────▼───────────────┐
 *                              │        选择工作模式           │
 *                              └─┬─────────┬─────────┬────────┘
 *                                │         │         │
 *                    ┌───────────▼─┐   ┌───▼───┐   ┌─▼────────┐
 *                    │  工厂模式   │   │工作模式│   │ 空闲模式 │
 *                    │ fact_thread │   │work_th│   │idle_th   │
 *                    └─────┬───────┘   └───┬───┘   └─┬────────┘
 *                          │               │         │
 *                   ┌──────▼──────┐        │         │
 *                   │ 垂直检测100次│        │         │
 *                   │  (10Hz采样) │        │         │
 *                   └──────┬──────┘        │         │
 *                          │               │         │
 *                     ┌────▼────┐          │         │
 *                     │检测通过?│          │         │
 *                     └─┬─────┬─┘          │         │
 *                  失败  │     │ 成功       │         │
 *               ┌───────▼─┐   │            │         │
 *               │2小时休眠│   │            │         │
 *               │再次检测 │   │            │         │
 *               └─────────┘   │            │         │
 *                             │            │         │
 *                        ┌────▼────┐       │         │
 *                        │切换工作模式      │         │
 *                        │并重启系统│       │         │
 *                        └─────────┘       │         │
 *                                          │         │
 *                        ┌─────────────────▼─────────▼─────────────────┐
 *                        │              网络初始化流程                 │
 *                        │    ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐   │
 *                        │    │4G上电│→ │LTE附着│→ │服务器│→ │连接  │   │
 *                        │    └──────┘  └──┬───┘  │配置  │  │建立  │   │
 *                        │                 │      └──────┘  └──┬───┘   │
 *                        │            失败 ▼                    │ 成功  │
 *                        │         ┌──────────┐                │       │
 *                        │         │网络不良休眠│                │       │
 *                        │         │t1_min间隔│                │       │
 *                        │         └──────────┘                │       │
 *                        └─────────────────────────────────────┼───────┘
 *                                                               │
 *                                    ┌──────────────────────────▼──────────────────────────┐
 *                                    │                   主事件循环                        │
 *                                    │                                                     │
 *                                    │  ┌─────────┐     ┌─────────┐     ┌─────────┐      │
 *                                    │  │震动事件 │     │消息事件 │     │定时器   │      │
 *                                    │  │vibrate  │     │message  │     │timeout  │      │
 *                                    │  └────┬────┘     └────┬────┘     └────┬────┘      │
 *                                    │       │               │               │          │
 *                                    │  ┌────▼────┐     ┌────▼────┐     ┌────▼────┐      │
 *                                    │  │上报状态 │     │解析协议 │     │主动上报 │      │
 *                                    │  │激活GNSS │     │执行命令 │     │或休眠   │      │
 *                                    │  └─────────┘     └─────────┘     └─────────┘      │
 *                                    └─────────────────────────────────────────────────────┘
 *                                                               │
 *                                              ┌────────────────▼────────────────┐
 *                                              │          协议处理分支           │
 *                                              │                                 │
 *                                              │ ┌─────────┐ ┌─────────┐        │
 *                                              │ │上传报告 │ │上传配置 │        │
 *                                              │ └─────────┘ └─────────┘        │
 *                                              │ ┌─────────┐ ┌─────────┐        │
 *                                              │ │修改配置 │ │配置模式 │        │
 *                                              │ └─────────┘ └─────────┘        │
 *                                              │ ┌─────────┐ ┌─────────┐        │
 *                                              │ │休眠命令 │ │模式切换 │        │
 *                                              │ └─────────┘ └─────────┘        │
 *                                              └─────────────────────────────────┘
 *
 * 关键特性说明 (AI生成):
 * ■ 三种工作模式: 工厂模式(检测安装)、工作模式(完整功能)、空闲模式(节能)
 * ■ 事件驱动架构: 震动检测、服务器消息、定时器超时
 * ■ 低功耗设计: 网络失败自动休眠、定时唤醒机制
 * ■ 容错机制: CRC校验、连接重试、配置备份
 * ■ 双服务器支持: 主服务器+备用服务器冗余设计
 * 
 * 交接：
 * 双服务器支持没怎么实现，但是后面可以尝试，留有接口。
 * 服务器的ip地址和端口好像是固定不能修改的，我不太记得了。
 * 工厂模式可能还要写一个运输固定休眠时间，比如说估计运输过程大约48h，那么在上电之后就首先休眠48h然后在进行摆正检测。
 * 
 * ================================================================================================
 */

// AI生成注释: 包含解决方案类的头文件定义
#include <Solution.h>
// AI生成注释: 包含工具函数库，提供系统级别的辅助功能
#include "utilties.h"
// AI生成注释: 包含CRC校验相关的函数和定义
#include "crc.h"

// AI生成注释: 包含辅助函数和重载操作符的定义
#include "helper_and_reload.hpp"

// AI生成注释: 包含magic_enum库，用于枚举类型的字符串转换
#include "magic_enum.hpp"
using namespace magic_enum;

// AI生成注释: 包含产品配置相关的宏定义和常量
#include "PRODUCT_CONFIG.h"

// AI生成注释: 解决方案任务句柄，用于FreeRTOS任务管理
TaskHandle_t solution_thread_handle = NULL;

// AI生成注释: 全局解决方案配置结构体实例，包含设备的所有配置参数
solution_handle fc_solution =
    {
	    .runningConfig =
		{
		// AI生成注释: 网络良好时的唤醒间隔（分钟），默认5分钟
		.t0_netGood_wakeup_min =
		PROD_CONFIG_FACTORY_NET_GOOD_WAKEUP_TIMEOUT_MIN, //5(单位[分钟])
			// AI生成注释: 网络不良时的唤醒间隔（分钟），默认3分钟
			.t1_netBad_wakeup_min =
				PROD_CONFIG_FACTORY_NET_BAD_WAKEUP_TIMEOUT_MIN, //3(单位[分钟])
			// AI生成注释: 服务器响应超时时间（秒），默认3秒
			.t2_serverRsps_timeout_sec =
				PROD_CONFIG_FACTORY_SERVER_RSP_TIMEOUT_SEC, //3(单位[秒])
			// AI生成注释: GNSS搜索休眠时间（秒），默认180秒
			.t3_gnssSearch_sleep_sec =
				PROD_CONFIG_FACTORY_GNSS_SEARCH_TIMEOUT, //180(单位[秒])
			// AI生成注释: GNSS定位完成后的休眠时间（秒），默认60秒
			.t4_gnssGood_sleep_sec =
			PROD_CONFIG_FACTORY_GNSS_SEARCH_COMPLETE_TIMEOUT, //60(单位[秒])
			// AI生成注释: RTC事件休眠时间（秒），此参数已不再使用
			.t6_rtcEvent_sleep_sec = 30, //60(单位[秒])$notice 这个参数不再使用了
			// AI生成注释: 震动检测防抖延迟时间（秒），默认5秒
			.t5_motionDetect_delay_sec =
			PROD_CONFIG_FACTORY_VIBRATION_DEBOUNCE_SEC, //5(单位[秒])
			// AI生成注释: GNSS更新唤醒计数阈值，此参数无效
			.n0_gnssUpdate_wakeup_count = 60, //60(单位[次])/*$notice parameter has no effect*/
			// AI生成注释: 电池低电压报警阈值（0.1V为单位），默认22表示2.2V
			.n1_vbatAlarm_threshold_volt =
			PROD_CONFIG_FACTORY_VBAT_LOW
//22(单位[0.1V])
		    },
	    .systemConfig =
		{
			// AI生成注释: 设备编码信息，包含主版本号、次版本号和索引
			.code =
			    {
			    .major =
			    PROD_CONFIG_FACTORY_DEFAULT_CODE_MAJOR, .minor =
			    PROD_CONFIG_FACTORY_DEFAULT_CODE_MINOR, .index =
			    PROD_CONFIG_FACTORY_DEFAULT_CODE_INDEX,
			    },
			// AI生成注释: 传感器倾斜检测的角度范围，默认15度
			. sensorReverseRange = 15,
			// AI生成注释: 传感器震动检测阈值，默认为1
			. sensorVibrationThreshold = 1,
			// AI生成注释: 备用服务器IP地址
			. backupServerIP =
			PROD_CONFIG_FACTORY_DEFAULT_AUX_IP,
			// AI生成注释: 备用服务器端口号
			. backupServerPort =
			PROD_CONFIG_FACTORY_DEFAULT_AUX_PORT,
			// AI生成注释: 主服务器IP地址
			. runServerIP =
			PROD_CONFIG_FACTORY_DEFAULT_MAIN_IP,
			// AI生成注释: 主服务器端口号
			. runServerPort =
			PROD_CONFIG_FACTORY_DEFAULT_MAIN_PORT
		},
	    .pwd =
		{
		// AI生成注释: 设备访问密码
		.item =
		PROD_CONFIG_FACTORY_DEFAULT_PASSWORD
		},
	    // AI生成注释: 设备工作模式，默认为工厂模式
	    .mode =
	    PROD_CONFIG_FACTORY_DEFAULT_WORK_MODE,
	    // AI生成注释: 配置编辑标志位，0表示未在编辑状态
	    .configEditting = 0,
	    // AI生成注释: 密码确认标志位，0表示密码未确认
	    .passwordConfirm = 0,
	    // AI生成注释: 新密码缓存区
	    .newPassword =
		{
		0
		},
	    // AI生成注释: 唤醒源类型，默认为定期唤醒
	    .wakeSource = util_lowpower_wake_source_e::regular,
	    // AI生成注释: 是否连接到主服务器的标志位
	    .connect_to_main_server = false,
	    // AI生成注释: 启动时是否更新位置信息的标志位
	    .updatePositionOnStart = false
    };

/**
 * AI生成注释: Solution类构造函数
 * 功能说明:
 * 1. 初始化非易失性存储器(NVM)用于参数保存和加载
 * 2. 如果是出厂默认状态，则生成唯一设备索引号
 * 3. 根据设备工作模式启动相应的任务线程
 * 4. 创建并启动解决方案主要工作任务
 */
Solution::Solution()
    {
    /* create NVM: parameters save and load
     *
     * 加载参数，如果加载失败，使用默认参数
     * */

    // AI生成注释: 记录解决方案初始化开始日志
    logInfo("solution init..");

    // AI生成注释: 创建NVM实例，用于参数的持久化存储，包含运行时和出厂配置
    nvm = new NVM(NVM::partition_solution, (uint8_t*) &this->rt_solution,
	    &fc_solution, sizeof(this->rt_solution));
    // AI生成注释: 确保NVM实例创建成功，否则触发断言
    configASSERT(nvm != NULL);

    // AI生成注释: 记录NVM创建成功日志
    logInfo("solution nvm created..");
    // AI生成注释: 从NVM加载配置参数
    nvm->load();
    // AI生成注释: 检查是否为出厂默认配置
    if (nvm->isFactoryDefault())
	{
	// AI生成注释: 记录使用出厂默认配置的日志
	logInfo("solution nvm factory default..");
	// AI生成注释: 恢复到出厂默认配置
	nvm->restoreDefault();
	// AI生成注释: 如果配置为生成唯一索引，则基于芯片UID生成设备唯一标识
	if (PROD_CONFIG_FACTORY_GENERATE_UNIQ_INDEX == 1)
	    {
	    /*根据芯片的Unique ID生成唯一的index */
	    /*unique index使用96位UID的每两个字节累加得出16位index */
	    /*无视溢出 */
	    /*获取uid */
	    // AI生成注释: 初始化UID数组，用于存储芯片的96位唯一标识符
	    uint32_t UID[3] =
		{
		    {
		    0
		    }
		};
	    // AI生成注释: 获取芯片UID的第一个32位字
	    UID[0] = HAL_GetUIDw0();
	    // AI生成注释: 获取芯片UID的第二个32位字
	    UID[1] = HAL_GetUIDw1();
	    // AI生成注释: 获取芯片UID的第三个32位字
	    UID[2] = HAL_GetUIDw2();
	    // AI生成注释: 初始化设备索引为0
	    this->rt_solution.systemConfig.code.index = 0;
	    // AI生成注释: 遍历3个32位UID，将每个32位分成高低16位进行累加
	    for (int i = 0; i < 3; i++)
		{
		// AI生成注释: 累加UID的低16位到设备索引
		this->rt_solution.systemConfig.code.index += UID[i] & 0xFFFF;
		// AI生成注释: 累加UID的高16位到设备索引，忽略溢出
		this->rt_solution.systemConfig.code.index += (UID[i] >> 16)
			& 0xFFFF;
		}
	    }
	// AI生成注释: 保存更新后的配置到NVM
	nvm->save();
	}

    /*
     * 解决方案任务启动
     * 模式启动不同任务
     */

    // AI生成注释: 根据当前设备工作模式创建相应的任务线程
    switch (rt_solution.mode)
	{
    // AI生成注释: 工作模式 - 创建工作例程线程，具备完整功能
    case solution_mode_e::wm_work:
	xTaskCreate(Solution::solution_work_routine_thread, "solution_work",
		512, this, osPriorityNormal, &solution_thread_handle);
	break;
    // AI生成注释: 空闲模式 - 创建空闲例程线程，功能受限以节省电能
    case solution_mode_e::wm_idle:
	xTaskCreate(Solution::solution_idle_routine_thread, "solution_idle",
		512, this, osPriorityNormal, &solution_thread_handle);
	break;
    // AI生成注释: 默认情况（包括工厂模式）- 创建工厂测试例程线程
    default:
	xTaskCreate(Solution::solution_fact_routine_thread, "solution_fact",
		512, this, osPriorityNormal, &solution_thread_handle);
	}
    // AI生成注释: 确保任务句柄创建成功，否则触发断言
    configASSERT(solution_thread_handle != NULL);

    // AI生成注释: 记录启动的例程模式日志，使用枚举名称转换为字符串
    logInfo("routine started: %s..", enum_name(rt_solution.mode).data());

    }

/**
 * AI生成注释: 设备状态报告函数
 * 功能说明:
 * 1. 收集传感器数据（加速度计、模拟信号、GNSS定位等）
 * 2. 组装协议数据包，包含设备状态、位置、时间等信息
 * 3. 计算CRC校验值并发送数据包到服务器
 * 4. 清除震动检测标志位，记录关键信息到日志
 */
void Solution::report(void)
    {

    // AI生成注释: 创建上报数据包结构体
    pb_packReport rsps;
    // AI生成注释: 获取加速度传感器当前状态（三轴加速度、倾斜角度、震动标志等）
    util_sc7a20_status_s sensor = util_sc7a20_get_status();
    // AI生成注释: 获取模拟信号状态（电池电压、温度等）
    util_analog_status_s analog = util_analog_get_status();

    // AI生成注释: 清除震动检测标志位，防止重复上报
    util_sc7a20_clear_vibration_flag();

    // AI生成注释: 设置数据包头部的设备编码信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    // AI生成注释: 设置数据长度为报告结构体大小
    rsps.body.header.dataLen = sizeof(pb_report);
    // AI生成注释: 设置功能码为上报数据
    rsps.body.header.function = e_pb_func::up_report;
    // AI生成注释: 设置三轴加速度数据
    rsps.body.report.acc[ID_AXIS_X] = sensor.x;
    rsps.body.report.acc[ID_AXIS_Y] = sensor.y;
    rsps.body.report.acc[ID_AXIS_Z] = sensor.z;
    // AI生成注释: 设置设备倾斜角度（转换为16位整数）
    rsps.body.report.angle = (int16_t) sensor.lean_angle;
    // AI生成注释: 设置地理位置坐标（经度）
    rsps.body.report.geo[ID_LONGITUDE] = util_atgm332d_get_status().longitude; //测试地理位置
    // AI生成注释: 设置地理位置坐标（纬度）
    rsps.body.report.geo[ID_LATITUDE] = util_atgm332d_get_status().latitude; //测试地理位置;
    // AI生成注释: 生成状态字节，包含工作模式、唤醒源、震动状态、倾斜状态、定位状态等信息
    rsps.body.report.status = helper_status_byte_maker(this->rt_solution.mode,
	    util_lowpower_get_wake_source(), sensor.vibration_occured,
	    sensor.leaned, util_atgm332d_get_status().position_fixed);
    // AI生成注释: 设置温度值（转换为8位有符号整数）
    rsps.body.report.temp = (int8_t) analog.temperture;
    // AI生成注释: 设置RTC时间戳
    rsps.body.report.time = (uint32_t) util_lowpower_get_rtc();
    // AI生成注释: 设置电池电压（转换为0.1V为单位的8位整数）
    rsps.body.report.vbat = (uint8_t) (analog.vbat * 10.0f);
    // AI生成注释: 设置4G模块信号强度
    rsps.body.report.csq = air->status.csq;
    // AI生成注释: 计算数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));

    /*log some critical information: accel data, angle, status byte, temp, time, csq*/
    // AI生成注释: 记录关键信息到日志：倾斜角度、温度、时间戳
    logInfo("angle: %d, temp: %d, time: %ld", rsps.body.report.angle,
	    rsps.body.report.temp, rsps.body.report.time);
    // AI生成注释: 记录地理坐标和电池电压信息到日志
    logInfo("geo: [%f,%f], vbat: %d ", rsps.body.report.geo[ID_LONGITUDE],
	    rsps.body.report.geo[ID_LATITUDE], rsps.body.report.vbat);

    // AI生成注释: 发送数据包到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packReport));
    }

/**
 * AI生成注释: 设备重启函数
 * 功能说明:
 * 1. 记录重启日志信息
 * 2. 等待50毫秒确保日志输出完成
 * 3. 同步Flash存储确保数据持久化
 * 4. 执行系统复位重启设备
 */
void Solution::restart(void)
    {
//	$todo simply reset chip
    // AI生成注释: 记录芯片重启日志
    logInfo("chip restart..");
    // AI生成注释: 延迟50毫秒，确保日志输出完成
    vTaskDelay(pdMS_TO_TICKS(50));
    /*$notice 重启时，写入一次flash*/
    // AI生成注释: 同步Flash存储，确保所有待写入数据被持久化
    __flash_sync();
    // AI生成注释: 执行NVIC系统复位，重启整个系统
    HAL_NVIC_SystemReset();
    }

/**
 * AI生成注释: 刷新服务器超时定时器函数
 * 功能说明:
 * 重置服务器响应超时定时器，用于在收到服务器响应时延长等待时间
 */
void Solution::refresh_server(void)
    {
    // AI生成注释: 重置服务器超时定时器，延长服务器响应等待时间
    xTimerReset(server_timeout_timer, portMAX_DELAY);
    }

/**
 * AI生成注释: 刷新设备超时定时器函数
 * 功能说明:
 * 重置设备超时定时器，用于延长设备活动时间防止意外休眠
 */
void Solution::refresh_device(void)
    {
    // AI生成注释: 重置设备超时定时器，延长设备活动时间
    xTimerReset(device_timeout_timer, portMAX_DELAY);
    }

/**
 * AI生成注释: 震动事件处理函数
 * 功能说明:
 * 1. 记录震动检测日志
 * 2. 刷新服务器响应超时定时器
 * 3. 立即上报设备状态到服务器
 * 4. 如果处于工作模式，激活GNSS定位系统
 */
void Solution::event_action_vibration(void)
    {
    /*震动发生后，刷新服务器响应超时定时器*/
    // AI生成注释: 记录检测到震动的日志
    logInfo("vibration detected..");
    // AI生成注释: 刷新服务器响应超时定时器，延长等待时间
    xTimerReset(server_timeout_timer, portMAX_DELAY);
    // AI生成注释: 立即上报当前设备状态到服务器
    this->report();
    /*
     * activate bd system if the running mode is "work"
     * 		if the mode is idle or factory, position
     * 		update will not work for saving power
     */
    // AI生成注释: 判断是否为工作模式，只有工作模式才启动定位功能以节省电能
    if (rt_solution.mode == solution_mode_e::wm_work)
	{
	// AI生成注释: 记录开始定位的日志
	logInfo("positioning..");
	// AI生成注释: 激活GNSS定位模块，参数10表示激活时间或尝试次数
	util_atgm332d_activate(10);
	}
    }

/**
 * AI生成注释: 消息接收处理函数
 * 功能说明:
 * 1. 从通信模块读取消息数据
 * 2. 解析协议头部和CRC校验
 * 3. 验证设备编码匹配和数据完整性
 * 4. 根据功能码分发到相应的处理函数
 * 5. 刷新服务器和设备超时定时器
 */
void Solution::event_action_message(void)
    {
    // AI生成注释: 分配64字节内存用于存储接收到的消息
    uint8_t *message = (uint8_t*) pvPortMalloc(64);
    // AI生成注释: 实际读取到的数据长度
    uint16_t read_size = 0;
    // AI生成注释: 目标CRC校验值
    uint16_t target_crc = 0;
    // AI生成注释: CRC数据指针
    uint8_t *crc_ptr = NULL;
    // AI生成注释: 协议数据指针，用于遍历消息内容
    uint8_t *protocol_ptr = NULL;
    // AI生成注释: 可能的协议头部指针
    pb_header *possible_header = NULL;
    // AI生成注释: 参数数据指针
    void *param_ptr = NULL;

    // AI生成注释: 确保内存分配成功，否则触发断言
    configASSERT(message != nullptr);

    // AI生成注释: 从通信模块读取消息，最大64字节
    read_size = read_message(message, 64);

    // AI生成注释: 检查读取的数据是否足够包含协议头部
    if (read_size < sizeof(pb_header))
	{
	// AI生成注释: 数据不足，释放内存并返回
	vPortFree(message);
	return;
	}

    // AI生成注释: 初始化协议指针为消息起始位置
    protocol_ptr = message;
    // AI生成注释: 遍历消息内容寻找协议起始标识符（0xFBFBFB）
    while (protocol_ptr < message + read_size - 2)
	{
	// AI生成注释: 检查是否找到协议起始标识符（3个连续的0xFB）
	if (*protocol_ptr == 0xFB && *(protocol_ptr + 1) == 0xFB
		&& *(protocol_ptr + 2) == 0xFB)
	    {
	    // AI生成注释: 跳过3字节的起始标识符
	    protocol_ptr += 3;
	    // AI生成注释: 将当前位置解析为协议头部
	    possible_header = (pb_header*) protocol_ptr;
	    // AI生成注释: 验证设备编码是否匹配
	    if (possible_header->code == this->rt_solution.systemConfig.code)
		{
		// AI生成注释: 计算CRC校验数据的位置
		crc_ptr = protocol_ptr + sizeof(pb_header)
			+ possible_header->dataLen;
		// AI生成注释: 提取16位CRC值（小端序）
		target_crc = (*(crc_ptr + 1) << 8) | *crc_ptr;
		// AI生成注释: 计算实际CRC值并与目标CRC比较
		if (HAL_CRC_Calculate(&hcrc, (uint32_t*) possible_header,
			sizeof(pb_header) + possible_header->dataLen)
			== target_crc)
		    {
		    // AI生成注释: CRC校验通过，计算参数数据的位置
		    param_ptr = ((uint8_t*) possible_header)
			    + sizeof(pb_header);
		    // AI生成注释: 记录消息确认日志
		    logInfo("message confirmed..");
		    // AI生成注释: 刷新服务器和设备超时定时器
		    this->refresh_server();
		    this->refresh_device();
		    // AI生成注释: 记录收到的消息功能码日志
		    logInfo("message received: %s",
			    enum_name((e_pb_func )(possible_header->function)).data());
		    // AI生成注释: 根据功能码分发到相应的处理函数
		    switch (possible_header->function)
			{
		    // AI生成注释: 服务器请求上传报告
		    case e_pb_func::down_uploadReport:
			this->message_report();
			break;
		    // AI生成注释: 服务器请求上传运行配置
		    case e_pb_func::down_uploadRunningConfig:
			this->message_upload_run_parameters();
			break;
		    // AI生成注释: 服务器请求修改运行配置
		    case e_pb_func::down_configRunning:
			this->message_change_run_parameters(
				(pb_runningConfig*) param_ptr);
			break;
		    // AI生成注释: 服务器请求设备休眠
		    case e_pb_func::down_sleep:
			this->message_sleep();
			break;
		    // AI生成注释: 服务器请求进入配置模式
		    case e_pb_func::down_configMode:
			this->message_enter_config_mode((password*) param_ptr);
			break;
		    // AI生成注释: 服务器请求上传系统配置
		    case e_pb_func::down_uploadSystemConfig:
			this->message_upload_sys_parameters();
			break;
		    // AI生成注释: 服务器请求修改系统配置
		    case e_pb_func::down_configSystem:
			this->message_change_sys_parameters(
				(pb_systemConfig*) param_ptr);
			break;
		    // AI生成注释: 服务器请求修改工作模式
		    case e_pb_func::down_configWorkMode:
			this->message_change_execute_mode(
				(solution_mode_e*) param_ptr);
			break;
			}
		    // AI生成注释: 消息处理完成，释放内存并返回
		    vPortFree(message);
		    return;
		    }
		}
	    }
	// AI生成注释: 继续搜索下一个可能的协议起始位置
	protocol_ptr++;
	}
    // AI生成注释: 未找到有效消息，释放内存
    vPortFree(message);
    // AI生成注释: 记录无效消息警告日志
    logWarning("bad message..");
    return;
    }

/**
 * AI生成注释: 处理服务器请求上传报告的消息
 * 功能说明:
 * 直接调用report函数上传当前设备状态报告
 */
void Solution::message_report(void)
    {
    // AI生成注释: 响应服务器请求，上传设备状态报告
    this->report();
    }

/**
 * AI生成注释: 处理服务器请求上传运行配置的消息
 * 功能说明:
 * 1. 组装运行配置数据包
 * 2. 设置协议头部信息
 * 3. 计算CRC校验值
 * 4. 发送配置数据到服务器
 */
void Solution::message_upload_run_parameters(void)
    {
    // AI生成注释: 创建运行配置上传数据包
    pb_packRunningConfig rsps;
    // AI生成注释: 设置设备编码
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    // AI生成注释: 设置数据长度为运行配置结构体大小
    rsps.body.header.dataLen = sizeof(pb_runningConfig);
    // AI生成注释: 设置功能码为运行配置上传
    rsps.body.header.function = e_pb_func::up_runningConfigUpload;
    // AI生成注释: 复制当前运行配置到数据包
    rsps.body.runningConfig = this->rt_solution.runningConfig;

    // AI生成注释: 计算数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));

    // AI生成注释: 发送运行配置数据包到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packRunningConfig));
    }

/**
 * AI生成注释: 处理服务器修改运行配置的消息
 * 功能说明:
 * 1. 更新设备运行配置参数
 * 2. 应用配置到相关硬件模块（传感器、定时器等）
 * 3. 保存配置到NVM
 * 4. 发送配置结果响应到服务器
 */
void Solution::message_change_run_parameters(pb_runningConfig *params)
    {
    // AI生成注释: 创建命令响应数据包
    pb_packCmdletOrResponse rsps;

    // AI生成注释: 获取当前传感器配置
    util_sc7a20_config_s sensor_config = util_sc7a20_get_config();
    // AI生成注释: 获取当前模拟信号配置
    util_analog_config_s analog_config = util_analog_get_config();

    // AI生成注释: 更新运行配置参数
    rt_solution.runningConfig = *params;
//	修改Solution相关配置
    // AI生成注释: 修改服务器响应超时定时器周期，单位转换为毫秒，立即生效
    xTimerChangePeriod(server_timeout_timer,
	    pdMS_TO_TICKS(rt_solution.runningConfig.t2_serverRsps_timeout_sec * 1000),
	    portMAX_DELAY); //    修改服务器响应超时定时器,单位为秒,立即生效
    // AI生成注释: 修改设备超时定时器，根据定位状态选择不同的超时时间
    xTimerChangePeriod(device_timeout_timer,
	    pdMS_TO_TICKS( util_atgm332d_get_status().position_fixed?rt_solution.runningConfig.t4_gnssGood_sleep_sec * 1000:rt_solution.runningConfig.t3_gnssSearch_sleep_sec * 1000),
	    portMAX_DELAY); //    修改设备超时定时器,单位为分钟,立即生效

//	修改传感器配置:震动消抖
    // AI生成注释: 更新传感器震动检测的冷却时间（防抖时间）
    sensor_config.cool_down_timeout = params->t5_motionDetect_delay_sec;
    // AI生成注释: 应用传感器配置
    util_sc7a20_set_config(sensor_config);
//	修改模拟信号配置:电池报警阈值
    // AI生成注释: 更新电池低电压报警阈值，从0.1V单位转换为V单位
    analog_config.low_battery_threshold = params->n1_vbatAlarm_threshold_volt
	    / 10.0f;
    // AI生成注释: 应用模拟信号配置
    util_analog_set_config(analog_config);

    // AI生成注释: 保存配置到NVM
    nvm->save();

    // AI生成注释: 设置响应数据包头部信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_runningConfigResult;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

    // AI生成注释: 设置响应结果，只有在配置编辑模式下才成功应用配置
    rsps.body.cmdletOrResponse =
	    this->rt_solution.configEditting ?
		    (this->rt_solution.runningConfig = *params) : 0xFF;
    // AI生成注释: 计算响应数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));

//    util_air780_net_write((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    // AI生成注释: 发送配置结果响应到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    }

/**
 * AI生成注释: 处理服务器休眠请求的消息
 * 功能说明:
 * 1. 发送休眠响应确认到服务器
 * 2. 保存所有配置到NVM
 * 3. 配置低功耗参数
 * 4. 进入待机休眠模式
 */
void Solution::message_sleep(void)
    {
    // AI生成注释: 创建简单响应数据包
    pb_packCmdletOrResponseSimple rsps;

//	log_i("requested sleep.");

    // AI生成注释: 设置响应数据包头部信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_responseSleep;
    rsps.body.header.dataLen = 0;
    // AI生成注释: 计算响应数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));

//    util_air780_net_write((uint8_t*) &rsps,
//	    sizeof(pb_packCmdletOrResponseSimple));
    // AI生成注释: 发送休眠响应到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponseSimple));
//	save all configurations before sleep
    // AI生成注释: 休眠前保存所有配置到NVM
    nvm->save();
//	sleep
    // AI生成注释: 获取当前低功耗配置
    auto lpconfig = util_lowpower_get_config();
    // AI生成注释: 设置请求的唤醒周期为网络良好时的唤醒间隔（分钟转秒）
    lpconfig.requested_wakeup_period =
	    this->rt_solution.runningConfig.t0_netGood_wakeup_min * 60;
    // AI生成注释: 设置剩余唤醒时间等于请求的唤醒周期
    lpconfig.wakeup_remain = lpconfig.requested_wakeup_period;
    // AI生成注释: 启用唤醒引脚，允许外部震动唤醒
    lpconfig.wake_pin_enable = true;
    // AI生成注释: 应用低功耗配置
    util_lowpower_set_config(lpconfig);
    // AI生成注释: 进入待机休眠模式
    util_lowpower_standby();
    }

/**
 * AI生成注释: 处理服务器配置模式请求的消息
 * 功能说明:
 * 1. 处理密码验证和配置模式切换
 * 2. 支持进入配置模式、退出配置模式、密码修改功能
 * 3. 实现密码确认机制防止误操作
 * 4. 发送配置模式操作结果到服务器
 */
void Solution::message_enter_config_mode(password *pwd)
    {
    // AI生成注释: 创建命令响应数据包
    pb_packCmdletOrResponse rsps;
    // AI生成注释: 定义退出配置模式的特殊密码（全零）
    password exitWord =
	{
	.item =
	    {
	    0
	    }
	};
    // AI生成注释: 设置响应数据包头部信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_responseConfigMode;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

//	非编辑模式，判断密码并进入编辑模式
    // AI生成注释: 如果当前不在配置编辑模式
    if (!this->rt_solution.configEditting)
	{
	// AI生成注释: 验证输入密码是否正确
	if (this->rt_solution.pwd == *pwd)
	    {
	    // AI生成注释: 密码正确，进入配置编辑模式
	    this->rt_solution.configEditting = true;
	    rsps.body.cmdletOrResponse = edit_enabled;
	    }
	else
	    {
	    // AI生成注释: 密码错误，配置编辑模式未启用
	    rsps.body.cmdletOrResponse = edit_notEnabled;
	    }
	}
//	编辑模式，识别退出密码，或更改密码
    else
	{
//	 编辑模式优先识别退出功能
	// AI生成注释: 如果输入的是退出密码（全零密码）
	if (*pwd == exitWord)
	    {
//	 	退出编辑模式也会清空newpassword
	    // AI生成注释: 退出配置编辑模式
	    this->rt_solution.configEditting = false;
	    // AI生成注释: 清空新密码缓存
	    this->rt_solution.newPassword = exitWord;
	    // AI生成注释: 清除密码确认标志
	    this->rt_solution.passwordConfirm = 0;
	    rsps.body.cmdletOrResponse = edit_exit;
	    }
//		若已经开始编辑密码则不进入下一分支（开始编辑密码）
	// AI生成注释: 如果正在进行密码确认流程
	else if (this->rt_solution.passwordConfirm)
	    {
//	 	若密码确认成功则返回密码修改成功
	    // AI生成注释: 验证两次输入的新密码是否一致
	    if (*pwd == this->rt_solution.newPassword)
		{
		// AI生成注释: 密码确认成功，修改密码
		rsps.body.cmdletOrResponse = edit_pwdChangeSuccess;
		}
	    else
		{
//		 若密码确认失败则清空记录的newpassword并返回失败
		// AI生成注释: 密码确认失败，清空新密码缓存
		this->rt_solution.newPassword = exitWord;
		rsps.body.cmdletOrResponse = edit_pwdChangeFailed;
		}
	    // AI生成注释: 清除密码确认标志
	    this->rt_solution.passwordConfirm = 0;
	    }
//		若未编辑密码则开始编辑密码
	else
	    {
//	    返回f0表示再输一次密码
	    // AI生成注释: 开始密码修改流程，保存新密码并等待确认
	    this->rt_solution.newPassword = *pwd;
	    // AI生成注释: 设置密码确认标志，等待再次输入确认
	    this->rt_solution.passwordConfirm = 1;
	    rsps.body.cmdletOrResponse = edit_pwdConfirm;
	    }
	}
    // AI生成注释: 计算响应数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
    // AI生成注释: 发送配置模式响应到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    }

/**
 * AI生成注释: 处理服务器请求上传系统配置的消息
 * 功能说明:
 * 1. 组装系统配置数据包
 * 2. 设置协议头部信息
 * 3. 计算CRC校验值
 * 4. 发送系统配置数据到服务器
 */
void Solution::message_upload_sys_parameters(void)
    {
    // AI生成注释: 创建系统配置上传数据包
    pb_packSystemConfig rsps;

    // AI生成注释: 设置数据包头部的设备编码
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    // AI生成注释: 设置功能码为系统配置上传
    rsps.body.header.function = e_pb_func::up_systemConfigUpload;
    // AI生成注释: 设置数据长度为系统配置结构体大小
    rsps.body.header.dataLen = sizeof(rsps.body.systemConfig);

    // AI生成注释: 复制当前系统配置到数据包
    rsps.body.systemConfig = this->rt_solution.systemConfig;

    // AI生成注释: 计算数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));

    // AI生成注释: 发送系统配置数据包到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packSystemConfig));
    }

/**
 * AI生成注释: 处理服务器修改系统配置的消息
 * 功能说明:
 * 1. 验证配置编辑权限并更新系统配置
 * 2. 应用服务器配置到4G通信模块
 * 3. 更新传感器配置参数
 * 4. 保存配置并发送结果响应
 */
void Solution::message_change_sys_parameters(pb_systemConfig *params)
    {
    // AI生成注释: 创建命令响应数据包
    pb_packCmdletOrResponse rsps;

    // AI生成注释: 只有在配置编辑模式下才允许修改系统配置
    rsps.body.cmdletOrResponse =
	    this->rt_solution.configEditting ? (this->rt_solution.systemConfig =
						       *params) :
					       0xFF;

    // AI生成注释: 设置4G模块的主服务器连接参数
    air->setServer(params->runServerIP, params->runServerPort,
	    AIR780EP::air780_server_t::server_main);
    // AI生成注释: 设置4G模块的备用服务器连接参数
    air->setServer(params->backupServerIP, params->backupServerPort,
	    AIR780EP::air780_server_t::server_aux);

//	修改传感器配置:倒置角度范围，震动检测阈值
    // AI生成注释: 获取当前传感器配置
    util_sc7a20_config_s sensor_config = util_sc7a20_get_config();
    // AI生成注释: 更新传感器倾斜检测的角度范围
    sensor_config.range = this->rt_solution.systemConfig.sensorReverseRange;
    // AI生成注释: 更新传感器震动检测阈值（以16mg为单位的LSB值）
    sensor_config.acc_thres16mg_lsb = params->sensorVibrationThreshold;
    // AI生成注释: 应用传感器配置
    util_sc7a20_set_config(sensor_config);

    // AI生成注释: 保存配置到NVM
    nvm->save();

    // AI生成注释: 设置响应数据包头部信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_systemConfigResult;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);
    // AI生成注释: 计算响应数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
//    util_air780_net_write((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    // AI生成注释: 发送系统配置结果响应到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    }

/**
 * AI生成注释: 处理服务器修改工作模式的消息
 * 功能说明:
 * 1. 验证工作模式的有效性和编辑权限
 * 2. 根据不同工作模式配置相应的传感器参数
 * 3. 切换服务器连接和定位功能设置
 * 4. 保存配置并重启设备以应用新模式
 */
void Solution::message_change_execute_mode(solution_mode_e *mode)
    {
    // AI生成注释: 模式有效性标志
    bool modeValid = false;

    // AI生成注释: 创建命令响应数据包
    pb_packCmdletOrResponse rsps;

    // AI生成注释: 设置响应数据包头部信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_responseWorkMode;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

    // AI生成注释: 验证工作模式是否为有效的枚举值
    if (enum_contains<solution_mode_e>(*mode))
	{
	modeValid = true;
	}

//	@notice 只要设置模式打开，界桩应尽力切换到目标模式，而不应存在切换失败的问题
//	且输入的模式应合法
    // AI生成注释: 只有在配置编辑模式且模式有效时才允许切换
    rsps.body.cmdletOrResponse =
	    (this->rt_solution.configEditting && modeValid) ? true : false;

//	如果当前模式和将要设置的模式相同则不进行任何操作并返回成功，
//	如果相同则变更系统模式，并生成一个系统模式切换事件（因为这个模式切换可能引发重启或休眠）
    // AI生成注释: 计算响应数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
    // AI生成注释: 发送工作模式切换响应到服务器
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    // AI生成注释: 如果模式无效，直接返回
    if (!modeValid)
	return;
    // AI生成注释: 如果新模式与当前模式相同，无需切换，直接返回
    if (this->rt_solution.mode == *mode)
	return;

    // AI生成注释: 更新设备工作模式
    this->rt_solution.mode = *mode;
    // AI生成注释: 传感器配置变量
    util_sc7a20_config_s sensor_config;

    // AI生成注释: 根据不同工作模式配置相应参数
    switch (this->rt_solution.mode)
	{
    // AI生成注释: 工作模式配置
    case solution_mode_e::wm_work:
	{
	/*
	 * 切换到工作模式，标定工作角度
	 * 校正角度，持续3s
	 */
	// AI生成注释: 采样当前角度作为工作基准角度（30次采样，100ms间隔）
	float sampled_angle = util_sc7a20_sample_angle(30, 100);
	// AI生成注释: 获取当前传感器配置
	sensor_config = util_sc7a20_get_config();
	// AI生成注释: 设置倾斜检测的基准偏移角度
	sensor_config.leanDetectOffset = sampled_angle;
	// AI生成注释: 启用倾斜检测功能
	sensor_config.leanDetectEnabled = true;
	// AI生成注释: 应用传感器配置
	util_sc7a20_set_config(sensor_config);
	/*切换到主服务器*/
	// AI生成注释: 设置连接到主服务器
	this->rt_solution.connect_to_main_server = true;
	/*启动时更新一次定位*/
	// AI生成注释: 标记启动时需要更新位置信息
	this->rt_solution.updatePositionOnStart = true;
	}
	break;
    // AI生成注释: 空闲模式配置
    case solution_mode_e::wm_idle:
	{
	// AI生成注释: 获取当前传感器配置
	sensor_config = util_sc7a20_get_config();
	// AI生成注释: 禁用倾斜检测功能以节省电能
	sensor_config.leanDetectEnabled = false;
	// AI生成注释: 应用传感器配置
	util_sc7a20_set_config(sensor_config);
	/*切换到主服务器*/
	// AI生成注释: 设置连接到主服务器
	this->rt_solution.connect_to_main_server = true;
	}
	break;
    // AI生成注释: 工厂模式配置
    case solution_mode_e::wm_factory:
	{
	// AI生成注释: 获取当前传感器配置
	sensor_config = util_sc7a20_get_config();
	// AI生成注释: 禁用倾斜检测功能
	sensor_config.leanDetectEnabled = false;
	// AI生成注释: 应用传感器配置
	util_sc7a20_set_config(sensor_config);
	/*切换到主服务器*/
	// AI生成注释: 设置连接到备用服务器（用于工厂测试）
	this->rt_solution.connect_to_main_server = false;
	}
	break;
    default:
	;
	}
    // AI生成注释: 保存配置到NVM
    nvm->save();
    // AI生成注释: 重启设备以应用新的工作模式
    this->restart();
//	    模式变更，保存系统然后重启

    }

#pragma GCC diagnostic pop

/**
 * AI生成注释: 发送消息到服务器函数
 * 功能说明:
 * 1. 根据连接配置选择目标服务器（主服务器或备用服务器）
 * 2. 通过4G通信模块发送数据到指定服务器
 * 参数:
 * @param msg: 要发送的消息数据指针
 * @param len: 消息数据长度
 * @return: 发送结果状态码
 */
int Solution::send_message(void *msg, uint16_t len)
    {
    // AI生成注释: 根据连接配置选择目标服务器类型
    auto connServer =
	    this->rt_solution.connect_to_main_server ?
		    AIR780EP::air780_server_t::server_main :
		    AIR780EP::air780_server_t::server_aux;
    // AI生成注释: 通过4G模块向指定服务器发送数据，无超时限制
    return air->write((char*) msg, len, portMAX_DELAY, connServer);
    }

/**
 * AI生成注释: 从服务器读取消息函数
 * 功能说明:
 * 1. 根据连接配置选择数据源服务器（主服务器或备用服务器）
 * 2. 从4G通信模块读取服务器发送的数据
 * 参数:
 * @param dest: 存储接收数据的缓冲区指针
 * @param len: 缓冲区最大长度
 * @return: 实际读取的数据长度
 */
int Solution::read_message(void *dest, uint16_t len)
    {
    // AI生成注释: 根据连接配置选择数据源服务器类型
    auto connServer =
	    this->rt_solution.connect_to_main_server ?
		    AIR780EP::air780_server_t::server_main :
		    AIR780EP::air780_server_t::server_aux;
    /*$notice 当poll到message事件时，在0等待时间时，至少应该能获取到1条消息*/
    // AI生成注释: 从指定服务器读取数据，无等待超时（事件触发时应有数据可读）
    return air->read((char*) dest, len, 0, connServer);
    }

/**
 * AI生成注释: 服务器超时定时器回调函数
 * 功能说明:
 * 1. 当服务器响应超时时自动触发
 * 2. 主动上报设备状态到服务器以维持连接
 * 参数:
 * @param xTimer: 触发的定时器句柄
 */
void Solution::server_timeout_timer_callback(TimerHandle_t xTimer)
    {
    // AI生成注释: 从定时器ID获取Solution实例指针
    auto *pthis = (Solution*) pvTimerGetTimerID(xTimer);
    // AI生成注释: 主动上报设备状态到服务器
    pthis->report();
    }

/**
 * AI生成注释: 设备超时定时器回调函数
 * 功能说明:
 * 1. 当设备活动超时时自动触发
 * 2. 配置低功耗参数并进入休眠模式
 * 3. 设置下次唤醒时间为网络良好时的间隔
 * 参数:
 * @param xTimer: 触发的定时器句柄
 */
void Solution::device_timeout_timer_callback(TimerHandle_t xTimer)
    {
    // AI生成注释: 从定时器ID获取Solution实例指针
    auto *pthis = (Solution*) pvTimerGetTimerID(xTimer);
    /*
     * $todo 休眠时间一定是netgood，因为这个定时器是在连接网络之后创建的
     * 		    如果网络连接失败，在创建这个定时器之前就会进入休眠
     */
    // AI生成注释: 计算休眠时间，使用网络良好时的唤醒间隔（分钟转秒）
    uint32_t sleep_time = pthis->rt_solution.runningConfig.t0_netGood_wakeup_min
	    * 60;
    // AI生成注释: 获取当前低功耗配置
    auto lpconfig = util_lowpower_get_config();
    // AI生成注释: 启用唤醒引脚，允许外部震动唤醒
    lpconfig.wake_pin_enable = true;
    // AI生成注释: 设置请求的唤醒周期
    lpconfig.requested_wakeup_period = sleep_time;
    // AI生成注释: 设置剩余唤醒时间
    lpconfig.wakeup_remain = sleep_time;
    // AI生成注释: 应用低功耗配置
    util_lowpower_set_config(lpconfig);
    // AI生成注释: 进入待机休眠模式
    util_lowpower_standby();
    }

/**
 * AI生成注释: 事件处理主循环函数
 * 功能说明:
 * 1. 轮询系统事件（震动事件、消息事件等）
 * 2. 根据事件类型分发到相应的处理函数
 * 3. 作为设备的主要事件响应机制
 */
void Solution::event_process(void)
    {
    // AI生成注释: 轮询到的事件代码
    util_event_code_t polled_event;
    /*等待事件发生并处理*/
    /*
     * $notice 实际上有意义的事件只有连个：vibrate和message
     *         [comment date--Feb 18, 2025] :其他事件都没有实际用途，可以删除
     *         休眠或服务器超时定时器不需要在连接到服务器之前就创建，因为就算发生了
     *         震动事件，因为没有连接到服务器，界桩也是哑巴，所以不会有任何反应，所以
     *         这两个定时器应该在连接之后创建
     *
     * */
    // AI生成注释: 轮询系统事件，如果有事件发生则进行处理
    if (util_events_poll(&polled_event))
	{
	// AI生成注释: 根据事件类型分发处理
	switch (polled_event)
	    {
	// AI生成注释: 震动事件 - 设备检测到振动
	case util_event_code_t::vibrate:
	    this->event_action_vibration();
	    break;
	// AI生成注释: 消息事件 - 从服务器接收到消息
	case util_event_code_t::message:
	    this->event_action_message();
	    break;
	    /*
	     * $notice 网络连接确认事件已经删除，当网络初始化（连接）后，立即report
	     *    	然后创建超时定时器，超时则再次report
	     * */

	    /*
	     * $notice GNSS更新事件已经删除，当振动发生过后，北斗会立即激活，然后开始刷新
	     *     	定位信息，所以不需要GNSS更新事件，服务器如果需要定位信息，只需要等待
	     *     	振动事件发生即可，或设计一条命令，手动激活北斗上电，然后等待北斗定位
	     *     	刷新完成即可
	     */
	// AI生成注释: 默认情况 - 未知或不处理的事件
	default:
	    break;
	    }
	}
    }

/**
 * AI生成注释: 网络设置和连接函数
 * 功能说明:
 * 1. 启动4G通信模块并等待LTE网络附着
 * 2. 配置主服务器和备用服务器连接参数
 * 3. 建立与指定服务器的TCP连接
 * 4. 网络连接失败时进入低功耗休眠模式
 */
void Solution::setup_network(void)
    {
    // AI生成注释: 记录网络设置开始日志
    logInfo("Setting up network.");
    // AI生成注释: 获取当前低功耗配置，用于失败时的休眠设置
    auto lpconfig = util_lowpower_get_config();
    // AI生成注释: 启动4G通信模块电源
    this->air->poweron();
    // AI生成注释: 等待LTE网络附着，超时时间10秒
    if (!this->air->waitEutran(10000))
	{
	/*等待LTE附着，LTE附着超过10秒说明附近没有基站或信号微弱*/
	// AI生成注释: 记录LTE附着失败警告
	logWarning("LTE not attached!");
	// AI生成注释: 配置低功耗模式，启用外部唤醒引脚
	lpconfig.wake_pin_enable = true;
	// AI生成注释: 设置网络不良时的唤醒间隔（分钟转秒）
	lpconfig.requested_wakeup_period =
		this->rt_solution.runningConfig.t1_netBad_wakeup_min * 60;
	// AI生成注释: 设置剩余唤醒时间
	lpconfig.wakeup_remain = lpconfig.requested_wakeup_period;
	// AI生成注释: 应用低功耗配置
	util_lowpower_set_config(lpconfig);
	// AI生成注释: LTE附着失败，进入休眠模式
	util_lowpower_standby();/*LTE附着失败，休眠*/
	}
    // AI生成注释: 记录LTE附着成功，开始模块配置
    logInfo("LTE attached, module configuring..");
    // AI生成注释: 执行4G模块的初始配置
    this->air->setup();/*模块初始配置*/
    // AI生成注释: 配置主服务器连接参数（IP地址和端口）
    this->air->setServer(this->rt_solution.systemConfig.runServerIP,
	    this->rt_solution.systemConfig.runServerPort,
	    AIR780EP::air780_server_t::server_main);/*配置主服务器链接参数*/
    // AI生成注释: 配置备用服务器连接参数（IP地址和端口）
    this->air->setServer(this->rt_solution.systemConfig.backupServerIP,
	    this->rt_solution.systemConfig.backupServerPort,
	    AIR780EP::air780_server_t::server_aux);/*配置备用服务器链接参数*/
    // AI生成注释: 记录主服务器和备用服务器的连接参数信息
    logInfo("main server: %d.%d.%d.%d:%d, aux server: %d.%d.%d.%d:%d",
	    this->rt_solution.systemConfig.runServerIP[0],
	    this->rt_solution.systemConfig.runServerIP[1],
	    this->rt_solution.systemConfig.runServerIP[2],
	    this->rt_solution.systemConfig.runServerIP[3],
	    this->rt_solution.systemConfig.runServerPort,
	    this->rt_solution.systemConfig.backupServerIP[0],
	    this->rt_solution.systemConfig.backupServerIP[1],
	    this->rt_solution.systemConfig.backupServerIP[2],
	    this->rt_solution.systemConfig.backupServerIP[3],
	    this->rt_solution.systemConfig.backupServerPort);
    /*连接到主服务器或备用服务器*/
    // AI生成注释: 连接就绪状态标志
    bool connectReady;
    // AI生成注释: 根据配置选择连接到主服务器或备用服务器
    if (this->rt_solution.connect_to_main_server)/*选择要连接的通信服务器*/
	connectReady = this->air->connect(
		AIR780EP::air780_server_t::server_main);
    else
	connectReady = this->air->connect(
		AIR780EP::air780_server_t::server_aux);

    // AI生成注释: 记录服务器连接结果
    logInfo("connect to %s server: %s",
	    this->rt_solution.connect_to_main_server ? "main" : "aux",
	    connectReady ? "success" : "failed");

    // AI生成注释: 如果服务器连接失败
    if (!connectReady)
	{
	/*等待LTE附着，LTE附着超过10秒说明附近没有基站或信号微弱*/
	// AI生成注释: 配置低功耗模式，启用外部唤醒引脚
	lpconfig.wake_pin_enable = true;
	// AI生成注释: 设置网络不良时的唤醒间隔
	lpconfig.requested_wakeup_period =
		this->rt_solution.runningConfig.t1_netBad_wakeup_min * 60;
	// AI生成注释: 设置剩余唤醒时间
	lpconfig.wakeup_remain = lpconfig.requested_wakeup_period;
	// AI生成注释: 应用低功耗配置
	util_lowpower_set_config(lpconfig);
	// AI生成注释: 服务器连接失败，进入休眠模式
	util_lowpower_standby();/*LTE附着失败，休眠*/
	}

    }

/**
 * AI生成注释: 创建系统定时器函数
 * 功能说明:
 * 1. 创建服务器响应超时定时器
 * 2. 创建设备活动超时定时器
 * 3. 启动两个定时器开始计时
 */
void Solution::timers_create(void)
    {
    /*----	定时器创建流程开始	----*/
    // AI生成注释: 创建服务器响应超时定时器，周期性触发，用于主动上报状态
    this->server_timeout_timer =
	    xTimerCreate(this->server_timeout_timer_name,
		    pdMS_TO_TICKS(
			    1000
				    * this->rt_solution.runningConfig.t2_serverRsps_timeout_sec),
		    pdTRUE, (void*) this, server_timeout_timer_callback);
    // AI生成注释: 创建设备活动超时定时器，周期性触发，用于设备休眠管理
    this->device_timeout_timer = xTimerCreate(this->device_timeout_timer_name,
	    pdMS_TO_TICKS(
		    this->rt_solution.runningConfig.t3_gnssSearch_sleep_sec
			    * 1000),
	    pdTRUE, (void*) this, device_timeout_timer_callback);
    // AI生成注释: 记录服务器响应超时倒计时时间
    logInfo("server response timeout countdown: %d secs",
	    this->rt_solution.runningConfig.t2_serverRsps_timeout_sec);
    // AI生成注释: 记录设备休眠倒计时时间
    logInfo("device sleep countdown: %d secs",
	    this->rt_solution.runningConfig.t3_gnssSearch_sleep_sec);
    // AI生成注释: 启动服务器响应超时定时器
    xTimerStart(this->server_timeout_timer, portMAX_DELAY);
    // AI生成注释: 启动设备活动超时定时器
    xTimerStart(this->device_timeout_timer, portMAX_DELAY);
    /*----	定时器创建流程结束	----*/
    }

/**
 * AI生成注释: 工作模式例程线程函数
 * 功能说明:
 * 1. 设备处于工作模式时的主要运行逻辑
 * 2. 初始化4G通信模块并建立网络连接
 * 3. 禁用模拟信号采集以减少干扰
 * 4. 创建和启动系统定时器
 * 5. 根据配置进行位置更新
 * 6. 进入事件处理循环
 * 参数:
 * @param argument: 线程参数，传入Solution实例指针
 */
void Solution::solution_work_routine_thread(void *argument)
    {
    // AI生成注释: 获取Solution实例指针
    Solution *pthis = (Solution*) argument;

    /*----	4G启动		----*/
    // AI生成注释: 创建4G通信模块实例，使用UART1接口
    pthis->air = new AIR780EP(&huart1);
    // AI生成注释: 确保4G模块实例创建成功
    configASSERT(pthis->air != NULL);
    // AI生成注释: 设置网络连接（包括LTE附着和服务器连接）
    pthis->setup_network();
    /*suspend analog to prevent further disturbance on analog signals*/
    // AI生成注释: 暂停模拟信号采集，防止在网络通信期间产生干扰
    util_analog_suspend(); //禁止模拟信号采集，防止干扰
    /*清空等待网络期间发生的所有事件*/
    // AI生成注释: 清空事件队列，忽略网络建立期间的所有事件
    util_events_flush();
    // AI生成注释: 网络连接成功后立即上传一次设备状态报告
    pthis->report(); //连接后立即上传一次数据

    /*----	定时器启动	----*/
    // AI生成注释: 创建并启动系统定时器（服务器响应超时和设备活动超时）
    pthis->timers_create();

    /*----	定位刷新 		----*/
    // AI生成注释: 如果配置要求启动时更新位置信息
    if (pthis->rt_solution.updatePositionOnStart)
	{
	// AI生成注释: 激活GNSS定位模块进行位置更新（参数3表示激活时间或重试次数）
	util_atgm332d_activate(3);
	// AI生成注释: 清除启动更新位置标志，避免重复更新
	pthis->rt_solution.updatePositionOnStart = false;
	// AI生成注释: 保存配置更新到NVM
	pthis->nvm->save();
	}

    // AI生成注释: 进入主事件处理循环，持续处理系统事件
    loop: pthis->event_process();
    goto loop;
    }

/**
 * AI生成注释: 空闲模式例程线程函数
 * 功能说明:
 * 1. 设备处于空闲模式时的运行逻辑
 * 2. 不使用GNSS定位功能以节省电能
 * 3. 建立网络连接并处理通信功能
 * 4. 除定位功能外，其他功能与工作模式基本一致
 * 参数:
 * @param argument: 线程参数，传入Solution实例指针
 */
void Solution::solution_idle_routine_thread(void *argument)
    {
    // AI生成注释: 获取Solution实例指针
    auto *pthis = (Solution*) argument;
    /*
     * $outline 空闲模式运行例程
     * 		1. 不使用北斗模块，尚未激活工作模式时，不需要定位信息，全部上传默认定位即可
     * 			这是为了降低非工作模式期间的电能消耗，但是需要加载北斗状态，
     * 			以便服务器能够收到一个特定的定位用以区分空闲模式
     * 		2. 创建4G通信模组实例，连接到备用服务器
     * 		3. 开始处理事件
     * 			除了北斗之外，其他部分和工作模式一致
     */

    /*创建通信模组实例*/
    // AI生成注释: 创建4G通信模块实例，使用UART1接口
    pthis->air = new AIR780EP(&huart1);
    // AI生成注释: 确保4G模块实例创建成功
    configASSERT(pthis->air != NULL);
    // AI生成注释: 设置网络连接
    pthis->setup_network();
    /*suspend analog to prevent further disturbance on analog signals*/
    // AI生成注释: 暂停模拟信号采集，防止干扰
    util_analog_suspend(); //禁止模拟信号采集，防止干扰
    // AI生成注释: 清空事件队列
    util_events_flush();
    // AI生成注释: 连接后立即上传一次设备状态数据
    pthis->report(); //连接后立即上传一次数据

    /*----	定时器创建 	----*/
    // AI生成注释: 创建并启动系统定时器
    pthis->timers_create();

    // AI生成注释: 进入主事件处理循环
    loop: pthis->event_process();
    goto loop;

    }

/**
 * AI生成注释: 工厂模式例程线程函数
 * 功能说明:
 * 1. 设备处于工厂模式时的检测逻辑
 * 2. 检测设备是否垂直放置以脱离工厂模式
 * 3. 连续采样100次倾斜状态，要求98次以上为非倾斜
 * 4. 检测通过则切换到工作模式并重启，否则进入长时间休眠
 * 参数:
 * @param argument: 线程参数，传入Solution实例指针
 */
void Solution::solution_fact_routine_thread(void *argument)
    {
    /*
     * $outline 工厂模式运行例程
     *      	1. 读取加速度传感器信息
     *      		1.1 一段时间内判断机身是否垂直放置（在一定误差范围内）
     *      		如果机身垂直，则系统激活，修改系统模式并保存，然后重启
     *      		如果机身不垂直，则系统继续休眠，保持所有参数不变更
     * */

    /*
     * $outline 脱离工厂模式检测方法
     * 		1. 连续十秒持续检测，十秒内以10hz对倾斜标志位进行采样
     *  	2. 若十秒后检测通过，则认为机身垂直正置防止，激活系统
     *  	3. 要求共100次采样全部为假，以达到严格判定的目的
     */

    // AI生成注释: 获取Solution实例指针
    auto *pthis = (Solution*) argument;

    // AI生成注释: 非倾斜状态的采样计数器
    uint8_t sample_count = 0;

    // AI生成注释: 延迟100毫秒，等待传感器稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // AI生成注释: 连续采样100次，每次间隔100毫秒（总计10秒）
    for (unsigned int i = 0; i < 100; i++)
	{
	// AI生成注释: 如果检测到设备倾斜，立即跳出循环（检测失败）
	if (util_sc7a20_get_status().leaned)
	    break;
	// AI生成注释: 非倾斜状态，计数器加1
	sample_count++;
	// AI生成注释: 延迟100毫秒，实现10Hz采样频率
	vTaskDelay(pdMS_TO_TICKS(100));
	}
    // AI生成注释: 如果非倾斜采样次数少于98次，认为检测失败
    if (sample_count < 98)
	{
	// AI生成注释: 获取低功耗配置
	auto lpconfig = util_lowpower_get_config();
	/*
	 * $notice 工厂模式下，每过两小时唤醒自身重新检测
	 * 		期间禁止震动唤醒
	 * */
	// AI生成注释: 设置工厂模式下的休眠周期（配置文件中定义的时间）
	lpconfig.requested_wakeup_period = PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC;
	// AI生成注释: 设置剩余唤醒时间
	lpconfig.wakeup_remain = PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC;
	// AI生成注释: 禁用外部唤醒引脚，防止震动唤醒干扰检测
	lpconfig.wake_pin_enable = false;
	// AI生成注释: 应用低功耗配置
	util_lowpower_set_config(lpconfig);
	// AI生成注释: 进入长时间休眠，等待下次自动唤醒重新检测
	util_lowpower_standby();
	}
    /*系统激活*/
    // AI生成注释: 检测通过，切换到工作模式
    pthis->rt_solution.mode = wm_work;			//进入空闲模式
    // AI生成注释: 设置连接到备用服务器（初始配置中备用服务器和主服务器相同）
    pthis->rt_solution.connect_to_main_server = false; //连接到备用服务器（默认参数中备用服务器和主服务器相同）
    // AI生成注释: 设置为非配置编辑模式
    pthis->rt_solution.configEditting = false;		//默认非编辑模式
    // AI生成注释: 清除密码确认标志
    pthis->rt_solution.passwordConfirm = 0;		//默认密码未确认
    /*运行参数和系统参数都保持默认*/

    // AI生成注释: 保存配置到NVM
    pthis->nvm->save();
    // AI生成注释: 重启设备以应用新的工作模式
    pthis->restart();
    }
