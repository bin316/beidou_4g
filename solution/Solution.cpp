/*
 * Solution.cpp
 *
 *  Created on: Jan 22, 2025
 *      Author: IRIS
 *      AI生成注释: 此文件实现了北斗界桩设备的核心解决方案类，包含设备的工作模式、通信协议、参数配置等主要功能
 *
 *
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
 * 修改运行配置流程：
 * 1. 服务器发送进入配置模式请求附带对应的设备密码，对应功能码为09
 * 2. 设备验证密码正确后进入配置编辑状态，等待服务器发送新的配置参数，功能码为08
 * 3. 服务器发送新的配置参数，功能码为05
 * 4. 设备验证新配置参数的合法性，合法则保存到NVM并回复成功响应，功能码为04
 * 由于使能界桩配置模式与修改密码对应的都是09功能码，要注意应答回复，如果失败有可能是服务器端未接收到08功能码，
 * 会再次发送09功能码，导致设备进入修改密码状态
 * 
 * 在定期唤醒和震动唤醒时：
 * 如果两分钟搜星没完成会命令上传数据  四分钟搜星没完成会进入休眠(低功耗)
 * 
 * 更改内容：
 * 添加对服务器端回复心跳包的处理操作
 * 非震动唤醒在收到回复心跳包之后进入休眠状态
 * 震动唤醒忽略收到的回复心跳包
 * ------------------------------------------------------------------------------
 * 定位状态标志位“上报后置否”设计说明
 * ------------------------------------------------------------------------------
 * 协议中 pb_report.status 的 bit0 为定位状态位（position_fixed），表示本帧
 * 上报的 geo[] 是否为新定位数据：1=新定位数据，0=非新/旧定位数据。
 * 设计策略：每次在 report() 中成功发送上报数据后，立即调用
 * util_atgm332d_clear_fix_flag() 将北斗模块内部的 position_fixed 置为 false。
 * 目的与效果：
 *   - 下一帧上报时，只有在上次上报之后又解析到新的 RMC，定位状态位才会
 *     再次为 1；否则为 0。避免多帧共用同一次定位结果却均标为“新”。 
 *   - 提升服务器端对“本帧是否为新定位数据”的语义准确性，减少将旧位置误判为
 *     新定位数据的情况。
 * 相关接口：util_atgm332d_clear_fix_flag() 定义于 bsp/util_atgm332d.cpp。
 *
 * ------------------------------------------------------------------------------
 * 卫星数量 sats 字段说明
 * ------------------------------------------------------------------------------
 * report() 中 rsps.body.report.sats = util_atgm332d_get_status().sats，
 * 将北斗解析的卫星数量填入上报数据包，用于表示定位质量。
 *
 * ------------------------------------------------------------------------------
 * 定位状态标志位“上报后置否”设计说明
 * ------------------------------------------------------------------------------
 * 协议 pb_report.status 的 bit0 为定位状态位，表示本帧 geo[] 是否为新定位数据。
 * 每次 report() 成功发送后调用 util_atgm332d_clear_fix_flag() 将其置否，使下一帧
 * 仅在有新 RMC 时才标为“新定位数据”，提升服务器对“是否为新定位数据”判断的准确性。
 * 详见 bsp/util_atgm332d.cpp、solution/Protocol.h、bsp/utilties.h 中的相关说明。
 * 
 * 288-313添加强制使用宏定义中的服务器地址和端口号的代码
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
#include "util_agnss.h"

/** locate_switch 有效位：bit0 GNSS / bit1 AGNSS / bit2 LBS（协议约定） */
static constexpr uint8_t kLocateSwitchMask = 0x07u;
static constexpr uint8_t kLocateBitGnss = 0x01u;
static constexpr uint8_t kLocateBitAgnss = 0x02u;
static constexpr uint8_t kLocateBitLbs = 0x04u;

static bool locate_gnss_on(uint8_t sw)
    {
    return (sw & kLocateBitGnss) != 0;
    }
static bool locate_agnss_on(uint8_t sw)
    {
    return (sw & kLocateBitAgnss) != 0;
    }
static bool locate_lbs_on(uint8_t sw)
    {
    return (sw & kLocateBitLbs) != 0;
    }

/** 开 AGNSS 必须同时开 GNSS */
static bool locate_switch_agnss_without_gnss(uint8_t sw)
    {
    return locate_agnss_on(sw) && !locate_gnss_on(sw);
    }

/** 清预留位；孤立 AGNSS 一并清掉（对齐 Slope，供出厂/上电/写 NVM） */
static uint8_t locate_switch_normalize(uint8_t locate_switch)
    {
    uint8_t v = (uint8_t) (locate_switch & kLocateSwitchMask);
    if (locate_switch_agnss_without_gnss(v))
	{
	v = (uint8_t) (v & (uint8_t) (~kLocateBitAgnss));
	}
    return v;
    }

/** 上位机改参：各分区已 save 进页缓冲后，统一擦写 Flash（对齐 Slope 应答前 sync） */
static void nvm_commit_host_config(void)
    {
    __flash_sync();
    logInfo("NVM: 上位机配置已刷入Flash");
    }

/** 进配置模式允许的意图：随后要写的下行功能码（不含改密 9） */
static bool config_enter_intent_valid(uint8_t intent)
    {
    return intent == e_pb_func::down_configRunning
	    || intent == e_pb_func::down_configSystem
	    || intent == e_pb_func::down_configWorkMode
	    || intent == e_pb_func::down_configLocateGeo
	    || intent == e_pb_func::down_configLocateSwitch;
    }

/**
 * 出厂默认 locate_switch：三宏 0/1 按位拼装后再 normalize
 * （改 PRODUCT_CONFIG 三宏只影响恢复出厂；已落盘 NVM 需协议改或擦除）
 */
static uint8_t locate_switch_factory_default(void)
    {
    const uint8_t raw = (uint8_t) (
	    ((PROD_CFG_DEFAULT_LOCATE_GNSS) ? 1u : 0u)
	    | (((PROD_CFG_DEFAULT_LOCATE_AGNSS) ? 1u : 0u) << 1)
	    | (((PROD_CFG_DEFAULT_LOCATE_LBS) ? 1u : 0u) << 2));
    return locate_switch_normalize(raw);
    }

/** 协议下行：AGNSS 无 GNSS 为非法（拒收且不落盘） */
static bool locate_switch_is_valid(uint8_t sw)
    {
    return !locate_switch_agnss_without_gnss(
	    (uint8_t) (sw & kLocateSwitchMask));
    }

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
	    // 默认连接到主服务器
	    .connect_to_main_server = true,
	    // AI生成注释: 启动时是否更新位置信息的标志位
	    .updatePositionOnStart = false,
	    /* 出厂 locateSwitch：PROD_CFG_DEFAULT_LOCATE_* 三宏 → factory_default */
	    .locate_switch = locate_switch_factory_default()
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
    logInfo("例程: 初始化");

    // AI生成注释: 创建NVM实例，用于参数的持久化存储，包含运行时和出厂配置
    nvm = new NVM(NVM::partition_solution, (uint8_t*) &this->rt_solution,
	    &fc_solution, sizeof(this->rt_solution));
    // AI生成注释: 确保NVM实例创建成功，否则触发断言
    configASSERT(nvm != NULL);

    // AI生成注释: 记录NVM创建成功日志
    logInfo("例程: NVM已创建");
    // AI生成注释: 从NVM加载配置参数
    nvm->load();

    /* 将运行参数 t5 消抖落到加计冷却（对齐 Slope）；仅变化时写 NVM */
    {
    util_sc7a20_config_s sensor_config = util_sc7a20_get_config();
    const uint32_t want_ms =
	    (uint32_t) this->rt_solution.runningConfig.t5_motionDetect_delay_sec
		    * 1000u;
    if (sensor_config.cool_down_timeout != want_ms)
	{
	sensor_config.cool_down_timeout = want_ms;
	util_sc7a20_set_config(sensor_config);
	}
    logInfo("震动消抖: %u秒",
	    (unsigned) this->rt_solution.runningConfig.t5_motionDetect_delay_sec);
    }

    // // AI生成注释: 强制设置IP和端口，确保使用宏定义的值
    // uint8_t default_main_ip[] = PROD_CONFIG_FACTORY_DEFAULT_MAIN_IP;
    // uint8_t default_aux_ip[] = PROD_CONFIG_FACTORY_DEFAULT_AUX_IP;
    // this->rt_solution.systemConfig.runServerIP[0] = default_main_ip[0];
    // this->rt_solution.systemConfig.runServerIP[1] = default_main_ip[1];
    // this->rt_solution.systemConfig.runServerIP[2] = default_main_ip[2];
    // this->rt_solution.systemConfig.runServerIP[3] = default_main_ip[3];
    // this->rt_solution.systemConfig.runServerPort = PROD_CONFIG_FACTORY_DEFAULT_MAIN_PORT;
    // this->rt_solution.systemConfig.backupServerIP[0] = default_aux_ip[0];
    // this->rt_solution.systemConfig.backupServerIP[1] = default_aux_ip[1];
    // this->rt_solution.systemConfig.backupServerIP[2] = default_aux_ip[2];
    // this->rt_solution.systemConfig.backupServerIP[3] = default_aux_ip[3];
    // this->rt_solution.systemConfig.backupServerPort = PROD_CONFIG_FACTORY_DEFAULT_AUX_PORT;
    // logInfo("IP和端口已配置: 主服务器 %d.%d.%d.%d:%d, 备用服务器 %d.%d.%d.%d:%d",
	//     this->rt_solution.systemConfig.runServerIP[0],
	//     this->rt_solution.systemConfig.runServerIP[1],
	//     this->rt_solution.systemConfig.runServerIP[2],
	//     this->rt_solution.systemConfig.runServerIP[3],
	//     this->rt_solution.systemConfig.runServerPort,
	//     this->rt_solution.systemConfig.backupServerIP[0],
	//     this->rt_solution.systemConfig.backupServerIP[1],
	//     this->rt_solution.systemConfig.backupServerIP[2],
	//     this->rt_solution.systemConfig.backupServerIP[3],
	//     this->rt_solution.systemConfig.backupServerPort);

    // AI生成注释: 检查是否为出厂默认配置
    if (nvm->isFactoryDefault())
	{
	// AI生成注释: 记录使用出厂默认配置的日志
	logInfo("例程: NVM恢复出厂默认");
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
     * 升级扩字段/非法组合：sanitize locate_switch 后写回
     * （出厂值来自 PROD_CFG_DEFAULT_LOCATE_*；已落盘合法值不改）
     */
    {
    const uint8_t sanitized =
	    locate_switch_normalize(this->rt_solution.locate_switch);
    if (sanitized != this->rt_solution.locate_switch)
	{
	logInfo("定位开关: 上电校正 0x%02X -> 0x%02X",
		(unsigned) this->rt_solution.locate_switch,
		(unsigned) sanitized);
	this->rt_solution.locate_switch = sanitized;
	nvm->save();
	}
    logInfo("定位开关: 0x%02X GNSS=%u AGNSS=%u LBS=%u (出厂默认=0x%02X)",
	    (unsigned) this->rt_solution.locate_switch,
	    locate_gnss_on(this->rt_solution.locate_switch) ? 1u : 0u,
	    locate_agnss_on(this->rt_solution.locate_switch) ? 1u : 0u,
	    locate_lbs_on(this->rt_solution.locate_switch) ? 1u : 0u,
	    (unsigned) locate_switch_factory_default());
    }

    /* 配置会话意图仅 RAM；唤醒后若 NVM 仍标 editing 则清掉，须重新进模式 */
    if (this->rt_solution.configEditting || this->rt_solution.passwordConfirm)
	{
	this->rt_solution.configEditting = 0;
	this->rt_solution.passwordConfirm = 0;
	this->rt_solution.newPassword =
	    {
	    0
	    };
	this->config_intent_ = 0;
	nvm->save();
	logInfo("配置会话: 上电清除残留 editing，需重新进模式");
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
	/* report/消息叠加；1536 使 RAM 溢出，取 1280 折中 */
	xTaskCreate(Solution::solution_work_routine_thread, "solution_work",
		1280, this, osPriorityNormal, &solution_thread_handle);
	break;
    // AI生成注释: 空闲模式 - 创建空闲例程线程，功能受限以节省电能
    case solution_mode_e::wm_idle:
	xTaskCreate(Solution::solution_idle_routine_thread, "solution_idle",
		768, this, osPriorityNormal, &solution_thread_handle);
	break;
    // AI生成注释: 默认情况（包括工厂模式）- 创建工厂测试例程线程
    default:
	xTaskCreate(Solution::solution_fact_routine_thread, "solution_fact",
		512, this, osPriorityNormal, &solution_thread_handle);
	}
    // AI生成注释: 确保任务句柄创建成功，否则触发断言
    configASSERT(solution_thread_handle != NULL);

    // AI生成注释: 记录启动的例程模式日志，使用枚举名称转换为字符串
    logInfo("例程: 已启动 %s", enum_name(rt_solution.mode).data());

    }

/**
 * AI生成注释: 设备状态报告函数
 * 功能说明:
 * 1. 收集传感器数据（加速度计、模拟信号、GNSS定位等）
 * 2. 组装协议数据包，包含设备状态、位置、时间等信息
 * 3. 计算CRC校验值并发送数据包到服务器
 * 4. 清除震动检测标志位，记录关键信息到日志
 * 5. 发送成功后清除定位状态标志位（上报后置否），见文件头部设计说明
 */
/**
 * @brief 分行打印原始 hex，避免单次撑爆 RTT/LOG 缓冲
 * @param tag  行前缀，如 "TX hex" / "RX hex"
 * @param raw  缓冲区
 * @param len  字节数
 */
static void log_hex_dump(const char *tag, const uint8_t *raw, unsigned len)
    {
    char line[80];
    unsigned pos = 0;
    line[0] = '\0';
    for (unsigned i = 0; i < len; i++)
	{
	if (pos + 4 >= sizeof(line))
	    {
	    logInfo("%s: %s", tag, line);
	    pos = 0;
	    line[0] = '\0';
	    }
	pos += (unsigned) snprintf(line + pos, sizeof(line) - pos, "%02X ",
		raw[i]);
	}
    if (pos > 0)
	{
	logInfo("%s: %s", tag, line);
	}
    }

/**
 * @brief 打印上报整包摘要；高频连报时跳过 hex，降低 snprintf/RTT 栈压（防 SoftReset）
 */
static void log_report_packet(const pb_packReport *pkt)
    {
    static TickType_t s_last_hex_tick = 0;
    const TickType_t now = xTaskGetTickCount();
    const bool dump_hex =
	    (s_last_hex_tick == 0)
		    || ((now - s_last_hex_tick) >= pdMS_TO_TICKS(2000));

    logInfo(
	    "上报字段: 时间=%lu 坐标_e4=[%ld,%ld] 状态=0x%02X 加速度=[%d,%d,%d] 倾角=%d 电压=%u 温度=%d CSQ=%u 卫星=%u",
	    (unsigned long) pkt->body.report.time,
	    (long) (pkt->body.report.geo[ID_LONGITUDE] * 10000.0f),
	    (long) (pkt->body.report.geo[ID_LATITUDE] * 10000.0f),
	    pkt->body.report.status, pkt->body.report.acc[ID_AXIS_X],
	    pkt->body.report.acc[ID_AXIS_Y], pkt->body.report.acc[ID_AXIS_Z],
	    pkt->body.report.angle, pkt->body.report.vbat, pkt->body.report.temp,
	    pkt->body.report.csq, pkt->body.report.sats);

    if (dump_hex)
	{
	s_last_hex_tick = now;
	log_hex_dump("上报hex", (const uint8_t*) pkt, sizeof(pb_packReport));
	}
    }

/**
 * @brief 打印从模组读到的原始收包（长度 + hex），便于对照 +RECEIVE
 */
static void log_rx_raw(const uint8_t *raw, uint16_t len)
    {
    static TickType_t s_last_rx_hex_tick = 0;
    const TickType_t now = xTaskGetTickCount();
    logInfo("收包原始 %u字节", (unsigned) len);
    /* 连报风暴时限频打 hex，减轻 solution 任务栈与 RTT 压力 */
    if (len > 0
	    && (s_last_rx_hex_tick == 0
		    || (now - s_last_rx_hex_tick) >= pdMS_TO_TICKS(2000)))
	{
	s_last_rx_hex_tick = now;
	log_hex_dump("收包hex", raw, len);
	}
    }

void Solution::report(void)
    {
    if (util_agnss_rx_is_active())
	{
	logInfo("上报: AGNSS进行中, 跳过");
	return;
	}

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
    /* geoStat：本周期 GNSS fix 或本周期 LBS 成功（对齐 Slope fill_for_report） */
    rsps.body.report.status = helper_status_byte_maker(this->rt_solution.mode,
	    util_lowpower_get_wake_source(), sensor.vibration_occured,
	    sensor.leaned, util_atgm332d_geo_valid_for_report());
    // AI生成注释: 设置温度值（转换为8位有符号整数）
    rsps.body.report.temp = (int8_t) analog.temperture;
    // AI生成注释: 设置RTC时间戳
    rsps.body.report.time = (uint32_t) util_lowpower_get_rtc();
    // AI生成注释: 设置电池电压（转换为0.1V为单位的8位整数）
    rsps.body.report.vbat = (uint8_t) (analog.vbat * 10.0f);
    /* 功能码1连报时避免每次 AT+CSQ（占 at_mutex，易与 RX/写包叠出 HardFault） */
    {
    static int s_csq_cache = 0;
    static TickType_t s_csq_tick = 0;
    const TickType_t now = xTaskGetTickCount();
    if (s_csq_tick == 0 || (now - s_csq_tick) >= pdMS_TO_TICKS(3000))
	{
	s_csq_cache = air->getCsq();
	s_csq_tick = now;
	}
    const int csq_now = s_csq_cache;
    rsps.body.report.csq = (csq_now < 0) ? 0 : (uint8_t) csq_now;
    }
    // AI生成注释: 设置用于定位的卫星数量（来自GGA的numSv，表示定位质量）
    rsps.body.report.sats = util_atgm332d_get_status().sats;
    // AI生成注释: 计算数据包的CRC校验值
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));

    log_report_packet(&rsps);

    // AI生成注释: 发送数据包到服务器
    const int send_ret = send_message((uint8_t*) &rsps, sizeof(pb_packReport));
    /* send_message 会清 awaiting；仅上报发送成功后再置位，供 OK 确认休眠 */
    if (send_ret >= 0)
	{
	mark_report_pending_ack();
	}

    // 上报完成后将定位状态标志位置否，使下一帧仅在有新 RMC 时才标为“新定位数据”
    util_atgm332d_clear_fix_flag();
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
    logInfo("芯片复位重启");
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
 * @brief 重置设备休眠倒计时（硬计时策略下收包不再调用，避免续命）
 */
void Solution::refresh_device(void)
    {
    xTimerReset(device_timeout_timer, portMAX_DELAY);
    }

/**
 * AI生成注释: 震动事件处理函数
 * 功能说明:
 * 1. 记录震动检测日志
 * 2. 刷新服务器响应超时定时器
 * 3. 如果处于工作模式，激活GNSS定位系统
 * 4. 立即上报设备状态到服务器
 */
void Solution::event_action_vibration(void)
    {
    logInfo("震动事件: 收到");
    xTimerReset(server_timeout_timer, portMAX_DELAY);
    /* 先 report，再 GNSS；LBS 仅本震动路径且看 locateSwitch（首包不含本周期新 LBS） */
    logInfo("震动事件: 立即上报");
    this->report();
    if (rt_solution.mode == solution_mode_e::wm_work)
	{
	logInfo("震动事件: 工作模式, 启动定位");
	this->start_locate();
	this->start_lbs_if_needed();
	}
    else
	{
	logInfo("震动事件: 非工作模式, 跳过定位");
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
    /* 先打原始长度与 hex（含 11B 短应答），再走心跳/协议解析 */
    log_rx_raw(message, read_size);

    // AI生成注释: 检查是否包含心跳包标识 "OK"（无功能码格式：FBFBFB + code + OK + CRC）
	if (read_size >= 11)
	{
	// AI生成注释: 查找心跳包起始标识符 FBFBFB
	uint8_t *hb_ptr = message;
	while (hb_ptr < message + read_size - 8)
		{
		if (hb_ptr[0] == 0xFB && hb_ptr[1] == 0xFB
			&& hb_ptr[2] == 0xFB)
		{
		// AI生成注释: 检查界桩编号是否匹配（4字节：major+minor+index）
		pb_code *hb_code = (pb_code*) (hb_ptr + 3);
		if (hb_code->major
			== this->rt_solution.systemConfig.code.major
			&& hb_code->minor
			== this->rt_solution.systemConfig.code.minor
			&& hb_code->index
			== this->rt_solution.systemConfig.code.index)
			{
			// AI生成注释: 检查是否是 "OK" 标记
			if (hb_ptr[7] == 'O' && hb_ptr[8] == 'K')
			{
			// AI生成注释: 验证CRC校验（从code到OK，共6字节）
			uint16_t hb_target_crc =
				(hb_ptr[10] << 8) | hb_ptr[9];
			uint16_t hb_calc_crc = HAL_CRC_Calculate(&hcrc,
				(uint32_t*) (hb_ptr + 3), 6);
			if (hb_calc_crc == hb_target_crc)
				{
				logInfo("确认收到OK回复");
				this->message_heartbeat();
				vPortFree(message);
				return;
				}
			}
			}
		}
		hb_ptr++;
		}
	}

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
		    logInfo("消息: 已确认");
		    /* 仅刷新上报超时；设备休眠为唤醒后硬计时，收包不续命 */
		    this->refresh_server();
		    /* 功能码同时打数值，避免 enum 名为空时与下一行 I(0) 粘成 "received: 0" */
		    {
		    auto fname = enum_name((e_pb_func) possible_header->function);
		    logInfo("消息: 已收到 功能码=%u 数据长=%u 名称=%s",
			    (unsigned) possible_header->function,
			    (unsigned) possible_header->dataLen,
			    fname.empty() ? "?" : fname.data());
		    }
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
			if (possible_header->dataLen == sizeof(pb_configModeReq))
			    {
			    this->message_enter_config_mode(
				    (pb_configModeReq*) param_ptr);
			    }
			else
			    {
			    logInfo("配置模式: 载荷长度错误 (期望 %u)",
				    (unsigned) sizeof(pb_configModeReq));
			    this->message_enter_config_mode(nullptr);
			    }
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
		    /* 服务器查询固件版本（func=17） */
		    case e_pb_func::down_uploadFirmwareVersion:
			this->message_upload_firmware_version();
			break;
		    case e_pb_func::down_configLocateGeo:
			this->message_config_locate_geo(
				possible_header->dataLen == sizeof(pb_locateGeo) ?
					(pb_locateGeo*) param_ptr : nullptr);
			break;
		    case e_pb_func::down_configLocateSwitch:
			this->message_config_locate_switch(
				possible_header->dataLen == 1 ?
					(uint8_t*) param_ptr : nullptr);
			break;
		    case e_pb_func::down_uploadLocateSwitch:
			this->message_upload_locate_switch();
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
    logWarning("消息: 格式错误");
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

    // AI生成注释: 设置响应数据包头部信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_runningConfigResult;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

    if (this->config_write_allowed(e_pb_func::down_configRunning))
	{
	// AI生成注释: 配置编辑模式下，应用运行配置
	rt_solution.runningConfig = *params;
	// AI生成注释: 修改服务器响应超时定时器周期
	xTimerChangePeriod(server_timeout_timer,
		pdMS_TO_TICKS(rt_solution.runningConfig.t2_serverRsps_timeout_sec * 1000),
		portMAX_DELAY);
	xTimerChangePeriod(device_timeout_timer,
		pdMS_TO_TICKS(util_atgm332d_get_status().position_fixed ?
			rt_solution.runningConfig.t4_gnssGood_sleep_sec * 1000 :
			rt_solution.runningConfig.t3_gnssSearch_sleep_sec * 1000),
		portMAX_DELAY);
	/* t5 单位秒 → 加计冷却毫秒（落实 PRODUCT_CONFIG / 协议消抖） */
	sensor_config.cool_down_timeout =
		(uint32_t) params->t5_motionDetect_delay_sec * 1000u;
	util_sc7a20_set_config(sensor_config);
	analog_config.low_battery_threshold = params->n1_vbatAlarm_threshold_volt / 10.0f;
	util_analog_set_config(analog_config);
	nvm->save();
	nvm_commit_host_config();
	rsps.body.cmdletOrResponse = 0; /* 0: 全部设定成功 */
	}
    else
	{
	rsps.body.cmdletOrResponse = 255; /* 255: 全部设定失败 */
	}
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
    util_atgm332d_nvm_flush();
    clear_config_session_for_standby();
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
 * @brief 服务器请求进/退配置模式或改密（功能码 9）
 * @param req 密码 8B + intentFunc 1B；null 或非法长度由调用方传 null，应答 00 00
 */
void Solution::message_enter_config_mode(pb_configModeReq *req)
    {
    pb_packConfigModeRsp rsps;
    password exitWord =
	{
	.item =
	    {
	    0
	    }
	};

    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_responseConfigMode;
    rsps.body.header.dataLen = sizeof(pb_configModeRsp);
    rsps.body.configMode.result = edit_notEnabled;
    rsps.body.configMode.intentFunc = 0;

    if (req == nullptr)
	{
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
		CFL(rsps.body));
	send_message((uint8_t*) &rsps, sizeof(pb_packConfigModeRsp));
	return;
	}

    password &pwd = req->pwd;
    const uint8_t intent = req->intentFunc;

    if (!this->rt_solution.configEditting)
	{
	/* A. 进模式：合法写意图 + 正确密码 */
	if (!config_enter_intent_valid(intent))
	    {
	    logInfo("配置模式进入失败: 非法意图 0x%02X", intent);
	    }
	else if (this->rt_solution.pwd == pwd)
	    {
	    this->rt_solution.configEditting = true;
	    this->config_intent_ = intent;
	    nvm->save();
	    nvm_commit_host_config();
	    rsps.body.configMode.result = edit_enabled;
	    rsps.body.configMode.intentFunc = intent;
	    }
	}
    else if (pwd == exitWord)
	{
	/* B. 退出：密码全 0 且 intent 必须为 0，成功回 02 00 */
	if (intent != 0u)
	    {
	    logInfo("配置模式退出失败: intent 须为 0 (收到 0x%02X)", intent);
	    /* 保持配置态，应答保持默认 00 00 */
	    }
	else
	    {
	    this->rt_solution.configEditting = false;
	    this->rt_solution.newPassword = exitWord;
	    this->rt_solution.passwordConfirm = 0;
	    this->config_intent_ = 0;
	    nvm->save();
	    nvm_commit_host_config();
	    rsps.body.configMode.result = edit_exit;
	    rsps.body.configMode.intentFunc = 0;
	    }
	}
    else if (intent == e_pb_func::down_configMode)
	{
	/* C. 改密：必须 intent=9 */
	if (this->rt_solution.passwordConfirm)
	    {
	    if (pwd == this->rt_solution.newPassword)
		{
		this->rt_solution.pwd = this->rt_solution.newPassword;
		nvm->save();
		nvm_commit_host_config();
		rsps.body.configMode.result = edit_pwdChangeSuccess;
		rsps.body.configMode.intentFunc = e_pb_func::down_configMode;
		logInfo("配置模式: 密码已更新并落盘");
		}
	    else
		{
		this->rt_solution.newPassword = exitWord;
		rsps.body.configMode.result = edit_pwdChangeFailed;
		rsps.body.configMode.intentFunc = 0;
		}
	    this->rt_solution.passwordConfirm = 0;
	    }
	else
	    {
	    this->rt_solution.newPassword = pwd;
	    this->rt_solution.passwordConfirm = 1;
	    rsps.body.configMode.result = edit_pwdConfirm;
	    rsps.body.configMode.intentFunc = e_pb_func::down_configMode;
	    }
	}
    else
	{
	/* D. 已在配置态且非退出/改密：禁止用本帧切换意图 */
	if (this->rt_solution.passwordConfirm)
	    {
	    this->rt_solution.newPassword = exitWord;
	    this->rt_solution.passwordConfirm = 0;
	    }
	logInfo("配置模式: 会话中非法帧 intent=0x%02X（保持原意图 0x%02X）",
		intent, this->config_intent_);
	}

    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
    send_message((uint8_t*) &rsps, sizeof(pb_packConfigModeRsp));
    }

/** 配置态且会话意图匹配时才允许写参 */
bool Solution::config_write_allowed(uint8_t write_func) const
    {
    return this->rt_solution.configEditting && this->config_intent_ == write_func;
    }

/** 进 Standby 前清 editing/intent/改密确认，避免跨睡无意图残留 */
void Solution::clear_config_session_for_standby(void)
    {
    if (!this->rt_solution.configEditting && !this->rt_solution.passwordConfirm
	    && this->config_intent_ == 0)
	{
	return;
	}
    logInfo("进 Standby 前清除配置会话 (intent=0x%02X)", this->config_intent_);
    this->rt_solution.configEditting = 0;
    this->rt_solution.passwordConfirm = 0;
    this->rt_solution.newPassword =
	{
	0
	};
    this->config_intent_ = 0;
    nvm->save();
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
 * @brief 应答服务器固件版本查询（down_uploadFirmwareVersion=17）
 * @note  上行功能码 16，数据域 year/month/revision 来自 PRODUCT_CONFIG
 */
void Solution::message_upload_firmware_version(void)
    {
    pb_packFirmwareVersion rsps;

    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_firmwareVersionUpload;
    rsps.body.header.dataLen = sizeof(pb_firmwareVersion);
    rsps.body.firmwareVersion.year = PROD_FW_VER_YEAR;
    rsps.body.firmwareVersion.month = PROD_FW_VER_MONTH;
    rsps.body.firmwareVersion.revision = PROD_FW_VER_REVISION;

    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));

    send_message((uint8_t*) &rsps, sizeof(pb_packFirmwareVersion));
    logInfo("固件版本应答: %02u.%02u.%02u",
	    (unsigned) PROD_FW_VER_YEAR, (unsigned) PROD_FW_VER_MONTH,
	    (unsigned) PROD_FW_VER_REVISION);
    }

/** 应答写入定位信息：配置模式下写入 GNSS 缓存并落 NVM */
void Solution::message_config_locate_geo(pb_locateGeo *geo)
    {
    pb_packCmdletOrResponse rsps;
    uint8_t ok = 0;

    if (this->config_write_allowed(e_pb_func::down_configLocateGeo)
	    && geo != nullptr)
	{
	if (util_atgm332d_set_manual_geo(geo->geo[ID_LONGITUDE],
		geo->geo[ID_LATITUDE]))
	    {
	    nvm_commit_host_config();
	    ok = 1;
	    }
	}

    rsps.body.cmdletOrResponse = ok;
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_locateGeoResult;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    logInfo("定位坐标写入应答: %u", (unsigned) ok);
    }

/**
 * @brief 应答修改定位开关（上行功能码 20）
 * @note  结果字节：1=成功；0=未进配置模式/参数缺失；2=非法(开AGNSS未开GNSS)
 */
void Solution::message_config_locate_switch(uint8_t *value)
    {
    pb_packCmdletOrResponse rsps;
    uint8_t result = 0;

    if (this->config_write_allowed(e_pb_func::down_configLocateSwitch)
	    && value != nullptr)
	{
	const uint8_t raw = (uint8_t) (*value & kLocateSwitchMask);
	/* 先判非法组合再 normalize，避免把「仅 AGNSS」静默改成全关 */
	if (!locate_switch_is_valid(raw))
	    {
	    result = 2; /* 开 AGNSS 须同时开 GNSS */
	    logWarning("定位开关: 拒绝 0x%02X (开AGNSS须开GNSS), 应答=2",
		    (unsigned) raw);
	    }
	else
	    {
	    this->rt_solution.locate_switch = locate_switch_normalize(raw);
	    util_atgm332d_deactivate();
	    nvm->save();
	    nvm_commit_host_config();
	    result = 1;
	    logInfo("定位开关: 已更新 0x%02X",
		    (unsigned) this->rt_solution.locate_switch);
	    }
	}

    rsps.body.cmdletOrResponse = result;
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_locateSwitchResult;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    logInfo("定位开关: 应答结果=%u", (unsigned) result);
    }

/** 上报当前定位开关（无需配置模式） */
void Solution::message_upload_locate_switch(void)
    {
    pb_packCmdletOrResponse rsps;

    rsps.body.cmdletOrResponse = locate_switch_normalize(
	    this->rt_solution.locate_switch);
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_locateSwitchUpload;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    logInfo("定位开关上传: 0x%02X",
	    (unsigned) rsps.body.cmdletOrResponse);
    }

/* LBS 会话状态（对齐 Slope sys_gnss：事件循环执行，不堵在 start_locate） */
static volatile bool s_lbs_due = false;
static bool s_lbs_active = false;
static uint8_t s_lbs_attempts = 0;

void Solution::lbs_retry_timer_callback(TimerHandle_t xTimer)
    {
    (void) xTimer;
    s_lbs_due = true;
    logInfo("LBS 重试计时到点");
    }

static void lbs_session_stop(void)
    {
    s_lbs_active = false;
    s_lbs_due = false;
    }

void Solution::lbs_schedule_retry(void)
    {
    if (this->lbs_retry_timer == nullptr)
	{
	this->lbs_retry_timer = xTimerCreate("lbs_retry",
		pdMS_TO_TICKS(PROD_CFG_LBS_RETRY_INTERVAL_SEC * 1000u),
		pdFALSE, this, Solution::lbs_retry_timer_callback);
	}
    if (this->lbs_retry_timer == nullptr)
	{
	return;
	}
    xTimerChangePeriod(this->lbs_retry_timer,
	    pdMS_TO_TICKS(PROD_CFG_LBS_RETRY_INTERVAL_SEC * 1000u), 0);
    xTimerStart(this->lbs_retry_timer, 0);
    logInfo("LBS %us 后重试 (已查询 %u/%u)",
	    (unsigned) PROD_CFG_LBS_RETRY_INTERVAL_SEC,
	    (unsigned) s_lbs_attempts,
	    (unsigned) PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE);
    }

void Solution::lbs_session_start(void)
    {
    if (util_atgm332d_get_status().position_fixed)
	{
	logInfo("LBS跳过: 北斗已定位");
	return;
	}
    if (!util_atgm332d_lbs_interval_elapsed())
	{
	return;
	}
    s_lbs_attempts = 0;
    s_lbs_active = true;
    s_lbs_due = true;
    logInfo("LBS 会话已启动（满 4h 后本唤醒最多 %u 次查询）",
	    (unsigned) PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE);
    }

void Solution::lbs_process_due(void)
    {
    if (!s_lbs_due || !s_lbs_active)
	{
	return;
	}
    s_lbs_due = false;

    if (util_atgm332d_get_status().position_fixed)
	{
	lbs_session_stop();
	return;
	}

    if (util_agnss_rx_is_active())
	{
	this->lbs_schedule_retry();
	return;
	}

    if (s_lbs_attempts >= PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE)
	{
	lbs_session_stop();
	logInfo("LBS：本唤醒结束 (attempts=%u/%u)",
		(unsigned) s_lbs_attempts,
		(unsigned) PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE);
	return;
	}

    if (this->air == nullptr)
	{
	lbs_session_stop();
	return;
	}

    const int csq = this->air->getCsq();
    if (csq < 0 || csq == 99 || csq < (int) PROD_CFG_LBS_MIN_CSQ)
	{
    logWarning("LBS跳过: CSQ=%d", csq);
	this->lbs_schedule_retry();
	return;
	}

    logInfo("定位: 查询LBS");
    AIR780EP::LbsResult lbs{};
    const bool got = this->air->query_lbs(&lbs, PROD_CFG_LBS_QUERY_TIMEOUT_MS);
    ++s_lbs_attempts;
    util_atgm332d_lbs_note_query_sent();

    if (util_atgm332d_get_status().position_fixed)
	{
	lbs_session_stop();
	return;
	}

    if (got && lbs.ok)
	{
	logInfo("定位: LBS成功, 按NVM门限处理坐标");
	(void) util_atgm332d_apply_lbs_geo(lbs.longitude, lbs.latitude);
	lbs_session_stop();
	logInfo("LBS：本唤醒查询完成，停止会话 (attempts=%u)",
		(unsigned) s_lbs_attempts);
	return;
	}

    logWarning("LBS查询失败 attempt=%u/%u", (unsigned) s_lbs_attempts,
	    (unsigned) PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE);

    if (s_lbs_attempts < PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE)
	{
	this->lbs_schedule_retry();
	return;
	}

    lbs_session_stop();
    logInfo("LBS：本唤醒结束 (attempts=%u/%u)", (unsigned) s_lbs_attempts,
	    (unsigned) PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE);
    }

/**
 * @brief 按 locate_switch 启 GNSS/AGNSS；LBS 不在此启（见 start_lbs_if_needed）
 */
void Solution::start_locate(void)
    {
    const uint8_t sw = locate_switch_normalize(this->rt_solution.locate_switch);
    const bool want_gnss = locate_gnss_on(sw);
    const bool want_agnss = locate_agnss_on(sw);

    logInfo("定位: 开关=0x%02X GNSS=%u AGNSS=%u LBS配置=%u",
	    (unsigned) sw, want_gnss ? 1u : 0u, want_agnss ? 1u : 0u,
	    locate_lbs_on(sw) ? 1u : 0u);

    if (!want_gnss)
	{
	logInfo("定位: 跳过(GNSS关)");
	return;
	}

    /* 定住后关模块延迟用 t4（0→activate 内兜底 3s） */
    const uint32_t post_fix_sec =
	    (uint32_t) this->rt_solution.runningConfig.t4_gnssGood_sleep_sec;
    logInfo("定位: 开启北斗, 定住后%us关模块",
	    (unsigned) ((post_fix_sec == 0u) ? 3u : post_fix_sec));
    util_atgm332d_activate(post_fix_sec);
    vTaskDelay(pdMS_TO_TICKS(PROD_CONFIG_AGNSS_PWR_SETTLE_MS));
    logInfo("定位: 北斗上电稳定结束");
    if (want_agnss && this->air != nullptr && !util_agnss_done_this_wake())
	{
	logInfo("定位: 拉取AGNSS");
	(void) util_agnss_fetch_and_inject_once(this->air);
	}
    else if (want_agnss && util_agnss_done_this_wake())
	{
	logInfo("定位: AGNSS本唤醒已做过, 跳过");
	}
    else if (want_agnss)
	{
	logInfo("定位: AGNSS跳过(无4G实例)");
	}
    logInfo("定位: GNSS流程结束");
    }

/**
 * @brief 震动唤醒路径调用：配置开 LBS 且当前未定住则启会话（AT 在事件循环）
 */
void Solution::start_lbs_if_needed(void)
    {
    const uint8_t sw = locate_switch_normalize(this->rt_solution.locate_switch);
    if (!locate_lbs_on(sw))
	{
	logInfo("LBS: 开关关, 跳过");
	return;
	}
    if (util_atgm332d_get_status().position_fixed)
	{
	logInfo("LBS: 北斗已定位, 跳过");
	return;
	}
    this->lbs_session_start();
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

    // AI生成注释: 设置响应数据包头部信息
    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_systemConfigResult;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

    if (this->config_write_allowed(e_pb_func::down_configSystem))
	{
	// AI生成注释: 配置编辑模式下，应用系统配置
	this->rt_solution.systemConfig = *params;
	air->setServer(params->runServerIP, params->runServerPort,
		AIR780EP::air780_server_t::server_main);
	air->setServer(params->backupServerIP, params->backupServerPort,
		AIR780EP::air780_server_t::server_aux);
	util_sc7a20_config_s sensor_config = util_sc7a20_get_config();
	sensor_config.range = this->rt_solution.systemConfig.sensorReverseRange;
	sensor_config.acc_thres16mg_lsb = params->sensorVibrationThreshold;
	util_sc7a20_set_config(sensor_config);
	nvm->save();
	nvm_commit_host_config();
	rsps.body.cmdletOrResponse = 0; /* 0: 全部设定成功 */
	}
    else
	{
	rsps.body.cmdletOrResponse = 255; /* 255: 全部设定失败 */
	}
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
    pb_packCmdletOrResponse rsps;
    const bool modeValid = enum_contains<solution_mode_e>(*mode);
    /* 配置模式 + 意图匹配 + 合法 + 与当前不同，才切换 */
    const bool willSwitch = this->config_write_allowed(
	    e_pb_func::down_configWorkMode) && modeValid
	    && this->rt_solution.mode != *mode;

    rsps.body.header.code = this->rt_solution.systemConfig.code;
    rsps.body.header.function = e_pb_func::up_responseWorkMode;
    rsps.body.header.dataLen = sizeof(rsps.body.cmdletOrResponse);

    if (!willSwitch)
	{
	rsps.body.cmdletOrResponse = 0; /* 功能码14：失败/未切换 */
	rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
		CFL(rsps.body));
	send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
	logInfo("运行模式未改变");
	return;
	}

    this->rt_solution.mode = *mode;
    util_sc7a20_config_s sensor_config;

    switch (this->rt_solution.mode)
	{
    case solution_mode_e::wm_work:
	{
	float sampled_angle = util_sc7a20_sample_angle(30, 100);
	sensor_config = util_sc7a20_get_config();
	sensor_config.leanDetectOffset = sampled_angle;
	sensor_config.leanDetectEnabled = true;
	util_sc7a20_set_config(sensor_config);
	this->rt_solution.connect_to_main_server = true;
	this->rt_solution.updatePositionOnStart = true;
	}
	break;
    case solution_mode_e::wm_idle:
	{
	sensor_config = util_sc7a20_get_config();
	sensor_config.leanDetectEnabled = false;
	util_sc7a20_set_config(sensor_config);
	this->rt_solution.connect_to_main_server = true;
	this->rt_solution.updatePositionOnStart = false;
	}
	break;
    case solution_mode_e::wm_factory:
	{
	sensor_config = util_sc7a20_get_config();
	sensor_config.leanDetectEnabled = false;
	util_sc7a20_set_config(sensor_config);
	this->rt_solution.connect_to_main_server = false;
	this->rt_solution.updatePositionOnStart = false;
	}
	break;
    default:
	break;
	}

    nvm->save();
    rsps.body.cmdletOrResponse = 1; /* 功能码14：成功 */
    rsps.body.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) (&rsps.body),
	    CFL(rsps.body));
    send_message((uint8_t*) &rsps, sizeof(pb_packCmdletOrResponse));
    /* restart() 内会再 __flash_sync；此处先刷一次保证应答前已落盘 */
    nvm_commit_host_config();
    this->restart();
    }

/**
 * @brief 处理服务器 OK：仅最近一次 up_report 的确认才可能休眠（对齐 Inclination）
 */
void Solution::message_heartbeat(void)
    {
    /* 引脚唤醒：OK 后继续在线搜星（勿进睡） */
    if (util_lowpower_get_wake_source() == util_lowpower_wake_source_e::pin)
	{
	clear_report_pending_ack();
	logInfo("震动唤醒收到上报 OK，继续在线");
	return;
	}

    if (!awaiting_report_ok_)
	{
	logInfo("收到 OK（非 up_report 确认），继续在线");
	return;
	}

    if (util_agnss_rx_is_active())
	{
	clear_report_pending_ack();
	logInfo("AGNSS 期间收到 OK，忽略");
	return;
	}

    awaiting_report_ok_ = false;

    /* 北斗仍上电：视为搜星会话，对齐 Inclination t3 会话不因 OK 进睡 */
    if (HAL_GPIO_ReadPin(BD_PWR_GPIO_Port, BD_PWR_Pin) == GPIO_PIN_SET)
	{
	logInfo("搜星中收到上报 OK，继续在线");
	return;
	}

    logInfo("收到 up_report OK，准备休眠");
    this->message_sleep();
    }

void Solution::mark_report_pending_ack(void)
    {
    awaiting_report_ok_ = true;
    }

void Solution::clear_report_pending_ack(void)
    {
    awaiting_report_ok_ = false;
    }

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
    /* 任意非 report 发送先清确认位；report 在发送成功后再 mark */
    clear_report_pending_ack();
    auto connServer =
	    this->rt_solution.connect_to_main_server ?
		    AIR780EP::air780_server_t::server_main :
		    AIR780EP::air780_server_t::server_aux;
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
    (void) xTimer;
    /* 只投递事件：report 含 AT/日志，放 Timer 任务会栈溢出且堵住喂狗定时器 */
    if (util_agnss_rx_is_active())
	{
	return;
	}
    util_events_generate(util_event_code_t::server_report_due);
    }

/**
 * 设备硬计时休眠：自 timers_create 起算，到期进入 standby（收包不延长）
 */
void Solution::device_timeout_timer_callback(TimerHandle_t xTimer)
    {
    auto *pthis = (Solution*) pvTimerGetTimerID(xTimer);
    if (util_agnss_rx_is_active())
	{
	/* 单次定时器：AGNSS 中错过则再开一轮，避免永远不睡 */
	xTimerStart(xTimer, 0);
	return;
	}
    (void) pthis;
    util_events_generate(util_event_code_t::device_sleep_due);
    }

/** 工作线程执行：服务器超时上报 */
void Solution::event_action_server_report(void)
    {
    if (util_agnss_rx_is_active())
	{
	return;
	}
    this->report();
    }

/** 工作线程执行：在线超时进休眠 */
void Solution::event_action_device_sleep(void)
    {
    if (util_agnss_rx_is_active())
	{
	return;
	}
    logInfo("定时器: 设备在线超时, 进入休眠");
    lbs_session_stop();
    util_atgm332d_nvm_flush();
    clear_config_session_for_standby();
    uint32_t sleep_time = this->rt_solution.runningConfig.t0_netGood_wakeup_min
	    * 60;
    auto lpconfig = util_lowpower_get_config();
    lpconfig.wake_pin_enable = true;
    lpconfig.requested_wakeup_period = sleep_time;
    lpconfig.wakeup_remain = sleep_time;
    util_lowpower_set_config(lpconfig);
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
	case util_event_code_t::server_report_due:
	    this->event_action_server_report();
	    break;
	case util_event_code_t::device_sleep_due:
	    this->event_action_device_sleep();
	    break;
	case util_event_code_t::gnss_pwr_off_commit:
	    /* t4 延时关电后：滤波已跑完，刷坐标进 Flash */
	    util_atgm332d_nvm_flush();
	    __flash_sync();
	    logInfo("北斗: 关模块后坐标已落盘");
	    break;
	default:
	    break;
	    }
	}
    /* LBS：对齐 Slope process_lbs_due，每圈检查到期查询 */
    this->lbs_process_due();
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
    logInfo("网络: 开始配置");
    // AI生成注释: 获取当前低功耗配置，用于失败时的休眠设置
    auto lpconfig = util_lowpower_get_config();
    // AI生成注释: 启动4G通信模块电源
    this->air->poweron();
    // AI生成注释: 等待LTE网络附着，超时时间10秒
    if (!this->air->waitEutran(10000))
	{
	/*等待LTE附着，LTE附着超过10秒说明附近没有基站或信号微弱*/
	// AI生成注释: 记录LTE附着失败警告
	logWarning("网络: LTE未附着!");
	// AI生成注释: 配置低功耗模式，启用外部唤醒引脚
	lpconfig.wake_pin_enable = true;
	// AI生成注释: 设置网络不良时的唤醒间隔（分钟转秒）
	lpconfig.requested_wakeup_period =
		this->rt_solution.runningConfig.t1_netBad_wakeup_min * 60;
	// AI生成注释: 设置剩余唤醒时间
	lpconfig.wakeup_remain = lpconfig.requested_wakeup_period;
	// AI生成注释: 应用低功耗配置
	util_lowpower_set_config(lpconfig);
	clear_config_session_for_standby();
	// AI生成注释: LTE附着失败，进入休眠模式
	util_lowpower_standby();/*LTE附着失败，休眠*/
	}
    // AI生成注释: 记录LTE附着成功，开始模块配置
    logInfo("网络: LTE已附着, 配置模组中");
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
    logInfo("网络: 主服 %d.%d.%d.%d:%d, 备服 %d.%d.%d.%d:%d",
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
    logInfo("网络: 连接%s服务器: %s",
	    this->rt_solution.connect_to_main_server ? "主" : "备",
	    connectReady ? "成功" : "失败");

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
	clear_config_session_for_standby();
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
    /* 唤醒后硬计时一次，到期必睡；pdFALSE=单次，收包不 reset */
    this->device_timeout_timer = xTimerCreate(this->device_timeout_timer_name,
	    pdMS_TO_TICKS(
		    this->rt_solution.runningConfig.t3_gnssSearch_sleep_sec
			    * 1000),
	    pdFALSE, (void*) this, device_timeout_timer_callback);
    // AI生成注释: 记录服务器响应超时倒计时时间
    logInfo("定时器: 服务器响应超时倒计时 %d秒",
	    this->rt_solution.runningConfig.t2_serverRsps_timeout_sec);
    logInfo("定时器: 设备硬休眠倒计时 %d秒(收包不续期)",
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
    Solution *pthis = (Solution*) argument;
    const util_lowpower_wake_source_e wake =
	    util_lowpower_get_wake_source();
    logInfo("工作例程: 启动, 唤醒源=%s", enum_name(wake).data());
    util_atgm332d_wake_session_begin();
    lbs_session_stop();

    pthis->air = new AIR780EP(&huart1);
    configASSERT(pthis->air != NULL);
    logInfo("工作例程: 开始联网");
    pthis->setup_network();
    util_analog_suspend();
    logInfo("工作例程: 清空联网期间事件");
    util_events_flush();

    /* 先上报再 AGNSS：避免 t2 report 与 link2 抢 AT（对齐 Slope） */
    logInfo("工作例程: 首次上报");
    pthis->report();

    /*
     * 产品只有两类唤醒：RTC 定时 / 震动（硬件记为 pin=加计 INT）。
     * Standby 震动唤醒=整机复位，不会自动产生 vibrate 事件，且 flush
     * 会丢掉联网期间加计事件，故此处按震动路径补 GNSS + LBS。
     */
    if (wake == util_lowpower_wake_source_e::pin)
	{
	logInfo("工作例程: 震动唤醒(引脚), 启动定位");
	logInfo("工作例程: 定位开关=0x%02X",
		(unsigned) pthis->rt_solution.locate_switch);
	util_sc7a20_mark_vibration();
	pthis->start_locate();
	pthis->start_lbs_if_needed();
	pthis->rt_solution.updatePositionOnStart = false;
	pthis->nvm->save();
	}
    else if (pthis->rt_solution.updatePositionOnStart)
	{
	/* RTC/上电等：只刷新 GNSS，不查 LBS */
	logInfo("工作例程: 定时/开机刷新定位(不含LBS)");
	pthis->start_locate();
	pthis->rt_solution.updatePositionOnStart = false;
	pthis->nvm->save();
	}
    else
	{
	logInfo("工作例程: 本周期不主动定位");
	}

    pthis->timers_create();
    logInfo("工作例程: 进入事件循环");

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
    /* 将4G模块实例赋值给全局指针，供其他模块（如北斗定位）调用 */
    //g_air780ep = pthis->air;
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
#ifdef DEBUG
    logInfo("当前角度: %.2f", util_sc7a20_get_status().lean_angle);
#endif
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
#ifdef DEBUG
    logInfo("脱离检测失败,继续休眠, 计数=%d", sample_count);
#endif
	auto lpconfig = util_lowpower_get_config();
	/*
	 * $notice 工厂模式下，每过两小时唤醒自身重新检测
	 * 		期间禁止震动唤醒
	 * */
	// AI生成注释: 设置工厂模式下的休眠周期（配置文件中定义的时间）
	lpconfig.requested_wakeup_period = PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC;
#ifdef DEBUG
    lpconfig.requested_wakeup_period = 30; //debug时设置为30s唤醒一次
#endif
	// AI生成注释: 设置剩余唤醒时间
	lpconfig.wakeup_remain = PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC;
	// AI生成注释: 禁用外部唤醒引脚，防止震动唤醒干扰检测
	lpconfig.wake_pin_enable = false;
	// AI生成注释: 应用低功耗配置
	util_lowpower_set_config(lpconfig);
	pthis->clear_config_session_for_standby();
	// AI生成注释: 进入长时间休眠，等待下次自动唤醒重新检测
	util_lowpower_standby();
	}
    /*系统激活*/
    // AI生成注释: 检测通过，切换到工作模式
    pthis->rt_solution.mode = wm_work;			//进入空闲模式
    
    pthis->rt_solution.connect_to_main_server = true; //默认连接到主服务器
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
