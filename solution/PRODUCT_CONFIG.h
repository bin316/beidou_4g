/*
 * PRODUCT_CONFIG.h
 *
 *  Created on: Feb 18, 2025
 *      Author: IRIS
 */

#ifndef PRODUCT_CONFIG_H_
#define PRODUCT_CONFIG_H_

/*版本代号*/
#define PROD_CONFIG_VERSION_CODE    0x02

/* 固件版本号（二进制整数） */
#define PROD_FW_VER_YEAR                                            (26)
#define PROD_FW_VER_MONTH                                           (7)
#define PROD_FW_VER_REVISION                                        (2)

/*工厂模式休眠周期*/
#define PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC    7200

/**
 * 目标测试项   目标 ID     后四位 (十进制)	转换后的 INDEX (十六进制)
 * 测试001      90884390    4390            0x1126
 * 测试002      90884391    4391            0x1127
 * 测试003      90884392    4392            0x1128
 */
/*出厂默认测试代号MAJOR*/
#define PROD_CONFIG_FACTORY_DEFAULT_CODE_MAJOR    'Z'
/*出厂默认测试代号MINOR*/
#define PROD_CONFIG_FACTORY_DEFAULT_CODE_MINOR    'X'
/*出厂默认测试代号INDEX*/
#define PROD_CONFIG_FACTORY_DEFAULT_CODE_INDEX    0x1126
/*出厂是否生成随机界桩编号*/
#define PROD_CONFIG_FACTORY_GENERATE_UNIQ_INDEX    0

/*出厂默认主连接IP*/
#define PROD_CONFIG_FACTORY_DEFAULT_MAIN_IP    {110, 40, 132, 159}
/*出厂默认主连接PORT*/
#define PROD_CONFIG_FACTORY_DEFAULT_MAIN_PORT    2000
/*出厂默认备用连接IP*/
#define PROD_CONFIG_FACTORY_DEFAULT_AUX_IP    {110, 40, 132, 159}
/*出厂默认备用连接PORT*/
#define PROD_CONFIG_FACTORY_DEFAULT_AUX_PORT    2000

/*出厂默认配置密码*/
#define PROD_CONFIG_FACTORY_DEFAULT_PASSWORD    {'p', 'a', 's', 's', 'w', 'o', 'r', 'd'}

/*出厂默认阈值低电压*/
#define PROD_CONFIG_FACTORY_VBAT_LOW    30
/*出厂默认定位搜索倒计时*/
#define PROD_CONFIG_FACTORY_GNSS_SEARCH_TIMEOUT    100
/*出厂默认定位搜索完成倒计时*/
#define PROD_CONFIG_FACTORY_GNSS_SEARCH_COMPLETE_TIMEOUT    15
/*出厂默认网络良好唤醒倒计时(分钟)*/
#define PROD_CONFIG_FACTORY_NET_GOOD_WAKEUP_TIMEOUT_MIN    720
/*出厂默认网络差唤醒倒计时(分钟)*/
#define PROD_CONFIG_FACTORY_NET_BAD_WAKEUP_TIMEOUT_MIN    15
/*出厂默认服务器响应超时 */
#define PROD_CONFIG_FACTORY_SERVER_RSP_TIMEOUT_SEC    10

/*出厂默认震动消抖秒数（写入 runningConfig.t5，并应用到 SC7A20 cool_down_timeout）*/
#define PROD_CONFIG_FACTORY_VIBRATION_DEBOUNCE_SEC    2

/*出厂默认工作模式 $notice 仅用于测试*/
#define PROD_CONFIG_FACTORY_DEFAULT_WORK_MODE    solution_mode_e::wm_work

/*是否启用了独立看门狗？*/
#define PROD_CONFIG_FACTORY_ENABLE_IWDG    1

/* 定位开关出厂默认（仅 NVM 全 0xFF 恢复出厂时写入；运行时以 NVM locate_switch 为准）
 * bit0 GNSS / bit1 AGNSS / bit2 LBS；bit1=1 须同时 bit0=1，拼装后会 normalize 清非法 AGNSS */
#define PROD_CFG_DEFAULT_LOCATE_GNSS                                (1)
#define PROD_CFG_DEFAULT_LOCATE_AGNSS                               (1)
#define PROD_CFG_DEFAULT_LOCATE_LBS                                 (0)

/* 中科微 AGNSS（TCP → CASBIN → UART 注入 ATGM332D） */
#define PROD_CONFIG_AGNSS_SERVER_IP                                 {121, 41, 40, 95}
#define PROD_CONFIG_AGNSS_SERVER_PORT                               (2621)
#define PROD_CONFIG_AGNSS_USER                                      "2734223046@qq.com"
#define PROD_CONFIG_AGNSS_PWD                                       "Jykj2026"
#define PROD_CONFIG_AGNSS_TCP_TIMEOUT_MS                            (15000u)
#define PROD_CONFIG_AGNSS_CONNECT_TIMEOUT_MS                        (12000u)
#define PROD_CONFIG_AGNSS_INJECT_DELAY_MS                           (30u)
/* 整包接收缓冲：放在 SRAM2（._ram2_area），不占主 RAM；可恢复 4KB */
#define PROD_CONFIG_AGNSS_RECV_BUF_SIZE                             (4096u)
/** 北斗上电后、拉 AGNSS 前的稳定等待（ms） */
#define PROD_CONFIG_AGNSS_PWR_SETTLE_MS                             (500u)

/** LBS：合宙 AT+CIPGSMLOC；仅震动/引脚路径 allow_lbs 时启会话（对齐 Slope） */
#define PROD_CFG_LBS_QUERY_TIMEOUT_MS                               (35000u)
#define PROD_CFG_LBS_MIN_CSQ                                        (10)
/** 合宙免费单基站最小查询间隔（s）；4h=14400；未到点则本唤醒不启 LBS 会话 */
#define PROD_CFG_LBS_MIN_INTERVAL_SEC                               (14400u)
/** 满间隔后的定位唤醒：CIPGSMLOC 最多次数（1 次 + 失败再试 2 次 = 3） */
#define PROD_CFG_LBS_MAX_ATTEMPTS_PER_WAKE                          (3u)
/** 失败重试 / CSQ·AGNSS 暂不可查时的改期间隔（s） */
#define PROD_CFG_LBS_RETRY_INTERVAL_SEC                             (10u)
#define PROD_CFG_LBS_PDP_CID                                        (1)

#endif /* PRODUCT_CONFIG_H_ */
