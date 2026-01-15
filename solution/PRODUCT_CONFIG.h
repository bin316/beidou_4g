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

/*工厂模式休眠周期*/
#define PROD_CONFIG_FACTORY_SLEEP_PERIOD_SEC    7200

/*出厂默认测试代号MAJOR*/
#define PROD_CONFIG_FACTORY_DEFAULT_CODE_MAJOR    'Z'
/*出厂默认测试代号MINOR*/
#define PROD_CONFIG_FACTORY_DEFAULT_CODE_MINOR    'X'
/*出厂默认测试代号INDEX*/
#define PROD_CONFIG_FACTORY_DEFAULT_CODE_INDEX    0x1124
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
#define PROD_CONFIG_FACTORY_NET_GOOD_WAKEUP_TIMEOUT_MIN    2880
/*出厂默认网络差唤醒倒计时(分钟)*/
#define PROD_CONFIG_FACTORY_NET_BAD_WAKEUP_TIMEOUT_MIN    15
/*出厂默认服务器响应超时 */
#define PROD_CONFIG_FACTORY_SERVER_RSP_TIMEOUT_SEC    10

/*出厂默认震动消抖秒数*/
#define PROD_CONFIG_FACTORY_VIBRATION_DEBOUNCE_SEC    2

/*出厂默认工作模式 $notice 仅用于测试*/
#define PROD_CONFIG_FACTORY_DEFAULT_WORK_MODE    solution_mode_e::wm_work

/*是否启用了独立看门狗？*/
#define PROD_CONFIG_FACTORY_ENABLE_IWDG    1

#endif /* PRODUCT_CONFIG_H_ */
