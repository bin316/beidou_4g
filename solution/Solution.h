/*
 * Solution.h
 *
 *  Created on: Jan 22, 2025
 *      Author: IRIS
 */

#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "Protocol.h"
#include "utilties.h"

//******FreeRTOS Family Bucket******//
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "queue.h"
#include "task.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "os_Allocator.hpp"
#include "NVM.h"

#include "AIR780EP.h"

typedef enum : uint8_t
    {
    wm_idle,
    wm_work,
    wm_factory = 0xFF
    } solution_mode_e;

typedef struct
    {
    pb_runningConfig runningConfig;
    pb_systemConfig systemConfig;
    password pwd;
    solution_mode_e mode;
    uint8_t configEditting = 0;
    uint8_t passwordConfirm = 0;
    password newPassword =
	{
	0
	};
    util_lowpower_wake_source_e wakeSource =
	    util_lowpower_wake_source_e::regular;
    bool connect_to_main_server = false;
    bool updatePositionOnStart = false;
    /** 定位开关 bit0 GNSS / bit1 AGNSS / bit2 LBS，随 solution NVM 持久化 */
    uint8_t locate_switch = 0;

    void clear_work(void)
	{
	pwd =
	    {
	    0
	    };
	mode = solution_mode_e::wm_factory;
	configEditting = 0;
	passwordConfirm = 0;
	newPassword =
	    {
	    0
	    };
	wakeSource = util_lowpower_wake_source_e::regular;
	}
    } solution_handle;

class Solution: public osAllocator<Solution>
    {
public:
    Solution(void);

private:

    solution_handle rt_solution;

    NVM *nvm;

    AIR780EP *air;

    bool solution_boot_fine = false;

    uint32_t parse_unix_time(void);
    void unique_identifier_generate(void);

    void report(void);
    void restart(void);
    void timers_create(void);
    void refresh_server(void);
    /** 重置设备休眠倒计时；硬计时策略下收包路径不再调用 */
    void refresh_device(void);

    int send_message(void *msg, uint16_t len);
    int read_message(void *dest, uint16_t len);

    void event_process(void);
    void setup_network(void);

    void event_action_vibration(void);
    void event_action_message(void);
    /** Timer 投递的服务器超时上报（工作线程执行） */
    void event_action_server_report(void);
    /** Timer 投递的在线超时休眠（工作线程执行） */
    void event_action_device_sleep(void);
    /** 按 locate_switch 启动定位；allow_lbs 仅震动/引脚唤醒为 true */
    void start_locate(bool allow_lbs = false);
    /** 对齐 Slope：启 LBS 会话（满 4h 后本唤醒最多 N 次，事件循环执行） */
    void lbs_session_start(void);
    /** 对齐 Slope：处理到期的 LBS 查询/重试 */
    void lbs_process_due(void);
    /** LBS 失败/暂不可用时改期重试 */
    void lbs_schedule_retry(void);

    void message_report(void);
    void message_upload_run_parameters(void);
    void message_change_run_parameters(pb_runningConfig *params);
    void message_sleep(void);
    void message_enter_config_mode(password *pwd);
    void message_upload_sys_parameters(void);
    void message_change_sys_parameters(pb_systemConfig *params);
    void message_change_execute_mode(solution_mode_e *mode);
    /** 服务器请求固件版本（func=17）时上行 year/month/revision */
    void message_upload_firmware_version(void);
    /** 下行19：写经纬度到 GNSS 缓存并落 NVM */
    void message_config_locate_geo(pb_locateGeo *geo);
    /** 下行21：改 locateSwitch；应答20：1成功/0一般失败/2非法AGNSS */
    void message_config_locate_switch(uint8_t *value);
    /** 下行23：查询 locateSwitch */
    void message_upload_locate_switch(void);

//	服务器响应超时定时器相关变量
    TimerHandle_t server_timeout_timer;
    const char *server_timeout_timer_name = "server_timeout_timer";
    static void server_timeout_timer_callback(TimerHandle_t xTimer);
//	设备超时休眠定时器相关变量
    TimerHandle_t device_timeout_timer;
    const char *device_timeout_timer_name = "device_timeout_timer";
    static void device_timeout_timer_callback(TimerHandle_t xTimer);
    /** LBS 失败/暂不可用时的改期定时器（对齐 Slope） */
    TimerHandle_t lbs_retry_timer = nullptr;
    static void lbs_retry_timer_callback(TimerHandle_t xTimer);

    TaskHandle_t solution_thread_handle = NULL;
    static void solution_work_routine_thread(void *argument);
    static void solution_idle_routine_thread(void *argument);
    static void solution_fact_routine_thread(void *argument);
    };

#endif /* SOLUTION_H_ */
