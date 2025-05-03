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
    void refresh_device(void);

    int send_message(void *msg, uint16_t len);
    int read_message(void *dest, uint16_t len);

    void event_process(void);
    void setup_network(void);

    void event_action_vibration(void);
    void event_action_message(void);

    void message_report(void);
    void message_upload_run_parameters(void);
    void message_change_run_parameters(pb_runningConfig *params);
    void message_sleep(void);
    void message_enter_config_mode(password *pwd);
    void message_upload_sys_parameters(void);
    void message_change_sys_parameters(pb_systemConfig *params);
    void message_change_execute_mode(solution_mode_e *mode);

//	服务器响应超时定时器相关变量
    TimerHandle_t server_timeout_timer;
    const char *server_timeout_timer_name = "server_timeout_timer";
    static void server_timeout_timer_callback(TimerHandle_t xTimer);
//	设备超时休眠定时器相关变量
    TimerHandle_t device_timeout_timer;
    const char *device_timeout_timer_name = "device_timeout_timer";
    static void device_timeout_timer_callback(TimerHandle_t xTimer);

    TaskHandle_t solution_thread_handle = NULL;
    static void solution_work_routine_thread(void *argument);
    static void solution_idle_routine_thread(void *argument);
    static void solution_fact_routine_thread(void *argument);
    };

#endif /* SOLUTION_H_ */
