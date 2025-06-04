/*
 * boot.cpp
 *
 *  Created on: Jan 13, 2025
 *      Author: IRIS
 */

//******FreeRTOS Family Bucket******//
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "timers.h"
#include "queue.h"
#include "task.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "semphr.h"

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

//******HAL Family Bucket******//
#include "main.h"
#include "gpio.h"

#include "NMEA0183.h"
#include "BufferedUart.h"

#include "utilties.h"
float longitude = 0;
float latitude = 0;

/*max and min cordinate*/
float long_max = 0;
float long_min = 0;
float lat_max = 0;
float lat_min = 0;
time_t localTime = 0;

#include "AIR780EP.h"

#include "Solution.h"

Solution *sol;

Xuart *testu = NULL;

extern "C" void boot_thread(void *argument) {

	/*
	 * 初始化流程：
	 * 		1. 初始化事件队列
	 * 		2. 初始化低功耗
	 * 	        $notice 这两个初始化安排在最开始，低功耗功能依赖事件队列
	 * 	        	初始化低功耗时，可能由于载入了未完成的休眠过程，需要立即继续休眠，
	 * 	        	后面的初始化就没有意义了
	 * 	        3. 初始化模拟量
	 * 	        4. 初始化SC7A20（加计）
	 */

//	__HAL_DBGMCU_FREEZE_IWDG();
//	__util_shell_init__();
	__util_events_init__();	//初始化事件队列
	__util_lowpower_init__();	//初始化低功耗

	__util_analog_init__();	//初始化模拟量
	__util_sc7a20_init__();	//初始化SC7A20（加速度传感器）

	__util_atgm332d_init__();

	/*
	 * 初始化解决方案
	 * 		应用从此处启动，解决方案对象的new重载为使用rtos分配内存，
	 * 		并记录到全局解决方案指针，因此结束boot线程后，解决方案对象
	 * 		不受影响
	 */
    sol = new Solution();
//    $notice 此处可以结束boot线程了，解决方案已经启动
//	testu = new Xuart(&huart2);
//	testu->open();
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(200));
//		testu->write((void*) "cao ni ma\r\n", 11);
		/* USER CODE END boot_thread */
	}

}
