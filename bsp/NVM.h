/*
 * NVM.h
 *
 *  Created on: Jan 24, 2025
 *      Author: IRIS
 */

#ifndef NVM_H_
#define NVM_H_

//******std Family Bucket******//
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdint-gcc.h"
#include "stdbool.h"
#include "math.h"

#include "magic_enum.hpp"
using namespace magic_enum;

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

#define NVM_BASE_ADDRESS				0x08000000
#define NVM_START_ADDRESS				0x0803F800
#define NVM_PARTITION_SIZE				256
#define NVM_PAGE_SIZE					2048
#define NVM_PARTITION_ADDRESS(x)			NVM_START_ADDRESS+NVM_PARTITION_SIZE*x
#define NVM_PARTITION_PAGE_ADDRESS(x)			NVM_START_ADDRESS+NVM_PAGE_SIZE*(x/4)
#define NVM_PARTITION_PAGE_OFFSET(x)			((x)*NVM_PARTITION_SIZE)

void __flash_sync(void);

class NVM: public osAllocator<NVM>
    {
public:
    typedef enum : uint8_t
	{
	partition_solution, /**< Protocol partition */
	partition_air780, /**< AIR780 module partition */
	partition_sc7a20, /**< SC7A20 sensor partition */
	partition_analog, /**< Analog sensor partition */
	partition_lowpower, /**< Low power partition */
	partition_bd /**< BD partition */
	} nvm_partition_t;

    NVM(nvm_partition_t partition, void *data, void *_defaultData,
	    uint16_t size);
    ~NVM();

    void save();
    void load();

    bool isFactoryDefault();
    void restoreDefault();

private:


    nvm_partition_t partition;
    void *data = NULL;
    void *defaultData = NULL;
    uint16_t size = 0;
    };

#endif /* NVM_H_ */
