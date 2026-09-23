/**
 * @file freertos_heap_regions.c
 * @brief FreeRTOS heap_5：主堆在 SRAM1，约 4KB 迁到 SRAM2，缓解 RAM1 近满
 */
#include "FreeRTOS.h"
#include "portable.h"

#ifndef configHEAP_RAM2_SIZE
#error "configHEAP_RAM2_SIZE must be defined in FreeRTOSConfig.h"
#endif
#ifndef configHEAP_RAM1_SIZE
#error "configHEAP_RAM1_SIZE must be defined in FreeRTOSConfig.h"
#endif

/* SRAM2：与 NVM/AGNSS 同段 ._ram2_area；heap_5 要求区域按地址升序登记 */
static uint8_t ucHeapRam2[configHEAP_RAM2_SIZE]
	__attribute__((section("._ram2_area"), aligned(8)));

/* SRAM1：主堆（仍占大部分任务/队列分配） */
static uint8_t ucHeapRam1[configHEAP_RAM1_SIZE]
	__attribute__((aligned(8)));

static const HeapRegion_t xHeapRegions[] = {
	{ ucHeapRam2, configHEAP_RAM2_SIZE },
	{ ucHeapRam1, configHEAP_RAM1_SIZE },
	{ NULL, 0 }
};

/** 必须在任何 pvPortMalloc / osKernelInitialize 创建对象之前调用 */
void freertos_heap_regions_init(void)
{
	vPortDefineHeapRegions(xHeapRegions);
}
