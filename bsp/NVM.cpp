/*
 * NVM.cpp
 *
 *  Created on: Jan 24, 2025
 *      Author: IRIS
 */

#include <NVM.h>

#include "main.h"
#include "rtc.h"

#define NVM_BASE_ADDRESS				0x08000000
#define NVM_START_ADDRESS				0x0803F800
#define NVM_PARTITION_SIZE				256
#define NVM_PAGE_SIZE					2048
#define NVM_PARTITION_ADDRESS(x)		NVM_START_ADDRESS+NVM_PARTITION_SIZE*x
#define NVM_PARTITION_PAGE_ADDRESS(x)	NVM_START_ADDRESS+NVM_PAGE_SIZE*(x/4)
#define NVM_PARTITION_PAGE_OFFSET(x)	((x)*NVM_PARTITION_SIZE)

static SemaphoreHandle_t flash_Sem;
static uint8_t *flash_pagebuffer;
static uint8_t __attribute__((section("._ram2_area"))) flash_pagebuffer_SRAM2[2048] =
    {
    0
    };
static const size_t flash_BaseAddress = 0x0803F800;
static const size_t flash_PartitionSize = 256;
static const size_t flash_PartitionTotal = 8;
static const size_t flash_PageSize = 2048;
static const size_t flash_PageNum = 127;

 void __flash_sync(void);
static void __flash_init(void);
static void __flash_read(void);
static void __flash_buffer_read(void *data, uint16_t offset, uint16_t size);
static void __flash_buffer_write(void *data, uint16_t offset, uint16_t size);

/*
 * @brief flash initialization
 * @param None
 * @retval None
 * @note
 * 	1. initialize only once at the beginning, use a static flag to ensure this
 * 	2. create a semaphore to protect the flash operation
 * 	3. create a page buffer to store the data
 */
__STATIC_INLINE void __flash_init(void)
    {
    static bool flash_init_flag = false;

    if (flash_init_flag)
	return;

    flash_Sem = xSemaphoreCreateBinary();
    configASSERT(flash_Sem != NULL);
    /*initial give*/
    xSemaphoreGive(flash_Sem);

//    flash_pagebuffer = (uint8_t*) pvPortMalloc(2048);
//    configASSERT(flash_pagebuffer != NULL);
    flash_pagebuffer = flash_pagebuffer_SRAM2;

    flash_init_flag = true;

    /*execute a initial load from flash*/
    __flash_read();

    }

/*
 * @function __flash_sync
 * @brief write the page buffer to the flash
 * @param None
 * @retval None
 * @note
 * 	1. erase the flash page
 * 	2. write the page buffer to the flash page
 */
void __flash_sync(void)
    {
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;
    /*$notice flash同步最好在芯片复位或休眠之前进行
     * 		可以将多次数据持久化合并为一次，延长flash寿命
     *
     * */
    xSemaphoreTake(flash_Sem, portMAX_DELAY);
    HAL_FLASH_Unlock();

    // Configure the erase parameters
    erase.Banks = FLASH_BANK_1;
    erase.NbPages = 1;
    erase.Page = flash_PageNum;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    // Erase the flash memory
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
	{
	HAL_FLASH_Lock();
	xSemaphoreGive(flash_Sem);
	return;
	}
    // Program the flash memory with the new data
    for (int i = 0; i < 256; i++)
	{
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
		(flash_BaseAddress) + i * 8,
		*((uint64_t*) &flash_pagebuffer[i * 8]));
	}
    HAL_FLASH_Lock();
    xSemaphoreGive(flash_Sem);
    return;
    }

/*
 * @function __flash_read
 * @brief read the flash page to the page buffer
 * @param None
 * @retval None
 * @note
 * 	1. read the flash page to the page buffer
 * */
__STATIC_INLINE void __flash_read(void)
    {
    memcpy(flash_pagebuffer, (const void*) flash_BaseAddress, 2048);
    }

/*
 * @function __flash_buffer_read
 * @brief read the flash page buffer
 * @param data: the data buffer to store the data
 * @param offset: the byte unit offset of the buffer
 * @param size: the byte unit size of the data to read
 * 		maximum size is 256
 * */
__STATIC_INLINE void __flash_buffer_read(void *data, uint16_t offset,
	uint16_t size)
    {
    /*offset+size should not exceed the page end*/
    configASSERT(
	    flash_BaseAddress + offset + size
		    <= flash_BaseAddress + flash_PageSize);
    /*size should not exceed 256*/
    configASSERT(size <= 256);
    memcpy(data, flash_pagebuffer + offset, size);
    }

/*
 * @function __flash_buffer_write
 * @brief write the flash page buffer
 * @param data: the data buffer to store the data
 * @param offset: the byte unit offset of the buffer
 * @param size: the byte unit size of the data to write
 * 		maximum size is 256
 */
__STATIC_INLINE void __flash_buffer_write(void *data, uint16_t offset,
	uint16_t size)
    {
    /*offset+size should not exceed the page end*/
    configASSERT(
	    flash_BaseAddress + offset + size
		    <= flash_BaseAddress + flash_PageSize);
    /*size should not exceed 256*/
    configASSERT(size <= 256);
    memcpy(flash_pagebuffer + offset, data, size);
    }

NVM::NVM(nvm_partition_t partition, void *data, void *_defaultData,
	uint16_t size)
    {
    /*
     * initialize static members
     */
    __flash_init();

    this->partition = partition;
    this->data = data;
    this->defaultData = _defaultData;
    this->size = size;

    }

NVM::~NVM()
    {
    }

void NVM::save()
    {
    __flash_buffer_write(this->data, static_cast<uint16_t>(partition) * 256,
	    size);
//    __flash_sync();
    /*$notice there is no need to sync every time*/
    }

void NVM::load()
    {
    __flash_read();
    __flash_buffer_read(this->data, static_cast<uint16_t>(partition) * 256,
	    size);
    /*$notice there is no need to sync every time*/
    }

bool NVM::isFactoryDefault()
    {
    uint8_t *_data = (uint8_t*) this->data;
    for (int i = 0; i < size; i++)
	{
	if (_data[i] != 0xFF)
	    {
	    return false;
	    }
	}
    return true;
    }

void NVM::restoreDefault()
    {
    if (this->defaultData != NULL)
	{
	memcpy(this->data, this->defaultData, size);
	}
    }
