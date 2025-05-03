/*
 * os_allocator.hpp
 *
 *  Created on: Nov 12, 2024
 *      Author: IRIS
 */

#ifndef UTILTIES_OS_ALLOCATOR_HPP_
#define UTILTIES_OS_ALLOCATOR_HPP_

#include "FreeRTOS.h"
#include "cmsis_os2.h"

template<typename T>
class osAllocator {
public:
	static inline void* operator new(size_t size) {
		return pvPortMalloc(size);
	}
	static inline void operator delete(void *ptr) {
		vPortFree(ptr);
	}
};


#endif /* UTILTIES_OS_ALLOCATOR_HPP_ */
