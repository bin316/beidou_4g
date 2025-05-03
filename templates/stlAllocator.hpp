/*
 * stlAllocator.hpp
 *
 *  Created on: Jan 23, 2025
 *      Author: IRIS
 */

#ifndef STLALLOCATOR_HPP_
#define STLALLOCATOR_HPP_

#include <cstdlib>
#include <new>
#include "FreeRTOS.h"

template <typename T>
class stlAllocator {
public:
    using value_type = T;

    stlAllocator() = default;

    template <typename U>
    stlAllocator(const stlAllocator<U>&) noexcept {}

    T* allocate(std::size_t n) noexcept {
        if (n == 0) {
            return nullptr;
        }
        if (n > static_cast<std::size_t>(-1) / sizeof(T)) {
            return nullptr;
        }
        void* ptr = pvPortMalloc(n * sizeof(T));
        return static_cast<T*>(ptr);
    }

    void deallocate(T* p, std::size_t) noexcept {
        vPortFree(p);
    }
};

template <typename T, typename U>
bool operator==(const stlAllocator<T>&, const stlAllocator<U>&) noexcept {
    return true;
}

template <typename T, typename U>
bool operator!=(const stlAllocator<T>&, const stlAllocator<U>&) noexcept {
    return false;
}


#endif /* STLALLOCATOR_HPP_ */
