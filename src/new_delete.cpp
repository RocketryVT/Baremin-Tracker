// new_delete.cpp — route C++ heap through FreeRTOS Heap4
//
// By default operator new calls the newlib malloc which uses a separate heap
// region and is NOT tracked by xPortGetFreeHeapSize().  Overriding new/delete
// here routes all C++ heap allocations (e.g. RadioLib SPI buffers) through
// pvPortMalloc/vPortFree so:
//   - Heap4 tracks all allocations in xPortGetFreeHeapSize()
//   - vApplicationMallocFailedHook fires on OOM instead of silent nullptr
//   - No second heap region is needed in the linker script

#include <cstddef>
#include <cstdlib>
#include <new>

#include "FreeRTOS.h"

// Throwing new — FreeRTOS malloc-failed hook handles OOM; we never throw.
void* operator new( std::size_t size )
{
    void* p = pvPortMalloc( size );
    return p;
}

void* operator new[]( std::size_t size )
{
    void* p = pvPortMalloc( size );
    return p;
}

// nothrow variants
void* operator new( std::size_t size, const std::nothrow_t& ) noexcept
{
    return pvPortMalloc( size );
}

void* operator new[]( std::size_t size, const std::nothrow_t& ) noexcept
{
    return pvPortMalloc( size );
}

void operator delete( void* p ) noexcept
{
    vPortFree( p );
}

void operator delete[]( void* p ) noexcept
{
    vPortFree( p );
}

// Sized delete (C++14) — Heap4 doesn't use the size hint, delegate to vPortFree.
void operator delete( void* p, std::size_t ) noexcept
{
    vPortFree( p );
}

void operator delete[]( void* p, std::size_t ) noexcept
{
    vPortFree( p );
}
