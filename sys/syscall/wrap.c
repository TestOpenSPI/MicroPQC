#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#if defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#elif defined(OS_FREERTOS)
#include "FreeRTOS.h"
#endif

// #define USE_NEWLIB_MEM

extern void *__real_malloc(size_t c);
extern void __real_free(void *p);
extern void *__real_calloc(size_t count, size_t eltsize);
extern void *__real_realloc(void *p, size_t newsize);

void *__wrap_malloc (size_t size)
{
#if defined(USE_NEWLIB_MEM)
    return __real_malloc(size);
#else
#if OS_FREERTOS
	return pvPortMalloc(size);
#elif defined(OS_CMSIS_RTX)
	return __real_malloc(size);
#else
	return __real_malloc(size);
#endif
#endif
}

void __wrap_free (void *p)
{
#if defined(USE_NEWLIB_MEM)
    __real_free(p);
#else
#if OS_FREERTOS
	vPortFree(p);
#elif defined(OS_CMSIS_RTX)
	__real_free(p);
#else
	__real_free(p);
#endif
#endif
}

void *__wrap_calloc(size_t count, size_t eltsize)
{
#if defined(USE_NEWLIB_MEM)
    return __real_calloc(count, eltsize);
#else
#if OS_FREERTOS
	void *p = NULL;
	p = pvPortMalloc(count*eltsize);
	if(p != NULL)
		memset(p, 0x00, count*eltsize);
	return p;
#elif defined(OS_CMSIS_RTX)
	return __real_calloc(count, eltsize);
#else
	return __real_calloc(count, eltsize);
#endif
#endif
}

void *__wrap_realloc(void *p, size_t newsize)
{
#if defined(USE_NEWLIB_MEM)
    return __real_realloc(p, newsize);
#else
#if OS_FREERTOS
	void *pvPortRealloc(void *pv,  size_t newsize);
	return pvPortRealloc(p, newsize);
#elif defined(OS_CMSIS_RTX)
	return __real_realloc(p, newsize);
#else
	return __real_realloc(p, newsize);
#endif
#endif
}
