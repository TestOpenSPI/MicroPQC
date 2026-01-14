/*
 * Copyright (C) 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup sys
 * @{
 * @file
 * @brief       ISR -> userspace pipe implementation
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include "isrpipe.h"

#if defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#elif defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include "cpu.h"
#elif defined(MODULE_SYSTICK)
#include "cpu.h"
#include "systick.h"
#include <string.h>
#elif defined(OS_LINUX)
#include <string.h>
#include <semaphore.h>
#include <stdio.h>
#else
#error Need time keeping method
#endif

int isrpipe_init(isrpipe_t *isrpipe, char *buf, size_t bufsize)
{
#if defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s;
	s = (osSemaphoreId_t *)(&isrpipe->sema);

    if ((*s = osSemaphoreNew(1, 0, NULL)) == NULL) {
        return 1;
    }
#elif defined(OS_FREERTOS)
	SemaphoreHandle_t *s;
	s = (SemaphoreHandle_t *)(&isrpipe->sema);

    if ((*s = xSemaphoreCreateBinary()) == NULL) {
        return 1;
    }
    //xSemaphoreTake(isrpipe->sema, portMAX_DELAY);
#elif defined(OS_LINUX)
    sem_t *s;
    s = (sem_t *)(&isrpipe->sema);
    sem_init(s, 0, 1);
#else
	memset(&isrpipe->sema, 0, sizeof(isrpipe->sema));
#endif

    tsrb_init(&isrpipe->tsrb, buf, bufsize);
    return 0;
}

void isrpipe_free(isrpipe_t *isrpipe)
{
#if defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s;
	s = (osSemaphoreId_t *)(&isrpipe->sema);
	if (*s)	{
		osSemaphoreDelete(*s);
	}
#elif defined(OS_FREERTOS)
	SemaphoreHandle_t *s;
	s = (SemaphoreHandle_t *)(&isrpipe->sema);
	if (*s)	{
		vSemaphoreDelete(*s);
	}
#elif defined(OS_LINUX)
	sem_t *s;
	s = (sem_t *)(&isrpipe->sema);
        sem_destroy(s);
#else
	(void)isrpipe;
#endif
}


int isrpipe_write(isrpipe_t *isrpipe, const char *src, size_t n)
{
    tsrb_add(&isrpipe->tsrb, src, n);

    /* `res` is either 0 on success or -1 when the buffer is full. Either way,
     * unlocking the mutex is fine.
     */
#if defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s;
	s = (osSemaphoreId_t *)(&isrpipe->sema);

    if (osSemaphoreRelease(*s) != osOK) {
        return -1;
    }

#elif defined(OS_FREERTOS) && defined(CPU_X86_64)
	SemaphoreHandle_t *s;
	s = (SemaphoreHandle_t *)(&isrpipe->sema);
    if (xSemaphoreGive(*s) != pdPASS) {
        return -1;
    }

#elif defined(OS_FREERTOS)
	SemaphoreHandle_t *s;
	s = (SemaphoreHandle_t *)(&isrpipe->sema);

    // temporary method to check the context is in the ISR
    if ((__get_IPSR() != 0) || (__get_PRIMASK() != 0)) {
        BaseType_t yield = pdFALSE;
        if (xSemaphoreGiveFromISR(*s, &yield) != pdPASS) {
            return -1;
        } else {
            portYIELD_FROM_ISR(yield);
        }
    }
    else
    {
        if (xSemaphoreGive(*s) != pdPASS) {
            return -1;
        }
    }
#elif defined(OS_LINUX)
    sem_t *s;
    s = (sem_t *)(&isrpipe->sema);
    sem_post(s);
#endif
    return 0;
}


int isrpipe_write_one(isrpipe_t *isrpipe, char c)
{
    tsrb_add_one(&isrpipe->tsrb, c);

    /* `res` is either 0 on success or -1 when the buffer is full. Either way,
     * unlocking the mutex is fine.
     */
#if defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s;
	s = (osSemaphoreId_t *)(&isrpipe->sema);

    if (osSemaphoreRelease(*s) != osOK) {
        return -1;
    }

#elif defined(OS_FREERTOS) && defined(CPU_X86_64)
	SemaphoreHandle_t *s;
	s = (SemaphoreHandle_t *)(&isrpipe->sema);
    if (xSemaphoreGive(*s) != pdPASS) {
        return -1;
    }

#elif defined(OS_FREERTOS)
	SemaphoreHandle_t *s;
	s = (SemaphoreHandle_t *)(&isrpipe->sema);

    // temporary method to check the context is in the ISR
    if ((__get_IPSR() != 0) || (__get_PRIMASK() != 0)) {
        BaseType_t yield = pdFALSE;
        if (xSemaphoreGiveFromISR(*s, &yield) != pdPASS) {
            return -1;
        } else {
            portYIELD_FROM_ISR(yield);
        }
    } else {
        if (xSemaphoreGive(*s) != pdPASS) {
            return -1;
        }
    }
#elif defined(OS_LINUX)
    sem_t *s;
    s = (sem_t *)(&isrpipe->sema);
    sem_post(s);
#endif
    return 0;
}

int isrpipe_read(isrpipe_t *isrpipe, char *buffer, size_t count, uint32_t timeout)
{
    int res;

#if defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s;
	s = (osSemaphoreId_t *)(&isrpipe->sema);
#elif defined(OS_FREERTOS)
	SemaphoreHandle_t *s;
	s = (SemaphoreHandle_t *)(&isrpipe->sema);
#elif defined(MODULE_SYSTICK)
    volatile uint32_t current;
    uint32_t start = get_systick();
#elif defined(OS_LINUX)
    sem_t *s;
    struct timespec ts;
    s = (sem_t *)(&isrpipe->sema);
#endif

    while (!(res = tsrb_get(&isrpipe->tsrb, buffer, count))) {
#if defined(OS_CMSIS_RTX)
        if (osSemaphoreAcquire(*s, (timeout!=0xFFFFFFFF)?timeout:osWaitForever) != osOK) {
            res = -1; /* osErrorTimeout, osErrorResource, osErrorParameter, ... */
            break;
        }
#elif defined(OS_FREERTOS)
        if (xSemaphoreTake(*s, (timeout!=0xFFFFFFFF)?timeout:portMAX_DELAY) != pdTRUE) {
            res = -1;
            break;
        }
#elif defined(MODULE_SYSTICK)
        if (timeout != 0xFFFFFFFF) {
            __NOP();
            current = get_systick();
            if (systick_diff(start, current) > timeout) {
                res = -1;
                break;
            }
        }
#elif defined(OS_LINUX)
        if (timeout == 0xFFFFFFFF) {
            sem_wait(s);
        } else {
            if (-1 == clock_gettime(CLOCK_REALTIME, &ts)) {
                perror("clock_gettime");
            }
            ts.tv_nsec += (timeout * 1000000);
            if (ts.tv_nsec > 1000000000) {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000;
            }
            sem_timedwait(s, &ts);
        }
#endif
    }

    return res;
}

