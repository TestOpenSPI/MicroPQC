/*
 * Copyright (C) 2023 SecurityPlatform
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     SecurityPlatform AMI
 * @{
 *
 * @file
 * @brief       RIOT --> FreeRTOS wrapper
 *
 * @author      Byungchul Cho
 *
 * @}
 */
#include <stdio.h>

#include "freertos_wrapper.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#if !defined(MSG_ERR)
    #define MSG_ERR(fmt, args...)
#endif

#if !defined(MSG_000)
    #define MSG_000(fmt, args...)
#endif

#if !defined(MG_ERR)
    #define MG_ERR(fmt, args...)
#endif

void msg_init_queue(QueueHandle_t *my_q, int num)
{
    *my_q = xQueueCreate(num, sizeof(msg_t));
    if (*my_q == NULL)
    {
        MSG_ERR("Could not make Queue");
    }
}

void msg_deinit_queue(QueueHandle_t *my_q)
{
    vQueueDelete(*my_q);
    *my_q = NULL;
}

int msg_send(msg_t *m, QueueHandle_t target_q)
{
    BaseType_t xStatus = xQueueSend(target_q, m, 0);
    if (xStatus != pdPASS)
    {
        MSG_ERR("Could not send to the queue");
        return -1;
    }

    return 1;
}

int msg_send_isr(msg_t *m, QueueHandle_t target_q)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    BaseType_t xStatus =
        xQueueSendFromISR(target_q, m, &xHigherPriorityTaskWoken);
    if (xStatus != pdPASS)
    {
        MSG_ERR("Could not send to the queue");
        return -1;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

int msg_receive_timeout(QueueHandle_t q, msg_t *m, uint32_t timeout_ms)
{
    BaseType_t xStatus = xQueueReceive(q, m, (TickType_t)timeout_ms);
    if (xStatus != pdPASS)
    {
        MSG_ERR("Could not receive from the queue");
        return -1;
    }

    return 0;
}

int msg_receive(QueueHandle_t q, msg_t *m)
{
    /* Setting xTicksToWait to portMAX_DELAY will cause the task to wait
     * indefinitely (without timing out) provided INCLUDE_vTaskSuspend is set to
     * 1 in FreeRTOSConfig.h. */
    BaseType_t xStatus = xQueueReceive(q, m, portMAX_DELAY);
    if (xStatus != pdPASS)
    {
        MSG_ERR("Could not receive from the queue");
        return -1;
    }

    return 0;
}

void xtimer_set_msg(xtimer_t *tm, uint32_t offset, QueueHandle_t tq)
{
    tm->msg.target_q = tq;

    vTimerSetTimerID(tm->handle, &tm->msg);
    xTimerChangePeriod(tm->handle, offset / 1000, 0);
}

void timer_cb(TimerHandle_t xTimer)
{
    msg_t *msg = pvTimerGetTimerID(xTimer);

    MSG_000("timer_cb");

    msg_send(msg, msg->target_q);
}

void create_timer(xtimer_t *tm, QueueHandle_t q, const char *name)
{
    tm->handle = xTimerCreate(name, 1000, pdFALSE, 0, timer_cb);
    if (tm->handle == NULL)
    {
        MSG_ERR("xTimerCreate Fail: %s", name);
    }
    tm->msg.sender_q = q;
}

void delete_timer(xtimer_t *tm)
{
    if(tm != NULL){
        xTimerDelete(tm->handle, 0);
        tm->handle = NULL;
    }
}

void xtimer_remove(TimerHandle_t xTimer) { xTimerStop(xTimer, 0); }

void xtimer_sleep(uint32_t seconds) { vTaskDelay(seconds * 1000); }

void xtimer_ms_sleep(uint32_t ms) { vTaskDelay(ms); }
