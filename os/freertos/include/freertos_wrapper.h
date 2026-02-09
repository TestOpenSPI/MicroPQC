#pragma once

#include "freertos_wrapper.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* 0 ~ 10 에서 중간. 큰 숫자가 높은 우선순위
   - 0: Idle Task
   - 10: Timer Task
   일반 Task는 1~9 사이의 우선순위를 가져야 함
*/
#define TASK_PRIORITY_MIDDLE 5

typedef struct
{
    QueueHandle_t sender_q; /**< Will be filled in by msg_send. */
    QueueHandle_t target_q;
    uint16_t type; /**< Type field. */
    union
    {
        void *ptr;      /**< Pointer content field. */
        uint32_t value; /**< Value content field. */
    } content;          /**< Content of the message. */
} msg_t;

typedef struct
{
    TimerHandle_t handle;
    msg_t msg;
} xtimer_t;

void msg_init_queue(QueueHandle_t *my_q, int num);
void msg_deinit_queue(QueueHandle_t *my_q);
int msg_send(msg_t *m, QueueHandle_t target_q);
int msg_send_isr(msg_t *m, QueueHandle_t target_q);
int msg_receive(QueueHandle_t q, msg_t *m);
void xtimer_set_msg(xtimer_t *tm, uint32_t offset, QueueHandle_t tq);
void timer_cb(TimerHandle_t xTimer);
void create_timer(xtimer_t *tm, QueueHandle_t q, const char *name);
void delete_timer(xtimer_t *tm);
void xtimer_remove(TimerHandle_t xTimer);
void xtimer_sleep(uint32_t seconds);
void xtimer_ms_sleep(uint32_t ms);
int msg_receive_timeout(QueueHandle_t q, msg_t *m, uint32_t timeout_ms);