#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "periph_api.h"
#include "time.h"
#include "hexdump.h"

#include "nsc_def.h"
#include "nscbroker.h"

#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#elif defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#elif defined(OS_NONE)
#else
#error "not support"
#endif

#if defined(OS_FREERTOS)
static QueueHandle_t queue = 0;
static TaskHandle_t task_id = 0;
#else
static uint32_t task_id = 0;
#endif


#if defined( __ICCARM__ )
#error Not Support!!

#elif defined( __ARMCC_VERSION )
extern unsigned int Image$$ARM_LIB_STACK$$Base;
extern unsigned int Image$$ARM_LIB_STACK$$ZI$$Limit;
static const uint32_t stack_start = (uint32_t)&Image$$ARM_LIB_STACK$$Base;
static const uint32_t stack_end = (uint32_t)&Image$$ARM_LIB_STACK$$ZI$$Limit;
#else
extern uint32_t __StackLimit;
extern uint32_t __StackTop;
static const uint32_t stack_start = (uint32_t)&__StackLimit;
static const uint32_t stack_end = (uint32_t)&__StackTop;
#endif

int nscbroker_is_init();

#if defined(OS_FREERTOS)
void nscbroker_thread(void *arg)
{
	(void)arg;
	// portALLOCATE_SECURE_CONTEXT( NSCBROKER_SECURE_STACK_SIZE );
	portALLOCATE_SECURE_CONTEXT( 82*1024 );
	
	nsc_msg_t *msg = NULL;
	while(1) {
		msg = NULL;

		// pop message
		if (xQueueReceive(queue, &msg, portMAX_DELAY) == pdPASS ) {
			// execute message
			extern void *nsc_dispatcher(void *magic, void *, void *param2, void *params);
#if 0			
			printf("call nsc_dispatcher(%d) in nscbroker_thread\n\r", msg->id);
#endif
			msg->ret = nsc_dispatcher(NULL, NULL, NULL, (void*)msg);
			// unlock
			xSemaphoreGive(msg->sema);
		}
	}
}
#endif

int nscbroker_is_in_task()
{
	uint32_t dummy;
	#if 0
	printf("nscbroker_is_in_task %p %X %X\n\r", &dummy, stack_limit, stack_base);
	#endif
	if (&dummy > (uint32_t*)stack_start && &dummy < (uint32_t*)stack_end)
		return 0;
	else 
		return 1;
}

void nscbroker_msg_disp(nsc_msg_t *msg)
{
	printf("*** nscbroker_msg_disp *** \n\r");
	printf("*** id %d, ret %p, param_len %d\n\r", msg->id, msg->ret, msg->param_len);

	for (int i = 0 ; i < msg->param_len ; i++) {
		printf("*** param[%d] %p \n\r", i, msg->param[i]);
	}
}

// caller
void* nscbroker_run(int num, ...)
{
	nsc_msg_t s = {0};
	
	void *ret = (void *)NSCBROKER_ERROR;

	if (num < 1) {
		goto end;
	}

	va_list ap;
	va_start(ap, num);

	// function pointer
	s.id = va_arg(ap, int);

	// function arg count
	s.param_len = num - 1;

	// arguments
	for (int i = 0 ; i < s.param_len ; i++) {
		s.param[i] = va_arg(ap, void*);
	}

	va_end(ap);

#if 0
	nscbroker_msg_disp(&s);
#endif

	if (nscbroker_is_init() == 0 || nscbroker_is_in_task() == 0) {
		extern void *nsc_dispatcher(void *magic, void *, void *param2, void *params);
		ret = nsc_dispatcher(NULL, NULL, NULL, (void*)&s);
	}
	else{
#if defined(OS_FREERTOS)		
		// create
		s.sema = xSemaphoreCreateBinary();
		if (s.sema == NULL) {
			goto end;
		}
		xSemaphoreGive(s.sema);
		
		// lock
		if (xSemaphoreTake(s.sema, portMAX_DELAY) != pdPASS) {
			goto end;
		}

		// push msg to securecall thread
		nsc_msg_t *pmsg = &s;
		if (xQueueSend(queue, (void*)&pmsg, ( TickType_t )10000) != pdPASS) {
			goto end;
		}
		// lock  <<== wait
		if (xSemaphoreTake(s.sema, portMAX_DELAY) != pdPASS) {
			goto end;
		}
		ret = pmsg->ret;
		vSemaphoreDelete(s.sema);
#endif
	}

end:
	return ret;
}

int nscbroker_is_init()
{
	if(task_id == 0)
		return 0;
	else 
		return 1;
}

int nscbroker_init()
{
    int ret = 1;

    if (task_id != 0) {
        ret = 2;
        goto end;
    }
#if defined(OS_FREERTOS)		
    queue = xQueueCreate(16, 4);
	if (queue == NULL) {
        ret = 3;
		goto end;
	}

    if(xTaskCreate(nscbroker_thread, "nscbroker", 256, NULL, configMAX_PRIORITIES - 1, &task_id) != pdTRUE) {
        ret = 3;
		goto end;
	}
#else
	task_id = 1;
#endif
    ret = 0;
end:
    return ret;
}

int nscbroker_deinit()
{
#if defined(OS_FREERTOS)		
    if(task_id != 0)
        vTaskDelete(task_id);

    if(queue != 0)
        vQueueDelete(queue);

    queue = 0;
    task_id = 0;
#endif
    return 0;
}
