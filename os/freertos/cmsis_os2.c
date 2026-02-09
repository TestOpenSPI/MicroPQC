#include "freertos/cmsis_os2.h"

uint32_t osEventFlagsWait( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait )

{
    EventBits_t uxBits;
    BaseType_t xClearOnExit = xWaitForAllBits & osFlagsNoClear ? pdFALSE : pdTRUE;
    BaseType_t _xWaitForAllBits = (xWaitForAllBits & osFlagsWaitAll)?pdTRUE:pdFALSE; 
    uxBits =  xEventGroupWaitBits(  xEventGroup,
                                    uxBitsToWaitFor,
                                    xClearOnExit,
                                    _xWaitForAllBits,
                                    xTicksToWait);
    if (uxBits & uxBitsToWaitFor) {
        return uxBits;
    }
    return osFlagsErrorTimeout;
}

BaseType_t osMessageQueuePut(QueueHandle_t xQueue, const void * pvItemToQueue, uint8_t msg_prio, TickType_t xTicksToWait)
{
    (void)msg_prio;
    return xQueueSend(xQueue, pvItemToQueue, xTicksToWait);
}

BaseType_t osMessageQueueGet(QueueHandle_t xQueue, void * pvBuffer, uint8_t *msg_prio, TickType_t xTicksToWait)
{
    BaseType_t ret;
    (void)msg_prio;
    ret = xQueueReceive(xQueue, pvBuffer, xTicksToWait);
    if (ret == pdTRUE)
        return osOK;
    return osError;
}
