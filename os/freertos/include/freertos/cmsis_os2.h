#pragma once

// compatibility with CMSIS_RTX

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"

#define osDelay vTaskDelay

#define osWaitForever         0xFFFFFFFFU ///< Wait forever timeout value.
 
// Flags options (\ref osThreadFlagsWait and \ref osEventFlagsWait).
#define osFlagsWaitAny        0x00000000U ///< Wait for any flag (default).
#define osFlagsWaitAll        0x00000001U ///< Wait for all flags.
#define osFlagsNoClear        0x00000002U ///< Do not clear flags which have been specified to wait for.
 
// Flags errors (returned by osThreadFlagsXxxx and osEventFlagsXxxx).
#define osFlagsError          0x80000000U ///< Error indicator.
#define osFlagsErrorUnknown   0xFFFFFFFFU ///< osError (-1).
#define osFlagsErrorTimeout   0xFFFFFFFEU ///< osErrorTimeout (-2).
#define osFlagsErrorResource  0xFFFFFFFDU ///< osErrorResource (-3).
#define osFlagsErrorParameter 0xFFFFFFFCU ///< osErrorParameter (-4).
#define osFlagsErrorISR       0xFFFFFFFAU ///< osErrorISR (-6).
 
// Thread attributes (attr_bits in \ref osThreadAttr_t).
#define osThreadDetached      0x00000000U ///< Thread created in detached mode (default)
#define osThreadJoinable      0x00000001U ///< Thread created in joinable mode
 
// Mutex attributes (attr_bits in \ref osMutexAttr_t).
#define osMutexRecursive      0x00000001U ///< Recursive mutex.
#define osMutexPrioInherit    0x00000002U ///< Priority inherit protocol.
#define osMutexRobust         0x00000008U ///< Robust mutex.

#define osMutexId_t             SemaphoreHandle_t
#define osMutexRelease          xSemaphoreGive
#define osMutexAcquire(a, b)    xSemaphoreTake((a), (b)!=osWaitForever?(b):portMAX_DELAY)
#define osMutexDelete           vSemaphoreDelete
#define osMutexNew(a)           xSemaphoreCreateMutex()

#define osSemaphoreRelease      xSemaphoreGive

#define osEventFlagsId_t            EventGroupHandle_t
#define osEventFlagsNew(a)          xEventGroupCreate()
#define osEventFlagsDelete          vEventGroupDelete
#define osEventFlagsSet             xEventGroupSetBits
#define osEventFlagsClear           xEventGroupClearBits
uint32_t osEventFlagsWait( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait );

#define osMessageQueueNew(a,b,c)    xQueueCreate((a), (b))
#define osMessageQueueDelete        vQueueDelete
//#define osMessageQueuePut(a,b,c,d)  xQueueSend((a), (b), (d)==osWaitForever?portMAX_DELAY:(d))
//#define osMessageQueueGet(a,b,c,d)  xQueueReceive((a), (b), (d)==osWaitForever?portMAX_DELAY:(d))

BaseType_t osMessageQueuePut(QueueHandle_t xQueue, const void * pvItemToQueue, uint8_t msg_prio, TickType_t xTicksToWait);
BaseType_t osMessageQueueGet(QueueHandle_t xQueue, void * pvBuffer, uint8_t *msg_prio, TickType_t xTicksToWait);
 
/// Status code values returned by CMSIS-RTOS functions.
typedef enum {
  osOK                      =  0,         ///< Operation completed successfully.
  osError                   = -1,         ///< Unspecified RTOS error: run-time error but no other error message fits.
  osErrorTimeout            = -2,         ///< Operation not completed within the timeout period.
  osErrorResource           = -3,         ///< Resource not available.
  osErrorParameter          = -4,         ///< Parameter error.
  osErrorNoMemory           = -5,         ///< System is out of memory: it was impossible to allocate or reserve memory for the operation.
  osErrorISR                = -6,         ///< Not allowed in ISR context: the function cannot be called from interrupt service routines.
  osStatusReserved          = 0x7FFFFFFF  ///< Prevents enum down-size compiler optimization.
} osStatus_t;
