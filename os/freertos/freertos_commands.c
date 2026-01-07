#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "inttypes.h"

void freertos_info(void)
{
	printf("Kernel Version: %s\n", tskKERNEL_VERSION_NUMBER);
}

void ps(void)
{
	char *buffer;
	buffer = pvPortMalloc(1024);
	vTaskList(buffer);
	puts(buffer);
	vPortFree(buffer);
}

void fheap_status()
{
	printf("*FreeHeapSize : %" PRId32 "\n\r", (uint32_t)xPortGetFreeHeapSize());
	printf("*MinimumEverFreeHeapSize : %" PRId32 "\n\r", (uint32_t)xPortGetMinimumEverFreeHeapSize());
}

#if defined(MODULE_SHELL)
#include "shell.h"

static int _freertos_info_handler(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	freertos_info();
	return 0;
}

static int _freertos_ps_handler(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	ps();
	return 0;
}
static int _freertos_heap_handler(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	fheap_status();
	return 0;
}


DEFINE_SHELL_CMD(finfo, "print FreeRTOS version",        _freertos_info_handler);
DEFINE_SHELL_CMD(fps,   "print FreeRTOS process status", _freertos_ps_handler);
DEFINE_SHELL_CMD(fheap, "print FreeRTOS heap status", _freertos_heap_handler);

#endif

