#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "nsc_api.h"
#include "FreeRTOS.h"

void secure_stack_init()
{
    secure_stack_set_init();
}
void secure_stack_disp()
{
    uint32_t totalStackSize = 0, usedStackSize = 0;
    secure_stack_get_info(&totalStackSize, &usedStackSize);
    uint32_t usage_percent = (usedStackSize * 100) / totalStackSize;
    printf("%2ld/%ldKB(%lu%%)\n", 
       usedStackSize/1024, totalStackSize/1024, usage_percent);
}

void print_elapsed_ms_ticks(TickType_t s, TickType_t e, uint32_t ntests)
{
    TickType_t dt = e - s;
    uint32_t ms = (uint32_t)pdTICKS_TO_MS(dt);
    uint32_t avg = (ntests != 0) ? (ms / ntests) : 0;

    printf("%8lums/op", (unsigned long)avg);
}
