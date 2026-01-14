#include "cpu.h"
#include "periph_api.h"
#if defined(MODULE_PRINTF)
#include "printf.h"
#endif
#if defined(MODULE_NSCBROKER)
#include "nscbroker.h"
#endif
#if defined( __ICCARM__ )
#pragma location="__vectors_in_ram"
#endif
static uint32_t __vectors_in_ram[256] __attribute__((section("vtor_ram")));

static void remap_vector(void)
{
	extern uint32_t __Vectors[];
	int i;

	for (i = 0; i < 256; i++) {
		__vectors_in_ram[i] = __Vectors[i];
	}
	__disable_irq();
	SCB->VTOR = (uint32_t)__vectors_in_ram;
	__DSB();
	__enable_irq();
}

#if defined(MODULE_SHELL)
#include "shell.h"

static int _reset_handler(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	//NVIC_SystemReset();
	reset_chip();

	return 0;
}

DEFINE_SHELL_CMD(reset, "reset the board", _reset_handler);
#endif


__attribute__((naked)) void HardFault_Handler(void)
{
	__asm__ volatile
		(
		 "mov r0, lr          \n"
		 "mrs r1, msp         \n"
		 "mrs r2, psp         \n"
		 "bl ProcessHardFault \n"
		 :
		 :
		 : "r0","r4","r5","r6","lr"
		);
}

void cpu_init(void)
{
	SystemCoreClockUpdate();

	remap_vector();

#if defined(MODULE_NSCBROKER)
	nscbroker_init();
#endif
}

