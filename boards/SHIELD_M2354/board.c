#include "board.h"
#include "periph_conf.h"
#include "periph_api.h"
#include "uart_stdout.h"
#if defined(MODULE_SYSTICK)
#include "systick.h"
#endif

void board_init(void)
{
	cpu_init();

#if defined(UART_STDIO_DEV)
	uart_stdout_init();
#endif

	spi_init(SPI_MASTER_0);
}

void board_fini(void)
{
#if defined(UART_STDIO_DEV)
	uart_fini(UART_STDIO_DEV);
#endif
}

