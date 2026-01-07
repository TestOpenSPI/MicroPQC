#include <stdio.h>
#include "periph_conf.h"
#include "periph_api.h"

void uart_stdout_init(void)
{
// #if !defined(MODULE_PRINTF)
	setbuf(stdout, 0);
// #endif

	uart_init(uart_stdio_dev, -1);
}

int uart_stdio_write(const unsigned char *buffer, unsigned int length)
{
	return uart_write(uart_stdio_dev, buffer, length);
}

