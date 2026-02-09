#include "periph_conf.h"
#include "periph_api.h"

void _putchar(char character)
{
	uart_write(UART_STDIO_DEV, (uint8_t *)&character, 1);
}
