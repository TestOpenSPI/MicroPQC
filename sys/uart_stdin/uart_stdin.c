#include <stdio.h>
#include "periph_conf.h"
#include "periph_api.h"
#if defined(MODULE_PRINTF)
#include "printf.h"
#endif

#include "isrpipe.h"
static char __buf[256]; /* must be power of 2 */
static isrpipe_t stdio_isrpipe;

static void uart_stdio_isr(void)
{
	unsigned char c;

	if (uart_irq_is_rx(uart_stdio_dev)) {
		uart_read(uart_stdio_dev, &c, 1);
		isrpipe_write_one(&stdio_isrpipe, c);
	}

	uart_irq_clear(uart_stdio_dev);
}

void uart_stdin_flush(void)
{
    tsrb_init(&stdio_isrpipe.tsrb, __buf, sizeof(__buf));
}

void uart_stdin_init(void)
{
	/* assume that uart_stdout_init is already done */

	if (isrpipe_init(&stdio_isrpipe, __buf, sizeof(__buf)) != 0) {
		printf("uart_stdin: isrpipe_init failed\n");
	}

	setbuf(stdin, 0);

	uart_irq_setup(uart_stdio_dev, UART_IRQ_RX, 0, uart_stdio_isr);
}

int uart_stdio_read(unsigned char *buffer, unsigned int length, uint32_t timeout)
{
	return isrpipe_read(&stdio_isrpipe, (char *)buffer, length, timeout);
}
