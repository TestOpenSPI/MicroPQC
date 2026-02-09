#ifndef _UART_STDIN_H_
#define _UART_STDIN_H_

void uart_stdin_init(void);

void uart_stdin_flush(void);

int uart_stdio_read(unsigned char *buffer, unsigned int length, uint32_t timeout);

#endif

