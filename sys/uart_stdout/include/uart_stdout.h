#ifndef _UART_STDOUT_H_
#define _UART_STDOUT_H_

void uart_stdout_init(void);

int uart_stdio_write(const unsigned char *buffer, unsigned int length);

#endif
