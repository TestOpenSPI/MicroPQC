#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "cpu.h"
#if !defined(CPU_X86_64)
#include <cmsis_compiler.h>
#endif
#if defined(MODULE_UART_STDIN)
#include "uart_stdin.h"
#endif
#if defined(MODULE_UART_STDOUT)
#include "uart_stdout.h"
#endif

#if defined(CPU_NUVOTON_M2351) || defined(CPU_NUVOTON_M2354)
int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    (void)n32In_R0;
    (void)n32In_R1;
    (void)pn32Out_R0;
    return 0;
}
#else

#endif

#if defined(__ICCARM__)
int __write(int fd, const char *buf, unsigned int length)
{
    (void)fd;

#if defined(MODULE_UART_STDOUT)
    return uart_stdio_write((void *)buf, length);
#else
    (void)buf;
    (void)length;
    return 0;
#endif
}

int __read(int fd, char *buf, unsigned int length)
{
    (void)fd;
#if defined(MODULE_UART_STDIN)
    return uart_stdio_read((void *)buf, length, 0xFFFFFFFF);
#else
    (void)buf;
    (void)length;
    return 0;
#endif
}
#elif defined (__CC_ARM)
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#elif defined(__GNUC__) && !defined(OS_LINUX) && !defined(BOARD_NATIVE)
extern char __HeapBase;
extern char __HeapLimit;
extern char __StackLimit;
static char *heap_top = &__HeapBase + 4;
void *_sbrk(ptrdiff_t incr)
{
    void *res;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    if ((heap_top + incr > &__HeapLimit) || (heap_top + incr < &__HeapBase)) {
        res = (void *)-1;
    } else {
        res = heap_top;
        heap_top += incr;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return res;
}

int _write(int fd, const char *buf, unsigned int length)
{
    (void)fd;

#if defined(MODULE_UART_STDOUT)
    return uart_stdio_write((void *)buf, length);
#else
    (void)buf;
    (void)length;
    return 0;
#endif
}

int _read(int fd, char *buf, unsigned int length)
{
    (void)fd;
#if defined(MODULE_UART_STDIN)
    return uart_stdio_read((void *)buf, length, 0xFFFFFFFF);
#else
    (void)buf;
    (void)length;
    return 0;
#endif
}

#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif
#include <sys/time.h>
int _gettimeofday (struct timeval* tp, void* tzvp)
{
    (void)tzvp;
    if (tp) {
        #if defined(OS_FREERTOS)
        uint32_t tick = xTaskGetTickCount();
        /* Ask the host for the seconds since the Unix epoch.  */
        tp->tv_sec = tick / 1000;
        tp->tv_usec = (tick % 1000) *1000;
        #endif
    }

    /* Return fixed data for the timezone.  */
#if 0
    if (tzp) {
        tzp->tz_minuteswest = 0;
        tzp->tz_dsttime = 0;
    }
#endif

    return 0;
}
#endif

#if defined (__CC_ARM)
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
void SendChar_ToUART(int ch)
{
}

int fputc(int ch, FILE *stream)
{
    (void)stream;

#if defined(MODULE_UART_STDOUT)
    uart_stdio_write((void *)&ch, 1);
    return ch;
#else
    (void)ch;
    return 0;
#endif
}

int fgetc(FILE *stream)
{
    (void)stream;
#if defined(MODULE_UART_STDIN)
    char ch;
    uart_stdio_read((void *)&ch, 1, 0xFFFFFFFF);
    return(int)ch;
#else
    return 0;
#endif
}

__attribute__((weak, noreturn))
void __aeabi_assert(const char* expr, const char* file, int line) {
    char str[12], * p;

    fputs("*** assertion failed: ", stderr);
    fputs(expr, stderr);
    fputs(", file ", stderr);
    fputs(file, stderr);
    fputs(", line ", stderr);

    p = str + sizeof(str);
    *--p = '\0';
    *--p = '\n';
    while(line > 0) {
        *--p = '0' + (line % 10);
        line /= 10;
    }
    fputs(p, stderr);

    abort();
}

__attribute__((weak))
void abort(void) {
    for(;;);
}
#elif defined(__GNUC__)
#include <sys/stat.h>
#include <errno.h>

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    errno = ESPIPE;
    return -1;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}
int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = ENOSYS; // "Function not implemented"
    return -1;      // 오류 반환
}
int _getpid(void) {
    return 1; // 아무 값(예: 1)이나 반환
}
#endif
