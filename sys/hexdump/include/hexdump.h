#ifndef _HEXDUMP_H_
#define _HEXDUMP_H_

#include <stdio.h>
#if defined(__ICCARM__)
#include <stdint.h>
#elif defined (__CC_ARM)
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#include <stdint.h>
#elif defined(__GNUC__)
#include <stdint.h>
#endif

#define HEXDUMP_ATTR_NONE    (0)
#define HEXDUMP_ATTR_ASCII    ((1<<0))
#define HEXDUMP_ATTR_DEFAULT  ((1<<1))
#define HEXDUMP_ATTR_PRESS    ((1<<2))
#define HEXDUMP_ATTR_XCOMMA   ((1<<3))
#define HEXDUMP_ATTR_SWAP32   ((1<<4))

void hexlog(char *str, void *data, size_t data_len);
void hexdump(void *target, size_t len, uint32_t attr);

#endif
