#ifndef _NSCBROKER_H_
#define _NSCBROKER_H_

#define NSCBROKER_SECURE_STACK_SIZE     2048

#define _NARGS(_1,_2,_3,_4,_5,_6_,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16,N,...) N
#define NARGS(...) _NARGS(__VA_ARGS__, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)
#define NSCBROKER_RUN(...) nscbroker_run(NARGS(__VA_ARGS__), __VA_ARGS__)

#include "nsc_def.h"

void* nscbroker_run(int num, ...);
int nscbroker_init();
int nscbroker_deinit();

#endif // _NSCBROKER_H_
