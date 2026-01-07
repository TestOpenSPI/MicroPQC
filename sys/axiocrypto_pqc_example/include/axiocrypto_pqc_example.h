#ifndef _AXIOCRYPTO_PQC_EXAMPLE_H_
#define _AXIOCRYPTO_PQC_EXAMPLE_H_

#include "hexdump.h"
#include "utils.h"
#include "FreeRTOS.h"

#define PQC_MSG_MAX_SIZE    1024
#define NTESTS              1

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

int axiocrypto_pqc_example(int argc, char *argv[]);

void secure_stack_init();
void secure_stack_disp();
void print_elapsed_ms_ticks(TickType_t s, TickType_t e, uint32_t ntests);

void axiocrypto_pqc_bench_list(void);
void axiocrypto_pqc_bench_all(void);
int axiocrypto_pqc_bench_one(const char* name);

void axiocrypto_pqc_aimer128f_bench();
void axiocrypto_pqc_haetae2_bench();
void axiocrypto_pqc_dilithium2_bench();
void axiocrypto_pqc_sphincs128f_bench();
void axiocrypto_pqc_falcon512_bench();
void axiocrypto_pqc_kyber512_bench();
void axiocrypto_pqc_ntru768_bench();
void axiocrypto_pqc_smaug1_bench();

void axiocrypto_pqc_kat_list(void);
void axiocrypto_pqc_kat_all(void);
int axiocrypto_pqc_kat_one(const char* name);

int axiocrypto_pqc_aimer128f_kat(void);
int axiocrypto_pqc_dilithium2_kat(void);
int axiocrypto_pqc_falcon512_kat(void);
int axiocrypto_pqc_haetae2_kat(void);
int axiocrypto_pqc_sphincs128f_kat(void);
int axiocrypto_pqc_kyber512_kat(void);
int axiocrypto_pqc_ntru768_kat(void);
int axiocrypto_pqc_smaug1_kat(void);

#endif
