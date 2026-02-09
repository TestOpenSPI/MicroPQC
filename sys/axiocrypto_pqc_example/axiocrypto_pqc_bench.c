#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "axiocrypto_pqc_example.h"

typedef void (*bench_fn_t)(void);
typedef struct {
    const char* name;
    const char* uname;
    bench_fn_t fn;
} bench_entry_t;

static const bench_entry_t g_benches[] = {
    {"aimer128f", "AIMer128F", axiocrypto_pqc_aimer128f_bench},
    {"haetae2", "HAETAE2", axiocrypto_pqc_haetae2_bench},
    {"dilithium2", "Dilithium2(ML-DSA-44)", axiocrypto_pqc_dilithium2_bench},
    {"sphincs128f", "SPHINCS+128F(SLH-DSA-SHAKE128F)", axiocrypto_pqc_sphincs128f_bench},
    {"falcon512", "Falcon512(FN-DSA-512)", axiocrypto_pqc_falcon512_bench},
    {"kyber512", "Kyber512(ML-KEM-512)", axiocrypto_pqc_kyber512_bench},
    {"ntru768", "NTRU+768", axiocrypto_pqc_ntru768_bench},
    {"smaug1", "SMAUG-T1", axiocrypto_pqc_smaug1_bench}
};

void axiocrypto_pqc_bench_list(void)
{
    for(size_t i = 0; i < sizeof(g_benches)/sizeof(g_benches[0]); i++)
        printf("%s\n", g_benches[i].name);
}

void axiocrypto_pqc_bench_all(void)
{
    for(size_t i = 0; i < sizeof(g_benches)/sizeof(g_benches[0]); i++) {
        printf("%s Benchmark", g_benches[i].uname);
        g_benches[i].fn();
    }
}

int axiocrypto_pqc_bench_one(const char* name)
{
    for(size_t i = 0; i < sizeof(g_benches)/sizeof(g_benches[0]); i++) {
        if (strcmp(name, g_benches[i].name) == 0) {
            printf("%s Benchmark", g_benches[i].uname);
            g_benches[i].fn();
            return 0;
        }
    }
    return -1;
}
