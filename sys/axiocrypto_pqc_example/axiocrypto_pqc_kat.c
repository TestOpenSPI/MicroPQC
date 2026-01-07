#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "axiocrypto_pqc_example.h"

typedef int (*kat_fn_t)(void);
typedef struct {
    const char* name;
    const char* uname;
    kat_fn_t fn;
} kat_entry_t;

static const kat_entry_t g_kats[] = {
    {"aimer128f", "AIMer128F", axiocrypto_pqc_aimer128f_kat},
    {"haetae2", "HAETAE2", axiocrypto_pqc_haetae2_kat},
    {"dilithium2", "Dilithium2(ML-DSA-44)", axiocrypto_pqc_dilithium2_kat},
    {"sphincs128f", "SPHINCS+128F(SLH-DSA-SHAKE128F)", axiocrypto_pqc_sphincs128f_kat},
    {"falcon512", "Falcon512(FN-DSA-512)", axiocrypto_pqc_falcon512_kat},
    {"kyber512", "Kyber512(ML-KEM-512)", axiocrypto_pqc_kyber512_kat},
    {"ntru768", "NTRU+768", axiocrypto_pqc_ntru768_kat},
    {"smaug1", "SMAUG-T1", axiocrypto_pqc_smaug1_kat}
};

void axiocrypto_pqc_kat_list(void)
{
    for(size_t i = 0; i < sizeof(g_kats)/sizeof(g_kats[0]); i++)
        printf("%s\n", g_kats[i].name);
}

void axiocrypto_pqc_kat_all(void)
{
    for(size_t i = 0; i < sizeof(g_kats)/sizeof(g_kats[0]); i++) {
        printf("%s KAT\n", g_kats[i].uname);
        g_kats[i].fn();
    }
}

int axiocrypto_pqc_kat_one(const char* name)
{
    for(size_t i = 0; i < sizeof(g_kats)/sizeof(g_kats[0]); i++) {
        if (strcmp(name, g_kats[i].name) == 0) {
            printf("%s KAT\n", g_kats[i].uname);
            g_kats[i].fn();
            return 0;
        }
    }
    return -1;
}
