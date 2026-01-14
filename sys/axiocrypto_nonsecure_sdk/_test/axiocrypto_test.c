

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <assert.h>
#include <stdint.h>

#if !defined(OS_FREERTOS)
    #error Have to use OS_FREERTOS!!
#endif

#include "shell.h"
#include "axiocrypto.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "axiocrypto.h"
#include "hexdump.h"

extern int aria_key_storage_example();
extern int example_axiocrypto_ecdsa_key();
extern int example_aria_gcm_test();


int example_axiocrypto_drbg()
{
    CRYPTO_STATUS ret;
    uint8_t buf[32];

    ret = axiocrypto_random(buf, 32);
    if (ret != CRYPTO_SUCCESS) {
	    printf("FAIL: axiocrypto_random %d \n\r", ret);
	    return 1;
    }

#if 1
    printf("random\n\r");
    hexdump(buf, 32, 0);
    printf("\n\r");
#endif

    return 0;
}

int example_axiocrypto_aria()
{
    CRYPTO_STATUS ret;
    ctx_handle_t handle = {0,};
    operation_mode_t opmode = OP_MODE_NOTHING;

    uint8_t key[16];
    uint32_t key_len = 16;
    
    uint8_t iv[16];
    uint32_t iv_len = 16;
    
    uint8_t plain_msg[32];
    uint32_t plain_msg_len = 32;
    
    uint8_t encrypt_msg[32];
    uint32_t encrypt_msg_len = 32;
    
    uint8_t decrypt_msg[32];
    uint32_t decrypt_msg_len = 32;

    handle[0] = 100;

    memset(iv, 0x00, iv_len);
    memset(key, 0x00, key_len);
    memset(plain_msg, 0xAB, plain_msg_len);

    ret = axiocrypto_info(NULL, 0, &opmode);
    if(ret != CRYPTO_SUCCESS){
        printf("FAIL: axiocrypto_info %d\n\r", ret);
        return 1;
    }

    if (opmode != OP_MODE_APPROVED_KCMVP && opmode != OP_MODE_NON_APPROVED) {
        printf("FAIL: op mode\n\r");
        return 1;
    }

    // printf("encrypt\n\r");
    ret = axiocrypto_allocate_slot(handle, SYM_ARIA, CTX_ATTR_NONE);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_allocate_slot %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_putkey(handle, key, key_len, 0, CTX_ATTR_NONE);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_putkey %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_enc_init(handle, SYM_ARIA, SYM_MODE_CBC, iv, iv_len);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_enc_init %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_enc_update(handle, plain_msg, plain_msg_len, encrypt_msg, &encrypt_msg_len);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_enc_update %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_enc_final(handle, NULL, 0);    
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_enc_final %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_free_slot(handle, SYM_ARIA);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_free_slot %d\n\r", ret);
        return 1;
    }

    // printf("decrypt\n\r");
    ret = axiocrypto_allocate_slot(handle, SYM_ARIA, CTX_ATTR_NONE);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_allocate_slot %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_putkey(handle, key, key_len, 0, CTX_ATTR_NONE);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_putkey %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_dec_init(handle, SYM_ARIA, SYM_MODE_CBC, iv, iv_len);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_dec_init %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_dec_update(handle, encrypt_msg, encrypt_msg_len, decrypt_msg, &decrypt_msg_len);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_dec_update %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_sym_dec_final(handle, NULL, 0);    
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_dec_final %d\n\r", ret);
        return 1;
    }
    ret = axiocrypto_free_slot(handle, SYM_ARIA);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_free_slot %d\n\r", ret);
        return 1;
    }

    if (plain_msg_len != decrypt_msg_len) {
        printf("FAIL : length\n\r");
        return 1;
    }

    if (memcmp(plain_msg, decrypt_msg, plain_msg_len) != 0) {
        printf("FAIL : message\n\r");
        return 1;
    }

    return 0;
}


int example_axiocrypto_hash()
{
    CRYPTO_STATUS ret = CRYPTO_ERROR;
    ctx_handle_t handle = {0,};

    uint8_t msg[16] = {0};
    uint32_t msg_len = sizeof(msg);

    uint8_t digest[32];

    handle[0] = 100;

    memset(msg, 0, msg_len);
    memset(digest, 0, 32);

    ret = axiocrypto_hash_init(handle, HASH_SHA_256);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL : axiocrypto_hash_init\n\r");
        return 1;
    }
        
    ret = axiocrypto_hash_update(handle, msg, msg_len);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL : axiocrypto_hash_update\n\r");
        return 1;
    }
        
    ret = axiocrypto_hash_final(handle, digest, 32);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL : axiocrypto_hash_final\n\r");
        return 1;
    }
   
    return 0;        
}

int example_axiocrypto_hmac()
{
    CRYPTO_STATUS ret;

    uint8_t digest[32];
    uint32_t digest_len = sizeof(digest);

    uint8_t key[32];
    uint32_t key_len = sizeof(key);

    uint8_t msg[18];
    uint32_t msg_len = sizeof(msg);

    memset(key, 0x00, key_len);
    memset(msg, 0x00, msg_len);

    ret = axiocrypto_hmac( HMAC_SHA_256, key, key_len, msg, msg_len, digest, digest_len);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL : axiocrypto_hmac\n\r");
        return 1;
    }

#if 0
    printf("key\n\r");
    hexdump(key, key_len, HEXDUMP_ATTR_NONE);
    printf("msg\n\r");
    hexdump(msg, msg_len, HEXDUMP_ATTR_NONE);
    printf("digest\n\r");
    hexdump(digest, digest_len, HEXDUMP_ATTR_NONE);
#endif

    return 0;
}


int example_axiocrypto_ecdsa()
{
    CRYPTO_STATUS ret;
    ctx_handle_t handle = {0,};
    int rdonly = 0;
    uint8_t msg[32] = {0,};
    uint8_t sig[64] = {0,};
    uint32_t siglen = 64;

    handle[0] = 1;

    ret = axiocrypto_allocate_slot(handle, ASYM_ECDSA_P256, 0);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_allocate_slot %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_asym_genkey(handle, ASYM_ECDSA_P256, rdonly);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_genkey %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_asym_sign(handle, msg, 32, RAW_MSG, sig, &siglen);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_sign %d\n\r", ret);
        return 1;
    }

    siglen = 64;
    ret = axiocrypto_asym_verify(handle, msg, 32, RAW_MSG,  sig, siglen);
    if (ret != CRYPTO_SIG_ACCEPT && ret != CRYPTO_SIG_REJECT) {
        printf("FAIL: axiocrypto_asym_verify %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_asym_getkey(handle, ASYM_ECDSA_P256, sig, 64);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_getkey %d\n\r", ret);
        return 1;
    }
#if 0
    hexdump(sig, 64, HEXDUMP_ATTR_NONE);
#endif

    ret = axiocrypto_free_slot(handle, ASYM_ECDSA_P256);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_free_slot %d\n\r", ret);
        return 1;
    }

    return 0;
}

int example_axiocrypto_ecdh()
{
    CRYPTO_STATUS ret;
    ctx_handle_t handle0 = {0,};
    ctx_handle_t handle1 = {0,};
    uint32_t algo = ASYM_ECDH_P256;
    int rdonly = 0;
    uint8_t kt0[64], kt1[64];
    uint8_t key0[64], key1[64];

    handle0[0] = 1;
    handle1[0] = 2;

    ret = axiocrypto_allocate_slot(handle0, ASYM_ECDH_P256, 0);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_allocate_slot %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_allocate_slot(handle1, ASYM_ECDH_P256, 0);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_allocate_slot %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_asym_genkey(handle0, algo, rdonly);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_genkey %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_asym_getkey(handle0, ASYM_ECDH_P256, kt0, 64);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_getkey %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_asym_genkey(handle1, algo, rdonly);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_genkey %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_asym_getkey(handle1, ASYM_ECDH_P256, kt1, 64);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_getkey %d\n\r", ret);
        return 1;
    }


    ret = axiocrypto_ecdh_computekey(handle0, kt1, 64, key0, 64);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_ecdh_computekey %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_ecdh_computekey(handle1, kt0, 64, key1, 64);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_ecdh_computekey %d\n\r", ret);
        return 1;
    }

    if (memcmp(key0, key1, 64) != 0) {
        printf("FAIL: axiocrypto_ecdh_computekey compare %d\n\r", ret);
        return 1;
    } 

    ret = axiocrypto_free_slot(handle0, ASYM_ECDH_P256);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_free_slot %d\n\r", ret);
        return 1;
    }

    ret = axiocrypto_free_slot(handle1, ASYM_ECDH_P256);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_free_slot %d\n\r", ret);
        return 1;
    }

    return 0;
}

int example_axiocrypto_info()
{
    CRYPTO_STATUS ret = CRYPTO_ERROR;
    operation_mode_t opmode;
    char versionstr[32] = {0,};

    ret = axiocrypto_info(versionstr, sizeof(versionstr), &opmode);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_info %d\n\r", ret);
        return 1;
    }

    printf("info : opmode(%d), ver(%s) \n\r", opmode, versionstr);

    return 0;
}

int example_axiocrypto_mode(operation_mode_t mode){
    CRYPTO_STATUS ret = CRYPTO_ERROR;
    operation_mode_t opmode = OP_MODE_NON_APPROVED;

    ret = axiocrypto_info(NULL, 0, &opmode);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_info %d\n\r", ret);
        return 1;
    }

    if (opmode != mode) {
        printf("reboot\n\r");
        ret = axiocrypto_set_mode((uint32_t)mode);
        if (ret != CRYPTO_SUCCESS) {
            printf("FAIL: axiocrypto_set_mode %d\n\r", ret);
            return 1;
        }
    } else {
        printf("Already set  %d\n\r", ret);
    }

    return 0;
}


int example_axiocrypto_init()
{
    CRYPTO_STATUS ret;

    ret = axiocrypto_init(NULL, 0);

    if (ret == CRYPTO_SUCCESS || ret == CRYPTO_ERR_ALREADY_INIT) {
	    printf("init: OK\n");
    } else {
	    printf("init: FAIL, %d, 0x%x, \n\r", ret, ret);
    }
  
    return 0;
}

void _example_axiocrypto_test()
{
    int ret = 1;

    printf("axiocrypto init\n\r");
    if (example_axiocrypto_init() != 0) {
        goto end;
    }

    printf("axiocrypto info\n\r");
    if (example_axiocrypto_info() != 0) {
        goto end;
    }

    printf("axiocrypto drbg\n\r");
    if (example_axiocrypto_drbg() != 0) {
        goto end;
    }

    printf("axiocrypto aria\n\r");
    if (example_axiocrypto_aria() != 0) {
        goto end;
    }

    printf("axiocrypto hash\n\r");
    if (example_axiocrypto_hash() != 0) {
        goto end;
    }

    printf("axiocrypto hmac\n\r");
    if (example_axiocrypto_hmac() != 0) {
        goto end;
    }

    printf("axiocrypto ecdsa\n\r");
    if (example_axiocrypto_ecdsa() != 0) {
        goto end;
    }

    printf("axiocrypto ecdh\n\r");
    if (example_axiocrypto_ecdh() != 0) {
        goto end;
    }

    ret = 0;

end:
    if (ret == 0) {
        printf("success\n\r");
    } else {
        printf("failed\n\r");
    }
}

void _example_axiocrypto_help()
{
	printf("Axiocrypto Usage : ac [option] \n\r");
    printf("Options : \n\r");
    printf("  test 				        basic test.\n\r");
    printf("  mode [none or kcmvp]      set mode.\n\r");
    printf("  aria_gcm 			        aria gcm mode test.\n\r");
    printf("  ks_ecdsa_putkey           input key in keystorage\n\r");
    printf("  ks_ecdsa 				    ks_ecdsa test.\n\r");
}

int _example_axiocrypto_handler(int argc, char *argv[])
{

    if (argc > 1) {
        if (strcmp(argv[1], "test") == 0) {
            _example_axiocrypto_test();
        } else if (strcmp(argv[1], "mode") == 0 && argc > 2) {
            if (strcmp(argv[2], "none") == 0) {
                example_axiocrypto_mode(OP_MODE_NON_APPROVED);
            } else if(strcmp(argv[1], "kcmvp") == 0) {
                example_axiocrypto_mode(OP_MODE_APPROVED_KCMVP);
            } else {
                printf("unknown mode\n\r");
            }
        } else if (strcmp(argv[1], "aria_gcm") == 0) {
                example_aria_gcm_test();
        } else if (strcmp(argv[1], "ks_ecdsa_putkey") == 0) {
                aria_key_storage_example();
        } else if (strcmp(argv[1], "ks_ecdsa") == 0) {
                example_axiocrypto_ecdsa_key();
        } else {
            _example_axiocrypto_help();
        }
    } else {
        _example_axiocrypto_help();
    }
    
    return 0;
}

DEFINE_SHELL_CMD(axiocrypto, "axiocrypto test", _example_axiocrypto_handler);
