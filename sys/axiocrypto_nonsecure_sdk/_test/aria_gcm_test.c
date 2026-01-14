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

int example_aria_gcm_test()
{
    CRYPTO_STATUS ret;
    ctx_handle_t handle = {0,};
    operation_mode_t opmode = OP_MODE_NOTHING;

	uint8_t aad_128[16] = {0};
	uint8_t tag[16] = {0};

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

    axiocrypto_init(NULL, 0);

    ret = axiocrypto_info(NULL, 0, &opmode);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_info %d\n\r", ret);
        return 1;
    }

    if (opmode != OP_MODE_APPROVED_KCMVP && opmode != OP_MODE_NON_APPROVED) {
        printf("FAIL: op mode\n\r");
        return 1;
    }

    printf("encrypt\n\r");
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

    printf("ARIA GCM mode encryption start\n\r");
	ret = axiocrypto_sym_enc_GCM(handle, SYM_ARIA, plain_msg, plain_msg_len, aad_128, sizeof(aad_128), tag, sizeof(tag), iv, iv_len, encrypt_msg, &encrypt_msg_len);
	if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_enc_GCM %d\n\r", ret);
		return 1;
    }
    printf("Input plain msg is:\r\n");
    hexdump(plain_msg, plain_msg_len, HEXDUMP_ATTR_NONE);

    printf("Tag:\r\n");
    hexdump(tag, 16, HEXDUMP_ATTR_NONE);

    printf("message after encryption msg is:\r\n");
    hexdump(encrypt_msg, encrypt_msg_len, HEXDUMP_ATTR_NONE);

    ret = axiocrypto_free_slot(handle, SYM_ARIA);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_free_slot %d\n\r", ret);
        return 1;
    }

    printf("decryption \n\r");
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

    printf("ARIA GCM mode decryption start\n\r");
    ret = axiocrypto_sym_dec_GCM(handle, SYM_ARIA, encrypt_msg, encrypt_msg_len, aad_128, sizeof(aad_128), tag, sizeof(tag), iv, iv_len, decrypt_msg, &decrypt_msg_len);
	if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_sym_dec_GCM %d\n\r", ret);
		return ret;
    }

    printf("Input encrypt msg is :\r\n");
    hexdump(encrypt_msg, encrypt_msg_len, HEXDUMP_ATTR_NONE);
    
    printf("Tag:\r\n");
    hexdump(tag, 16, HEXDUMP_ATTR_NONE);

    printf("message after decryption msg(plain msg) is:\r\n");
    hexdump(plain_msg, plain_msg_len, HEXDUMP_ATTR_NONE);

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
