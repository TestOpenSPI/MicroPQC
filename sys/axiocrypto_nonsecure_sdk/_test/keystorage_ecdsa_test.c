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
const uint8_t ecdsa_p256_qx5[] = {0x02,0x3A,0x11,0x92,0xD3,0xCF,0x64,0x20,0x44,0x77,0xA4,0xE2,0x53,0xF8,0x36,0xBB,0xEA,0xA0,0x92,0xEA,0x66,0x57,0x1E,0xD7,0x40,0x92,0x6F,0x4F,0xA8,0x56,0xA1,0xC9};
const uint8_t ecdsa_p256_qy5[] = {0x07,0x33,0xA9,0x23,0x58,0x60,0xA6,0x00,0xC3,0x2D,0x5B,0xF4,0xD0,0xEF,0x39,0x68,0xAE,0xE3,0x9E,0xE7,0x66,0x0D,0x46,0x0C,0x5E,0xF2,0xF3,0x29,0x8D,0x41,0xD4,0xE0};

const uint8_t ECDSAP256Kpg_x1[] = {0x57,0x11,0x9E,0xB1,0x07,0x4A,0x23,0xBA,0x5F,0x2E,0x3B,0x3E,0xEF,0x11,0x74,0x00,0x20,0xE0,0xE8,0x30,0x91,0x1E,0xBB,0x40,0xB3,0xE9,0xA4,0xD3,0xB8,0xB8,0xA0,0x06};

static uint16_t CRC_IMPL(uint16_t sum, const uint8_t *p, const uint32_t len)
{
    uint32_t l = len;
    while (l--) {
        uint8_t byte = *p++;

        for (int i = 0; i < 8; i++) {
            uint16_t osum = sum;

            sum <<= 1;

            if (byte & 0x80)
                sum |= 1;

            if (osum & 0x8000)
                sum ^= 0x1021;  // the polynomial

            byte <<= 1;
        }
    }
    return sum;
}

uint16_t axiocrypto_crc(const uint8_t *p, const uint32_t len)
{
    uint8_t zeroes[] = {0, 0};
    if (NULL == p || 0 == len) {
        return 0;
    }

    return CRC_IMPL(CRC_IMPL(0, p, len), zeroes, 2);
}

int example_axiocrypto_drbg_gcm()
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

int aria_key_storage_example()
{
	CRYPTO_STATUS ret;
	ctx_handle_t handle = {0};		

	handle[0] = 25;

    axiocrypto_init(NULL, 0);

	printf("context slot memory allocate\n\r");
	ret = axiocrypto_allocate_slot(handle, ASYM_ECDSA_P256, CTX_ATTR_PERSISTENT);
	if (ret != CRYPTO_SUCCESS) {
        printf("Fail: axiocrypto_allocate_slot: %d\r\n", ret);
		return ret;
    }

	/*symmetric put Private key algorithm*/
    printf("ECDSA put Private key alrorithm start\n\r");
	ret = axiocrypto_asym_putkey(handle, ASYM_ECDSA_P256,
        ECDSAP256Kpg_x1, 32, axiocrypto_crc(ECDSAP256Kpg_x1, 32), NULL, 0, 0, CTX_ATTR_PERSISTENT);
	if (ret != CRYPTO_SUCCESS) {
        printf("Fail: axiocrypto_asym_putkey: %d\r\n", ret);
		return ret;
	}

	return 0;
}

int example_axiocrypto_ecdsa_key()
{

    CRYPTO_STATUS ret;
    ctx_handle_t handle = {0,};
    ctx_handle_t handle1 = {0,};
    uint8_t msg[32] = {0,};
    uint8_t sig[64] = {0,};
    uint32_t siglen = 64;
    uint8_t Q[64];

    handle[0] = 25;
    handle1[0] = 26;

    axiocrypto_init(NULL, 0);

    // sign
    printf("load keystorage key \r\n");
    ret = axiocrypto_allocate_slot(handle, ASYM_ECDSA_P256, CTX_ATTR_PERSISTENT);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_allocate_slot %d\n\r", ret);
        return 1;
    }

	ret = axiocrypto_asym_sign(handle, msg, sizeof(msg), RAW_MSG, sig, &siglen);
	if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_asym_sign %d\n\r", ret);
        return ret;
    }
    printf("msg is: \n\r");
    hexdump(msg, sizeof(msg), HEXDUMP_ATTR_NONE);

    printf("digital signature is: \n\r");
    hexdump(sig, sizeof(sig), HEXDUMP_ATTR_NONE);

//==============================================================================
    // verify
    ret = axiocrypto_allocate_slot(handle1, ASYM_ECDSA_P256, CTX_ATTR_NONE);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_allocate_slot %d\n\r", ret);
        return 1;
    }

    memcpy(Q, ecdsa_p256_qx5, 32);
	memcpy(&Q[32], ecdsa_p256_qy5, 32);
    /*symmetric put Public key algorithm*/
    printf("ECDSA put Public key alrorithm start\n\r");
	ret = axiocrypto_asym_putkey(handle1, ASYM_ECDSA_P256, NULL, 0, 0, Q, 64, axiocrypto_crc(Q, 64), CTX_ATTR_NONE);
	if (ret != CRYPTO_SUCCESS) {
        printf("Fail: axiocrypto_asym_putkey: %d\r\n", ret);
		return ret;
	}

    ret = axiocrypto_asym_verify(handle1, msg, sizeof(msg), RAW_MSG, sig, siglen);
    if (ret != CRYPTO_SIG_ACCEPT && ret != CRYPTO_SIG_REJECT) {
        printf("FAIL: axiocrypto_asym_verify %d\n\r", ret); 
        return ret;
    }

    printf("Signature verification successful\r\n");

    ret = axiocrypto_free_slot(handle1, ASYM_ECDSA_P256);
    if (ret != CRYPTO_SUCCESS) {
        printf("FAIL: axiocrypto_free_slot %d\n\r", ret);
        return ret;
    }
    printf("success\r\n");

    return 0;
}
