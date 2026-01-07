#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "periph_api.h"
#include "time.h"
#include "hexdump.h"

#include "nsc_def.h"
#include "nscbroker.h"

#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#elif defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#else
#error need time keeping method
#endif
#include "axiocrypto.h"
#define AXIOCRYPTO_MAGIC 0xdcd7504d

CRYPTO_STATUS axiocrypto_init(uint8_t *password, uint32_t sz)
{	
	uint32_t verbose=0;
    void *v[] = {(void *)verbose, (void *)password, (void *)sz};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_INIT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_finish(void)
{
	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_FINISH);
}

CRYPTO_STATUS axiocrypto_allocate_slot(ctx_handle_t handle, ALGORITHM algo, ctx_attr_t attr)
{
	uint32_t p = attr;
    void *v[] = {(void *)handle, (void *)algo, (void *)p};
	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ALLOCATE_SLOT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_free_slot(ctx_handle_t handle, ALGORITHM algo)
{
    void *v[] = {(void *)handle, (void *)algo};
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_FREE_SLOT, AXIOCRYPTO_MAGIC, v);
}


CRYPTO_STATUS axiocrypto_asym_genkey(const ctx_handle_t handle, const ALGORITHM algo, const ctx_attr_t attr)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)attr};

    if (algo == ASYM_ECDSA_P256) {
		return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ASYM_GENKEY, AXIOCRYPTO_MAGIC, v);
        
    } else if (algo == ASYM_ECDH_P256) {
		return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ECDH_GENKEY, AXIOCRYPTO_MAGIC, v);
    } else {
        return CRYPTO_ERR_NOT_SUPPORT_ALGORITHM;
    }
}

CRYPTO_STATUS axiocrypto_ecdsa_genkey(const ctx_handle_t handle, const ALGORITHM algo, const ctx_attr_t attr)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)attr};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ASYM_GENKEY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_ecdh_genkey(const ctx_handle_t handle, const ALGORITHM algo, const ctx_attr_t attr)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)attr};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ECDH_GENKEY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_asym_putkey(const ctx_handle_t handle, const ALGORITHM algo,
                    const uint8_t *d, const uint32_t dsz, uint16_t dcrc,
                    const uint8_t *Q, const uint32_t Qsz, uint16_t Qcrc, const ctx_attr_t attr)
{
    uint32_t _dcrc = dcrc;
    uint32_t _Qcrc = Qcrc;
    void * v[] = {(void *)handle, (void *)algo, (void *)d, (void *)dsz, (void *)_dcrc,
		  (void *)Q, (void *)Qsz, (void *)_Qcrc, (void *)attr};

    if (algo == ASYM_ECDSA_P256) {
		return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ASYM_PUTKEY, AXIOCRYPTO_MAGIC, v);
    } else if (algo == ASYM_ECDH_P256) {
		return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ECDH_PUTKEY, AXIOCRYPTO_MAGIC, v);
    } else {
        return CRYPTO_ERR_NOT_SUPPORT_ALGORITHM;
    }
}

CRYPTO_STATUS axiocrypto_ecdsa_putkey(const ctx_handle_t handle, const ALGORITHM algo,
                    const uint8_t *d, const uint32_t dsz, uint16_t dcrc,
                    const uint8_t *Q, const uint32_t Qsz, uint16_t Qcrc, const ctx_attr_t attr)
{
    uint32_t _dcrc = dcrc;
    uint32_t _Qcrc = Qcrc;
    void * v[] = {(void *)handle, (void *)algo, (void *)d, (void *)dsz, (void *)_dcrc,
		  (void *)Q, (void *)Qsz, (void *)_Qcrc, (void *)attr};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ASYM_PUTKEY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_ecdh_putkey(const ctx_handle_t handle, const ALGORITHM algo,
                    const uint8_t *d, const uint32_t dsz, uint16_t dcrc,
                    const uint8_t *Q, const uint32_t Qsz, uint16_t Qcrc, const ctx_attr_t attr)
{
    uint32_t _dcrc = dcrc;
    uint32_t _Qcrc = Qcrc;
    void * v[] = {(void *)handle, (void *)algo, (void *)d, (void *)dsz, (void *)_dcrc,
		  (void *)Q, (void *)Qsz, (void *)_Qcrc, (void *)attr};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ECDH_PUTKEY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_asym_sign(const ctx_handle_t handle,
                            const uint8_t* inMessage, const uint32_t inMsz, const uint32_t hashedinM,
			    uint8_t *sig, uint32_t *sigsz)
{
    void *v[] = {(void *)handle, (void *)inMessage, (void *)inMsz, (void *)hashedinM,
		 (void *)sig, (void *)sigsz};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ASYM_SIGN, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_asym_verify(
                const ctx_handle_t handle, const uint8_t* inMessage, const uint32_t inMsz,
                const uint32_t hashedinM, const uint8_t *sig, const uint32_t sigsz)
{
    void *v[] = {(void *)handle, (void *)inMessage, (void *)inMsz, (void *)hashedinM,
		(void *)sig, (void *)sigsz};

    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ASYM_VERIFY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_asym_getkey(const ctx_handle_t handle,  ALGORITHM algo, uint8_t *Q, const uint32_t Qsz)
{
    void *v[] = {(void *)handle, (void *)Q, (void *)Qsz};

    if (algo == ASYM_ECDSA_P256) {
		return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ASYM_GETKEY, AXIOCRYPTO_MAGIC, v);
    } else if (algo == ASYM_ECDH_P256) {
		return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ECDH_GETKEY, AXIOCRYPTO_MAGIC, v);
    } else {
        return CRYPTO_ERR_NOT_SUPPORT_ALGORITHM;
    }
}

CRYPTO_STATUS axiocrypto_ecdh_computekey(const ctx_handle_t handle, const uint8_t *KT, const uint32_t KTsz,
		  uint8_t *out, const uint32_t outsz)
{
    void *v[] = {(void *)handle, (void *)KT, (void *)KTsz, (void *)out, (void *)outsz};

    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_ECDH_COMPUTEKEY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_putkey(const ctx_handle_t handle, const uint8_t* key, const uint32_t keysz, const uint16_t crc, const ctx_attr_t attr)
{
    uint32_t _crc = crc;
    void *v[] = {(void *)handle, (void *)key, (void *)keysz, (void *)_crc, (void *)attr};

    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_PUTKEY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_enc_init(const ctx_handle_t handle, const ALGORITHM algo, const SYM_MODE mode,
                                 const uint8_t* iv, const uint32_t ivsz)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)mode,
                 (void *)iv, (void *)ivsz};

    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_ENC_INIT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_enc_update(const ctx_handle_t handle,
                                 const uint8_t*  pt, const uint32_t  ptsz,
                                 uint8_t* ct, uint32_t* ctsz)
{
    void *v[] = {(void *)handle, (void *)pt, (void *)ptsz, (void *)ct, (void *)ctsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_ENC_UPDATE, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_enc_final(const ctx_handle_t handle,
                                 uint8_t* ct, uint32_t* ctsz)
{
    void *v[] = {(void *)handle, (void *)ct, (void *)ctsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_ENC_FINAL, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_dec_init(const ctx_handle_t handle, const ALGORITHM algo, const SYM_MODE mode,
                                 const uint8_t* iv, const uint32_t ivsz)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)mode,
                 (void *)iv, (void *)ivsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_DEC_INIT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_dec_update(const ctx_handle_t handle,
                                 const uint8_t*  ct, const uint32_t  ctsz,
                                 uint8_t* pt, uint32_t* ptsz)
{
    void *v[] = {(void *)handle, (void *)ct, (void *)ctsz, (void *)pt, (void *)ptsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_DEC_UPDATE, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_dec_final(const ctx_handle_t handle,
                                 uint8_t*  pt, uint32_t *ptsz)
{
    void *v[] = {(void *)handle, (void *)pt, (void *)ptsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_DEC_FINAL, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_enc_ECB(ctx_handle_t handle, ALGORITHM algo,
		  uint8_t*  pt, uint32_t  ptsz, uint8_t* ct, uint32_t* ctsz)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)pt, (void *)ptsz, (void *)ct, (void *)ctsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_ENC_ECB, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_dec_ECB(const ctx_handle_t handle, const ALGORITHM algo,
                                     const uint8_t*  ct, const uint32_t  ctsz,
                                     uint8_t* pt, uint32_t* ptsz)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)ct, (void *)ctsz, (void *)pt, (void *)ptsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_DEC_ECB, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_enc_GCM(ctx_handle_t handle, ALGORITHM algo,
                                const uint8_t* pt,  const uint32_t  ptsz,
                                const uint8_t* aad, const uint32_t  aadsz,
                                      uint8_t* tag, const uint32_t  tagsz,
                                const uint8_t* iv,  const uint32_t  ivsz,
                                      uint8_t* ct,        uint32_t* ctsz)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)pt, (void *)ptsz,
                 (void *)aad, (void *)aadsz, (void *)tag, (void *)tagsz,
                 (void *)iv,  (void *)ivsz,
                 (void *)ct, (void *)ctsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_ENC_GCM, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_sym_dec_GCM(const ctx_handle_t handle, const ALGORITHM algo,
                                     const uint8_t* ct,  const uint32_t  ctsz,
                                     const uint8_t* aad, const uint32_t  aadsz,
                                     const uint8_t* tag, const uint32_t  tagsz,
                                     const uint8_t* iv,  const uint32_t  ivsz,
                                           uint8_t* pt,        uint32_t* ptsz)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)ct, (void *)ctsz,
                 (void *)aad, (void *)aadsz, (void *)tag, (void *)tagsz,
                 (void *)iv,  (void *)ivsz,
                 (void *)pt, (void *)ptsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SYM_DEC_GCM, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hash_init(ctx_handle_t handle, const ALGORITHM algo)
{
    void *v[] = {(void *)handle, (void *)algo};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HASH_INIT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hash_update(const ctx_handle_t handle, const uint8_t *in, const uint32_t sz)
{
    void *v[] = {(void *)handle, (void *)in, (void *)sz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HASH_UPDATE, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hash_final(const ctx_handle_t handle, uint8_t* out, const uint32_t sz)
{
    void *v[] = {(void *)handle, (void *)out, (void *)sz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HASH_FINAL, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hash(const ALGORITHM algo, const uint8_t *in, const uint32_t insz, uint8_t *out, const uint32_t outsz)
{
    void *v[] = {(void *)algo, (void *)in, (void *)insz, (void *)out, (void *)outsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HASH, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hash_clone(const ctx_handle_t src_handle, ctx_handle_t dst_handle)
{
    void *v[] = {(void *)src_handle, (void *)dst_handle};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HASH_CLONE, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hmac_putkey(ctx_handle_t handle, const ALGORITHM algo, const uint8_t *key, const uint32_t keysz, uint16_t crc)
{
    uint32_t _crc = crc;
    void *v[] = {(void *)handle, (void *)algo, (void *)key, (void *)keysz, (void *)_crc};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HMAC_PUTKEY, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hmac_init(const ctx_handle_t handle)
{
    void *v[] = {(void *)handle};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HMAC_INIT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hmac_update(const ctx_handle_t handle, const uint8_t * in, const uint32_t  insz)
{
    void *v[] = {(void *)handle, (void *)in, (void *)insz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HMAC_UPDATE, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hmac_final(const ctx_handle_t handle, uint8_t *out, const uint32_t outsz)
{
    void *v[] = {(void *)handle, (void *)out, (void *)outsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HMAC_FINAL, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_hmac(const ALGORITHM algo, const uint8_t *key, const uint32_t keysz,
     const uint8_t *in, const uint32_t insz, uint8_t *out, const uint32_t outsz)
{
    void *v[] = {(void *)algo, (void *)key, (void *)keysz, (void *)in, (void *)insz, (void *)out, (void *)outsz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_HMAC, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_random(uint8_t *out,  const uint32_t outsz)
{
    void *v[] = {(void *)outsz, (void *)out};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_RANDOM, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_info( char *versionstr, uint32_t versionstrlen, operation_mode_t *opmode )
{
    void *v[] = {(void *)versionstr, (void *)versionstrlen, (void *)opmode};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_INFO, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_set_mode(operation_mode_t opmode)
{
    uint32_t m = (uint32_t)opmode;
    void *v[] = {(void *)m};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SET_MODE, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_clear_all(void)
{
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_CLEAR_ALL, AXIOCRYPTO_MAGIC);
}

CRYPTO_STATUS axiocrypto_set_entity_info(uint8_t *entityinfo)
{
    void *v[] =  {(void *)entityinfo};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SET_ENTITY_INFO, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_pbkdf(uint8_t *pw, uint32_t pwsz, uint8_t *salt, uint32_t saltsz, uint32_t iter, uint8_t *key, uint32_t keysz)
{
    void *v[] = {pw, (void *)pwsz, salt, (void *)saltsz, (void *)iter, key, (void *)keysz};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_PBKDF, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_get_slotinfo(const ctx_handle_t handle, const ALGORITHM algo, uint16_t *info)
{
    void *v[] = {(void *)handle, (void *)algo, (void *)info};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_GET_SLOTINFO, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_trng_random(unsigned char *output, uint32_t len, uint32_t *olen)
{
    void *v[] = {(void *)output, (void *)len, (void *)olen};
    
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_TRNG_RANDOM, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_self_test(void)
{
    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SELF_TEST, AXIOCRYPTO_MAGIC);
}

CRYPTO_STATUS axiocrypto_get_module_status(void)
{
	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_GET_MODULE_STATUS, AXIOCRYPTO_MAGIC);
}
#if defined(AXIOCRYPTO_FAULT_INDUCTION)
CRYPTO_STATUS axiocrypto_set_error(int e, int save)
{
    void *v[2] = {(void *)e, (void *)save};

    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SET_ERROR, AXIOCRYPTO_MAGIC, v);
}
CRYPTO_STATUS axiocrypto_show_keystorage(void)
{
	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_SHOW_KEYSTORAGE, AXIOCRYPTO_MAGIC);
}
CRYPTO_STATUS axiocrypto_drbg(uint32_t reqsz, uint8_t* nonce, uint32_t noncesz, uint8_t* perStr, uint32_t perStrsz, uint8_t* eInput, uint32_t eInputsz, uint8_t* addReseed, uint32_t addReseedsz, uint8_t* eInputReseed, uint32_t eInputReseedsz,  uint8_t* addInput, uint32_t addInputsz, uint8_t* out, uint32_t outsz)
{
    void *v[15] = { (void *) reqsz,
                    nonce, (void *) noncesz,
                    perStr, (void *) perStrsz,
                    eInput, (void *) eInputsz,
                    addReseed, (void *) addReseedsz,
                    eInputReseed, (void *) eInputReseedsz,
                    addInput, (void *) addInputsz,
                    out, (void *) outsz};
	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_DRBG, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_drbg_set_context(ALGORITHM mode, uint32_t prMode, uint32_t updatePeriod[2])
{
    uint32_t v0 = (uint32_t)mode;
    void *v[3] = {(void *)v0, (void *)prMode, updatePeriod};

    return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_DRBG_SET_CONTEXT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_drbg_init(uint8_t* entropyInput, uint32_t entropyInputBitSize,
		  uint8_t* nonce, uint32_t nonceBitSize, uint8_t* str, uint32_t strBitSize)
{
    void *v[6] = {entropyInput, (void *)entropyInputBitSize,
		  nonce, (void *)nonceBitSize,
		  str, (void *)strBitSize};

return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_DRBG_INIT, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_drbg_update(uint8_t* entropyInput, uint32_t entropyInputBitSize,
		  uint8_t* addInput, uint32_t addInputBitSize)
{
    void *v[4] = {entropyInput, (void *)entropyInputBitSize,
		  addInput, (void *)addInputBitSize};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_DRBG_UPDATE, AXIOCRYPTO_MAGIC, v);
}

CRYPTO_STATUS axiocrypto_drbg_final(uint32_t reqBitSize,
                                    uint8_t* addReseed, uint32_t addReseedBitSize,
                                    uint8_t* eInputReseed, uint32_t eInputReseedBitSize,
                                    uint8_t* addInput, uint32_t addInputBitSize,
                                    uint8_t* out, uint32_t outBitSize)
{
    void *v[9] = {(void *)reqBitSize,
                  addReseed, (void *) addReseedBitSize,
                  eInputReseed, (void *) eInputReseedBitSize,
                  addInput, (void *) addInputBitSize,
                  out, (void *) outBitSize};

	return (CRYPTO_STATUS)NSCBROKER_RUN(NSC_AXIOCRYPTO_DRBG_FINAL, AXIOCRYPTO_MAGIC, v);
}
#else
CRYPTO_STATUS axiocrypto_set_error(int e, int save)
{
    (void)e;
    (void)save;
    printf("NOT implemented\n");
    return 0;
}
CRYPTO_STATUS axiocrypto_show_keystorage(void)
{
    printf("NOT implemented\n");
    return 0;
}
#endif

