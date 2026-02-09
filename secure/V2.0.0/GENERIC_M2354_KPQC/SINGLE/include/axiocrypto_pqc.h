
#ifndef __AXIOCRYPTO_PQC_H
#define __AXIOCRYPTO_PQC_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    PQC_AIMER128F           = 0x1101,
    PQC_HAETAE2             = 0x1201,
    PQC_DILITHIUM2          = 0x1301,
    PQC_SPHINCS128F         = 0x1401,
    PQC_FALCON512           = 0x1501,
    PQC_NTRU768             = 0x8101,
    PQC_SMAUG1              = 0x8201,
    PQC_KYBER512            = 0x8301,

    PQC_FN_DSA_512          = PQC_FALCON512, // FALCON512
    PQC_SLH_DSA_SHAKE128F   = PQC_SPHINCS128F, // SPHINCS128F
    PQC_ML_DSA_44           = PQC_DILITHIUM2, // DILITHIUM2
    PQC_ML_KEM_512          = PQC_KYBER512, // KYBER512
}pqc_algo_t;

#define PQC_SUCCESS            0
#define PQC_ERROR_INVALID_ARG  -1
#define PQC_ERROR_KEYGEN_FAIL  -2
#define PQC_ERROR_SIGN_FAIL    -3
#define PQC_ERROR_VERIFY_FAIL  -4
#define PQC_ERROR_ENCAPS_FAIL  -5
#define PQC_ERROR_DECAPS_FAIL  -6

#define PQC_AIMER128F_PUBLICKEY_SIZE    32
#define PQC_AIMER128F_SECRETKEY_SIZE    48
#define PQC_AIMER128F_SIGNATURE_SIZE    5888

#define PQC_HAETAE2_PUBLICKEY_SIZE      992
#define PQC_HAETAE2_SECRETKEY_SIZE      1408
#define PQC_HAETAE2_SIGNATURE_SIZE      1474

#define PQC_NTRU768_PUBLICKEY_SIZE      1152
#define PQC_NTRU768_SECRETKEY_SIZE      2337
#define PQC_NTRU768_CIPHERTEXT_SIZE     1152
#define PQC_NTRU768_SHAREDSECRET_SIZE   32

#define PQC_SMAUG1_PUBLICKEY_SIZE       672
#define PQC_SMAUG1_SECRETKEY_SIZE       832
#define PQC_SMAUG1_CIPHERTEXT_SIZE      672
#define PQC_SMAUG1_SHAREDSECRET_SIZE    32

#define PQC_KYBER512_PUBLICKEY_SIZE     800
#define PQC_KYBER512_SECRETKEY_SIZE     1632
#define PQC_KYBER512_CIPHERTEXT_SIZE    768
#define PQC_KYBER512_SHAREDSECRET_SIZE  32

#define PQC_DILITHIUM2_PUBLICKEY_SIZE   1312
#define PQC_DILITHIUM2_SECRETKEY_SIZE   2560
#define PQC_DILITHIUM2_SIGNATURE_SIZE   2420

#define PQC_SPHINCS128F_PUBLICKEY_SIZE  32
#define PQC_SPHINCS128F_SECRETKEY_SIZE  64
#define PQC_SPHINCS128F_SIGNATURE_SIZE  17088

#define PQC_FALCON512_PUBLICKEY_SIZE    897
#define PQC_FALCON512_SECRETKEY_SIZE    1281
#define PQC_FALCON512_SIGNATURE_SIZE    690

/**
 * @brief Generate a post-quantum cryptography key pair
 * 
 * @param algo      PQC algorithm type (see pqc_algo_t)
 * @param pk        Buffer to store the public key
 * @param pklen     Size of the public key buffer (must match algorithm requirement)
 * @param sk        Buffer to store the secret key
 * @param sklen     Size of the secret key buffer (must match algorithm requirement)
 * @return int      PQC_SUCCESS on success, error code otherwise
 *                  - PQC_ERROR_INVALID_ARG: Invalid parameters
 *                  - PQC_ERROR_KEYGEN_FAIL: Key generation failed
 */
int axiocrypto_pqc_keypair(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *sk, size_t sklen);

/**
 * @brief Sign a message and produce a detached signature (Digital Signature)
 * 
 * @param algo      PQC algorithm type (must be signature algorithm)
 * @param sk        Secret key for signing
 * @param sklen     Size of the secret key
 * @param m         Message to be signed
 * @param mlen      Length of the message
 * @param sig       Buffer to store the signature
 * @param siglen    Input: buffer size, Output: actual signature size
 * @return int      PQC_SUCCESS on success, error code otherwise
 *                  - PQC_ERROR_INVALID_ARG: Invalid parameters
 *                  - PQC_ERROR_SIGN_FAIL: Signing failed
 */
int axiocrypto_pqc_sign_signature(pqc_algo_t algo, uint8_t *sk, size_t sklen, uint8_t *m, size_t mlen, uint8_t *sig, size_t *siglen);

/**
 * @brief Verify a detached signature
 * 
 * @param algo      PQC algorithm type (must be signature algorithm)
 * @param pk        Public key for verification
 * @param pklen     Size of the public key
 * @param m         Original message
 * @param mlen      Length of the message
 * @param sig       Signature to verify
 * @param siglen    Size of the signature
 * @return int      PQC_SUCCESS if signature is valid, error code otherwise
 *                  - PQC_ERROR_INVALID_ARG: Invalid parameters
 *                  - PQC_ERROR_VERIFY_FAIL: Signature verification failed
 */
int axiocrypto_pqc_verify(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *m, size_t mlen, uint8_t *sig, size_t siglen); 

/**
 * @brief Sign a message and produce signed message (message + signature)
 * 
 * @param algo      PQC algorithm type (must be signature algorithm)
 * @param sk        Secret key for signing
 * @param sklen     Size of the secret key
 * @param m         Message to be signed
 * @param mlen      Length of the message
 * @param sm        Buffer to store signed message (message || signature)
 * @param smlen     Input: buffer size, Output: actual signed message size
 * @return int      PQC_SUCCESS on success, error code otherwise
 *                  - PQC_ERROR_INVALID_ARG: Invalid parameters
 *                  - PQC_ERROR_SIGN_FAIL: Signing failed
 */
int axiocrypto_pqc_sign(pqc_algo_t algo, uint8_t *sk, size_t sklen, uint8_t *m, size_t mlen, uint8_t *sm, size_t *smlen);

/**
 * @brief Open and verify a signed message
 * 
 * @param algo      PQC algorithm type (must be signature algorithm)
 * @param pk        Public key for verification
 * @param pklen     Size of the public key
 * @param sm        Signed message (message || signature)
 * @param smlen     Length of the signed message
 * @param m         Buffer to store the extracted message
 * @param mlen      Input: buffer size, Output: actual message size
 * @return int      PQC_SUCCESS if signature is valid and message extracted, error code otherwise
 *                  - PQC_ERROR_INVALID_ARG: Invalid parameters
 *                  - PQC_ERROR_VERIFY_FAIL: Signature verification failed
 */
int axiocrypto_pqc_open(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *sm, size_t smlen, uint8_t *m, size_t *mlen);

/**
 * @brief Encapsulate a shared secret using KEM (Key Encapsulation Mechanism)
 * 
 * @param algo      PQC algorithm type (must be KEM algorithm like KYBER)
 * @param pk        Public key for encapsulation
 * @param pklen     Size of the public key
 * @param ct        Buffer to store the ciphertext
 * @param ctlen     Input: buffer size, Output: actual ciphertext size
 * @param ss        Buffer to store the shared secret
 * @param sslen     Size of the shared secret buffer
 * @return int      PQC_SUCCESS on success, error code otherwise
 *                  - PQC_ERROR_INVALID_ARG: Invalid parameters
 *                  - PQC_ERROR_ENCAPS_FAIL: Encapsulation failed
 */
int axiocrypto_pqc_encapsulate(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *ct, size_t *ctlen, uint8_t *ss, size_t sslen);

/**
 * @brief Decapsulate a shared secret using KEM
 * 
 * @param algo      PQC algorithm type (must be KEM algorithm like KYBER)
 * @param sk        Secret key for decapsulation
 * @param sklen     Size of the secret key
 * @param ct        Ciphertext to decapsulate
 * @param ctlen     Size of the ciphertext
 * @param ss        Buffer to store the recovered shared secret
 * @param sslen     Input: buffer size, Output: actual shared secret size
 * @return int      PQC_SUCCESS on success, error code otherwise
 *                  - PQC_ERROR_INVALID_ARG: Invalid parameters
 *                  - PQC_ERROR_DECAPS_FAIL: Decapsulation failed
 */
int axiocrypto_pqc_decapsulate(pqc_algo_t algo, uint8_t *sk, size_t sklen, uint8_t *ct, size_t ctlen, uint8_t *ss, size_t *sslen);

#endif // __AXIOCRYPTO_PQC_H