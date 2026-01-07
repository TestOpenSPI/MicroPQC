#pragma once
/**
 * @file axiocrypto.h
 *
 * @brief API functions in AxioCrypto
 */

#include <stdint.h>


/**
 * @~korean     @brief 알고리즘을 나타내는 상수
 * @~english    @brief enum types for constants representing algorithms
 */
typedef enum crypto_algorithm{
        CRYPTO_ALGO_INITIALIZED = 0x00,         /**< Initialized Algorithm :0 */
        ASYM_ECDSA_P256         = 0x02,         /**< ECDSA P-256 */
        ASYM_ECDSA_SM2          = 0x03,         /**< SM2 */
        HASH_SHA_256            = 0x05,         /**< SHA-256 */
        DRBG_HASH_SHA256        = 0x06,         /**< DRBG */
        ASYM_ECDH_P256          = 0x07,         /**< ECDH P-256*/
        HMAC_SHA_256            = 0x08,         /**< HMAC_SHA-256 */
        HASH_SM3                = 0x09,         /**< SM3 */
        KDF_PBKDF2              = 0x0A,
        KDF_HKDF                = 0x0B,
        KDF_SM2KDF2             = 0x0C,
        ASYM_ECDH_SM2           = 0x0D,
        SYM_ARIA                = 0x10,         /**< ARIA Block Cipher */
        SYM_LEA                 = 0x20,         /**< LEA Block Cipher */
        SYM_AES                 = 0x30,         /**< AES Block Cipher */
        SYM_SM4                 = 0x40,         /**< SM4 Block Cipher */
        SYM_MASK                = 0xf0,
} ALGORITHM;

/**
 * @~korean     @brief 블록 암호 알고리즘 운영 모드(CBC, CTR, GCM)를 나타내는 상수
 * @~english    @brief enum type for block cipher mode of operation
 */
typedef enum symmetric_encryption_operation_mode{
        SYM_MODE_INITIALIZED	= 0,            /**< Initialized Block operation : -1 */
        SYM_MODE_CBC            = 0x01,         /**< Block operation : CBC */
        SYM_MODE_CTR            = 0x02,         /**< Block operation : CTR */
        SYM_MODE_GCM            = 0x03,         /**< Block operation : GCM */
        SYM_MODE_ECB            = 0x04,         /**< Block operation : ECB: ony for AES, SM4 */
        SYM_MODE_MAX            = 0x05,
} SYM_MODE;

/**
 * @~korean     @brief 컨텍스트 슬롯의 아이디로 사용할 데이터 타입.
 * @~english    @brief data type to be used as the id of context slot
 */
typedef uint8_t ctx_handle_t[32];

/**
 * @brief entity id의 데이터 타입
 */
typedef uint8_t entity_info_t[32];

/**
 * @~korean     @brief 파라미터의 속성에 관한 상수
 * @~english    @brief enum type for error status value.
 */
typedef enum {
    CTX_ATTR_NONE        = 0,
    CTX_ATTR_RDONLY      = 0x01,
    CTX_ATTR_PERSISTENT  = 0x02,
} ctx_attr_t;

typedef enum {
    SLOTINFO_ATTR_KEY_LEN    = 0x07FF,
    SLOTINFO_ATTR_DATA_EXIST = 0x0800,
    SLOTINFO_ATTR_PRIV_EXIST = 0x1000,
    SLOTINFO_ATTR_PUBK_EXIST = 0x2000,
    SLOTINFO_ATTR_READ_ONLY  = 0x4000,
    SLOTINFO_ATTR_PERSISTENT = 0x8000,
} slotinfo_attr_t;

/**
 * @~korean     @brief <code>axiocrypto_asym_sign()</code>, <code>axiocrypto_asym_verify()</code>에서 <code>msghashed</code>에 사용할 상수
 * @~english    @brief enum type for error status value.
 */
enum msg_type_in_ecdsa {
    HASHED_MSG = 1,
    RAW_MSG = 0,
};

/**
 * @~korean     @brief sm2 key exchange protocol에서 사용할 상수
 */
enum type_in_sm2kep {
        INITIATOR_MODE = 0,
        RESPONDER_MODE = 1,
        NO_CHECHSUM_KEP = 0,
        CHECKSUM_KEP    = 1,
};

/**
 * @~korean     @brief 암호모듈의 동작모드에 관한 상수 정의
 * @~english    @brief enum type for representing operation mode.
 */
typedef enum {
    OP_MODE_NON_APPROVED=0,
    OP_MODE_APPROVED_FIPS1402=1,
    OP_MODE_APPROVED_KCMVP=2,
    OP_MODE_APPROVED=3,
    OP_MODE_APPROVED_OSCCA=4,
    OP_MODE_NOTHING=0xff,
} operation_mode_t;

typedef enum module_status {
    MODULE_POWER_ONOFF                          = 0,
    MODULE_CRYPTO_INIT                          = 1,
    MODULE_PRE_OPERATIONAL_SELFTEST             = 3,
    MODULE_CONDITIONAL_SELFTEST                 = 4,
    MODULE_APPROVED_MODE                        = 5,
    MODULE_NON_APPROVED_MODE                    = 6,
    MODULE_MODE_CHANGE                          = 7,
    MODULE_MODE_CRITICAL_ERROR                  = 8,
    MODULE_NOTHING                              = 0xFF,
} module_status_t;

/**
  * @~korean @brief 이 모듈에서 상태를 나타내기 위해 사용하는 상수
  * @~english @brief constants showing the module's status
  */
typedef enum crypto_status {
    CRYPTO_SUCCESS				    = 0,    /**< @~korean 암호 모듈 동작 성공 @~english */
    CRYPTO_OK				            = 0,    /**< @~korean 암호 모듈 동작 성공 @~english */
    CRYPTO_ERROR				    = -1,   /**< @~korean 암호 모듈 동작 실패 @~english */
    CRYPTO_ERR_NOT_INITIALIZED			    = -2,   /**< @~korean 초기화되지 않음. @~english */

    // SIGNATURE STATUS CODE
    CRYPTO_SIG_ACCEPT				    = 0,    /**< @~korean 서명 검증 성공 @~english */
    CRYPTO_SIG_REJECT				    = -10,  /**< @~korean 서명 검증 결과 서명 거절 @~english */

    // CBC, GCM AUTH. STATUS CODE
    CRYPTO_GCM_ACCEPT				    = 0,    /**< @~korean GCM 모드 인증 성공 @~english */
    CRYPTO_GCM_REJECT				    = -8,   /**< @~korean GCM 모드 인증 실패 @~english */

    CRYPTO_ERR_PWD_POLICY_VIOLATION                 = -16,
    CRYPTO_ERR_TOO_SHORT_LENGTH                     = -17,

    CRYPTO_ERR_INVALID_PARAM			    = -20,  /* invalid parameter */
    CRYPTO_ERR_BUFFER_NULL			    = -21,  /* buffer is null */
    CRYPTO_ERR_INVALID_LENGTH			    = -22,  /* invalid parameter length */
    CRYPTO_ERR_ALREADY_INIT			    = -25,  /* invalid parameter */
    CRYPTO_ERR_SLOT_FULL			    = -28,  /* slot full */
    CRYPTO_ERR_HANDLE_INVALID_VALUE		    = -31,  /* the value of given handle is invalid */
    CRYPTO_ERR_NO_SUCH_HANDLE                       = -32,  /* the value of given handle is invalid */

    CRYPTO_ERR_KEY_RDONLY			    = -54,  /* Key is not-erasable */
    CRYPTO_ERR_SYM_NOKEY			    = -55,  /* Symmetric key doesn't exist */
    CRYPTO_ERR_SIG_NOKEY			    = -56,  /* Signature key doesn't exist */
    CRYPTO_ERR_KEX_NOKEY			    = -57,  /* Key Exchange key doesn't exist */
    CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE         = -60,  /* op. not allowd in approved mode */
    CRYPTO_ERR_KEY_STORAGE_ALGORITHM_NO_SUPPORT     = -64,
    CRYPTO_ERR_NOT_SUPPORT_ALGORITHM                = -66,

    CRYPTO_ERR_SIGNATURE_INVALID		    = -110, /**<  Signature Sturct Invalid */
    CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM	    = -113, /**<  @~korean 지원하지 않는 비대칭키 알고리즘 선택 에러 @~english*/
    CRYPTO_ERR_SIG_PUBLICKEY_INVALID		    = -114, /**<  Asymmetric Public Key Invalid */
    CRYPTO_ERR_SIG_PRIVATEKEY_INVALID		    = -115, /**<  Asymmetric Private Key Invalid */
    CRYPTO_ERR_SIG_KEY_EMPTY			    = -118, /**<  Private or Public Key not set */

    CRYPTO_ERR_SIG_MESSAGE_INVALID		    = -122, /**< @~korean 사용할 수 없는 서명 메시지  @~english*/
    CRYPTO_ERR_SIG_CTX_HANDLE_INVALID		    = -123, /**< @~korean 유효하지 않은 Context ID  @~english*/
    CRYPTO_ERR_ECC_PUBLICKEY_VALIDATION_PKEY_IS_NOT_VALID = -125, /**< ECC 공개키 검증 중 사용할 수 없는 PKEY 발견 */

    // ASYM (ECDH) ERR CODE
    CRYPTO_ERR_KEY_EXCHANGE_NOT_SUPPORT_ALGORITHM   = -126, /**< @~korean 지원하지 않는 비대칭 키 교환 알고리즘 선택 에러 @~english*/
    CRYPTO_ERR_KEY_EXCHANGE_PRIVATEKEY_INVALID	    = -128, /**<  Asymmetric Private Key Invalid */
    CRYPTO_ERR_KEY_EXCHANGE_PUBLICKEY_INVALID	    = -129, /**<  Asymmetric Public Key Invalid */
    CRYPTO_ERR_KEY_EXCHANGE_CTX_HANDLE_INVALID	    = -130, /**< @~korean 유효하지 않은 Context ID  @~english*/
    CRYPTO_ERR_KEY_EXCHANGE_BUFFER_SIZE_INVALID	    = -131, /**< @~korean 버퍼 크기 오류  @~english*/
    CRYPTO_ERR_KEY_EXCHANGE_BUFFER_NULL 	    = -132, /**< @~korean 버퍼 오류  @~english*/

    /**< SYM(ARIA / LEA) ERR CODE */
    CRYPTO_ERR_SYM_NOT_SUPPORT_MODE		    = -200, /**< @~korean 지원하지 않는 대칭키의 운영 모드 선택 에러 @~english*/
    CRYPTO_ERR_SYM_NOT_SUPPORT_ALGORITHM	    = -201, /**< @~korean 지원하지 않는 대칭키 알고리즘 선택 에러 @~english*/
    CRYPTO_ERR_SYM_KEY_INVALID			    = -202, /**< @~korean 유효하지 않은 대칭키  @~english*/
    CRYPTO_ERR_SYM_NOT_SUPPORT_SECURITY_LEVEL	    = -203, /**< @~korean 지원하지 않는 안전성  @~english*/
    CRYPTO_ERR_SYM_KEYGEN			    = -204, /**< @~korean 라운드키 생성 오류  @~english*/
    CRYPTO_ERR_SYM_IV_INVALID			    = -205, /**< @~korean 초기벡터 설정 오류  @~english*/
    CRYPTO_ERR_SYM_CONTEXT_INVALID                  = -206,
    CRYPTO_ERR_SYM_IV_SIZE_INVALID		    = -207, /**< @~korean 유효하지 않은 초기벡터 길이  @~english*/
    CRYPTO_ERR_SYM_PT_SIZE_INVALID		    = -209, /**< @~korean 평문의 길이가 8의 배수가 아닌 경우  @~english*/
    CRYPTO_ERR_SYM_CT_SIZE_INVALID		    = -210, /**< @~korean 암호문의 길이가 8의 배수가 아닌 경우  @~english*/
    CRYPTO_ERR_SYM_TAG_SIZE_INVALID		    = -216, /**< @~korean Tag의 길이가 유효하지 않은 경우  @~english*/
    CRYPTO_ERR_SYM_CTX_HANDLE_INVALID		    = -220, /**< @~korean 유효하지 않은 Context ID  @~english*/
    CRYPTO_ERR_SYM_PT_NULL		            = -221, /**< @~korean 평문의 길이가 8의 배수가 아닌 경우  @~english*/
    CRYPTO_ERR_SYM_CT_NULL		            = -222, /**< @~korean 암호문의 길이가 8의 배수가 아닌 경우  @~english*/
    CRYPTO_ERR_SYM_AAD_INVALID                      = -227,
    CRYPTO_ERR_SYM_TAG_INVALID                      = -228,

    CRYPTO_ERR_HASH_CTX_HANDLE_INVALID		    = -302, /**<  SHA CTX id Invalid */
    CRYPTO_ERR_HASH_NOT_SUPPORT_ALGORITHM	    = -303, /**< @~korean 지원하지 않는 HASH 알고리즘 요청 에러  @~english*/
    CRYPTO_ERR_HASH_CTX_NOT_INIT		    = -304, /**<  try to use HASH without Init() */
    CRYPTO_ERR_HASH_IN_PARAMETER_INVALID	    = -305, /**< @~korean HASH에서 지원하지 않는 입력 파라미터 선택 에러  @~english*/
    CRYPTO_ERR_HASH_OUT_PARAMETER_INVALID	    = -306, /**< @~korean HASH에서 지원하지 않는 출력 파라미터 선택 에러  @~english*/
    CRYPTO_ERR_HASH_CTX_IN_USE			    = -307, /**<  SHA CTX in use */

    CRYPTO_ERR_HMAC_CTX_HANDLE_INVALID		    = -352, /**<  HMAC CTX id Invalid */
    CRYPTO_ERR_HMAC_NOT_SUPPORT_ALGORITHM	    = -353, /**< @~korean 지원하지 않는 알고리즘 요청 에러  @~english*/
    CRYPTO_ERR_HMAC_CTX_NOT_INIT		    = -354, /**<  try to use HMAC without Init() */

    // DRBG ERR CODE
    CRYPTO_ERR_DRBG_NOT_SUPPORT_ALGORITHM           = -402, /**<  지원하지 않는 DRBG 알고리즘 선택 에러*/
    CRYPTO_ERR_DRBG_NOT_SUPPORT_PR_MODE             = -404, /**< DRBG가 지원하지 않는 PR_MODE 선택 에러 */
    CRYPTO_ERR_DRBG_UPDATE_ENTROPY_INVALID          = -406, /**< 지원하지 않는 DRBG Entropy 갱신 요청 에러 */
    CRYPTO_ERR_DRBG_INIT_PARAMETER_INVALID          = -407, /**< 예측내성이 비활성화 되어, 추가로 필요한 >파라미터가 부족한 경우 */
    CRYPTO_ERR_DRBG_MAXIMUM_REQUEST_BIT_SIZE_OVER   = -408, /**< DRBG가 생성할 수 있는 비트 수를 초과한 경우 */
    CRYPTO_ERR_DRBG_OVER_MAXIMUM_RESEED_INTERVAL    = -409, /**< DRBG에 설정할 수 있는 reseed interval를 >초과한 경우 */
    CRYPTO_ERR_DRBG_REQUEST_BIT_SIZE_INVALID        = -411, /**< DRBG에 요청한 Request Bit가 적절하지 않은 경우(예, Request비트가 0인 경우) */
    CRYPTO_ERR_DRBG_OUT_PARAMETER_INVALID           = -412, /**< DRBG가 생성한 랜덤한 수가 저장되어야 하는 변수와 변수비트사이즈가 올바르지 않은 경우(예, 변수가 NULL이거나 변수비트사이즈가 0인경우) */
    CRYPTO_ERR_DRBG_HEALTHTEST_FAIL                 = -413, /**< DRBG test fail */
    CRYPTO_ERR_TRNG_HEALTHTEST_FAIL                 = -490, /**<  @~korean TRNG health test 실패 */

    CRYPTO_ERR_PBKDF_SHORT_LENGTH                   = -801, /**< iter count or salt length is too short */
} CRYPTO_STATUS;

/**
 * @defgroup M_GENERAL_API 일반 함수
 * @{
 */

/**
 * @~korean @brief 주어진 알고리즘에서 사용할 키슬롯을 할당
 * @~korean @param handle @c ctx_handle_t 의 버퍼.
 * @~korean @param algo [in]  사용할 알고리즘.
 *                            이 값에 따라 어느 그룹을 사용할지 결정함<br>
 *                            SYM_ARIA/SYM_LEA/SYM_AES, ASYM_ECDSA_P256, ASYM_ECDH_P256
 * @~korean @param persistent 1 또는 0, 1이면 플래시메모리에 저장할 키슬롯을 할당함.
 * @~korean @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 동작 완료.
 *      @li <code>CRYPTO_ERR_BUFFER_NULL:</code> phandle이 NULL임
 *      @li <code>CRYPTO_ERR_SLOT_FULL:</code> 할당할 슬롯이 없음.
 * @~korean @details 주어진 @c handle 이 이미 있으면 새로 할당하지 않고 @c CRYPTO_SUCCESS 를 리턴함.
 *                   새로 할당해야 한다면 기존에 존재하는 handle과 겹치지 않도록 만드는 것은
 *                   응용프로그램 제작자의 몫으로 남김.
 *
 * @~english @brief  This function allocates a slot for given <code>algo</code><br>
 *                   and returns random number used as handle to this slot.
 * @~english @param  handle     [in] buffer with handle value.
 * @~english @param  algo       [in] algorithm to use the allocated slot with.
 * @~english @param  persistent [in] if this value is non-zero, this function allocates a slot to be stored in flash.
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS:</code> OK 
 *      @li <code>CRYPTO_ERR_BUFFER_NULL:</code> <code>phandle</code> is NULL
 *      @li <code>CRYPTO_ERR_SLOT_FULL:</code> no slot is free.
 */
CRYPTO_STATUS axiocrypto_allocate_slot(ctx_handle_t handle, ALGORITHM algo, ctx_attr_t attr);

/**
 * @~korean @brief   슬롯을 모두 사용하고 나서 반환할 때 사용할 함수.<br>
 *                   <code>handle</code>값과 일치하는 핸들이 있는 슬롯이 있으면
 *                   그 내용을 삭제하고 비어 있는 것으로 표시함.<br>
 *
 * <b>세부 사항</b> 사용 후 이 함수를 실행하지 않으면 슬롯을 해제하지 않으므로
 *                   더 이상 사용하지 않는 슬롯을 파기하지 않고 할당을 계속하면 
 *                   나중에는 <code>axiocrypto_allocate_slot()</code>이
 *                   <code>CRYPTO_ERR_SLOT_FULL</code>을 리턴하는데, 이 때 정상적인
 *                   <code>handle</code>이 알지 못한다면
 *                   <code>axiocrypto_clear_all()</code>을 사용하여
 *                   전체 키 저장공간을 모두 지워야 새로운 키를 저장할 수 있음.
 *
 * @~korean @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 동작 수행
 *      @li <code>CRYPTO_ERR_SLOT_NOTHING</code>: 주어진 <code>handle</code>에 해당하는 키가 없음.
 * @~korean @param  handle [in] 파기할 슬롯의 @c ctx_handle_t
 * @~korean @param  algo   [in] 파기할 슬롯에 지정된 알고리즘.
 *
 * @~english @brief   This function is used to free the allocated slot after use.
 *                    This searches for slot with given <code>handle</code>, 
 *                    deletes the contents and marks the slot as being blank.
 * @~english @details If user doesn't execute this function when the key is no use
 *                    any more, finally <code>axiocrypto_allocate_slot()</code>
 *                    will return <code>CRYPTO_ERR_SLOT_FULL</code>.
 *                    In this case, if the user doesn't know the handle for existing slots, he/she can use <code>axiocrypto_clear_all()</code> to erase all the keys stored in this module.
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: OK
 *      @li <code>CRYPTO_ERR_SLOT_NOTHING</code>: there is no key slot with given <code>handle</code>
 * @~english @param  handle [in] the <code>ctx_handle_t</code> value of the slot to free
 * @~english @param  algo   [in] the algorithm bind to the slot.
 */
CRYPTO_STATUS axiocrypto_free_slot(ctx_handle_t handle, ALGORITHM algo);

/**
 * @}
 */

/**
 * @defgroup M_ASYM_API 공개키 알고리즘
 * @{
 */

/**
 * @~korean  @brief  ECC 키를 생성함
 *
 * <b>세부 사항 </b>
 *      @li 오류상태와 인증모드 우선 검사.
 *      @li 검증 대상 난수생성기를 이용하여 개인키를 생성하고, 이에 대응하는 공개키를 계산함.
 *      @li 생성 후 키페어가 적절한지 테스트하고 나서 지정된 <code>handle</code>와 연결된 슬롯에 키를 저장하고 리턴함.
 *
 * @~korean  @param  handle [in] 키를 생성한 다음 저장할 슬롯의 핸들
 * @~korean  @param  algo   [in] 생성한 키와 함께 사용할 알고리즘.
 * @~korean  @param  attr   [in] <code>CTX_ATTR_RDONLY</code>: 이면 키를 저장하고 나서 읽기 전용으로 설정함. 0이면 동작에 영향 없음.
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 종료.
 *      @li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle 
 *	@li <code>CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM</code>: 유효하지 않은 알고리즘. 
 *      @li <code>CRYPTO_ERR_KEY_RDONLY:</code> 주어진 키 슬롯이 읽기 전용.
 * @~english @brief  This function is used to generate ECC key in given key handle.
 * @~english @param  handle [in] key handle in slot to store generated key in
 * @~english @param  algo   [in] algorithm to use with. ASYM_ECDSA_P256 or ASYM_ECDH_P256.
 * @~english @param  attr   [in] if <code>CTX_ATTR_RDONLY</code> the key is configured as read-only and not overwritten. if 0, no effect.
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: function returned OK.
 *      @li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: <code>handle</code> is invalid.
 *	@li <code>CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM</code>: <code>algo</code> is invalid.
 *      @li <code>CRYPTO_ERR_KEY_RDONLY</code>: given handle is already read-only.
 */
CRYPTO_STATUS axiocrypto_asym_genkey(
    /* Inputs  */ const ctx_handle_t handle, const ALGORITHM algo, const ctx_attr_t attr);

/**
 * @~korean  @brief  ECC 개인키, 공개키 키 쌍을 지정함.
 *
 * <b>세부 사항</b>
 *      @li 오류상태와 인증모드 우선 검사.
 *      @li 슬롯을 찾아서 키를 기록하고 플래시메모리에 저장함.
 *
 * @~korean  @param  handle [in] 키를 저장할 슬롯의 핸들
 * @~korean  @param  algo   [in] 생성한 키와 함께 사용할 알고리즘.
 * @~korean  @param  d      [in] 개인키가 저장된 주소. NULL 이면 개인키를 저장하지 않음.
 * @~korean  @param  dsz    [in] 개인키의 길이. 32
 * @~korean  @param  dcrc   [in] 개인키의 crc-16 xmodem 계산값.
 * @~korean  @param  Q      [in] 공개키의 값이 저장된 주소. x좌표와 y좌표를 연달아 붙여 저장함. NULL이면 공개키를 저장하지 않음.
 * @~korean  @param  Qsz    [in] 공개키버퍼의 길이, 64
 * @~korean  @param  Qcrc   [in] 공개키의 crc-16 xmodem 계산값.
 * @~korean  @param  attr   [in] <code>CTX_ATTR_RDONLY</code>이면 키를 저장하고 나서 읽기 전용으로 설정하여 덮어쓸 수 없도록 함. 0이면 동작에 영향 없음.
 *
 * @~korean  @return
 *      @li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *	@li <code>CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM</code>: 유효하지 않은 알고리즘.
 *	@li <code>CRYPTO_ERR_SIG_PRIVATEKEY_INVALID</code>: 유효하지 않은 개인키
 *	@li <code>CRYPTO_ERR_INVALID_LENGTH </code>: 주어진 개인키 또는 공개키 길이의 오류
 *
 * @~english @brief  This function is used to provide ECC key to given key slot.
 * @~english @param  handle [in] key handle in slot to generate and store ECDSA/ECDH key.
 * @~english @param  algo   [in] algorithm to use with. ASYM_ECDSA_P256 or ASYM_ECDH_P256.
 * @~english @param  d      [in] the address of private key. if NULL, the module doesn't store private key.
 * @~english @param  dsz    [in] the length of <code>d</code>
 * @~english @param  dcrc   [in]  CRC16 / XMODEM value of <code>d</code>
 * @~english @param  Q      [in] the address of public key. x-cordinate and y-cordinate must be concatenated. if NULL, the module doesn't store public key.
 * @~english @param  Qsz    [in] the length of public key.
 * @~english @param  Qcrc   [in]  CRC16 / XMODEM value of <code>Q</code>
 * @~english @param  attr   [in] if <code>CTX_ATTR_RDONLY</code> the key is configured as read-only and not overwritten. if 0, no effect.
 * @~english @return
 *      @li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid.
 *	@li <code>CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM</code>: <code>algo</code> is not valid.
 *	@li <code>CRYPTO_ERR_SIG_PRIVATEKEY_INVALID</code>: d is not valid
 *	@li <code>CRYPTO_ERR_INVALID_LENGTH </code>: dsz or Qsz is not valid.
 *	@li <code>CRYPTO_ERR_ECC_PUBLICKEY_VALIDATION_PKEY_IS_NOT_VALID</code>: public key in Q is not valid.
 */
CRYPTO_STATUS axiocrypto_asym_putkey(
    /* Inputs  */ const ctx_handle_t handle, const ALGORITHM algo,
                  const uint8_t *d, const uint32_t dsz, uint16_t dcrc,
		  const uint8_t *Q, const uint32_t Qsz, uint16_t Qcrc, const ctx_attr_t attr);

/**
 * @~korean  @brief  메세지에 대해 서명연산을 진행
 *
 * <b>세부 사항</b>
 *  @li 오류상태와 인증모드 우선 검사.
 *  @li <code>axiocrypto_asym_putkey</code>에서 지정한 알고리즘에 따라 동작이 변경됨.
 *  @li 알고리즘이 <code>ASYM_ECDSA_P256</code>면 <code>ECDSA_Sign</code>을 호출함.
 *      <code>ECDSA_Sign</code>의 동작:
 *      -# 입력 메시지에 대해 HASH를 진행한다.
 *      -# 랜덤한 k를 선택한다(HASH기반 DRBG 이용)
 *      -# r = px mod n를 연산한다.
 *      -# s = k^-1(e + dr) mod n를 연산한다.
 *      -# 연산한 (r, s)를 ECC_SIG 구조체에 복사한다.
 *
 * @~korean  @param  handle      [in]  서명연산에 사용할 키가 저장된 슬롯의 핸들
 * @~korean  @param  msg         [in]  메세지가 저장된 주소
 * @~korean  @param  msgsz       [in]  메세지의 크기 <code>msghashed</code>이 0이면 > 0, 아니면 32
 * @~korean  @param  msghashed   [in]  메세지가 해시연산 결과임.
 * @~korean  @param  sig         [out] 서명을 저장할 주소. ECDSA의 경우, r과 s를 붙여서 저장함.
 * @~korean  @param  sigsz       [out] 서명의 크기. p256r1 커브의 경우 32 * 2 = 64
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상적으로 동작 수행.
 *	@li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *	@li <code>CRYPTO_ERR_SIGNATURE_INVALID</code>: 유효하지 않은 서명의 버퍼
 *	@li <code>CRYPTO_ERR_SIG_MESSAGE_INVALID</code>: 유효하지 않은 메시지 버퍼
 *	@li <code>CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM</code>: 지원하지 않는 알고리즘이 설정된 컨텍스트
 *	@li <code>CRYPTO_ERR_SIG_KEY_EMPTY</code>: 선택한 슬롯에 개인키가 설정되어 있지 않음. 
 * @~english @brief  This function is used to execute signing on given message.
 * @~english @param  handle    [in] key handle in slot with key to sign with.
 * @~english @param  msg [in] the address of message
 * @~english @param  msgsz     [in] the length of <code>msg</code>
 * @~english @param  msghashed [in] if 0, the module calc hash value before signing. if 1 the module skip hash calc.
 * @~english @param  sig       [out] the address to store signature. r and s values are stored in concatented form.
 * @~english @param  sigsz     [out] the length of signature. in p256r1 curve, it's 64 Bytes.
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: function returned OK.
 *	@li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid.
 *	@li <code>CRYPTO_ERR_SIGNATURE_INVALID</code>: the address of <code>sig</code> buffer is not valid.
 *	@li <code>CRYPTO_ERR_SIG_MESSAGE_INVALID</code>: the address of <code>msg</code> is not valid.
 *	@li <code>CRYPTO_ERR_SIG_KEY_EMPTY</code>: private key is not stored in <code>handle</code>
 */
CRYPTO_STATUS axiocrypto_asym_sign(
    /* Inputs  */ const ctx_handle_t handle,
                  const uint8_t* msg, const uint32_t msgsz, const uint32_t msghashed,
    /* Outputs */ uint8_t *sig, uint32_t *sigsz);

/**
 * @~korean  @brief  주어진 메세지와 서명에 대해 일치 여부를 검증함
 *
 * <b>세부 사항</b>
 *      @li 오류상태와 인증모드 우선 검사.
 *      @li <code>axiocrypto_asym_putkey</code>에서 지정한 알고리즘에 따라 동작이 변경됨.
 *      @li 알고리즘이 <code>ASYM_ECDSA_P256</code>면 <code>ECDSA_Verify</code>를 호출함.
 *        -# 서명 r과 s가 1 <= r,s < n 인지 확인한다.  아닐 경우 CRYPTO_SIG_REJECT를 반환한다.
 *        -# 입력 메시지에 대해 해시를 수행한다.
 *        -# w = s^-1 mod n를 연산한다.
 *        -# u1 = ew mod n , u2 = rw mod n을 연산한다.
 *        -# X = u1*P + u2*Q 를 연산한다.  u1과 u2는 scalar이며, P, Q, X는 ECC POINT 이다.
 *        -# 위의 5.의 계산 결과 X가 무한원점일 경우 <code>CRYPTO_SIG_REJECT</code>를 반환한다.
 *        -# v = x1 mod n를 계산한다.
 *        -# v == r을 확인 후, 참이라면 <code>CRYPTO_SIG_ACCEPT</code>를 반환하고 아니면 <code>CRYPTO_SIG_REJECT</code>를 반환한다.
 *
 * @~korean  @param  handle      [in] 공개키가 저장된 서명키 슬롯의 핸들
 * @~korean  @param  msg   [in] 입력 메시지 포인터
 * @~korean  @param  msgsz       [in]  메세지의 크기 <code>msghashed</code>이 0이면 > 0, 아니면 32
 * @~korean  @param  msghashed   [in] 메세지가 해시연산 결과임.
 * @~korean  @param  sig         [in] 서명이 있는 버퍼. ECDSA의 겨우 r과 s를 합쳐놓은 공간.
 * @~korean  @param  sigsz       [in] 서명이 있는 버퍼의 크기
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SIG_ACCEPT</code>: 서명이 메시지와 일치함. 
 *	@li <code>CRYPTO_SIG_REJECT</code>: 서명이 메시지와 일치하지 않음.
 *	@li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: 유효하지 않은 슬롯.
 *	@li <code>CRYPTO_ERR_SIGNATURE_INVALID</code>: 유효하지 않은 서명의 버퍼
 *	@li <code>CRYPTO_ERR_SIG_MESSAGE_INVALID</code>: 유효하지 않은 메시지 버퍼
 *	@li <code>CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM</code>: 지원하지 않는 알고리즘이 설정된 컨텍스트
 *	@li <code>CRYPTO_ERR_SIG_KEY_EMPTY</code>: 선택한 슬롯에 공개키가 들어있지 않음.
 * @~english @brief  This function is used to verify the signature of given message.
 * @~english @param  handle    [in] key handle in slot where the public key is stored. 
 * @~english @param  msg [in] the address of message
 * @~english @param  msgsz     [in] the length of <code>msg</code>
 * @~english @param  msghashed [in] if 0, the module calc hash value before signing. if 1 the module skip hash calc.
 * @~english @param  sig       [in] the address of signature. r and s are considered as being concatenated.
 * @~english @param  sigsz     [in] the length of <code>sig</code>
 * @~english @return
 *      @li <code>CRYPTO_SIG_ACCEPT</code>: the signature matches the message.
 *	@li <code>CRYPTO_SIG_REJECT</code>: the signature doesn't match the message
 *	@li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid.
 *	@li <code>CRYPTO_ERR_SIGNATURE_INVALID</code>: the buffer containing <code>sig</code> is not valid
 *	@li <code>CRYPTO_ERR_SIG_MESSAGE_INVALID</code>: the buffer containing <code>msg</code> is not valid.
 *	@li <code>CRYPTO_ERR_SIG_NOT_SUPPORT_ALGORITHM</code>: The algorithm bind to the key slot is not supported
 *	@li <code>CRYPTO_ERR_SIG_KEY_EMPTY</code>: no public key in <code>handle</code>
 */
CRYPTO_STATUS axiocrypto_asym_verify(
    /* Inputs  */ const ctx_handle_t handle,
                  const uint8_t* msg, const uint32_t msgsz, const uint32_t msghashed,
		  const uint8_t *sig, const uint32_t sigsz);

/**
 * @~korean  @brief   슬롯에 할당된 공개키를 리턴
 *
 * <b>세부 사항</b>
 *  @li 리턴하는 형태는 X || Y 처럼 연접한 형태.
 *  @li P-256의 경우 X와 Y의 크기는 각각 32바이트.
 *
 * @~korean  @param   handle [in]  공개키가 저장된 서명키 슬롯의 핸들
 * @~korean  @param   Q      [out] 공개키를 리턴할 버퍼의 포인터
 * @~korean  @param   Qsz    [in]  공개키를 리턴할 버퍼의 크기. >= 64
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS </code>성공
 *      @li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *      @li <code>CRYPTO_ERR_BUFFER_NULL</code>: Q가 NULL
 *      @li <code>CRYPTO_ERR_INVALID_LENGTH</code>: 유효하지 않은 Qsz
 * @~english @brief   This function is used to get ECDSA public key in given key slot.
 * @~english @param   handle [in]  key handle in slot where the public key is stored. 
 * @~english @param   Q      [out] the buffer address for returning public key
 * @~english @param   Qsz    [in]  the length of buffer <code>Q</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS </code>function returned OK
 *      @li <code>CRYPTO_ERR_SIG_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_BUFFER_NULL</code>: <code>Q</code> is NULL
 *      @li <code>CRYPTO_ERR_INVALID_LENGTH</code>: <code>Qsz</code> is not valid
 * @~english @details in <code>Q</code>, the public key is returned concatenated.
 *                    in case of P-256, the size of <code>X</code> and <code>Y</code> is 32Byte each.
 */
CRYPTO_STATUS axiocrypto_asym_getkey(
    /* Inputs  */ const ctx_handle_t handle, ALGORITHM algo,
    /* Outputs */uint8_t *Q, const uint32_t Qsz);

/**
 * @}
 */

/**
 * @defgroup M_ECDH_API 키교환 알고리즘
 * @{
 */

/**
 * @~korean  @brief   지정된 슬롯을 이용하여 상대방의 키토큰으로 공유키를 계산함.
 *
 * <b>세부 사항</b>
 * -# Point Multiplication을 이용하여 outP = d * KT를 연산한다.
 * -# 결과값 outP가 곡선 위에 있는지 확인한다.
 *
 * @li  keysz가 0이 아니면 계산한 공유키 중 [offset..keysz]를
 * @li  대칭키컨텍스트의 symhandle이 있는 슬롯에 저장함.
 * @li  out이 NULL이 아니면 계산한 공유키를 반환함. 권장하지 않음.<br>
 *      out은 개발용으로만 사용하는 것을 권장함.
 *
 * @~korean  @param   handle    [in]  사용할 컨텍스트가 저장된 슬롯의 핸들
 * @~korean  @param   KT        [in]  키토큰 버퍼의 주소
 * @~korean  @param   KTsz      [in]  KT의 크기, 64
 * @~korean  @param   symhandle [in]  계산한 공유키를 지정할 대칭키 컨텍스트의 슬롯 핸들.
 * @~korean  @param   offset    [in]  계산한 공유키에서 대칭키 컨텍스트에 저장할 내용의 위치
 * @~korean  @param   keysz     [in]  계산한 공유키에서 대칭키 컨텍스트에 저장할 길이 정보
 * @~korean  @param   out       [out] 공유키 버퍼의 주소
 * @~korean  @param   outsz     [out] out의 크기, >= 64
 *
 * @~korean  @return
 *      @li<code>CRYPTO_SUCCESS</code>: 정상적으로 키를 생성함.
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_NOT_SUPPORT_ALGORITHM</code>: 유효하지 않은 알고리즘.
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_PRIVATEKEY_INVALID</code>: 유효하지 않은 개인키
 *	@li <code>CRYPTO_ERR_INVALID_PARAM </code>주어진 키토큰이 유효하지 않음.
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_BUFFER_SIZE_INVALID</code>: 잘못된 버퍼의 크기.
 * @~english @brief   This function is used to calculate shared key with given key token and key in given key slot.
 * @~english @param   handle    [in]  key handle in slot where the ECDH key is stored.
 * @~english @param   KT        [in]  the buffer address containing key token.
 * @~english @param   KTsz      [in]  the size of <code>KT</code>
 * @~english @param   symhandle [in]  the symmetric key handle in slot to store calculated key
 * @~english @param   offset    [in]  the offset information of the key from calculated blob.
 * @~english @param   keysz     [in]  the length information of the key from calculated blob.
 * @~english @param   out       [out] the address of buffer to store key in.
 * @~english @param   outsz     [out] the length of <code>out</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returned OK
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_NOT_SUPPORT_ALGORITHM</code>: <code>algo</code> is not valid
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_PRIVATEKEY_INVALID</code>: <code>d</code> is not valid
 *	@li <code>CRYPTO_ERR_INVALID_PARAM</code>: <code>KT</code> is not valid
 *	@li <code>CRYPTO_ERR_KEY_EXCHANGE_BUFFER_SIZE_INVALID</code>: <code>KTsz</code> is not valid
 * @~english @details if <code>keysz != 0</code>, the function stores [offset..keysz] from calculated blob <br>
 *                    in the slot with <code>symhandle</code> in symmetric key slot<br>
 *                    if <code>out != NULL</code> the function returns calculated key here.<br>
 *                    recommendation: use <code>out</code> only for development.
 */
CRYPTO_STATUS axiocrypto_ecdh_computekey(
    /* Inputs  */ const ctx_handle_t handle, const uint8_t *KT, const uint32_t KTsz,
    /* Outputs */ uint8_t *out, const uint32_t outsz);

/**
 * @}
 */

/**
 * @defgroup M_SYM_API 대칭키 알고리즘
 * @{
 */

/**
 * @~korean  @brief   암호연산에 필요한 암호키를 지정함.
 *
 * <b>세부 사항</b>
 * @li sym_enc() 또는 sym_dec()를 실행하기 전에 먼저 실행해야 함.<br>
 *     ecdh_computekey()를 실행해서 원하는 슬롯에 키를 저장한 경우 이 함수를 실행할 필요 없음.
 *
 * @~korean  @param   handle [in] 사용할 키 슬롯의 핸들
 * @~korean  @param   key    [in] 키가 저장된 주소
 * @~korean  @param   keysz  [in] 키의 길이, 16, 24, 32 중 하나.
 * @~korean  @param   crc    [in] HMAC용 키의 crc-16 xmodem 계산값.
 * @~korean  @param   attr   [in] <code>CTX_ATTR_RDONLY</code>이면 키를 저장하고 나서 읽기 전용으로 설정하여 덮어쓸 수 없도록 함. 0이면 동작에 영향 없음.
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 성공 
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: <code>key</code>가 NULL 
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_SECURITY_LEVEL</code>: 유효하지 않은 <code>keysz</code>
 *      @li <code>CRYPTO_ERR_KEY_RDONLY</code>: <code>handle</code>이 읽기 전용으로 설정되어 있음.
 *      @li <code>CRYPTO_ERR_SAVE_SYM_KEY</code>: 저장 과정에 오류 발생
 * @~english @brief   This function is used to provide secret key to given key slot.
 * @~english @param   handle [in] key handle in slot to store the secret key.
 * @~english @param   key    [in] the address of buffer containing key.
 * @~english @param   keysz  [in] the length of <code>key</code>
 * @~english @param   crc    [in]  CRC16 / XMODEM value of <code>key</code>
 * @~english @param   attr   [in] if <code>CTX_ATTR_RDONLY</code> the key is configured as read-only and not overwritten. if 0, no effect.
 * @~english @return  <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 * @~english @details this function must be called before axiocrypto_sym_enc_XXX() or axiocrypto_sym_dec_XXX().
 *	              if axiocrypto_ecdh_computekey() stored key in <code>handle</code>, this function can be skipped.
 */
CRYPTO_STATUS axiocrypto_sym_putkey(
    /* Inputs  */ const ctx_handle_t handle, 
                  const uint8_t* key, const uint32_t keysz, uint16_t crc, 
                  const ctx_attr_t attr);

/**
 * @~korean  @brief   암호화를 수행할 수 있도록 지정된 슬롯을 준비하는 함수
 *
 * <b>세부 사항</b>
 * @li <code>init()</code> - <code>update()</code> - <code>final</code>로 이어지는 호출 순서 중 첫번째 함수.
 * @li GCM 모드에서는 사용할 수 없음.
 * @li axiocrypto_sym_putkey()를 먼저 실행해야 함.
 *
 * @~korean  @param   handle [in] 사용할 키 슬롯의 핸들
 * @~korean  @param   algo   [in] 사용할 알고리즘
 * @~korean  @param   mode   [in] 사용할 운영 모드
 * @~korean  @param   iv     [in] 암호화에 사용할 초기벡터가 저장된 주소
 * @~korean  @param   ivsz   [in] iv의 길이. mode == SYM_MODE_CBC면 16, mode == SYM_MODE_CTR면 16의 배수, mode == SYM_MODE_GCM면 양수인 임의의 값.
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 성공
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: 사용할 수 없는 운영 모드
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: 검증모드에서 사용할 수 없는 알고리즘이나 운영모드
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: 지정된 슬롯에 저장된 키가 없음.
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: 암호키가 지정되어 있지 않음.
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: iv가 유효하지 않음.
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: iv의 길이가 유효하지 않음.
 * @~english @brief   This function is used to initialize symmetric key slot for cipher operation.
 * @~english @param   handle [in] key handle in slot to use.
 * @~english @param   algo   [in] algorithm constant to use.
 * @~english @param   mode   [in] cipher mode of operation.
 * @~english @param   iv     [in] the address of IV contents.
 * @~english @param   ivsz   [in] the length of <code>iv</code>
 * @~english @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: <code>mode</code> is not valid
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: <code>mode</code> is not available in approved mode.
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: there is no stored key in slot with <code>handle</code>
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: the key in slot with <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: <code>iv</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: <code>ivsz</code> is not valid
 * @~english @details this function requires axiocrypto_sym_putkey() or axiocrypto_ecdh_computekey() called before this.
 */
CRYPTO_STATUS axiocrypto_sym_enc_init(
                const ctx_handle_t handle, const ALGORITHM algo, const SYM_MODE mode,
                const uint8_t* iv, const uint32_t ivsz);

/**
 * @~korean  @brief   암호화를 수행하는 함수
 *
 * <b>세부 사항</b>
 * @li <code>init()</code> - <code>update()</code> - <code>final</code>로 이어지는 호출 순서 중 두번째 함수.
 * @li GCM 모드에서는 사용할 수 없음.
 *
 * @~korean  @param   handle [in]    사용할 키 슬롯의 핸들
 * @~korean  @param   pt     [in]    평문이 저장된 주소
 * @~korean  @param   ptsz   [in]    평문의 길이.
 * @~korean  @param   ct     [out]   암호문을 저장할 주소
 * @~korean  @param   ctsz   [inout] 암호문을 저장할 버퍼의 길이를 전달할 주소, 생성된 암호문의 길이
 *
 * @~korean  @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_enc_init()</code>를 호출하지 않았음.
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: 유효하지 않은 평문의 길이
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL</code>: 유효하지 않은 암호문을 저장할 버퍼
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: 암호문 버퍼의 길이가 유효하지 않음.
 * @~english @brief   This function is used to execute cipher encryption.
 *
 * this function requires axiocrypto_sym_enc_init() called before this.
 *
 * @~english @param   handle [in]    key handle in slot to use.
 * @~english @param   pt     [in]    the address of buffer containing plaintext
 * @~english @param   ptsz   [in]    the length of <code>pt</code>
 * @~english @param   ct     [out]   the address of buffer to store ciphertext in.
 * @~english @param   ctsz   [inout] the length of <code>ct</code>, the function returns the length of ciphertext here.
 * @~english @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_enc_init()</code> is not called before this function.
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: <code>ptsz</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL</code>: <code>ct</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: <code>ctsz</code> is not vaild
 */
CRYPTO_STATUS axiocrypto_sym_enc_update(
                const ctx_handle_t handle,
                const uint8_t*  pt, const uint32_t  ptsz, uint8_t* ct, uint32_t* ctsz);

/**
 * @~korean  @brief  암호화를 수행하고 정리하는 함수
 *
 * <b>세부 사항</b>
 * @li <code>init()</code> - <code>update()</code> - <code>final</code>로 이어지는 호출 순서 중 마지막 함수.
 * @li <code>axiocrypto_sym_enc_update()</code> 에서 출력하지 않고 남은 암호문이 있다면 이를 ct에 출력한다.
 * @li GCM 모드에서는 사용할 수 없음.
 *
 * @~korean  @param  handle [in]  사용할 키 슬롯의 핸들
 * @~korean  @param  ct     [out] 암호문을 저장할 주소
 * @~korean  @param  ctsz   [inout]  암호문을 저장할 버퍼의 길이를 전달할 주소<br>
 *                   ctsz   [out] 생성된 암호문의 길이
 *
 * @~korean  @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_enc_init()</code>를 호출하지 않았음.
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL  </code>: 유효하지 않은 암호문을 저장할 버퍼
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: 암호문 버퍼의 길이가 암호문을 저장하기에는 작음.
 * @~english @brief  This function is used to execute and finalize cipher encryption.
 *
 * this function requires axiocrypto_sym_enc_init() called before this.
 *                    if there is remaining ciphertext, it is output here.
 *
 * @~english @param  handle  [in]    key handle in slot to use.
 * @~english @param  ct     [out]   the address of buffer to store ciphertext in.
 * @~english @param  ctsz   [inout] the length of <code>ct</code>, the function returns the length of ciphertext here.
 * @~english @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_enc_init()</code> is not called before this function.
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL</code>: <code>ct</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: <code>ctsz</code> is not vaild
 */
CRYPTO_STATUS axiocrypto_sym_enc_final(
                const ctx_handle_t handle, uint8_t* ct, uint32_t* ctsz);

/**
 * @~korean  @brief   암호 해독을 수행할 수 있도록 지정된 슬롯을 준비하는 함수
 *
 * <b>세부 사항</b>
 * @li <code>init()</code> - <code>update()</code> - <code>final</code>로 이어지는 호출 순서 중 첫번째 함수.
 * @li GCM 모드에서는 사용할 수 없음.
 * @li axiocrypto_sym_putkey()를 먼저 실행해야 함.
 *
 * @~korean  @param   handle [in] 사용할 키 슬롯의 핸들
 * @~korean  @param   algo   [in] 사용할 알고리즘
 * @~korean  @param   mode   [in] 사용할 운영 모드
 * @~korean  @param   iv     [in] 암호화에 사용할 초기벡터가 저장된 주소
 * @~korean  @param   ivsz   [in] iv의 길이. mode == SYM_MODE_CBC면 16, mode == SYM_MODE_CTR면 16의 배수, mode == SYM_MODE_GCM면 양수인 임의의 값.
 *
 * @~korean  @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: 사용할 수 없는 운영 모드
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: 검증모드에서 사용할 수 없는 알고리즘이나 운영모드
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: 지정된 슬롯에 저장된 키가 없음.
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: iv의 길이가 유효하지 않음.
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: 암호키가 지정되어 있지 않음.
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: iv가 유효하지 않음.
 * @~english @brief   This function is used to init symmetric key slot for cipher operation.
 * @~english @param   handle [in] key handle in slot to use.
 * @~english @param   algo   [in] algorithm constant to use.
 * @~english @param   mode   [in] cipher mode of operation.
 * @~english @param   iv     [in] the address of IV contents.
 * @~english @param   ivsz   [in] the length of <code>iv</code>
 * @~english @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: <code>mode</code> is not valid
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: <code>mode</code> is not available in approved mode.
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: there is no stored key in slot with <code>handle</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: <code>ivsz</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: the key in slot with <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: <code>iv</code> is not valid
 * @~english @details this function requires axiocrypto_sym_putkey() or axiocrypto_ecdh_computekey() called before this.
 */
CRYPTO_STATUS axiocrypto_sym_dec_init(
                const ctx_handle_t handle, const ALGORITHM algo, const SYM_MODE mode,
                const uint8_t* iv, const uint32_t ivsz);

/**
 * @~korean  @brief   암호 해독을 수행하는 함수
 *
 * <b>세부 사항</b>
 * @li <code>init()</code> - <code>update()</code> - <code>final</code>로 이어지는 호출 순서 중 두 번째 함수.
 * @li GCM 모드에서는 사용할 수 없음.
 * @li axiocrypto_sym_dec_init()를 먼저 실행해야 함.
 *
 * @~korean  @param   handle [in]    사용할 키 슬롯의 핸들
 * @~korean  @param   ct     [in]    암호문이 저장된 주소
 * @~korean  @param   ctsz   [in]    암호문의 길이.
 * @~korean  @param   pt     [out]   평문을 저장할 주소
 * @~korean  @param   ptsz   [inout] 평문을 저장할 버퍼의 길이를 전달할 주소, 생성된 평문의 길이 리턴.
 *
 * @~korean  @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_dec_init()</code>를 호출하지 않았음.
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: 유효하지 않은 암호문의 길이
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL  </code>: 유효하지 않은 평문을 저장할 버퍼
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: 평문 버퍼의 길이가 유효하지 않음.
 * @~english @brief   This function is used to execute cipher operation.
 * @~english @param   handle [in]    key handle in slot to use.
 * @~english @param   ct     [in]    the address of containing ciphertext
 * @~english @param   ctsz   [in]    the length of <code>ct</code>
 * @~english @param   pt     [out]   the address of buffer to store plaintext in.
 * @~english @param   ptsz   [inout] the length of <code>pt</code>. the function returns the length of <code>pt</code> here.
 * @~english @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_dec_init()</code> is not called before this function.
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: <code>ctsz</code> is not vaild
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL  </code>: <code>pt</code> is not valid
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: <code>ptsz</code> is not valid
 * @~english @details this function requires axiocrypto_sym_dec_init() called before this.
 */
CRYPTO_STATUS axiocrypto_sym_dec_update(
                const ctx_handle_t handle,
                const uint8_t*  ct, const uint32_t  ctsz, uint8_t* pt, uint32_t* ptsz);

/**
 * @~korean  @brief   암호 해독을 수행하고 정리하는 함수
 *
 * <b>세부 사항</b>
 * @li <code>init()</code> - <code>update()</code> - <code>final</code>로 이어지는 호출 순서 중 마지막 함수.
 * @li GCM 모드에서는 사용할 수 없음.
 * @li axiocrypto_sym_dec_init()를 먼저 실행해야 함.
 *
 * @~korean  @param   handle [in]    사용할 키 슬롯의 핸들
 * @~korean  @param   pt     [out] 평문을 저장할 주소
 * @~korean  @param   ptsz   [inout]  평문을 저장할 버퍼의 길이를 전달할 주소, 생성된 평문의 길이
 *
 * @~korean  @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle<br>
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_dec_init()</code>를 호출하지 않았음.<br>
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL  </code>: 평문을 저장할 버퍼가 유효하지 않음.<br>
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: 평문 버퍼의 길이가 평문을 저장하기에는 작음.<br>
 *                    axiocrypto_sym_dec_update() 에서 출력하지 않고 남은 평문이 있다면 이를 pt에 출력한다.
 * @~english @brief   This function is used to execute and finalize cipher operation.
 * @~english @param   handle [in]    key handle in slot to use.
 * @~english @param   pt     [out]   the address of buffer to store plaintext in.
 * @~english @param   ptsz   [inout] the length of <code>pt</code>. the function returns the length of <code>pt</code> here.
 * @~english @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid<br>
 *      @li <code>CRYPTO_ERR_SYM_CTX_NOT_INITIALIZED</code>: <code>axiocrypto_sym_dec_init()</code> is not called before this function.<br>
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL  </code>: <code>pt</code> is not valid<br>
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: <code>ptsz</code> is not valid<br>
 * @~english @details this function requires axiocrypto_sym_dec_init() called before this.
 *                    if there is remaining plaining, it is output here.
 */
CRYPTO_STATUS axiocrypto_sym_dec_final(
                const ctx_handle_t handle, uint8_t* pt, uint32_t* ptsz);

/**
 * @~korean  @brief   암호화를 수행하는 함수.
 * 
 * <b>세부 사항</b>
 *      @li sym_putkey()을 먼저 실행해야 함.
 *      @li 한줄짜리 간단한 인터페이스를 제공하는 함수
 *      @li AES 알고리즘 대상으로, 비검증대상모드에서만 사용할 수 있음.
 *
 * @~korean  @param   handle [in]    사용할 키 슬롯의 핸들
 * @~korean  @param   algo   [in]    사용할 알고리즘
 * @~korean  @param   pt     [in]    평문이 저장된 주소
 * @~korean  @param   ptsz   [in]    평문의 길이
 * @~korean  @param   ct     [out]   암호문을 저장할 주소
 * @~korean  @param   ctsz   [inout] 암호문을 저장할 버퍼의 길이를 전달하고 반환할 주소
 *
 * @~korean  @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 * @~english @brief   This function is for simplified encryption in ECB mode of operation.
 *
 *      @li this function requires axiocrypto_sym_putkey() or axiocrypto_ecdh_computekey() called before this.
 *      @li this function provides one-line function for simple encryption.
 *      @li this function is available only for AES algorithm.
 * @~english @param   handle [in]    key handle in slot to use.
 * @~english @param   algo   [in]    algorithm constant to use.
 * @~english @param   pt     [in]    the address of buffer containing plaintext
 * @~english @param   ptsz   [in]    the length of <code>pt</code>
 * @~english @param   ct     [out]   the address of buffer to store ciphertext in.
 * @~english @param   ctsz   [inout] the length of <code>ct</code>, the function returns the length of ciphertext here.
 * @~english @return
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 */
CRYPTO_STATUS axiocrypto_sym_enc_ECB( ctx_handle_t handle, ALGORITHM algo,
                                     uint8_t*  pt, uint32_t  ptsz,
                                     uint8_t* ct, uint32_t* ctsz);
/**
 * @~korean @brief	GCM 모드 암호화를 수행하는 함수.
 *
 * <b>세부 사항</b>
 *     @li axiocrypto_sym_putkey()를 먼저 실행해야 함.
 *
 * @~korean @param      handle [in]    사용할 키의 핸들
 * @~korean @param	algo   [in]    사용할 알고리즘
 * @~korean @param      pt     [in]    평문이 저장된 주소
 * @~korean @param      ptsz   [in]    평문의 길이
 * @~korean @param      aad    [in]    Additional Authentication Data가 저장된 주소
 * @~korean @param      aadsz  [in]    aad의 길이
 * @~korean @param      tag    [out]   TAG를 저장할 주소
 * @~korean @param      tagsz  [in]    암호화를 마친 후 생성할 TAG의 길이
 * @~korean @param      iv     [in]    암호화에 사용할 초기벡터가 저장된 주소
 * @~korean @param      ivsz   [in]    iv의 길이
 * @~korean @param      ct     [out]   암호문을 저장할 주소
 * @~korean @param      ctsz   [inout] 암호문을 저장할 버퍼의 길이를 전달할 주소,
 *                                     암호문의 길이를 반환할 주소
 * @~korean @return
 *      @li <code>CRYPTO_SUCCESS</code>: 성공
 *      @li 이미 복구할 수 없는 에러가 발생한 에러상태이면 그 에러를 리턴함.
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: 인증모드에서는 사용할 수 없음.
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 <code>handle</code>
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: 유효하지 않은 <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: 키가 지정되어 있지 않음.
 *      @li <code>CRYPTO_ERR_SYM_CONTEXT_INVALID</code>: 초기화 과정 오류
 *      @li <code>CRYPTO_ERR_SYM_AAD_INVALID</code>: 유효하지 않은 <code>aad</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: 유효하지 않은 <code>ivsz</code>
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: 키가 지정되어 있지 않음.
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: 유효하지 않은 <code>iv</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_IS_ALREADY_SET</code>: 초기화 과정 오류
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_ALGORITHM</code>: 유효하지 않은 <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL</code>: <code>pt</code>가 NULL 임.
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: 유효하지 않은 <code>ptsz</code>
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL</code>: <code>ct</code>가 NULL 임.
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: 유효하지 않은 <code>ctsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_SIZE_INVALID</code>: 유효하지 않은 <code>tagsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_INVALID</code>: 유효하지 않은 <code>tag</code>
 * @~english @brief	This function encrypts plaintext in GCM mode.
 *               	axiocrypto_sym_putkey() must be called before this function is called.
 * @~english @param      handle [in]    the handle of key
 * @~english @param	 algo   [in]    algorithm to use
 * @~english @param      pt     [in]    buffer with plaintext
 * @~english @param      ptsz   [in]    the length of plaintext
 * @~english @param      aad    [in]    buffer with Additional Authentication Data
 * @~english @param      aadsz  [in]    the length of aad
 * @~english @param      tag    [out]   buffer addres to store TAG in.
 * @~english @param      tagsz  [in]    the length of TAG to return
 * @~english @param      iv     [in]    buffer with IV
 * @~english @param      ivsz   [in]    the length of IV
 * @~english @param      ct     [out]   buffer address to store ciphertext in
 * @~english @param      ctsz   [inout] in: the length of buffer, out: the length of ct
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: Success
 *      @li this function returns if there is already any unrecoverable error before this function is called.
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: this function is not available in non-approved mode.
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: invalid <code>handle</code>
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: invalid <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: no stored key or the key is invalid..
 *      @li <code>CRYPTO_ERR_SYM_CONTEXT_INVALID</code>: initialization error
 *      @li <code>CRYPTO_ERR_SYM_AAD_INVALID</code>: invalid <code>aad</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: invalid <code>ivsz</code>
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: no key
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: invalid <code>iv</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_IS_ALREADY_SET</code>: initialization error
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_ALGORITHM</code>: invalid <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL</code>: <code>pt</code> is NULL
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: invalid <code>ptsz</code>
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL</code>: <code>ct</code> is NULL
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: invalid <code>ctsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_SIZE_INVALID</code>: invalid <code>tagsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_INVALID</code>: invalid <code>tag</code>
 *
 */
CRYPTO_STATUS axiocrypto_sym_enc_GCM(ctx_handle_t handle, ALGORITHM algo,
                                const uint8_t* pt,  const uint32_t  ptsz,
                                const uint8_t* aad, const uint32_t  aadsz,
                                      uint8_t* tag, const uint32_t  tagsz,
                                const uint8_t* iv,  const uint32_t  ivsz,
                                uint8_t* ct,  uint32_t* ctsz);

/**
 * @~korean  @brief   암호문을 해독하는 함수.
 *
 * <b>세부 사항</b>
 *      @li sym_putkey()을 먼저 실행해야 함.
 *      @li 한줄짜리 간단한 인터페이스를 제공하는 함수
 *      @li AES 알고리즘 대상으로 비검증대상모드에서만 사용할 수 있음.
 *
 * @~korean  @param   handle [in]    사용할 키 슬롯의 핸들
 * @~korean  @param   algo   [in]    사용할 알고리즘
 * @~korean  @param   ct     [in]    암호문이 저장된 주소
 * @~korean  @param   ctsz   [in]    암호문의 길이가 저장된  주소
 * @~korean  @param   pt     [out]   해독이 끝난 후 평문을 저장할 주소
 * @~korean  @param   ptsz   [inout] 평문을 저장할 버퍼의 길이를 전달하고 반환할 주소
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 성공
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 handle
 *
 * @~english @brief   This function is for simplified decryption in ECB mode of operation.
 *
 *      @li this function requires axiocrypto_sym_putkey() or axiocrypto_ecdh_computekey() called before this.
 *      @li this function provides one-line function for simple decryption.
 *      @li this function is available only for AES algorithm.
 * @~english @param   handle [in]    key handle in slot to use
 * @~english @param   algo   [in]    algorithm constant to use.
 * @~english @param   ct     [in]    the address of containing ciphertext
 * @~english @param   ctsz   [in]    the length of <code>ct</code>
 * @~english @param   pt     [out]   the address of buffer to store plaintext in.
 * @~english @param   ptsz   [inout] the length of <code>pt</code>. the function returns the length of <code>pt</code> here.
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: Success
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 */
CRYPTO_STATUS axiocrypto_sym_dec_ECB(const ctx_handle_t handle, const ALGORITHM algo,
                                     const uint8_t* ct, const uint32_t  ctsz,
                                           uint8_t* pt,        uint32_t* ptsz);

/**
 * @~korean @brief	GCM 모드 암호문을 해독하는 함수.
 *
 * <b>세부 사항</b>
 *      @li axiocrypto_sym_putkey()를 먼저 실행해야 함.
 *
 * @~korean @param      handle [in]    사용할 키의 핸들
 * @~korean @param	algo   [in]    사용할 알고리즘
 * @~korean @param      ct     [in]    암호문이 저장된 주소
 * @~korean @param      ctsz   [in]    암호문의 길이
 * @~korean @param      aad    [in]    Additional Authentication Data가 저장된 주소
 * @~korean @param      aadsz  [in]    aad의 길이
 * @~korean @param      tag    [in]    TAG가 저장된 주소
 * @~korean @param      tagsz  [in]    TAG의 길이
 * @~korean @param      iv     [in]    암호화에 사용할 초기벡터가 저장된 주소
 * @~korean @param      ivsz   [in]    iv의 길이
 * @~korean @param      pt     [out]   평문을 저장할 주소
 * @~korean @param      ptsz   [inout] 평문을 저장할 버퍼의 길이를 전달할 주소<br>
 *                             평문의 길이를 반환할 주소
 *
 * @~korean @return
 *      @li <code>CRYPTO_GCM_ACCEPT</code>: 성공
 *      @li 오류 상태에 있을 때 호출되면 오류 상태를 초래한 에러를 리턴함.
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: 인증모드에서는 사용할 수 없음.
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: 유효하지 않은 <code>handle</code>
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: 유효하지 않은 <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: 키가 지정되어 있지 않음.
 *      @li <code>CRYPTO_ERR_SYM_CONTEXT_INVALID</code>: 초기화 과정 오류
 *      @li <code>CRYPTO_ERR_SYM_AAD_INVALID</code>: 유효하지 않은 <code>aad</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: 유효하지 않은 <code>ivsz</code>
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: 키가 지정되어 있지 않음.
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: 유효하지 않은 <code>iv</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_IS_ALREADY_SET</code>: 초기화 과정 오류
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_ALGORITHM</code>: 유효하지 않은 <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL</code>: <code>pt</code>가 NULL 임.
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: 유효하지 않은 <code>ptsz</code>
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL</code>: <code>ct</code>가 NULL 임.
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: 유효하지 않은 <code>ctsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_SIZE_INVALID</code>: 유효하지 않은 <code>tagsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_INVALID</code>: 유효하지 않은 <code>tag</code>
 *
 * @~english @brief	This function restores plaintext from ciphertext in GCM mode.
 *               	axiocrypto_sym_putkey() must be called before this function is called.
 * @~english @param      handle [in]    the handle of key
 * @~english @param	 algo   [in]    algorithm to use
 * @~english @param      ct     [in]    buffer with ciphertext
 * @~english @param      ctsz   [in]    the length of ciphertext
 * @~english @param      aad    [in]    buffer with Additional Authentication Data
 * @~english @param      aadsz  [in]    the length of aad
 * @~english @param      tag    [in]   buffer addres to store TAG in.
 * @~english @param      tagsz  [in]    the length of TAG to return
 * @~english @param      iv     [in]    buffer with IV
 * @~english @param      ivsz   [in]    the length of IV
 * @~english @param      pt     [out]   buffer address to store plaintext in
 * @~english @param      ptsz   [inout] in: the length of buffer, out: the length of pt
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: Success
 *      @li this function returns if there is already any unrecoverable error before this function is called.
 *      @li <code>CRYPTO_ERR_NOT_ALLOWED_IN_APPROVED_MODE</code>: this function is not available in non-approved mode.
 *      @li <code>CRYPTO_ERR_SYM_CTX_HANDLE_INVALID</code>: invalid <code>handle</code>
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_MODE</code>: invalid <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_NOKEY</code>: no stored key or the key is invalid..
 *      @li <code>CRYPTO_ERR_SYM_CONTEXT_INVALID</code>: initialization error
 *      @li <code>CRYPTO_ERR_SYM_AAD_INVALID</code>: invalid <code>aad</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_SIZE_INVALID</code>: invalid <code>ivsz</code>
 *      @li <code>CRYPTO_ERR_SYM_KEY_INVALID</code>: no key
 *      @li <code>CRYPTO_ERR_SYM_IV_INVALID</code>: invalid <code>iv</code>
 *      @li <code>CRYPTO_ERR_SYM_IV_IS_ALREADY_SET</code>: initialization error
 *      @li <code>CRYPTO_ERR_SYM_NOT_SUPPORT_ALGORITHM</code>: invalid <code>algo</code>
 *      @li <code>CRYPTO_ERR_SYM_PT_NULL</code>: <code>pt</code> is NULL
 *      @li <code>CRYPTO_ERR_SYM_PT_SIZE_INVALID</code>: invalid <code>ptsz</code>
 *      @li <code>CRYPTO_ERR_SYM_CT_NULL</code>: <code>ct</code> is NULL
 *      @li <code>CRYPTO_ERR_SYM_CT_SIZE_INVALID</code>: invalid <code>ctsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_SIZE_INVALID</code>: invalid <code>tagsz</code>
 *      @li <code>CRYPTO_ERR_SYM_TAG_INVALID</code>: invalid <code>tag</code>
 */
CRYPTO_STATUS axiocrypto_sym_dec_GCM(const ctx_handle_t handle, const ALGORITHM algo,
                                     const uint8_t* ct,  const uint32_t  ctsz,
                                     const uint8_t* aad, const uint32_t  aadsz,
                                     const uint8_t* tag, const uint32_t  tagsz,
                                     const uint8_t* iv,  const uint32_t  ivsz,
                                           uint8_t* pt,        uint32_t* ptsz);

/**
 * @}
 */

/**
 * @defgroup M_HASH_API 해시 알고리즘
 * @{
 */

/**
 * @~korean  @brief   SHA 동작 전, SHA Context 내부 변수 설정을 초기화함.
 *
 * <b>세부 사항</b>
 *      @li SHA 동작 전, SHA Context 내부 변수를 초기화한다.
 *      @li hash_init()이 실행하면 다음에 hash_final()을 실행하기 전까지는 다시 hash_init()을 반복실행할 수 없다.
 *      @li SHA-256 기능만을 이용하여 해시를 수행한다.
 *
 * @~korean  @param   handle [in] 컨텍스트의 핸들. 임의의 값을 줄 수 있음.
 * @~korean  @param   algo [in] 알고리즘 모드 상수
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 성공
 *	@li <code>CRYPTO_ERR_HASH_CTX_HANDLE_INVALID</code>: 주어진 <code>handle</code>이 유효하지 않음
 *	@li <code>CRYPTO_ERR_HASH_NOT_SUPPORT_ALGORITHM</code>: algo가 해시 알고리즘을 나타내는 상수가 아닌 경우
 *      @li <code>CRYPTO_ERR_HASH_ALL_CTX_IN_USE</code>: 사용할 수 있는 HASH 슬롯이 없음. 모두 사용중.
 *      @li <code>CRYPTO_ERR_HANDLE_ALREADY_EXIST</code>: <code>handle</code>를 사용하는 슬롯이 이미 있음.
 * @~english @brief   This function is used to init hash context for hash operation.
 *
 *      @li this function initializes the <code>handle</code> hash context
 *      @li this function is not callable repeatedly before calling axiocrypto_hash_final()
 *      @li this function supports only HASH_SHA_256
 *
 * @~english @param   handle [inout] handle to hash context.
 * @~english @param   algo  [in] algorithm constant to use. HASH_SHA_256
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returned OK
 *	@li <code>CRYPTO_ERR_HASH_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *	@li <code>CRYPTO_ERR_HASH_NOT_SUPPORT_ALGORITHM</code>: <code>algo</code> is not valid
 *      @li <code>CRYPTO_ERR_HASH_ALL_CTX_IN_USE</code>: There is no available handle
 *	@li <code>CRYPTO_ERR_HANDLE_ALREADY_EXIST</code>: <code>handle</code> is now in use.
 */
CRYPTO_STATUS axiocrypto_hash_init(
    /* Inputs  */ ctx_handle_t handle, const ALGORITHM algo);

/**
 * @~korean  @brief   메세지에 대한 HASH를 수행
 *
 * <b>세부 사항</b>
 *      @li 입력 메시지에 대한 해시를 수행하여 SHA Context의 state를 갱신한다.
 *      @li SHA-256 기능만을 이용하여 해시를 수행한다.
 *
 * @~korean  @param   handle [in] 컨텍스트가 저장된 슬롯의 번호
 * @~korean  @param   in   [in] 메세지
 * @~korean  @param   sz   [in] 메세지의 크기.
 *
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 성공
 *	@li <code>CRYPTO_ERR_HASH_CTX_HANDLE_INVALID</code>: 주어진 <code>handle</code>이 유효하지 않음
 *	@li <code>CRYPTO_ERR_HASH_CTX_NOT_OPEN</code>: <code>axiocrypto_hash_init()</code>을 실행하지 않았음.
 * @~english @brief   This function is used to execute hash operation.
 *
 *      @li this function calculates using the context in <code>handle</code> handle
 *
 * @~english @param   handle [in] context handle to use.
 * @~english @param   in   [in] the address of message buffer.
 * @~english @param   sz   [in] the length of <code>in</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returned OK
 *	@li <code>CRYPTO_ERR_HASH_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *	@li <code>CRYPTO_ERR_HASH_CTX_NOT_OPEN</code>: <code>axiocrypto_hash_init()</code> was not called
 */
CRYPTO_STATUS axiocrypto_hash_update(
    /* Inputs  */ const ctx_handle_t handle, const uint8_t *in, uint32_t const sz);

/**
 * @~korean  @brief  hash 함수의 결과를 out에 복사한 후 ctx 초기화
 *
 * <b>세부 사항</b>
 * @li SHA Context에 저장되어 있는 HASH의 state로 digest를 생성하고 out에 복사한다.
 * @li SHA-256 기능만을 이용하여 해시를 수행한다.
 *
 * @~korean  @param  handle  [in]  컨텍스트가 저장된 슬롯의 번호
 * @~korean  @param  out   [out] digest를 저장할 버퍼의 포인터
 * @~korean  @param  outsz [in]  digest를 저장할 버퍼의 크기 >= 32
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 성공
 *	@li <code>CRYPTO_ERR_HASH_CTX_HANDLE_INVALID</code>: 주어진 handle이 유효하지 않음.
 *	@li <code>CRYPTO_ERR_HASH_CTX_NOT_OPEN</code>: <code>axiocrypto_hash_init()</code>을 실행하지 않았음.
 *
 * @~english @brief  This function is used to execute and finalize hash operation.
 *
 * @li this function generates digest using <code>handle</code> hash context
 * @li this function supports only HASH_SHA_256
 *
 * @~english @param  handle  [in]  context handle to use.
 * @~english @param  out   [out] the address of buffer to store digest.
 * @~english @param  outsz [in]  the length of <code>out</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returned OK
 *	@li <code>CRYPTO_ERR_HASH_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid
 *	@li <code>CRYPTO_ERR_HASH_CTX_NOT_OPEN</code>: <code>axiocrypto_hash_init()</code> was not called
 */
CRYPTO_STATUS axiocrypto_hash_final(
    /* Inputs  */ const ctx_handle_t handle,
    /* Outputs */ uint8_t* out, const uint32_t outsz);

/**
 * @~korean  @brief  hash_init, hash_update, hash_final를 순서대로 호출
 * @~korean  @param  algo  [in]  알고리즘 명시
 * @~korean  @param  in    [in]  입력 메세지
 * @~korean  @param  insz  [in]  입력 메세지의 크기
 * @~korean  @param  out   [out] sha output
 * @~korean  @param  outsz [in]  sha output 크기
 * @~korean  @return 성공 시 0(<code>CRYPTO_SUCCESS</code>) 반환, 실패 시 에러 상수 반환
 * @~english @brief  This function is for simplified hash operation.
 * @~english @param  algo  [in]  algorithm constant to use. HASH_SHA_256
 * @~english @param  in    [in]  the address of message buffer.
 * @~english @param  insz  [in]  the length of <code>in</code>
 * @~english @param  out   [out] the address of buffer to store digest.
 * @~english @param  outsz [in]  the length of <code>out</code>
 * @~english @return <code>CRYPTO_SUCCESS</code>: the function returns OK<br>
 */
CRYPTO_STATUS axiocrypto_hash(const ALGORITHM algo,
    /* Inputs  */ const uint8_t *in, const uint32_t insz,
    /* Outputs */ uint8_t *out, const uint32_t outsz);

/**
 * @}
 */

/**
 * @defgroup M_HMAC_API 메시지 인증 알고리즘
 * @{
 */

/**
 * @~korean  @brief  HMAC 연산에 필요한 컨텍스트를 할당하고, 키를 설정.
 * @~korean  @param  handle [in] 컨텍스트의 핸들. 임의의 값을 줄 수 있음.
 * @~korean  @param  algo   [in] HMAC연산에 사용할 알고리즘 지정.
 * @~korean  @param  key    [in] HMAC용 키
 * @~korean  @param  keysz  [in] HMAC용 키의 크기 != 0
 * @~korean  @param  crc    [in] HMAC용 키의 crc-16 xmodem 계산값
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 수행.
 *      @li <code>CRYPTO_ERR_HASH_NOT_SUPPORT_ALGORITHM</code>: 유효하지 않은 알고리즘 <code>algo</code>.  현재는 HASH_SHA_256만 지원함.
 *      @li <code>CRYPTO_ERR_HMAC_CTX_HANDLE_INVALID</code>: <code>handle</code>값이 유효하지 않음.
 *      @li <code>CRYPTO_ERR_HANDLE_ALREADY_EXIST</code>: 주어진 <code>handle</code> 값이 이미 존재함
 *      @li <code>CRYPTO_ERR_HMAC_ALL_CTX_IN_USE</code>:  HMAC 컨텍스트를 모두 사용하고 있음
 *      @li <code>CRYPTO_ERR_INVALID_LENGTH</code>: 키의 길이가 유효하지 않음.
 * @~english @brief  This function is used to provide key to given hmac key handle.
 * @~english @param  handle [in] the address of key handle buffer containing or to store handle number.<br>
 *                                if the contents of the handle is CTX_HANDLE_NOT_SPECIFIED,
 *                                the module finds empty handle and stores the number of the handle
 *                                to the variable <code>handle</code> points.
 * @~english @param  algo   [in]  algorithm constant to use. HMAC_SHA_256 only.
 * @~english @param  key    [in]  the address of buffer containing HMAC key.
 * @~english @param  keysz  [in]  the length of <code>key</code>
 * @~english @param  crc    [in]  CRC16 / XMODEM value of <code>key</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returns OK
 *      @li <code>CRYPTO_ERR_HASH_NOT_SUPPORT_ALGORITHM</code>: <code>algo</code> is not valid
 *      @li <code>CRYPTO_ERR_HMAC_CTX_HANDLE_INVALID</code>: <code>handle</code> is not valid.
 *      @li <code>CRYPTO_ERR_HANDLE_ALREADY_EXIST</code>: <code>handle</code> is aloreadey in use.
 *      @li <code>CRYPTO_ERR_HMAC_ALL_CTX_IN_USE</code>:  all HMAC contexts are in use.
 *      @li <code>CRYPTO_ERR_INVALID_LENGTH</code>: <code>keysz</code> is not valid
 */
CRYPTO_STATUS axiocrypto_hmac_putkey(
    /* Inputs  */ ctx_handle_t handle, const ALGORITHM algo,
    /* Inputs  */ const uint8_t *key, const uint32_t keysz, const uint16_t crc) ;

/**
 * @~korean     @brief  HMAC 연산을 수행할 준비를 수행. 키에 대한 해시 계산하고 준비함.
 * @~korean     @param  handle [in]  사용할 컨텍스트의 슬롯
 * @~korean     @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 수행.
 *      @li <code>CRYPTO_ERR_HASH_CTX_INVALID</code>: 유효하지 않은 컨텍스트 슬롯.
 * @~english    @brief  This function is used to init hmac context for hmac operation.
 * @~english    @param  handle [in] context handle to use.
 * @~english    @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returns OK
 *      @li <code>CRYPTO_ERR_HASH_CTX_INVALID</code>: <code>handle</code> is not valid
 */
CRYPTO_STATUS axiocrypto_hmac_init(
    /* Inputs  */ const ctx_handle_t handle);

/**
 * @~korean  @brief  HMAC 연산을 수행함.
 * @~korean  @param  handle [in] 사용할 컨텍스트의 슬롯
 * @~korean  @param  in   [in] HMAC를 계산할 메시지
 * @~korean  @param  insz [in] HMAC를 계산할 메시지의 크기
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 수행.
 *      @li <code>CRYPTO_ERR_HASH_CTX_INVALID</code>: 유효하지 않은 컨텍스트 슬롯.
 * @~english @brief  This function is used to execute hmac operation.
 * @~english @param  handle [in] context handle to use.
 * @~english @param  in   [in] the address of message buffer.
 * @~english @param  insz [in] the length of <code>in</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returns OK
 *      @li <code>CRYPTO_ERR_HASH_CTX_INVALID</code>: <code>handle</code> is not valid
 */
CRYPTO_STATUS axiocrypto_hmac_update(
    /* Inputs  */ const ctx_handle_t handle,
    /* Inputs  */ const uint8_t * in, const uint32_t  insz);

/**
 * @~korean  @brief  HMAC 연산을 수행함.
 * @~korean  @param  handle  [in]  사용할 컨텍스트의 슬롯
 * @~korean  @param  out   [out] HMAC값을 저장할 버퍼
 * @~korean  @param  outsz [out] HMAC값을 저장할 버퍼의 크기 >= 32
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 수행.
 *      @li <code>CRYPTO_ERR_HASH_CTX_INVALID</code>: 유효하지 않은 컨텍스트 슬롯.
 *                   <code>CRYPTO_ERR_INVALID_LENGTH</code>: 출력 버퍼의 크기가 필요한 것보다 작음.
 * @~english @brief  This function is used to execute and finalize hmac operation.
 * @~english @param  handle  [in]  context handle to use.
 * @~english @param  out   [out] the address of buffer to store hmac value.
 * @~english @param  outsz [out] the length of <code>out</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returns OK
 *      @li <code>CRYPTO_ERR_HASH_CTX_INVALID</code>: <code>handle</code> is not valid
 *      @li <code>CRYPTO_ERR_INVALID_LENGTH</code>: <code>outsz</code> is too small
 */
CRYPTO_STATUS axiocrypto_hmac_final(
    /* Inputs  */ const ctx_handle_t handle,
    /* Outputs */ uint8_t *out, const uint32_t outsz);

/**
 * @~korean  @brief   HMAC 연산을 수행함.
 *
 * <b>세부 사항</b>
 * @li 내부에서 수행하는 hmac_init나 hmac_update가 실패할 경우 HMCA_final을 수행한 다음 먼저 발생한 에러값을 리턴함.
 *
 * @~korean  @param   algo  [in]  HMAC연산에 사용할 알고리즘 지정.
 * @~korean  @param   key   [in]  HMAC용 키
 * @~korean  @param   keysz [in]  HMAC용 키의 크기 != 0
 * @~korean  @param   in    [in]  HMAC를 계산할 메시지
 * @~korean  @param   insz  [in]  HMAC를 계산할 메시지의 크기
 * @~korean  @param   out   [out] HMAC값을 저장할 버퍼
 * @~korean  @param   outsz [out] HMAC값을 저장할 버퍼의 크기 >= 32
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 수행.
 *      @li <code>CRYPTO_ERR_INVALID_LENGTH</code>: 출력 버퍼의 크기가 필요한 것보다 작음.
 * @~english @brief   This function is for simplified hmac operation.
 * @~english @param   algo  [in]  algorithm constant to use. HMAC_SHA_256 only.
 * @~english @param   key   [in]  the address of buffer containing HMAC key.
 * @~english @param   keysz [in]  the length of <code>key</code>
 * @~english @param   in    [in]  the address of message buffer.
 * @~english @param   insz  [in]  the length of <code>in</code>
 * @~english @param   out   [out] the address of buffer to store hmac value.
 * @~english @param   outsz [out] the length of <code>out</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returns OK
 *      @li <code>CRYPTO_ERR_INVALID_LENGTH</code>: <code>outsz</code> is too small
 */
CRYPTO_STATUS axiocrypto_hmac(
    /* Inputs  */ const ALGORITHM algo,
    /* Inputs  */ const uint8_t *key, const uint32_t keysz,
    /* Inputs  */ const uint8_t *in, const uint32_t insz,
    /* Outputs */ uint8_t *out, const uint32_t outsz);

/**
 * @}
 */

/**
 * @defgroup M_DRBG_API 난수 발생기
 * @{
 */


/**
 * @~korean  @brief  HASH기반 DRBG와 TRNG를 이용하여 랜덤한 수를 생성한다.
 *
 * <b>세부 사항</b>
 *      @li HASH기반 DRBG와 TRNG를 이용하여 랜덤한 수를 생성한다.<br>
 *          랜덤한 수를 생성하기 위한 과정은 다음과 같다.
 *
 * -# TRNG API를 이용하기 위하여 TRNG를 초기화하는 TRNG_init 함수를 호출한다.
 * -# DRBG의 입력값으로 전달할 엔트로피 입력값과 추가입력, 논스, 개별화 문자열을
 *    TRNG로부터 엔트로피 소스를 받아와 설정한다.
 *    TRNG로부터 랜덤한 데이터를 가져오는 함수는 TRNG_GetRandomData이다.
 *    만약 TRNG_GetRandomData 함수에서 에러 발생 시 <code>CRYPTO_ERR_FIRMWARE_TRNG_API_FAIL</code>를 반환한다.
 * -# DRBG를 사용하기 위해 DRBG_Set_Context를 호출하여 DRBG Context를 설정한다.
 *    랜덤함수에서는 예측 내성 없음, 갱신주기 0으로 설정한다.
 * -# 호출하여 요청받은 outsz * 8만큼의 랜덤한 비트를 생성한다.
 * -# 4.의 과정을 통해 랜덤한 수를 생성하였다면 DRBG_final를 호출하여
 *    DRBG Context를 Finalize한다.
 *
 * @~korean  @param  out   [out]  난수를 저장할 주소
 * @~korean  @param  outsz [out]  버퍼의 크기
 * @~korean  @return
 *      @li <code>CRYPTO_SUCCESS</code>: 정상 수행.
 *      @li <code>CRYPTO_ERR_FIRMWARE_TRNG_API_FAIL</code>: SEED 생성을 위한 TRNG 동작 실패 
 *
 * @~english @brief  This function is used to generate random number using DRBG and TRNG.
 *
 * this function generates random number according to NIST SP 800-90A.
 *
 * @~english @param  out   [out]  the address of buffer to store random number.
 * @~english @param  outsz [out]  the length of <code>out</code>
 * @~english @return
 *      @li <code>CRYPTO_SUCCESS</code>: the function returned OK
 */
CRYPTO_STATUS axiocrypto_random(
    /* Outputs */ uint8_t* out,
    /* Inputs  */ const uint32_t outsz);

/**
 * @}
 */

/**
 * @addtogroup M_GENERAL_API 일반 함수
 * @{
 */

/**
 * @~korean  @brief 현재 암호모듈의 현재 상태를 보여줌.
 *
 * <b>세부 사항</b>
 * @li '현재 상태'는 무결성 정보, 버전 정보, 동작 모드를 포함.
 *
 * @~korean  @param versionstr    버전정보를 저장할 버퍼의 포인터. NULL 이면 버전 정보를 제공하지 않음.
 * @~korean  @param versionstrlen versionstr의 길이. 0 이면 버전 정보를 제공하지 않음.
 * @~korean  @param popmode       동작 모드를 저장할 변수의 포인터, NULL 이면 모드 정보를 제공하지 않음.
 * @~korean  @return <code>CRYPTO_SUCCESS </code>무결성 검증 성공
 *
 * @~english @brief This function is used to current status of crypto module.
 *
 * <b>세부 사항</b>
 * @li 'current status' includes integrity, version and operation mode.
 *
 * @~english @param versionstr    the address of buffer to store versioninformation. if NULL, version information is not returned.
 * @~english @param versionstrlen the length of <code>versionstr</code>
 * @~english @param popmode       the address of variable to store the operation mode. if NULL, operation mode information is not returned.
 * @~english @return <code>CRYPTO_SUCCESS </code>the function returned OK.
 */
CRYPTO_STATUS axiocrypto_info(char *versionstr, uint32_t versionstrlen, operation_mode_t *popmode);

/**
 * @~korean     @brief 동작모드 변경
 *
 * <b>세부 사항</b>
 * @li approved 모드와 Non-Approved 모드 등 동작 모드를 변경함.
 * @li opmode가 현재 모드와 다르면 저장된 키를 삭제하고 리붓을 실행해서 내부 자료구조를 모두 초기화함.
 *
 * @~korean     @param opmode  변경할 동작모드. @c OP_MODE_APPROVED_KCMVP 또는 @c OP_MODE_NON_APPROVED 
 * @~korean     @return  <code>CRYPTO_SUCCESS</code>: 성공
 * @~english    @brief This function is used to change the operation mode of module.
 *
 * @li this function changes the operation mode of the module.
 * @li if <code>opmode</code> is different from the current mode, this function switches to <code>opmode</code>. while switching, all the keys stored are erased and the module reboots to clean all the data structures in SRAM.
 *
 * @~english    @param opmode  the operation mode to switch to. OP_MODE_APPROVED_FIPS1402 or OP_MODE_NON_APPROVED 
 * @~english    @return  <code>CRYPTO_SUCCESS</code>: the function returned OK
 */
CRYPTO_STATUS axiocrypto_set_mode(operation_mode_t opmode);

/**
 * @~korean     @brief 모든 키 소거
 *
 * <b>세부 사항</b>
 * @li 저장된 키를 삭제하고 리붓을 실행해서 내부 자료구조를 모두 초기화함.
 *
 * @~korean     @return <code>CRYPTO_SUCCESS </code>
 * @~english    @brief This function is used to delete all the stored keys.
 * @~english    @return <code>CRYPTO_SUCCESS
 </code>* @~english    @details this function erases all the keys stored and reboots to clean all the data structures in SRAM.
 */
CRYPTO_STATUS axiocrypto_clear_all(void);

/**
 * @~korean     @brief 초기화 함수.
 *
 * <b>세부 사항</b>
 * @li     초기화와 자기시험 함수를 실행한다.
 * @li 자가시험이 실패하면 암호모듈은 더 이상 동작하지 않는다.
 *
 * @~korean     @return <code>CRYPTO_SUCCESS </code>정상 동작<br>
 *         에러가 발행한 경우, 에러 코드
 *
 * @~english    @brief Crypto module initializing function.
 *
 * @li this function executes initialization and self-test of the module.
 * @li if self-test fails, the crypto function of the module will not operate anymore.
 *
 * @~english    @return <code>CRYPTO_SUCCESS </code>the function returned OK
 */
CRYPTO_STATUS axiocrypto_init(uint8_t *password, uint32_t sz);
/**
 * @~korean     @brief 정리함수.
 *
 * <b>세부 사항</b>
 *      @li 내부적으로 할당한 메모리를 회수하고 사용하던 메모리를 청소함.
 *      @li M235x의 전원을 끄는 것으로 대신할 수 있음.
 *
 * @~english    @brief Crypto module finalizing function.
 *
 *      @li this function cleans and deallocates all memories.
 *      @li powering-off M235x has the same effect with this function.
 */
CRYPTO_STATUS axiocrypto_finish(void);

/**
 * @~korean     @brief
 *      @li uint8_t[32] 형태의 엔티티 정보를 등록함. 필수 아님.
 *      @li 이 함수를 사용하면 axiocrypto_sym_putkey(), axiocrypto_asym_putkey(),
 *          axiocrypto_ecdh_putkey(), axiocrypto_hmac_putkey()를 호출할 때
 *          axiocrypto_xor_key()를 이용하여 key 와 entity info 를 xor한 결과를 전달해야 함.
 *  @~korean    @param entityinfo 32바이트 크기 버퍼의 주소.
 *
 * @~english    @brief
 *      @li this function registers 32B entity info. Not mandatory.
 *      @li After calling this function, when axiocrypto_sym_putkey(), axiocrypto_asym_putkey(),
 *          axiocrypto_ecdh_putkey(), axiocrypto_hmac_putkey() are callled,
 *          the key value must be  xor'ed with entity info using axiocrypto_xor_key().
 *  @~english   @param entityinfo Theh address of 32B sized buffer.
 */
CRYPTO_STATUS axiocrypto_set_entity_info(uint8_t *entityinfo);

CRYPTO_STATUS axiocrypto_pbkdf(uint8_t *pw, uint32_t pwsz, uint8_t *salt, uint32_t saltsz, uint32_t iter, uint8_t *key, uint32_t keysz);

CRYPTO_STATUS axiocrypto_get_slotinfo(const ctx_handle_t handle, const ALGORITHM algo, uint16_t *info);
CRYPTO_STATUS axiocrypto_trng_random(unsigned char *output, uint32_t len, uint32_t *olen);
CRYPTO_STATUS axiocrypto_self_test(void);
CRYPTO_STATUS axiocrypto_get_module_status(void);

CRYPTO_STATUS axiocrypto_ecdh_putkey(const ctx_handle_t handle, const ALGORITHM algo,
                    const uint8_t *d, const uint32_t dsz, uint16_t dcrc,
                    const uint8_t *Q, const uint32_t Qsz, uint16_t Qcrc, const ctx_attr_t attr);

CRYPTO_STATUS axiocrypto_ecdsa_putkey(const ctx_handle_t handle, const ALGORITHM algo,
                    const uint8_t *d, const uint32_t dsz, uint16_t dcrc,
                    const uint8_t *Q, const uint32_t Qsz, uint16_t Qcrc, const ctx_attr_t attr);
CRYPTO_STATUS axiocrypto_ecdsa_genkey(const ctx_handle_t handle, const ALGORITHM algo, const ctx_attr_t attr);
CRYPTO_STATUS axiocrypto_ecdh_genkey(const ctx_handle_t handle, const ALGORITHM algo, const ctx_attr_t attr);
/**
 * @}
 */
