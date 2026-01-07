#ifndef _NSC_DEF_H_
#define _NSC_DEF_H_

#include <stdio.h>
#include <stdint.h>

#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#else
#endif

#define NSCBROKER_ERROR             -9999

typedef struct{
    int id;
    void *param[16];
    int param_len;
    void *ret;
    
#if defined(OS_FREERTOS)
	SemaphoreHandle_t sema;
#else    

#endif
}nsc_msg_t;

void *nsc_dispatcher(void *magic, void *p1, void *p2, void *p3);


#define NSC_TESTER                          0001

#define NSC_SecureContext_LoadContext           101
#define NSC_SecureContext_SaveContext           102
#define NSC_SecureContext_Init                  103
#define NSC_SecureContext_AllocateContext       104
#define NSC_SecureContext_FreeContext           105
#define NSC_SecureInit_DePrioritizeNSExceptions 106
#define NSC_SecureInit_EnableNSFPUAccess        107


#define NSC_RTC_GET                         1000
#define NSC_RTC_SET                         1001
#define NSC_RESET_CHIP                      1002
#define NSC_PIN_FUNCTION                    1003
#define NSC_I2C_CLOSE                       1004
#define NSC_GET_FLASHINFO                   1005
#define NSC_PM_GET_WAKEUP                   1006
#define NSC_PM_SET_ENV                      1007
#define NSC_PM_POWERDOWN                    1008
#define NSC_PM_SET_WAKEUP_PIN               1009
#define NSC_PM_SET_WAKEUP_RTC               1010
#define NSC_SCR_LOCK                        1011
#define NSC_SCR_IS_LOCK                     1012
#define NSC_USB_SELECT_MODE                 1013
#define NSC_MCU_WDT_INIT                    1014
#define NSC_MCU_WDT_FINI                    1015
#define NSC_TRNG_RANDOM                     1016
#define NSC_FWUPDATE_BEGIN                  1017
#define NSC_FWUPDATE_NEXT                   1018
#define NSC_HARDFAULT_SET_MODE              1019
#define NSC_HARDFAULT_SET_FUNC              1020
#define NSC_HARDFAULT_GET_MODE              1021
#define NSC_RTC_SET_CLOCK                   1022
#define NSC_RTC_EnableSpareAccess           1023
#define NSC_RTC_DynamicTamperEnable         1024
#define NSC_RTC_DynamicTamperConfig         1025
#define NSC_RTC_EnableInt                   1026
#define NSC_RTC_SetGPIOMode                 1027
#define NSC_RTC_WRITE_SPARE_REGISTER        1028
#define NSC_RTC_READ_SPARE_REGISTER         1029
#define NSC_RTC_GET_TAMPER_INT_FLAG         1030
#define NSC_RTC_GET_TAMPER_INT_STATUS       1031
#define NSC_RTC_CLEAR_TAMPER_INT_FLAG       1032

#define NSC_SECUREBOOT_STATUS               1031
#define NSC_SECUREBOOT_ENABLE               1032

#define NSC_DICE_GET_PUBLICKEY              1041
#define NSC_DICE_SIGN                       1042
#define NSC_DICE_VERIFY                     1043

#define NSC_ABM_CONFIG_LOAD                 1101
#define NSC_ABM_CONFIG_SAVE                 1102



#define NSC_SYS_ResetModule                 2001
#define NSC_CLK_SetModuleClock              2002
#define NSC_CLK_EnableModuleClock           2003
#define NSC_CLK_DisableModuleClock          2004

#define NSC_CLK_DisableCKO                  3001
#define NSC_CLK_EnableCKO                   3002
#define NSC_CLK_PowerDown                   3003
#define NSC_CLK_Idle                        3004
#define NSC_CLK_SetCoreClock                3011
#define NSC_CLK_SetHCLK                     3012
#define NSC_CLK_SetSysTickClockSrc          3013
#define NSC_CLK_EnableXtalRC                3014
#define NSC_CLK_DisableXtalRC               3015
#define NSC_CLK_EnablePLL                   3016
#define NSC_CLK_DisablePLL                  3017
#define NSC_CLK_WaitClockReady              3018
#define NSC_CLK_EnableSysTick               3019
#define NSC_CLK_DisableSysTick              3020
#define NSC_CLK_SetPowerDownMode            3021
#define NSC_CLK_EnableDPDWKPin              3022
#define NSC_CLK_GetPMUWKSrc                 3023
#define NSC_CLK_EnableSPDWKPin              3024
#define NSC_SYS_ClearResetSrc               3026
#define NSC_SYS_GetBODStatus                3027
#define NSC_SYS_GetResetSrc                 3028
#define NSC_SYS_IsRegLocked                 3029
#define NSC_SYS_ReadPDID                    3030
#define NSC_SYS_SetPowerLevel               3031
#define NSC_SYS_SetPowerRegulator           3032
#define NSC_SYS_SetSSRAMPowerMode           3033
#define NSC_SYS_SetPSRAMPowerMode           3034
#define NSC_SYS_SetVRef                     3035
#define NSC_RTC_SetClockSource              3036
#define NSC_SET_REGISTER                    3037
#define NSC_GET_REGISTER                    3038
#define NSC_SET_TEMPERATURE_ENABLE          3039

#define NSC_CLK_GetHXTFreq                  4001
#define NSC_CLK_GetLXTFreq                  4002
#define NSC_CLK_GetHCLKFreq                 4003
#define NSC_CLK_GetPCLK0Freq                4004
#define NSC_CLK_GetPCLK1Freq                4005
#define NSC_CLK_GetCPUFreq                  4006
#define NSC_CLK_GetPLLClockFreq             4007
#define NSC_CLK_GetModuleClockSource        4008
#define NSC_CLK_GetModuleClockDivider       4009
#define NSC_CLK_ENABLE_RTCWK                4010
#define NSC_CLK_DISABLE_RTCWK               4011

#define NSC_SDF_INIT                        5001
#define NSC_SDF_READ                        5002
#define NSC_SDF_WRITE                       5003

#define NSC_TAMPER_IOSEL_RTC                6001
#define NSC_TAMPER_SET_ENV                  6002
#define NSC_TAMPER_INIT                     6003
#define NSC_TAMPER_DEINIT                   6004
#define NSC_TAMPER_GET_INTERRUPT_FLAG       6005
#define NSC_TAMPER_HANDLER_SET_FUNC         6006

#define NSC_AXIOCRYPTO_INIT                 9001
#define NSC_AXIOCRYPTO_FINISH               9002
#define NSC_AXIOCRYPTO_ALLOCATE_SLOT        9003
#define NSC_AXIOCRYPTO_FREE_SLOT            9004
#define NSC_AXIOCRYPTO_ASYM_GENKEY          9005
#define NSC_AXIOCRYPTO_ECDH_GENKEY          9006
#define NSC_AXIOCRYPTO_ASYM_PUTKEY          9007
#define NSC_AXIOCRYPTO_ECDH_PUTKEY          9008
#define NSC_AXIOCRYPTO_ASYM_SIGN            9009
#define NSC_AXIOCRYPTO_ASYM_VERIFY          9010
#define NSC_AXIOCRYPTO_ASYM_GETKEY          9011
#define NSC_AXIOCRYPTO_ECDH_GETKEY          9012
#define NSC_AXIOCRYPTO_ECDH_COMPUTEKEY      9013
#define NSC_AXIOCRYPTO_SYM_PUTKEY           9014
#define NSC_AXIOCRYPTO_SYM_ENC_INIT         9015
#define NSC_AXIOCRYPTO_SYM_ENC_UPDATE       9016
#define NSC_AXIOCRYPTO_SYM_ENC_FINAL        9017
#define NSC_AXIOCRYPTO_SYM_DEC_INIT         9018
#define NSC_AXIOCRYPTO_SYM_DEC_UPDATE       9019
#define NSC_AXIOCRYPTO_SYM_DEC_FINAL        9020
#define NSC_AXIOCRYPTO_SYM_ENC_ECB          9021
#define NSC_AXIOCRYPTO_SYM_DEC_ECB          9022
#define NSC_AXIOCRYPTO_SYM_ENC_GCM          9023
#define NSC_AXIOCRYPTO_SYM_DEC_GCM          9024
#define NSC_AXIOCRYPTO_HASH_INIT            9025
#define NSC_AXIOCRYPTO_HASH_UPDATE          9026
#define NSC_AXIOCRYPTO_HASH_FINAL           9027
#define NSC_AXIOCRYPTO_HASH                 9028
#define NSC_AXIOCRYPTO_HMAC_PUTKEY          9029
#define NSC_AXIOCRYPTO_HMAC_INIT            9030
#define NSC_AXIOCRYPTO_HMAC_UPDATE          9031
#define NSC_AXIOCRYPTO_HMAC_FINAL           9032
#define NSC_AXIOCRYPTO_HMAC                 9033
#define NSC_AXIOCRYPTO_RANDOM               9034
#define NSC_AXIOCRYPTO_INFO                 9035
#define NSC_AXIOCRYPTO_SET_MODE             9036
#define NSC_AXIOCRYPTO_CLEAR_ALL            9037
#define NSC_AXIOCRYPTO_SET_ENTITY_INFO      9038
#define NSC_AXIOCRYPTO_PBKDF                9039
#define NSC_AXIOCRYPTO_GET_SLOTINFO         9040
#define NSC_AXIOCRYPTO_TRNG_RANDOM          9041
#define NSC_AXIOCRYPTO_SELF_TEST            9042
#define NSC_AXIOCRYPTO_GET_MODULE_STATUS    9043
#define NSC_AXIOCRYPTO_SET_ERROR            9044
#define NSC_AXIOCRYPTO_SHOW_KEYSTORAGE      9045
#define NSC_AXIOCRYPTO_DRBG                 9046
#define NSC_AXIOCRYPTO_DRBG_SET_CONTEXT     9047
#define NSC_AXIOCRYPTO_DRBG_INIT            9048
#define NSC_AXIOCRYPTO_DRBG_UPDATE          9049
#define NSC_AXIOCRYPTO_DRBG_FINAL           9050
#define NSC_AXIOCRYPTO_HASH_CLONE           9051

#define NSC_AXIOCRYPTO_PQC_KEYPAIR          9101
#define NSC_AXIOCRYPTO_PQC_SIGN             9102
#define NSC_AXIOCRYPTO_PQC_OPEN             9103
#define NSC_AXIOCRYPTO_PQC_SIGN_SIGNATURE   9104
#define NSC_AXIOCRYPTO_PQC_VERIFY           9105

#define NSC_AXIOCRYPTO_PQC_ENCAPSULATE      9106
#define NSC_AXIOCRYPTO_PQC_DECAPSULATE      9107


#define NSC_SECURE_STACK_SET_INIT           9901
#define NSC_SECURE_STACK_GET_INFO           9902
#define NSC_SECURE_STACK_TEST               9903

#define NSC_END                             9999
#endif // _NSCAPI_H_
