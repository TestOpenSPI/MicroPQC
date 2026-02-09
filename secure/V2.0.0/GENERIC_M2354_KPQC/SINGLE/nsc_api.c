#if defined(TRUSTZONE_NONSECURE)

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "nsc_def.h"
#include "nsc_api.h"
#include "nscbroker.h"
#include "axiocrypto_pqc.h"

int rtc_set_clock(rtc_clock_type_t rtc_clock_type)
{
    return (int)NSCBROKER_RUN(NSC_RTC_SET_CLOCK, rtc_clock_type);
}
int rtc_get(struct tm *t) 
{ 
    return (int)NSCBROKER_RUN(NSC_RTC_GET, t);
}

int rtc_set(struct tm *t)
{ 
    return (int)NSCBROKER_RUN(NSC_RTC_SET, t); 
}


int reset_chip() { return (int)NSCBROKER_RUN(NSC_RESET_CHIP); }

int scr_lock(int is_all) { return (int)NSCBROKER_RUN(NSC_SCR_LOCK, is_all); }

int scr_is_lock() { return (int)NSCBROKER_RUN(NSC_SCR_IS_LOCK); }

int usb_select_mode_s(int mode)
{
    return (int)NSCBROKER_RUN(NSC_USB_SELECT_MODE, mode);
}

int pin_function_s(int32_t port, int32_t pin, int32_t data)
{
    return (int)NSCBROKER_RUN(NSC_PIN_FUNCTION, port, pin, data);
}

int mcu_wdt_init(void) { return (int)NSCBROKER_RUN(NSC_MCU_WDT_INIT); }

int mcu_wdt_fini(void) { return (int)NSCBROKER_RUN(NSC_MCU_WDT_FINI); }

int fwupdate_begin(uint8_t *data, uint32_t size, uint32_t total_size)
{
    return (int)NSCBROKER_RUN(NSC_FWUPDATE_BEGIN, 0, data, size, total_size);
}

int fwupdate_next(uint8_t *data, uint32_t size)
{
    return (int)NSCBROKER_RUN(NSC_FWUPDATE_NEXT, data, size);
}

int SYS_ResetModule_S(uint32_t u32ModuleIndex)
{
    return (int)NSCBROKER_RUN(NSC_SYS_ResetModule, u32ModuleIndex);
}

int CLK_SetModuleClock_S(uint32_t u32ModuleIdx, uint32_t u32ClkSrc,
                         uint32_t u32ClkDiv)
{
    return (int)NSCBROKER_RUN(NSC_CLK_SetModuleClock, u32ModuleIdx, u32ClkSrc,
                              u32ClkDiv);
}

int CLK_EnableModuleClock_S(uint32_t u32ModuleIdx)
{
    return (int)NSCBROKER_RUN(NSC_CLK_EnableModuleClock, u32ModuleIdx);
}

int CLK_DisableModuleClock_S(uint32_t u32ModuleIdx)
{
    return (int)NSCBROKER_RUN(NSC_CLK_DisableModuleClock, u32ModuleIdx);
}

int I2C_Close_S(I2C_T *i2c) { return (int)NSCBROKER_RUN(NSC_I2C_CLOSE, i2c); }

#if defined(MODULE_SPIFLASH)
#include "spiflash.h"
int get_flashinfo_s(uint8_t id[6], struct flash_info *info)
{
    return (int)NSCBROKER_RUN(NSC_GET_FLASHINFO, id, info);
}
#endif

pm_status_t pm_get_wakeup(gpio_id_t *wakeup_pin_id)
{
    return (pm_status_t)(int)NSCBROKER_RUN(NSC_PM_GET_WAKEUP, wakeup_pin_id);
}

int pm_set_env(void) { return (int)NSCBROKER_RUN(NSC_PM_SET_ENV); }

int pm_powerdown(pm_mode_t mode)
{
    return (int)NSCBROKER_RUN(NSC_PM_POWERDOWN, mode);
}

int pm_set_wakeup_pin(gpio_id_t id, pm_wakeup_pin_mode_t wp_mode)
{
    return (int)NSCBROKER_RUN(NSC_PM_SET_WAKEUP_PIN, id, wp_mode);
}

int pm_set_wakeup_rtc(struct tm *t)
{
    return (int)NSCBROKER_RUN(NSC_PM_SET_WAKEUP_RTC, t);
}

int CLK_DisableCKO_S(void) { return (int)NSCBROKER_RUN(NSC_CLK_DisableCKO); }

int CLK_EnableCKO_S(uint32_t u32ClkSrc, uint32_t u32ClkDiv,
                    uint32_t u32ClkDivBy1En)
{
    return (int)NSCBROKER_RUN(NSC_CLK_EnableCKO, u32ClkSrc, u32ClkDiv,
                              u32ClkDivBy1En);
}

int CLK_PowerDown_S(void) { return (int)NSCBROKER_RUN(NSC_CLK_PowerDown); }

int CLK_Idle_S(void) { return (int)NSCBROKER_RUN(NSC_CLK_Idle); }

int CLK_GetCPUFreq_S(uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_CLK_GetCPUFreq, output);
}

int CLK_SetCoreClock_S(uint32_t u32Hclk, uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_CLK_SetCoreClock, u32Hclk, output);
}

int CLK_SetHCLK_S(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    return (int)NSCBROKER_RUN(NSC_CLK_SetHCLK, u32ClkSrc, u32ClkDiv);
}

int CLK_SetSysTickClockSrc_S(uint32_t u32ClkSrc)
{
    return (int)NSCBROKER_RUN(NSC_CLK_SetSysTickClockSrc, u32ClkSrc);
}

int CLK_EnableXtalRC_S(uint32_t u32ClkMask)
{
    return (int)NSCBROKER_RUN(NSC_CLK_EnableXtalRC, u32ClkMask);
}

int CLK_DisableXtalRC_S(uint32_t u32ClkMask)
{
    return (int)NSCBROKER_RUN(NSC_CLK_DisableXtalRC, u32ClkMask);
}

int CLK_EnablePLL_S(uint32_t u32PllClkSrc, uint32_t u32PllFreq,
                    uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_CLK_EnablePLL, u32PllClkSrc, u32PllFreq,
                              output);
}

int CLK_DisablePLL_S(void) { return (int)NSCBROKER_RUN(NSC_CLK_DisablePLL); }

int CLK_WaitClockReady_S(uint32_t u32ClkMask, uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_CLK_WaitClockReady, u32ClkMask, output);
}

int CLK_EnableSysTick_S(uint32_t u32ClkSrc, uint32_t u32Count)
{
    return (int)NSCBROKER_RUN(NSC_CLK_EnableSysTick, u32ClkSrc, u32Count);
}

int CLK_DisableSysTick_S(void)
{
    return (int)NSCBROKER_RUN(NSC_CLK_DisableSysTick);
}

int CLK_SetPowerDownMode_S(uint32_t u32PDMode)
{
    return (int)NSCBROKER_RUN(NSC_CLK_SetPowerDownMode, u32PDMode);
}

int CLK_EnableDPDWKPin_S(uint32_t u32TriggerType)
{
    return (int)NSCBROKER_RUN(NSC_CLK_EnableDPDWKPin, u32TriggerType);
}

int CLK_GetPMUWKSrc_S(uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_CLK_GetPMUWKSrc, output);
}

int CLK_EnableSPDWKPin_S(uint32_t u32Port, uint32_t u32Pin,
                         uint32_t u32TriggerType, uint32_t u32DebounceEn)
{
    return (int)NSCBROKER_RUN(NSC_CLK_EnableSPDWKPin, u32Port, u32Pin,
                              u32TriggerType, u32DebounceEn);
}

// default __NONSECURE_ENTRY_WEAK in BSP
uint32_t CLK_GetHXTFreq(void)
{
    return (uint32_t)NSCBROKER_RUN(NSC_CLK_GetHXTFreq);
}

uint32_t CLK_GetLXTFreq(void) { return (int)NSCBROKER_RUN(NSC_CLK_GetLXTFreq); }

uint32_t CLK_GetHCLKFreq()
{
    return (uint32_t)NSCBROKER_RUN(NSC_CLK_GetHCLKFreq);
}

uint32_t CLK_GetPCLK0Freq(void)
{
    return (uint32_t)NSCBROKER_RUN(NSC_CLK_GetPCLK0Freq);
}

uint32_t CLK_GetPCLK1Freq(void)
{
    return (uint32_t)NSCBROKER_RUN(NSC_CLK_GetPCLK1Freq);
}

uint32_t CLK_GetCPUFreq() { return (int)NSCBROKER_RUN(NSC_CLK_GetCPUFreq); }

uint32_t CLK_GetPLLClockFreq(void)
{
    return (uint32_t)NSCBROKER_RUN(NSC_CLK_GetPLLClockFreq);
}

uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx)
{
    return (uint32_t)NSCBROKER_RUN(NSC_CLK_GetModuleClockSource, u32ModuleIdx);
}

uint32_t CLK_GetModuleClockDivider(uint32_t u32ModuleIdx)
{
    return (uint32_t)NSCBROKER_RUN(NSC_CLK_GetModuleClockDivider, u32ModuleIdx);
}

// end

// SYS
int SYS_ClearResetSrc_S(uint32_t u32Src)
{
    return (int)NSCBROKER_RUN(NSC_SYS_ClearResetSrc, u32Src);
}

int SYS_GetBODStatus_S(uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_SYS_GetBODStatus, output);
}

int SYS_GetResetSrc_S(uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_SYS_GetResetSrc, output);
}

int SYS_IsRegLocked_S(uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_SYS_IsRegLocked, output);
}

int SYS_ReadPDID_S(uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_SYS_ReadPDID, output);
}

int SYS_SetPowerLevel_S(uint32_t u32PowerLevel)
{
    return (int)NSCBROKER_RUN(NSC_SYS_SetPowerLevel, u32PowerLevel);
}

int SYS_SetPowerRegulator_S(uint32_t u32PowerRegulator, uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_SYS_SetPowerRegulator, u32PowerRegulator,
                              output);
}

int SYS_SetSSRAMPowerMode_S(uint32_t u32SRAMSel, uint32_t u32PowerMode)
{
    return (int)NSCBROKER_RUN(NSC_SYS_SetSSRAMPowerMode, u32SRAMSel,
                              u32PowerMode);
}

int SYS_SetPSRAMPowerMode_S(uint32_t u32SRAMSel, uint32_t u32PowerMode)
{
    return (int)NSCBROKER_RUN(NSC_SYS_SetPSRAMPowerMode, u32SRAMSel,
                              u32PowerMode);
}

int SYS_SetVRef_S(uint32_t u32VRefCTL)
{
    return (int)NSCBROKER_RUN(NSC_SYS_SetVRef, u32VRefCTL);
}

// RTC
int RTC_SetClockSource_S(uint32_t u32ClkSrc, uint32_t *output)
{
    return (int)NSCBROKER_RUN(NSC_RTC_SetClockSource, u32ClkSrc, output);
}

// Register
int setRegister_S(uint32_t address, uint32_t value)
{
    // *(volatile uint32_t *)address = value;
    return (int)NSCBROKER_RUN(NSC_SET_REGISTER, address, value);
}

int getRegister_S(uint32_t address, uint32_t *output)
{
    // return *(volatile uint32_t *)address;
    return (int)NSCBROKER_RUN(NSC_GET_REGISTER, address, output);
}

int hardfault_set_mode(int mode)
{
    return (int)NSCBROKER_RUN(NSC_HARDFAULT_SET_MODE, mode);
}

int hardfault_set_func(hardfault_nonsecure_func_t func)
{
    return (int)NSCBROKER_RUN(NSC_HARDFAULT_SET_FUNC, func);
}

int hardfault_get_mode() { return (int)NSCBROKER_RUN(NSC_HARDFAULT_GET_MODE); }

int secureboot_status() { return (int)NSCBROKER_RUN(NSC_SECUREBOOT_STATUS); }

int secureboot_enable() { return (int)NSCBROKER_RUN(NSC_SECUREBOOT_ENABLE); }


int sdf_init(char *passwd){return (int)NSCBROKER_RUN(NSC_SDF_INIT, passwd);}
int sdf_read(int num, uint8_t *out, int out_len, uint16_t *rt_len){{return (int)NSCBROKER_RUN(NSC_SDF_READ, num, out, out_len, rt_len);}}
int sdf_write(int num, uint8_t *in, int in_len){{return (int)NSCBROKER_RUN(NSC_SDF_WRITE, num, in, in_len);}}

int axiocrypto_pqc_keypair(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *sk, size_t sklen)
{
    return (int)NSCBROKER_RUN(NSC_AXIOCRYPTO_PQC_KEYPAIR, algo, pk, pklen, sk, sklen);
}

int axiocrypto_pqc_sign_signature(pqc_algo_t algo, uint8_t *sk, size_t sklen, uint8_t *m, size_t mlen, uint8_t *sig, size_t *siglen)
{
    return (int)NSCBROKER_RUN(NSC_AXIOCRYPTO_PQC_SIGN_SIGNATURE, algo, sk, sklen, m, mlen, sig, siglen);
}

int axiocrypto_pqc_verify(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *m, size_t mlen, uint8_t *sig, size_t siglen)
{
    return (int)NSCBROKER_RUN(NSC_AXIOCRYPTO_PQC_VERIFY, algo, pk, pklen, m, mlen, sig, siglen);
}

int axiocrypto_pqc_sign(pqc_algo_t algo, uint8_t *sk, size_t sklen, uint8_t *m, size_t mlen, uint8_t *sm, size_t *smlen)
{
    return (int)NSCBROKER_RUN(NSC_AXIOCRYPTO_PQC_SIGN, algo, sk, sklen, m, mlen, sm, smlen);
}

int axiocrypto_pqc_open(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *sm, size_t smlen, uint8_t *m, size_t *mlen)
{
    return (int)NSCBROKER_RUN(NSC_AXIOCRYPTO_PQC_OPEN, algo, pk, pklen, sm, smlen, m, mlen);
}

int axiocrypto_pqc_encapsulate(pqc_algo_t algo, uint8_t *pk, size_t pklen, uint8_t *ct, size_t *ctlen, uint8_t *ss, size_t sslen)
{
    return (int)NSCBROKER_RUN(NSC_AXIOCRYPTO_PQC_ENCAPSULATE, algo, pk, pklen, ct, ctlen, ss, sslen);
}

int axiocrypto_pqc_decapsulate(pqc_algo_t algo, uint8_t *sk, size_t sklen, uint8_t *ct, size_t ctlen, uint8_t *ss, size_t *sslen)
{
    return (int)NSCBROKER_RUN(NSC_AXIOCRYPTO_PQC_DECAPSULATE, algo, sk, sklen, ct, ctlen, ss, sslen);
}

int secure_stack_set_init()
{
    return (int)NSCBROKER_RUN(NSC_SECURE_STACK_SET_INIT);
}
    
int secure_stack_get_info(uint32_t *totalStackSize, uint32_t *usedStackSize)
{
    return (int)NSCBROKER_RUN(NSC_SECURE_STACK_GET_INFO, totalStackSize, usedStackSize);
}

int secure_stack_test(int cnt)
{
    return (int)NSCBROKER_RUN(NSC_SECURE_STACK_TEST, cnt);
}

#endif