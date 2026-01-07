#ifndef _NSC_API_H_
#define _NSC_API_H_
#if defined(TRUSTZONE_NONSECURE)

#include <time.h>
#include "NuMicro.h"
#include "nsc_def.h"

typedef enum
{
    RTC_CLOCK_INTERNAL = 0,
    RTC_CLOCK_EXTERNAL,
} rtc_clock_type_t;

int rtc_set_clock(rtc_clock_type_t rtc_clock_type);
int rtc_get(struct tm *t);
int rtc_set(struct tm *t);

/**
 * @brief @~Korean 칩을 리셋합니다.
 */
int reset_chip(void);


// power down mode
typedef enum{
	PDM_WAKEUP_NORMAL = 0,
	PDM_WAKEUP_RTC,
	PDM_WAKEUP_TIMER,
	PDM_WAKEUP_PIN,
	PDM_WAKEUP_UNKNOWN,
}pm_status_t;

typedef struct{
    uint32_t mode;
    S_RTC_TIME_DATA_T rtc;
    uint32_t wakeup[3];    //gpio_id_t
}pdm_conf_t;

typedef enum{
	PM_MODE_DP = 0,
	PM_MODE_LLPD = 1,
	PM_MODE_FWPD = 2,
	PM_MODE_ULLPD = 3,
	PM_MODE_SPD = 4,
	PM_MODE_DPD = 6,
}pm_mode_t;

typedef enum{
	WP_MODE_DISABLE = 0,
	WP_MODE_RISING = 1,
	WP_MODE_FALLING = 2,
	WP_MODE_BOTHEDGE = 3,
}pm_wakeup_pin_mode_t;

int pm_set_env(void);
pm_status_t pm_get_wakeup(int *wakeup_pin_id);
int pm_set_wakeup_pin(int id, pm_wakeup_pin_mode_t wp_mode);
int pm_set_wakeup_rtc(struct tm *t);
int pm_powerdown(pm_mode_t mode);

int mcu_wdt_init(void);
int mcu_wdt_fini(void);

int fwupdate_begin(uint8_t *data, uint32_t size, uint32_t total_size);
int fwupdate_next(uint8_t *data, uint32_t size);


int SYS_ResetModule_S(uint32_t u32ModuleIndex);
int CLK_SetModuleClock_S(uint32_t u32ModuleIdx, uint32_t u32ClkSrc,
                         uint32_t u32ClkDiv);
int CLK_EnableModuleClock_S(uint32_t u32ModuleIdx);
int CLK_DisableModuleClock_S(uint32_t u32ModuleIdx);
int pin_function_s(int32_t port, int32_t pin, int32_t data);
int I2C_Close_S(I2C_T *i2c);

typedef __NONSECURE_CALL uint32_t (*hardfault_nonsecure_func_t)(uint32_t *);

#define HARDFAULT_MODE_HOLD 0
#define HARDFAULT_MODE_RESET 1
#define HARDFAULT_MODE_FUNC 2

/**
 * @brief @~Korean hardfault가 발생한 경우 처리 방법을 설정합니다.
 * @param[in] mode HARDFAULT_MODE_HOLD 	: 대기로 설정
 *                 HARDFAULT_MODE_RESET : 리셋으로 설정
 *                 HARDFAULT_MODE_FUNC 	: nonsecure fuction을 호출하도록 설정
 */
int hardfault_set_mode(int mode);

/**
 * @brief @~Korean  hardfault가 발생한 경우 호출할 nonsecure func을 설정합니다.
 * @param[in] func nonsecure fuction 포인터
 */
int hardfault_set_func(hardfault_nonsecure_func_t func);

/**
 * @brief @~Korean hardfault가 발생한 경우 현재 설정값을 가져옵니다.
 */
int hardfault_get_mode();

/**
 * @brief @~Korean 칩을 리셋합니다.
 */
int reset_chip(void);

/**
 * @brief @~Korean 디버그모듈을 이용해서 flash의 사용을 못하도록 설정합니다.
 * <br> 리셋후에 반영됩니다.
 * @param[in] is_all 0 : Lock APROM region(disable to read APROM region via ICE)
 *                   1 : Lock all region(disable to read all region via ICE)
 */
int scr_lock(int is_all);

/**
 * @brief @~Korean 디버그모듈을 이용해서 flash의 접근 가능여부를 확인합니다.
 *
 * @retval  0  Unlocked
 * @retval  1  APROM region locked
 * @retval  2  All region locked
 * @retval  3  APROM and all region locked
 */
int scr_is_lock();

int usb_select_mode_s(int mode);

int setRegister_S(uint32_t address, uint32_t value);
int getRegister_S(uint32_t address, uint32_t *value);

// CLK M2354 library
int CLK_DisableCKO_S(void);
int CLK_EnableCKO_S(uint32_t u32ClkSrc, uint32_t u32ClkDiv,
                    uint32_t u32ClkDivBy1En);
int CLK_PowerDown_S(void);
int CLK_Idle_S(void);
int CLK_GetHXTFreq_S(uint32_t *output);
int CLK_GetLXTFreq_S(uint32_t *output);
int CLK_GetHCLKFreq_S(uint32_t *output);
int CLK_GetPCLK0Freq_S(uint32_t *output);
int CLK_GetPCLK1Freq_S(uint32_t *output);
int CLK_GetCPUFreq_S(uint32_t *output);
int CLK_SetCoreClock_S(uint32_t u32Hclk, uint32_t *output);
int CLK_SetHCLK_S(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
int CLK_SetSysTickClockSrc_S(uint32_t u32ClkSrc);
int CLK_EnableXtalRC_S(uint32_t u32ClkMask);
int CLK_DisableXtalRC_S(uint32_t u32ClkMask);
int CLK_EnablePLL_S(uint32_t u32PllClkSrc, uint32_t u32PllFreq,
                    uint32_t *output);
int CLK_DisablePLL_S(void);
int CLK_WaitClockReady_S(uint32_t u32ClkMask, uint32_t *output);
int CLK_EnableSysTick_S(uint32_t u32ClkSrc, uint32_t u32Count);
int CLK_DisableSysTick_S(void);
int CLK_SetPowerDownMode_S(uint32_t u32PDMode);
int CLK_EnableDPDWKPin_S(uint32_t u32TriggerType);
int CLK_GetPMUWKSrc_S(uint32_t *output);
int CLK_EnableSPDWKPin_S(uint32_t u32Port, uint32_t u32Pin,
                         uint32_t u32TriggerType, uint32_t u32DebounceEn);
int CLK_GetPLLClockFreq_S(uint32_t *output);

// SYS M2354 library
int SYS_ClearResetSrc_S(uint32_t u32Src);
int SYS_GetBODStatus_S(uint32_t *output);
int SYS_GetResetSrc_S(uint32_t *output);
int SYS_IsRegLocked_S(uint32_t *output);
int SYS_ReadPDID_S(uint32_t *output);
int SYS_SetPowerLevel_S(uint32_t u32PowerLevel);
int SYS_SetPowerRegulator_S(uint32_t u32PowerRegulator, uint32_t *output);
int SYS_SetSSRAMPowerMode_S(uint32_t u32SRAMSel, uint32_t u32PowerMode);
int SYS_SetPSRAMPowerMode_S(uint32_t u32SRAMSel, uint32_t u32PowerMode);
int SYS_SetVRef_S(uint32_t u32VRefCTL);
int RTC_SetClockSource_S(uint32_t u32ClkSrc, uint32_t *output);

#define SECUREBOOT_DISABLE 0
#define SECUREBOOT_ENABLE 1
int secureboot_status();
int secureboot_enable();

// SDF(secure stroage flash data)
int sdf_init(char *passwd);
int sdf_read(int num, uint8_t *out, int out_len, uint16_t *rt_len);
int sdf_write(int num, uint8_t *in, int in_len);

int secure_stack_set_init();
int secure_stack_get_info(uint32_t *totalStackSize, uint32_t *usedStackSize);
int secure_stack_test(int cnt);

#endif
#endif


