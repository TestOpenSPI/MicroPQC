#ifndef _PIN_DEF_H_
#define _PIN_DEF_H_

#include "NuMicro.h"

#if 0
#if defined(TRUSTZONE_NONSECURE)
	#define PERIPH_GPIO_DECLARE(port,pin) (((uint32_t)(port##_NS)) | pin)
#else
	#define PERIPH_GPIO_DECLARE(port,pin) (((uint32_t)(port##_S)) | pin)
#endif
#endif

#if defined(TRUSTZONE_SECURE) || defined(TRUSTZONE_NONSECURE)
#define PERIPH_GPIO_DECLARE_PA(pin) (((uint32_t)(PA_S)) | pin | ((SCU_INIT_IONSSET0_VAL&(1<<pin))?NS_OFFSET:0))
#define PERIPH_GPIO_DECLARE_PB(pin) (((uint32_t)(PB_S)) | pin | ((SCU_INIT_IONSSET1_VAL&(1<<pin))?NS_OFFSET:0))
#define PERIPH_GPIO_DECLARE_PC(pin) (((uint32_t)(PC_S)) | pin | ((SCU_INIT_IONSSET2_VAL&(1<<pin))?NS_OFFSET:0))
#define PERIPH_GPIO_DECLARE_PD(pin) (((uint32_t)(PD_S)) | pin | ((SCU_INIT_IONSSET3_VAL&(1<<pin))?NS_OFFSET:0))
#define PERIPH_GPIO_DECLARE_PE(pin) (((uint32_t)(PE_S)) | pin | ((SCU_INIT_IONSSET4_VAL&(1<<pin))?NS_OFFSET:0))
#define PERIPH_GPIO_DECLARE_PF(pin) (((uint32_t)(PF_S)) | pin | ((SCU_INIT_IONSSET5_VAL&(1<<pin))?NS_OFFSET:0))
#define PERIPH_GPIO_DECLARE_PG(pin) (((uint32_t)(PG_S)) | pin | ((SCU_INIT_IONSSET6_VAL&(1<<pin))?NS_OFFSET:0))
#define PERIPH_GPIO_DECLARE_PH(pin) (((uint32_t)(PH_S)) | pin | ((SCU_INIT_IONSSET7_VAL&(1<<pin))?NS_OFFSET:0))

#define PERIPH_GPIO_DECLARE(port,pin) PERIPH_GPIO_DECLARE_##port(pin)
#else
#define PERIPH_GPIO_DECLARE(port,pin) (((uint32_t)(port##_S)) | pin)
#endif

#define PERIPH_GPIO_IDX(gpio)         ((((uint32_t)(gpio)) & (0x1c0)) >> 6)
#define PERIPH_GPIO_PORT(gpio)        (((uint32_t)(gpio)) & (~0xf))
#define PERIPH_GPIO_PIN(gpio)         (((uint32_t)(gpio)) & (0xf))
#define PERIPH_GPIO_UNDEF             (0)

typedef uint32_t gpio_t;

typedef struct pin_af
{
	void *device;
	gpio_t gpio;
	int af;
} pin_af_t;

typedef struct mod_def
{
	void *device;
	uint32_t clkidx;
	uint32_t clksrc;
	uint32_t clkdiv;
	uint32_t rstidx;
	IRQn_Type irqn;
} mod_desc_t;

extern const pin_af_t pin_af_uart_rx[];
extern const pin_af_t pin_af_uart_tx[];
extern const pin_af_t pin_af_uart_rts[];
extern const pin_af_t pin_af_uart_cts[];

extern const pin_af_t pin_af_spi_mosi[];
extern const pin_af_t pin_af_spi_miso[];
extern const pin_af_t pin_af_spi_sclk[];
extern const pin_af_t pin_af_spi_ssel[];

extern const pin_af_t pin_af_qspi_mosi0[];
extern const pin_af_t pin_af_qspi_miso0[];
extern const pin_af_t pin_af_qspi_sclk[];
extern const pin_af_t pin_af_qspi_ssel[];

extern const pin_af_t pin_af_i2c_sda[];
extern const pin_af_t pin_af_i2c_scl[];

extern const pin_af_t pin_af_sc_pwr[];
extern const pin_af_t pin_af_sc_rst[];
extern const pin_af_t pin_af_sc_dat[];
extern const pin_af_t pin_af_sc_clk[];
extern const pin_af_t pin_af_sc_ncd[];

extern const pin_af_t pin_af_lcd_com0[];
extern const pin_af_t pin_af_lcd_com1[];
extern const pin_af_t pin_af_lcd_com2[];
extern const pin_af_t pin_af_lcd_com3[];
extern const pin_af_t pin_af_lcd_seg0[];
extern const pin_af_t pin_af_lcd_seg1[];
extern const pin_af_t pin_af_lcd_seg2[];
extern const pin_af_t pin_af_lcd_seg3[];
extern const pin_af_t pin_af_lcd_seg4[];
extern const pin_af_t pin_af_lcd_seg5[];
extern const pin_af_t pin_af_lcd_seg6[];
extern const pin_af_t pin_af_lcd_seg7[];
extern const pin_af_t pin_af_lcd_seg8[];
extern const pin_af_t pin_af_lcd_seg9[];
extern const pin_af_t pin_af_lcd_seg10[];
extern const pin_af_t pin_af_lcd_seg11[];
extern const pin_af_t pin_af_lcd_seg12[];
extern const pin_af_t pin_af_lcd_seg13[];
extern const pin_af_t pin_af_lcd_seg14[];
extern const pin_af_t pin_af_lcd_seg15[];
extern const pin_af_t pin_af_lcd_seg16[];
extern const pin_af_t pin_af_lcd_seg17[];
extern const pin_af_t pin_af_lcd_seg18[];
extern const pin_af_t pin_af_lcd_seg19[];
extern const pin_af_t pin_af_lcd_seg20[];
extern const pin_af_t pin_af_lcd_seg21[];
extern const pin_af_t pin_af_lcd_seg22[];
extern const pin_af_t pin_af_lcd_seg23[];
extern const pin_af_t pin_af_lcd_seg24[];
extern const pin_af_t pin_af_can_rx[];
extern const pin_af_t pin_af_can_tx[];
extern const pin_af_t pin_af_eadc[];
extern const pin_af_t pin_af_usbd[];

/* TODO: sd, adc, ... */

extern const mod_desc_t modtbl_uart[];
extern const mod_desc_t modtbl_spi[];
extern const mod_desc_t modtbl_qspi[];
extern const mod_desc_t modtbl_i2c[];
extern const mod_desc_t modtbl_sc[];
extern const mod_desc_t modtbl_timer[];
extern const mod_desc_t modtbl_lcd[];
extern const mod_desc_t modtbl_can[];
extern const mod_desc_t modtbl_usbd_pll[];


#define MAX_MOD_UART   6
#define MAX_MOD_SPI    4
#define MAX_MOD_QSPI   1
#define MAX_MOD_I2C    3
#define MAX_MOD_SC     3
#define MAX_MOD_TIMER  6
#define MAX_MOD_LCD    1
#define MAX_MOD_CAN	   1
#define MAX_MOD_ADC    4
#define MAX_MOD_USBD   1

/* TODO: sd, adc, ... */

int find_pin_af(void *dev, gpio_t gpio, const pin_af_t tb[]);
const mod_desc_t *find_mod_desc(void *dev, const mod_desc_t tb[]);

#if 0
/*
#define _MFP(periph,port,pin,afname) \
{ periph, M2351_GPIO_DECLARE(port,pin), periph##_##afname##_##port##pin }
*/

#if defined(TRUSTZONE_NONSECURE)
	#define _MFP(periph,port,pin,afname) \
	{ periph,  (((uint32_t)(port##_NS)) | pin), periph##_##afname##_##port##pin }
#else
	#define _MFP(periph,port,pin,afname) \
	{ periph,  (((uint32_t)(port##_S)) | pin), periph##_##afname##_##port##pin }
#endif
#endif
#if defined(TRUSTZONE_SECURE) || defined(TRUSTZONE_NONSECURE)
	#define _MFP(periph,port,pin,afname) \
	{ periph, PERIPH_GPIO_DECLARE_##port(pin), periph##_##afname##_##port##pin }
#else
	#define _MFP(periph,port,pin,afname) \
	{ periph, (((uint32_t)(port##_S)) | pin), periph##_##afname##_##port##pin }
#endif



#define _MFP_NONE \
{ NULL, 0, 0 }

#endif

