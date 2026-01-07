// clang-format off
/* sungmin, 2023-11-06 more information in periph_api.h 402 line*/
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "NuMicro.h" /* Device header */
#include "periph_def.h"
#include "periph_api.h"
#include "periph_conf.h"
#if defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#elif defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif
#include "tsrb.h"
#include "lcdlib.h"
#include <stdlib.h>
#if defined(MODULE_PRINTF)
#include "printf.h"
#endif
#if !defined(TRUSTZONE_NONSECURE) && defined(MODULE_ENTROPYHEALTH)
#include "entropyhealth.h"
#endif

extern const int numof_GPIO;
extern const int numof_UART;
extern const int numof_SPI;
extern const int numof_I2C;
extern const int numof_SC;
extern const int numof_TIMER;
extern const int numof_CAN;
extern const int numof_ADC;
extern const int numof_USBD;

#define EXCEED_GPIO_IDX(idx) ((idx) < 0 || (idx >= numof_GPIO))
#define EXCEED_UART_IDX(idx) EXCEED_MOD_IDX(UART, idx)
#define EXCEED_SPI_IDX(idx) EXCEED_MOD_IDX(SPI, idx)
#define EXCEED_I2C_IDX(idx) EXCEED_MOD_IDX(I2C, idx)
#define EXCEED_SC_IDX(idx) EXCEED_MOD_IDX(SC, idx)
#define EXCEED_CAN_IDX(idx) EXCEED_MOD_IDX(CAN, idx)
#define EXCEED_ADC_IDX(idx) EXCEED_MOD_IDX(ADC, idx)
#define EXCEED_USBD_IDX(idx) EXCEED_MOD_IDX(USBD, idx)
#define EXCEED_MOD_IDX(periph, idx) \
    ((idx) < 0 || (idx >= MAX_MOD_##periph) || (idx >= numof_##periph))

#if defined(OS_CMSIS_RTX)
__attribute__((unused)) static osMutexId_t spi_lock[MAX_MOD_SPI] = {
    NULL,
};
__attribute__((unused)) static osMutexId_t i2c_lock[MAX_MOD_I2C] = {
    NULL,
};

static inline int _create_periph_lock(osMutexId_t *lock)
{
    if ((*lock) != NULL)
        return -1;

    *lock = osMutexNew(NULL);
    if ((*lock) == NULL)
        return -1;

    return 0;
}

static inline void _destroy_periph_lock(osMutexId_t *lock)
{
    if ((*lock) != NULL)
    {
        osMutexDelete(*lock);
        *lock = NULL;
    }
}

static inline int _acquire_periph_lock(osMutexId_t *lock, uint32_t timeout)
{
    if (osMutexAcquire(*lock, (timeout != 0xFFFFFFFF) ? timeout : 0) != osOK)
        return -1;
    return 0;
}

static inline int _release_periph_lock(osMutexId_t *lock)
{
    if (osMutexRelease(*lock) != osOK)
        return -1;
    return 0;
}
#elif defined(OS_FREERTOS)
__attribute__((unused)) static SemaphoreHandle_t spi_lock[MAX_MOD_SPI] = {
    NULL,
};
__attribute__((unused)) static SemaphoreHandle_t i2c_lock[MAX_MOD_I2C] = {
    NULL,
};

__STATIC_INLINE int _create_periph_lock(SemaphoreHandle_t *lock)
{
    if ((*lock) != NULL)
        return -1;

    *lock = xSemaphoreCreateMutex();
    if ((*lock) == NULL)
        return -1;

    return 0;
}

__STATIC_INLINE void _destroy_periph_lock(SemaphoreHandle_t *lock)
{
    if ((*lock) != NULL)
    {
        vSemaphoreDelete(*lock);
        *lock = NULL;
    }
}

__STATIC_INLINE int _acquire_periph_lock(SemaphoreHandle_t *lock,
                                         TickType_t timeout)
{
    if (xSemaphoreTake(
            *lock, (timeout != 0xFFFFFFFF) ? timeout : portMAX_DELAY) != pdPASS)
        return -1;
    return 0;
}

__STATIC_INLINE int _release_periph_lock(SemaphoreHandle_t *lock)
{
    if (xSemaphoreGive(*lock) != pdPASS)
        return -1;
    return 0;
}
#else
__attribute__((unused)) static int spi_lock[MAX_MOD_SPI] = {
    0,
};
__attribute__((unused)) static int i2c_lock[MAX_MOD_I2C] = {
    0,
};

static inline int _create_periph_lock(int *lock)
{
    *lock = 0;
    return 0;
}

static inline void _destroy_periph_lock(int *lock) { (void)lock; }

static inline int _acquire_periph_lock(int *lock, uint32_t timeout)
{
    (void)timeout;
    while (*lock)
        ;
    *lock = 1;
    return 0;
}

static inline int _release_periph_lock(int *lock)
{
    *lock = 0;
    return 0;
}
#endif

int uart_irq_setup(uart_id_t idx, uart_irq_type_t type, int priority,
                   void (*isr)(void))
{
    const uart_conf_t *conf = &uart_config[idx];
    UART_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_uart);
    if (!mod)
    {
        return -1;
    }

    (void)priority;
    NVIC_SetVector(mod->irqn, (uint32_t)isr);
    NVIC_EnableIRQ(mod->irqn);

    if (type == UART_IRQ_RX)
        UART_EnableInt(dev, UART_INTEN_RDAIEN_Msk);

    return 0;
}

typedef enum
{
    PARITY_NONE = 0,
    PARITY_ODD = 1,
    PARITY_EVEN = 2,
    PARITY_FORCED1 = 3,
    PARITY_FORCED0 = 4,
    PARITY_MARK = 5,
    PARITY_SPACE = 6
} serial_parity_t;

int uart_line_config(uart_id_t idx, int data_bits, int parity, int stop_bits)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    UART_T *uart = uart_config[idx].dev;

    // Flush Tx FIFO.
    while (!UART_IS_TX_EMPTY(uart))
    {
    };

    // check parameters
    // 1. data_bits
    if ((data_bits < 5) || (data_bits > 8))
    {
        // printf("ERR] data_bits of UARD_DEV(%d) should be between 5 and 8 in
        // uart_line_config.\n", idx);
        return -2;
    }

    // 2. parity
    if ((parity != PARITY_NONE) && (parity != PARITY_ODD) &&
        (parity != PARITY_EVEN) && (parity != PARITY_FORCED1) &&
        (parity != PARITY_FORCED0) && (parity != PARITY_MARK) &&
        (parity != PARITY_SPACE))
    {
        // printf("ERR] parity of UARD_DEV(%d) is invalid in
        // uart_line_config.\n", idx);
        return -3;
    }

    // 3. stop_bits
    if ((stop_bits != 1) && (stop_bits != 2))
    {
        // printf("ERR] stop_bits of UARD_DEV(%d) is invalid in
        // uart_line_config.\n", idx);
        return -4;
    }

    uint32_t databits_intern = (data_bits == 5)   ? UART_WORD_LEN_5
                               : (data_bits == 6) ? UART_WORD_LEN_6
                               : (data_bits == 7) ? UART_WORD_LEN_7
                                                  : UART_WORD_LEN_8;
    uint32_t parity_intern =
        (parity == PARITY_ODD || parity == PARITY_FORCED1)    ? UART_PARITY_ODD
        : (parity == PARITY_EVEN || parity == PARITY_FORCED0) ? UART_PARITY_EVEN
        : (parity == PARITY_MARK)                             ? UART_PARITY_MARK
        : (parity == PARITY_SPACE) ? UART_PARITY_SPACE
                                   : UART_PARITY_NONE;
    uint32_t stopbits_intern =
        (stop_bits == 2) ? UART_STOP_BIT_2 : UART_STOP_BIT_1;

    UART_SetLineConfig(uart,
                       0,  // Don't change baudrate
                       databits_intern, parity_intern, stopbits_intern);

    return 0;
}

void uart_fini(uart_id_t idx)
{
    if (EXCEED_UART_IDX(idx))
        return;

    const uart_conf_t *conf = &uart_config[idx];
    UART_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_uart);
    if (!mod)
        return;

    NVIC_EnableIRQ(mod->irqn);
    UART_Close(dev);
    CLK_DisableModuleClock_S(mod->clkidx);
}

int uart_init(uart_id_t idx, int baudrate)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    const uart_conf_t *conf = &uart_config[idx];
    UART_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_uart);
    if (!mod)
    {
        return -1;
    }

    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);
    CLK_EnableModuleClock_S(mod->clkidx);
    SYS_ResetModule_S(mod->rstidx);

    pin_function_s(PERIPH_GPIO_IDX(conf->rx_pin), PERIPH_GPIO_PIN(conf->rx_pin),
                   find_pin_af(dev, conf->rx_pin, pin_af_uart_rx));

    pin_function_s(PERIPH_GPIO_IDX(conf->tx_pin), PERIPH_GPIO_PIN(conf->tx_pin),
                   find_pin_af(dev, conf->tx_pin, pin_af_uart_tx));

    if (conf->rts_pin != PERIPH_GPIO_UNDEF)
    {
        pin_function_s(PERIPH_GPIO_IDX(conf->rts_pin),
                       PERIPH_GPIO_PIN(conf->rts_pin),
                       find_pin_af(dev, conf->rts_pin, pin_af_uart_rts));
    }
    if (conf->cts_pin != PERIPH_GPIO_UNDEF)
    {
        pin_function_s(PERIPH_GPIO_IDX(conf->cts_pin),
                       PERIPH_GPIO_PIN(conf->cts_pin),
                       find_pin_af(dev, conf->cts_pin, pin_af_uart_cts));
    }

    if (baudrate == -1)
        UART_Open(dev, uart_config[idx].baudrate);
    else
        UART_Open(dev, baudrate);
    
    if (conf->type == UART_TYPE_485TRANCEIVER){
		gpio_init(conf->gpio_dir_id);
		gpio_write(conf->gpio_dir_id, 0);
	}

#if !defined(LOADER_ABMV2)
    if (conf->type == UART_TYPE_485MASTER)
    {
        /* Set RS485-Master as AUD mode */
        /* Enable AUD mode to HW control RTS pin automatically */
        /* You also can use GPIO to control RTS pin for replacing AUD mode */
        UART_SelectRS485Mode(dev, UART_ALTCTL_RS485AUD_Msk, 0);

        /* Set RTS pin active level as high level active */
        dev->MODEM = (dev->MODEM & (~UART_MODEM_RTSACTLV_Msk)) |
                     UART_RTS_IS_HIGH_LEV_ACTIVE;
    }
    else if ((conf->type == UART_TYPE_485SLAVE_AAD) ||
             (conf->type == UART_TYPE_485SLAVE_NMM))
    {
        uart_line_config(idx, 8, PARITY_EVEN, 1);

        /* Set RX Trigger Level as 1 byte */
        dev->FIFO =
            (dev->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;

        /* Set RTS pin active level as high level active */
        dev->MODEM = (dev->MODEM & (~UART_MODEM_RTSACTLV_Msk)) |
                     UART_RTS_IS_HIGH_LEV_ACTIVE;

        if (conf->type == UART_TYPE_485SLAVE_NMM)
        {
            /* Set Receiver disabled before set RS485-NMM mode */
            dev->FIFO |= UART_FIFO_RXOFF_Msk;

            /* Set RS485-NMM Mode */
            UART_SelectRS485Mode(
                dev, UART_ALTCTL_RS485NMM_Msk | UART_ALTCTL_RS485AUD_Msk, 0);
        }
        else
        {
            /* Set RS485-AAD Mode and address match is 0xC0 */
            UART_SelectRS485Mode(
                dev, UART_ALTCTL_RS485AAD_Msk | UART_ALTCTL_RS485AUD_Msk,
                (uint8_t)conf->slaveaddr);
        }

        /* Set RS485 address detection enable */
        dev->ALTCTL |= UART_ALTCTL_ADDRDEN_Msk;
    }
#endif

    return 0;
}

int uart_write(uart_id_t idx, const unsigned char *buf, unsigned int length)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    UART_T *dev = uart_config[idx].dev;
#if !defined(LOADER_ABMV2)
    int uart_type = uart_config[idx].type;

    if (uart_type >= UART_TYPE_485MASTER && uart_type <= UART_TYPE_485SLAVE_NMM)
    {
        if (length > 1)
        {
            uart_line_config(idx, 8, PARITY_MARK, 1);
            UART_Write(dev, (uint8_t *)buf, 1);

            uart_line_config(idx, 8, PARITY_SPACE, 1);
            UART_Write(dev, (uint8_t *)buf + 1, length - 1);

            return length;
        }
        return 0;
    }
#endif

    if (uart_config[idx].type == UART_TYPE_485TRANCEIVER){
		uint32_t ret;
		gpio_write(uart_config[idx].gpio_dir_id, 1);
		ret = UART_Write(dev, (uint8_t *)buf, length);
		UART_WAIT_TX_EMPTY(uart_config[idx].dev);
		gpio_write(uart_config[idx].gpio_dir_id, 0);
		return  ret;
	}
	else{
		return UART_Write(dev, (uint8_t *)buf, length);
	}
}

int uart_read(uart_id_t idx, unsigned char *buf, unsigned int length)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    return UART_Read(uart_config[idx].dev, buf, length);
}

int uart_close(uart_id_t idx)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    const mod_desc_t *mod = find_mod_desc(uart_config[idx].dev, modtbl_uart);
    if (!mod)
        return -1;

    UART_Close(mod->device);
    NVIC_DisableIRQ(mod->irqn);
    // CLK_DisableModuleClock_S(mod->clkidx);

    return 0;
}

int uart_readable(uart_id_t idx)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    UART_T *uart = uart_config[idx].dev;

    return !UART_GET_RX_EMPTY(uart);
}

int uart_writable(uart_id_t idx)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    UART_T *uart = uart_config[idx].dev;

    return !UART_IS_TX_FULL(uart);
}

int uart_set_flowcontrol(uart_id_t idx)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    const uart_conf_t *conf = &uart_config[idx];
    UART_T *dev = conf->dev;

    if (conf->cts_pin == PERIPH_GPIO_UNDEF ||
        conf->rts_pin == PERIPH_GPIO_UNDEF)
    {
        return -2;
    }

    UART_EnableFlowCtrl(dev);

    return 0;
}

int uart_set_rts(uart_id_t idx, int val)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    const uart_conf_t *conf = &uart_config[idx];
    UART_T *dev = conf->dev;

    if (conf->rts_pin == PERIPH_GPIO_UNDEF)
    {
        return -2;
    }

    if (val == 0)
    {
        UART_CLEAR_RTS(dev);
    }
    else
    {
        UART_SET_RTS(dev);
    }

    return 0;
}

int uart_rs485_write(uart_id_t idx, uint8_t *addr, const unsigned char *buf,
                     unsigned int length)
{
    if (EXCEED_UART_IDX(idx))
        return -1;

    /* Write address */
    uart_line_config(idx, 8, PARITY_MARK, 1);
    uart_write(idx, addr, 1);

    /* Send data */
    uart_line_config(idx, 8, PARITY_SPACE, 1);
    uart_write(idx, buf, length);

    return 0;
}

int timer_init(timer_id_t idx)
{
    if (idx > numof_TIMER)
        return -1;

    const timer_conf_t *conf = &timer_config[idx];
    TIMER_T *timer = conf->dev;

    const mod_desc_t *mod = find_mod_desc(timer, modtbl_timer);
    if (!mod)
    {
        return -1;
    }

    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);
    CLK_EnableModuleClock_S(mod->clkidx);
    SYS_ResetModule_S(mod->rstidx);

    return 0;
}

void timer_fini(timer_id_t idx)
{
    if (idx > numof_TIMER)
        return;

    const timer_conf_t *conf = &timer_config[idx];
    TIMER_T *timer = conf->dev;

    const mod_desc_t *mod = find_mod_desc(timer, modtbl_timer);
    if (!mod)
        return;

    NVIC_DisableIRQ(mod->irqn);
    TIMER_Close(timer);
    CLK_DisableModuleClock_S(mod->clkidx);
}

int timer_irq_setup(timer_id_t idx, int priority, void (*isr)(void))
{
    if (idx > numof_TIMER)
        return -1;

    const timer_conf_t *conf = &timer_config[idx];
    TIMER_T *timer = conf->dev;

    const mod_desc_t *mod = find_mod_desc(timer, modtbl_timer);
    if (!mod)
    {
        return -1;
    }

    (void)priority;
    if (isr != NULL)
    {
        NVIC_SetVector(mod->irqn, (uint32_t)isr);
    }
    NVIC_EnableIRQ(mod->irqn);

    TIMER_EnableInt(timer);

    return 0;
}

int timer_start(timer_id_t idx, timer_mode_t mode, uint32_t freq)
{
    if (idx > numof_TIMER)
        return -1;

    const timer_conf_t *conf = &timer_config[idx];
    TIMER_T *timer = conf->dev;

    const mod_desc_t *mod = find_mod_desc(timer, modtbl_timer);
    if (!mod)
    {
        return -1;
    }

    if (mode == TIMER_MODE_ONESHOT)
    {
        TIMER_Open(timer, TIMER_ONESHOT_MODE, freq);
    }
    else if (mode == TIMER_MODE_CONTINUOUS)
    {
        TIMER_Open(timer, TIMER_CONTINUOUS_MODE, freq);
    }
    else if (mode == TIMER_MODE_PERIODIC)
    {
        TIMER_Open(timer, TIMER_PERIODIC_MODE, freq);
    }
    else
    {
        return -1;
    }

    if (NVIC_GetEnableIRQ(mod->irqn))
        TIMER_EnableInt(timer);

    TIMER_Start(timer);

    return 0;
}

int timer_stop(timer_id_t idx)
{
    if (idx > numof_TIMER)
        return -1;

    const timer_conf_t *conf = &timer_config[idx];
    TIMER_T *timer = conf->dev;

    TIMER_Stop(timer);

    return 0;
}

int timer_read_counter(timer_id_t idx, uint32_t *count)
{
    if (idx > numof_TIMER)
        return -1;

    const timer_conf_t *conf = &timer_config[idx];
    TIMER_T *timer = conf->dev;
    *count = timer->CNT;

    return 0;
}

#if !defined(TRUSTZONE_NONSECURE)

#if defined(HW_TRNG)
static char trng_buf[128]; /* must be power of 2 */
static tsrb_t trng_rb;
#endif

#if defined(HW_TRNG)
static inline void trng_enable(void)
{
    TRNG->CTL |= TRNG_CTL_TRNGEN_Msk;  // Enable
    TRNG->CTL |= TRNG_CTL_DVIEN_Msk;

    NVIC_EnableIRQ(TRNG_IRQn);
}

static inline void trng_disable(void)
{
    TRNG->CTL &= ~TRNG_CTL_TRNGEN_Msk;  // Enable
}

void TRNG_IRQHandler(void)
{
    uint8_t r;
    if (TRNG->CTL & TRNG_CTL_DVIF_Msk)
    {
        r = TRNG->DATA & TRNG_DATA_DATA_Msk;
#if defined(MODULE_ENTROPYHEALTH)
        if (entropy_health_test(&trng_rb, r) < 0)
        {
            return;
        }
#endif
        if (tsrb_add_one(&trng_rb, r))
        {
            /* disable trng */
            trng_disable();
        }
    }
}
#endif

int trng_fini(void)
{
#if defined(HW_TRNG)
    TRNG->CTL &= ~TRNG_CTL_TRNGEN_Msk;  // disable gen
    TRNG->CTL &= ~TRNG_CTL_DVIEN_Msk;   // disable interrupt
    TRNG->ACT &= ~TRNG_ACT_ACT_Msk;     // deactivation
#endif
    return 0;
}

int trng_init(void)
{
#ifdef HW_TRNG
    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;
    CLK->APBCLK1 |= CLK_APBCLK1_TRNGCKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

#if 1  // def USE_LIRC_AS_TRNG_SOURCE
    RTC->LXTCTL |=
        RTC_LXTCTL_RTCCKSEL_Msk | RTC_LXTCTL_LIRC32KEN_Msk;  // Enable LIRC32K
#else
    RTC->LXTCTL &= ~(RTC_LXTCTL_RTCCKSEL_Msk | RTC_LXTCTL_LIRC32KEN_Msk |
                     RTC_LXTCTL_C32KSEL_Msk);  // Enable LXT
#endif

    TRNG->ACT |= TRNG_ACT_ACT_Msk;  // activation
    // TRNG->CTL = 0x4; // set CLKP

    // while((TRNG->CTL & TRNG_CTL_READY_Msk) == 0);

    tsrb_init(&trng_rb, trng_buf, sizeof(trng_buf));

#if defined(MODULE_ENTROPYHEALTH)
    entropy_health_test_init();
#endif
    trng_enable();
#else
    srand(1234);
#endif
    return 0;
}

int trng_random(unsigned char *output, size_t len, size_t *olen)
{
#if defined(HW_TRNG)
    size_t remain = len;

#if defined(MODULE_ENTROPYHEALTH)
    if (entropy_health_get_status(&trng_rb) == TRNG_STATUS_IN_STARTUP_TEST)
    {
        return 0;
    }
#endif
    while (remain > 0)
    {
        remain -= tsrb_get(&trng_rb, (char *)(output + len - remain), remain);
    }
    *olen = len;

    trng_enable();
    return 0;
#else
    unsigned int remain = len;
    unsigned char *p = output;
    int r;

    while (remain)
    {
        r = rand();
        if (remain < sizeof(int))
        {
            memcpy(p, &r, remain);
            break;
        }

        memcpy(p, &r, sizeof(int));
        remain -= sizeof(int);
        p += sizeof(int);
    }

    *olen = len;
    return 0;
#endif
}
#endif

#if !defined(TRUSTZONE_NONSECURE)

int crpt_init(int priority, void (*isr)(void))
{
    CLK_EnableModuleClock(CRPT_MODULE);

    ECC_ENABLE_INT(CRPT);
    AES_ENABLE_INT(CRPT);
    SHA_ENABLE_INT(CRPT);

    if (isr)
    {
        (void)priority;
        NVIC_SetVector(CRPT_IRQn, (uint32_t)isr);
        NVIC_EnableIRQ(CRPT_IRQn);
    }

    return 0;
}

#endif

static void __gpio_write(uint32_t port, uint32_t pin, int value)
{
    uint32_t ctl =
        GPIO_PIN_DATA_BASE + (((uint32_t)port) & 0x100001c0) + (pin << 2);
    *(volatile uint32_t *)ctl = value;
}

static void __gpio_toggle(uint32_t port, uint32_t pin)
{
    uint32_t ctl =
        GPIO_PIN_DATA_BASE + (((uint32_t)port) & 0x100001c0) + (pin << 2);
    *(volatile uint32_t *)ctl ^= 1;
}

static int __gpio_read(uint32_t port, uint32_t pin)
{
    uint32_t ctl =
        GPIO_PIN_DATA_BASE + (((uint32_t)port) & 0x100001c0) + (pin << 2);
    return *(volatile uint32_t *)ctl;
}

int gpio_init(gpio_id_t idx)
{
    if (EXCEED_GPIO_IDX(idx))
        return -1;

    const gpio_conf_t *cfg = &gpio_config[idx];

    uint32_t port = PERIPH_GPIO_PORT(cfg->gpio);
    uint32_t pin = PERIPH_GPIO_PIN(cfg->gpio);

    __gpio_write(port, pin, (cfg->invert) ? (!cfg->value) : (!!cfg->value));

    GPIO_SetMode((GPIO_T *)port, 1 << pin, cfg->mode);

    return 0;
}

typedef void (*gpio_isr_t)(void);
static gpio_isr_t gpio_isr[8][16] = {
    {
        NULL,
    },
}; /* 512 byte */

static void gpio_irq_handler(void)
{
    int i, j;
    int k;
    uint32_t flag;
    GPIO_T *gpio[8] = {PA, PB, PC, PD, PE, PF, PG, PH};

    k = 0;

    for (i = 0; i < 8; i++)
    {
        flag = gpio[i]->INTSRC;
        for (j = 0; j < 16; j++)
        {
            if ((flag & (1 << j)) && gpio_isr[i][j])
            {
                gpio_isr[i][j]();
                k++;
            }
        }
        gpio[i]->INTSRC = flag;
    }
    if (!k)
        printf("unexpected interrupt occured!\n");
}

int gpio_irq_setup(gpio_id_t idx, gpio_intr_type_t type, int priority,
                   void (*isr)(void))
{
    if (EXCEED_GPIO_IDX(idx))
        return -1;

    const gpio_conf_t *cfg = &gpio_config[idx];

    uint32_t attr;
    uint32_t port = PERIPH_GPIO_PORT(cfg->gpio);
    uint32_t pin = PERIPH_GPIO_PIN(cfg->gpio);
    uint32_t port_idx = PERIPH_GPIO_IDX(cfg->gpio);

    int irqn[] = {GPA_IRQn, GPB_IRQn, GPC_IRQn, GPD_IRQn,
                  GPE_IRQn, GPF_IRQn, GPG_IRQn, GPH_IRQn};

    if (port_idx > 7)
        return -1;

    /* irq mode implies input mode */
    GPIO_SetMode((GPIO_T *)port, 1 << pin, 0);

    gpio_isr[port_idx][pin] = isr;

    /* gpio_intr_type is same as M23xx BSP GPIO_INT_xxxx */

    switch (type)
    {
    case GPIO_INTR_TYPE_RISING:
        attr = GPIO_INT_RISING;
        break;
    case GPIO_INTR_TYPE_FALLING:
        attr = GPIO_INT_FALLING;
        break;
    case GPIO_INTR_TYPE_BOTH:
        attr = GPIO_INT_BOTH_EDGE;
        break;

    default:
        return -2;
    }

    GPIO_EnableInt((GPIO_T *)port, pin, attr);

    NVIC_SetVector(irqn[port_idx], (uint32_t)gpio_irq_handler);
    NVIC_EnableIRQ(irqn[port_idx]);
    NVIC_SetPriority(irqn[port_idx], priority);

    return 0;
}

int gpio_write(gpio_id_t idx, int value)
{
    if (EXCEED_GPIO_IDX(idx))
        return -1;

    const gpio_conf_t *cfg = &gpio_config[idx];

    uint32_t port = PERIPH_GPIO_PORT(cfg->gpio);
    uint32_t pin = PERIPH_GPIO_PIN(cfg->gpio);

    __gpio_write(port, pin, (cfg->invert) ? (!value) : (!!value));

    return 0;
}

int gpio_toggle(gpio_id_t idx)
{
    if (EXCEED_GPIO_IDX(idx))
        return -1;

    const gpio_conf_t *cfg = &gpio_config[idx];

    uint32_t port = PERIPH_GPIO_PORT(cfg->gpio);
    uint32_t pin = PERIPH_GPIO_PIN(cfg->gpio);

    __gpio_toggle(port, pin);

    return 0;
}

int gpio_read(gpio_id_t idx)
{
    if (EXCEED_GPIO_IDX(idx))
        return -1;

    const gpio_conf_t *cfg = &gpio_config[idx];

    uint32_t port = PERIPH_GPIO_PORT(cfg->gpio);
    uint32_t pin = PERIPH_GPIO_PIN(cfg->gpio);
    int value = __gpio_read(port, pin);

    return (cfg->invert) ? (!value) : (!!value);
}

void gpio_set(gpio_id_t idx)
{
    gpio_write(idx, 1);
}
void gpio_clear(gpio_id_t idx)
{
    gpio_write(idx, 0);
}

static const uint32_t spi_mode[4] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2,
                                     SPI_MODE_3};
static const uint32_t qspi_mode[4] = {QSPI_MODE_0, QSPI_MODE_1, QSPI_MODE_2,
                                      QSPI_MODE_3};

void spi_fini(spi_id_t idx)
{
    if (EXCEED_SPI_IDX(idx))
        return;

    const spi_conf_t *cfg = &spi_config[idx];
    SPI_T *dev = cfg->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_spi);
    if (!mod)
        return;

    NVIC_EnableIRQ(mod->irqn);
    SPI_Close(cfg->dev);
    CLK_DisableModuleClock_S(mod->clkidx);

    _destroy_periph_lock(&spi_lock[idx]);
}

int spi_init(spi_id_t idx)
{
    if (EXCEED_SPI_IDX(idx))
        return -1;

    const spi_conf_t *cfg = &spi_config[idx];
    void *dev = cfg->dev;

    const mod_desc_t *mod = NULL;

    if (cfg->type == SPI_TYPE_QUAD)
        mod = find_mod_desc(dev, modtbl_qspi);
    else
        mod = find_mod_desc(dev, modtbl_spi);
    if (!mod)
        return -1;

    if (_create_periph_lock(&spi_lock[idx]) != 0)
        return -1;

    const gpio_t *slaves = cfg->ssel;

    if (cfg->is_master)
    {
        int i;
        for (i = 0; i < cfg->ssel_cnt; i++)
        {
            GPIO_SetMode((GPIO_T *)PERIPH_GPIO_PORT(slaves[i]),
                         1 << PERIPH_GPIO_PIN(slaves[i]), 1);
            __gpio_write(PERIPH_GPIO_PORT(slaves[i]),
                         PERIPH_GPIO_PIN(slaves[i]), 1);
        }
    }

    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);
    CLK_EnableModuleClock_S(mod->clkidx);
    SYS_ResetModule_S(mod->rstidx);

    if (cfg->type == SPI_TYPE_QUAD)
    {
        pin_function_s(PERIPH_GPIO_IDX(cfg->mosi), PERIPH_GPIO_PIN(cfg->mosi),
                       find_pin_af(dev, cfg->mosi, pin_af_qspi_mosi0));
        pin_function_s(PERIPH_GPIO_IDX(cfg->miso), PERIPH_GPIO_PIN(cfg->miso),
                       find_pin_af(dev, cfg->miso, pin_af_qspi_miso0));
        pin_function_s(PERIPH_GPIO_IDX(cfg->sclk), PERIPH_GPIO_PIN(cfg->sclk),
                       find_pin_af(dev, cfg->sclk, pin_af_qspi_sclk));
    }
    else
    {
        pin_function_s(PERIPH_GPIO_IDX(cfg->mosi), PERIPH_GPIO_PIN(cfg->mosi),
                       find_pin_af(dev, cfg->mosi, pin_af_spi_mosi));
        pin_function_s(PERIPH_GPIO_IDX(cfg->miso), PERIPH_GPIO_PIN(cfg->miso),
                       find_pin_af(dev, cfg->miso, pin_af_spi_miso));
        pin_function_s(PERIPH_GPIO_IDX(cfg->sclk), PERIPH_GPIO_PIN(cfg->sclk),
                       find_pin_af(dev, cfg->sclk, pin_af_spi_sclk));
    }

    if (!cfg->is_master)
    {
        if (cfg->type == SPI_TYPE_QUAD)
        {
            pin_function_s(PERIPH_GPIO_IDX(*slaves), PERIPH_GPIO_PIN(*slaves),
                           find_pin_af(dev, *slaves, pin_af_qspi_ssel));

            QSPI_Open(cfg->dev, SPI_SLAVE, qspi_mode[cfg->mode], cfg->width, 0);
        }
        else
        {
            pin_function_s(PERIPH_GPIO_IDX(*slaves), PERIPH_GPIO_PIN(*slaves),
                           find_pin_af(dev, *slaves, pin_af_spi_ssel));

            SPI_Open(cfg->dev, SPI_SLAVE, spi_mode[cfg->mode], cfg->width, 0);
        }
    }

    return 0;
}

void spi_deinit(spi_id_t idx)
{
    if (EXCEED_SPI_IDX(idx))
        return;

    const spi_conf_t *cfg = &spi_config[idx];
    SPI_T *dev = cfg->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_spi);
    if (!mod)
        return;

    _destroy_periph_lock(&spi_lock[idx]);

    if (!cfg->is_master)
    {
        SPI_Close(cfg->dev);
    }

    CLK_DisableModuleClock_S(mod->clkidx);
}

#define SPI_IO_FN(t, x)                                                 \
    static int t##_IO##x(t##_T *dev, uint##x##_t *out, uint##x##_t *in, \
                         uint32_t len)                                  \
    {                                                                   \
        unsigned i;                                                     \
        for (i = 0; i < len; i++)                                       \
        {                                                               \
            if (out)                                                    \
            {                                                           \
                t##_WRITE_TX(dev, (uint32_t)(out[i]));                  \
            }                                                           \
            else                                                        \
            {                                                           \
                t##_WRITE_TX(dev, 0);                                   \
            }                                                           \
            while (t##_IS_BUSY(dev))                                    \
                ;                                                       \
            while (t##_GET_RX_FIFO_EMPTY_FLAG(dev))                     \
                ;                                                       \
            if (in)                                                     \
            {                                                           \
                in[i] = t##_READ_RX(dev);                               \
            }                                                           \
            else                                                        \
            {                                                           \
                volatile uint32_t tmp = t##_READ_RX(dev);               \
                (void)tmp;                                              \
            }                                                           \
            while (t##_IS_BUSY(dev))                                    \
                ;                                                       \
        }                                                               \
        t##_ClearRxFIFO(dev);                                           \
        return 0;                                                       \
    }

SPI_IO_FN(SPI, 8)
SPI_IO_FN(SPI, 16)
SPI_IO_FN(SPI, 32)

SPI_IO_FN(QSPI, 8)
SPI_IO_FN(QSPI, 16)
SPI_IO_FN(QSPI, 32)

int spi_acquire(const spi_dev_t *spi)
{
    if (EXCEED_SPI_IDX(spi->id))
        return -1;

    const spi_conf_t *cfg = &spi_config[spi->id];

    if (!cfg->is_master)
        return -1;

    if (spi->mode > 3)
        return -1;

#if defined(OS_CMSIS_RTX) || defined(OS_FREERTOS)
    if (spi_lock[spi->id] == NULL)
    {
        /*
         * spi_lock is not initialized because
         * spi_init is not called in the NonSecure world
         */
        if (_create_periph_lock(&spi_lock[spi->id]) != 0)
        {
            return -1;
        }
    }
#endif

    if (_acquire_periph_lock(&spi_lock[spi->id], 0xFFFFFFFF) != 0)
        return -1;

    if (cfg->type == SPI_TYPE_QUAD)
    {
        QSPI_Open(cfg->dev, QSPI_MASTER, QSPI_MODE_0, spi->width, spi->clk);
        QSPI_DisableAutoSS(cfg->dev);
    }
    else
    {
        SPI_Open(cfg->dev, SPI_MASTER, spi_mode[spi->mode], spi->width,
                 spi->clk);
        SPI_DisableAutoSS(cfg->dev);
    }

    return 0;
}

int spi_release(const spi_dev_t *spi)
{
    if (EXCEED_SPI_IDX(spi->id))
        return -1;

    const spi_conf_t *cfg = &spi_config[spi->id];

    if (!cfg->is_master)
        return -1;

    if (cfg->type == SPI_TYPE_QUAD)
        QSPI_Close(cfg->dev);
    else
        SPI_Close(cfg->dev);

    _release_periph_lock(&spi_lock[spi->id]);

    return 0;
}

int spi_transfer_bytes(const spi_dev_t *spi, spi_io_transaction_t cont,
                       spi_stream_t *out, spi_stream_t *in)
{
    if (EXCEED_SPI_IDX(spi->id))
        return -1;

    const spi_conf_t *cfg = &spi_config[spi->id];
    SPI_T *dev = cfg->dev;
    uint32_t width = (dev->CTL >> SPI_CTL_DWIDTH_Pos) & 0x1F;

    if (spi->slave_idx >= cfg->ssel_cnt)
        return -1;

    gpio_t ssel = cfg->ssel[spi->slave_idx];

    /* if both out & in are present, in->len is ignored.
       that is, in->len be should greater than or equal to out->len */
    if (out && in && in->len < out->len)
        return -1;

    int ret = 0;

    if (__gpio_read(PERIPH_GPIO_PORT(ssel), PERIPH_GPIO_PIN(ssel)) != 0)
        __gpio_write(PERIPH_GPIO_PORT(ssel), PERIPH_GPIO_PIN(ssel), 0);

    if (out)
    {
        if (in)
        {
            if (width <= 8)
            {
                if (cfg->type == SPI_TYPE_QUAD)
                    ret = QSPI_IO8((QSPI_T *)dev, out->buf, in->buf, out->len);
                else
                    ret = SPI_IO8(dev, out->buf, in->buf, out->len);
            }
            else if (width <= 16)
            {
                if (cfg->type == SPI_TYPE_QUAD)
                    ret = QSPI_IO16((QSPI_T *)dev, out->buf, in->buf, out->len);
                else
                    ret = SPI_IO16(dev, out->buf, in->buf, out->len);
            }
            else
            {
                if (cfg->type == SPI_TYPE_QUAD)
                    ret = QSPI_IO32((QSPI_T *)dev, out->buf, in->buf, out->len);
                else
                    ret = SPI_IO32(dev, out->buf, in->buf, out->len);
            }
        }
        else
        {
            if (width <= 8)
            {
                if (cfg->type == SPI_TYPE_QUAD)
                    ret = QSPI_IO8((QSPI_T *)dev, out->buf, NULL, out->len);
                else
                    ret = SPI_IO8(dev, out->buf, NULL, out->len);
            }
            else if (width <= 16)
            {
                if (cfg->type == SPI_TYPE_QUAD)
                    ret = QSPI_IO16((QSPI_T *)dev, out->buf, NULL, out->len);
                else
                    ret = SPI_IO16(dev, out->buf, NULL, out->len);
            }
            else
            {
                if (cfg->type == SPI_TYPE_QUAD)
                    ret = QSPI_IO32((QSPI_T *)dev, out->buf, NULL, out->len);
                else
                    ret = SPI_IO32(dev, out->buf, NULL, out->len);
            }
        }
    }

    if (ret == 0 && in)
    {
        if (width <= 8)
        {
            if (cfg->type == SPI_TYPE_QUAD)
                ret = QSPI_IO8((QSPI_T *)dev, NULL, in->buf, in->len);
            else
                ret = SPI_IO8(dev, NULL, in->buf, in->len);
        }
        else if (width <= 16)
        {
            if (cfg->type == SPI_TYPE_QUAD)
                ret = QSPI_IO16((QSPI_T *)dev, NULL, in->buf, in->len);
            else
                ret = SPI_IO16(dev, NULL, in->buf, in->len);
        }
        else
        {
            if (cfg->type == SPI_TYPE_QUAD)
                ret = QSPI_IO32((QSPI_T *)dev, NULL, in->buf, in->len);
            else
                ret = SPI_IO32(dev, NULL, in->buf, in->len);
        }
    }

    if (ret == 0 && cont == SPI_IO_TRANSACTION_CONTINUE)
    {
        return ret;
    }

    __gpio_write(PERIPH_GPIO_PORT(ssel), PERIPH_GPIO_PIN(ssel), 1);

    return ret;
}

int i2c_init(i2c_id_t idx)
{
    if (EXCEED_I2C_IDX(idx))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[idx];
    I2C_T *dev = cfg->dev;
    if (NULL == dev)
        return -1;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_i2c);
    if (NULL == mod)
        return -1;

    if (_create_periph_lock(&i2c_lock[idx]) != 0)
    {
        return -1;
    }

    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);
    // CLK_EnableModuleClock_S(mod->clkidx);
    SYS_ResetModule_S(mod->rstidx);

    pin_function_s(PERIPH_GPIO_IDX(cfg->scl), PERIPH_GPIO_PIN(cfg->scl),
                   find_pin_af(dev, cfg->scl, pin_af_i2c_scl));
    pin_function_s(PERIPH_GPIO_IDX(cfg->sda), PERIPH_GPIO_PIN(cfg->sda),
                   find_pin_af(dev, cfg->sda, pin_af_i2c_sda));

    // TODO, Jin Jung, 20191204, I2C Master case should be configured.

    I2C_EnableInt(dev);
    NVIC_EnableIRQ(mod->irqn);

    return 0;
}

int i2c_deinit(i2c_id_t idx)
{
    if (EXCEED_I2C_IDX(idx))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[idx];
    I2C_T *dev = cfg->dev;
    if (NULL == dev)
        return -1;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_i2c);
    if (NULL == mod)
        return -1;

    _destroy_periph_lock(&i2c_lock[idx]);

    CLK_DisableModuleClock_S(mod->clkidx);

    I2C_DisableInt(dev);

    return 0;
}

int i2c_acquire(const i2c_dev_t *i2c)
{
    if (NULL == i2c)
        return -1;

    if (EXCEED_I2C_IDX(i2c->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c->id];
    I2C_T *dev = cfg->dev;
    if (NULL == dev)
        return -1;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_i2c);
    if (NULL == mod)
        return -1;

#if defined(OS_CMSIS_RTX) || defined(OS_FREERTOS)
    if (i2c_lock[i2c->id] == NULL)
    {
        /*
         * i2c_lock is not initialized because
         * i2c_init is not called in the NonSecure world
         */
        if (_create_periph_lock(&i2c_lock[i2c->id]) != 0)
        {
            return -1;
        }
    }
#endif

    if (_acquire_periph_lock(&i2c_lock[i2c->id], 0xFFFFFFFF) != 0)
        return -1;

    CLK_EnableModuleClock_S(mod->clkidx);
    /* Open I2C module and set bus clock */
    I2C_Open(dev, i2c->clk);

    /* Get I2C0 Bus Clock */
    // printf("I2C clock %ld Hz\n", I2C_GetBusClockFreq(dev));

    return 0;
}

int i2c_release(const i2c_dev_t *i2c)
{
    if (NULL == i2c)
        return -1;

    if (EXCEED_I2C_IDX(i2c->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c->id];
    I2C_T *dev = cfg->dev;
    if (NULL == dev)
        return -1;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_i2c);
    if (NULL == mod)
        return -1;

    I2C_Close_S(dev);
    // int dev_index = ((uint32_t)dev >> 12) & 0x3;
    // I2C_Close_S(dev_index);

    CLK_DisableModuleClock_S(mod->clkidx);

    _release_periph_lock(&i2c_lock[i2c->id]);

    return 0;
}

/**
 * @brief Read multi bytes from EEPROM Slave
 * @param[in] *i2c Point to I2C peripheral
 * @param[in] ctx Point to I2C input/output context structure
 * @retval -1 Failure
 * @retval >0 A length of how many bytes have been received
 */
int i2c_eeprom_rx_bytes(I2C_T *i2c, const i2c_io_ctx_t *ctx)
{
    uint8_t xfering = 1, err = 0, ctl = 0;
    uint32_t txlen = 0, rxlen = 0;

    if (NULL == i2c)
        goto exit;

    if (NULL == ctx)
        goto exit;

    /* Send START */
    I2C_START(i2c);
    while (xfering && (0 == err))
    {
        I2C_WAIT_READY(i2c) {}
        switch (I2C_GET_STATUS(i2c))
        {
        /* Write SLA+W to Register I2CDAT */
        case I2C_CTL_SI:  // 0x08
            I2C_SET_DATA(i2c, (uint8_t)(ctx->slave_addr << 1u | 0x00u));
            ctl = I2C_CTL_SI; /* Clear SI */
            break;
            /* SLA+W has been transmitted and ACK has been received */
        case I2C_CTL_STO_SI:  // 0x18
            I2C_SET_DATA(i2c, ctx->tx.buf[txlen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            break;
            /* SLA+W has been transmitted and NACK has been received */
        case I2C_CTL_STA:  // 0x20
                           /* Master transmit data NACK */
        case 0x30u:
            ctl = I2C_CTL_STO_SI;
            err = 1;
            break;
            /* DATA has been transmitted and ACK has been received */
        case I2C_CTL_STA_SI:  // 0x28
            if (txlen < ctx->tx.len)
            {
                I2C_SET_DATA(i2c, ctx->tx.buf[txlen++]);
                ctl = I2C_CTL_SI;
            }
            else
            {
                ctl = I2C_CTL_STA_SI;
            }
            break;
            /* Repeat START has been transmitted and prepare SLA+R */
        case I2C_CTL_STO:  // 0x10
            I2C_SET_DATA(i2c, (uint8_t)(ctx->slave_addr << 1u | 0x01u));
            ctl = I2C_CTL_SI;
            break;
            /* SLA+R has been transmitted and ACK has been received */
        case 0x40:
            if (1 == ctx->rx.len)
                ctl = I2C_CTL_SI;
            else
                ctl = I2C_CTL_SI_AA;
            break;
            /* DATA has been received and ACK has been returned */
        case 0x50:
            // printf("0x50 rxlen=%lx, rx.len=%x\n", rxlen, ctx->rx.len);
            if (rxlen < ctx->rx.len)
            {
                ctx->rx.buf[rxlen++] = (uint8_t)I2C_GET_DATA(i2c);

                // check final read operation or not
                if ((rxlen + 1) < ctx->rx.len)
                {
                    ctl = I2C_CTL_SI_AA;
                }
                else
                {
                    ctl = I2C_CTL_SI;
                }
            }
            break;
            /* DATA has been received and NACK has been returned */
        case 0x58:
            // printf("0x58 rxlen=%lx, rx.len=%x\n", rxlen, ctx->rx.len);
            ctx->rx.buf[rxlen++] = (uint8_t)I2C_GET_DATA(i2c);
            ctl = I2C_CTL_STO_SI;
            xfering = 0;
            break;
            /* Unknow status */
        default:
            /* Clear SI and send STOP */
            ctl = I2C_CTL_STO_SI;
            err = 1;
            break;
        }
        /* Write controlbit to I2C_CTL register */
        I2C_SET_CONTROL_REG(i2c, ctl);
    }

exit:
    // printf("%s rxlen=%lx--\n", __FUNCTION__, rxlen);
    /* Return bytes length that have been transmitted */
    return rxlen;
}

int i2c_tx_bytes(const i2c_dev_t *i2c, const i2c_io_ctx_t *ctx)
{
    int ret = -1;

    if (NULL == i2c)
        return -1;

    if (NULL == ctx)
        return -1;

    if (EXCEED_I2C_IDX(i2c->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c->id];
    I2C_T *dev = cfg->dev;
    if (NULL == dev)
        goto exit;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_i2c);
    if (NULL == mod)
        goto exit;

    ret = (int)I2C_WriteMultiBytes(dev, ctx->slave_addr, ctx->tx.buf,
                                   ctx->tx.len);

exit:

    return ret;
}

int i2c_rx_bytes(const i2c_dev_t *i2c, const i2c_io_ctx_t *ctx)
{
    int ret = -1;

    if (NULL == i2c)
        return -1;

    if (NULL == ctx)
        return -1;

    if (EXCEED_I2C_IDX(i2c->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c->id];
    I2C_T *dev = cfg->dev;
    if (NULL == dev)
        return -1;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_i2c);
    if (NULL == mod)
        return -1;

    ret = i2c_eeprom_rx_bytes(dev, ctx);

    return ret;
}

int i2c_read_byte(const i2c_dev_t *i2c_dev, uint8_t *data, uint32_t dlen)
{
    if (!i2c_dev || !data || !dlen)
        return -1;

    if (EXCEED_I2C_IDX(i2c_dev->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c_dev->id];

    if (i2c_acquire(i2c_dev) != 0)
        return -1;

    if (dlen == 1)
        *data = I2C_ReadByte(cfg->dev, i2c_dev->slave_addr);
    else
        I2C_ReadMultiBytes(cfg->dev, i2c_dev->slave_addr, data, dlen);

    i2c_release(i2c_dev);

    return 0;
}

int i2c_write_byte(const i2c_dev_t *i2c_dev, uint8_t *data, uint32_t dlen)
{
    if (!i2c_dev || !data || !dlen)
        return -1;

    if (EXCEED_I2C_IDX(i2c_dev->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c_dev->id];

    if (i2c_acquire(i2c_dev) != 0)
        return -1;

    if (dlen == 1)
        I2C_WriteByte(cfg->dev, i2c_dev->slave_addr, data[0]);
    else
        I2C_WriteMultiBytes(cfg->dev, i2c_dev->slave_addr, data, dlen);

    i2c_release(i2c_dev);

    return 0;
}

int i2c_read_reg(const i2c_dev_t *i2c_dev, uint16_t reg, uint16_t rlen,
                 uint8_t *data, uint32_t dlen)
{
    int ret;

    if (!i2c_dev || !data || !dlen)
        return -1;

    if (rlen == 0 || rlen > 2)
        return -1;

    if (EXCEED_I2C_IDX(i2c_dev->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c_dev->id];

    if (i2c_acquire(i2c_dev) != 0)
        return -1;

    if (rlen == 1)
    {
        if (dlen == 1)
        {
            *data = I2C_ReadByteOneReg(cfg->dev, i2c_dev->slave_addr,
                                       (uint8_t)(reg & 0xff));
            ret = 1;
        }
        else
        {
            ret = I2C_ReadMultiBytesOneReg(cfg->dev, i2c_dev->slave_addr,
                                           (uint8_t)(reg & 0xff), data, dlen);
        }
    }
    else if (rlen == 2)
    {
        if (dlen == 1)
        {
            *data = I2C_ReadByteTwoRegs(cfg->dev, i2c_dev->slave_addr, reg);
            ret = 1;
        }
        else
        {
            ret = I2C_ReadMultiBytesTwoRegs(cfg->dev, i2c_dev->slave_addr, reg,
                                            data, dlen);
        }
    }

    i2c_release(i2c_dev);

    return ret;
}

int i2c_write_reg(const i2c_dev_t *i2c_dev, uint16_t reg, uint16_t rlen,
                  uint8_t *data, uint32_t dlen)
{
    int ret;

    if (!i2c_dev || !data || !dlen)
        return -1;

    if (rlen == 0 || rlen > 2)
        return -1;

    if (EXCEED_I2C_IDX(i2c_dev->id))
        return -1;

    const i2c_conf_t *cfg = &i2c_config[i2c_dev->id];

    if (i2c_acquire(i2c_dev) != 0)
        return -1;

    if (rlen == 1)
    {
        if (dlen == 1)
        {
            I2C_WriteByteOneReg(cfg->dev, i2c_dev->slave_addr,
                                (uint8_t)(reg & 0xff), data[0]);
            ret = 1;
        }
        else
        {
            ret = I2C_WriteMultiBytesOneReg(cfg->dev, i2c_dev->slave_addr,
                                            (uint8_t)(reg & 0xff), data, dlen);
        }
    }
    else if (rlen == 2)
    {
        if (dlen == 1)
        {
            I2C_WriteByteTwoRegs(cfg->dev, i2c_dev->slave_addr, reg, data[0]);
            ret = 1;
        }
        else
        {
            ret = I2C_WriteMultiBytesTwoRegs(cfg->dev, i2c_dev->slave_addr, reg,
                                             data, dlen);
        }
    }

    i2c_release(i2c_dev);

    return ret;
}

#if defined(MODULE_SYSTICK)
void systick_config(uint32_t hz, uint32_t vector)
{
    NVIC_SetVector(SysTick_IRQn, vector);
    SysTick_Config(SystemCoreClock / hz);
}
#endif

int get_cpu_uid(unsigned char *buf, unsigned int buf_len, unsigned int *uid_len)
{
    if (!buf || buf_len < 12 || !uid_len)
        return -1;

    uint32_t data[3];

    data[0] = FMC_ReadUID(0);
    data[1] = FMC_ReadUID(1);
    data[2] = FMC_ReadUID(2);

    memcpy(buf, data, 12);
    *uid_len = 12;

    return 0;
}

static inline int addr_is_apromns(uint32_t addr, uint32_t len)
{
    if ((int32_t)addr >= (int32_t)FMC_NON_SECURE_BASE && addr < (0x10000000 + FMC_APROM_END) &&
        addr + len -1 < (0x10000000 + FMC_APROM_END))
        return 1;
    return 0;
}

static inline int addr_is_ldrom(uint32_t addr, uint32_t len)
{
    if (addr >= FMC_LDROM_BASE && addr < FMC_LDROM_END &&
        addr + len -1 < FMC_LDROM_END)
        return 1;
    return 0;
}

int flash_erase(uint32_t addr, uint32_t len)
{
    int is_aprom = 0;

    if (len == 0)
        return 0;

    if (addr % FMC_FLASH_PAGE_SIZE)
        return -1;

    if (addr_is_apromns(addr, len))
        is_aprom = 1;
    else if (addr_is_ldrom(addr, len))
        is_aprom = 0;
    else
        return -1;

    int i, ret, pages;
    addr = addr & 0x00FFFFFF;

    pages = ((len + FMC_FLASH_PAGE_SIZE - 1) & FMC_PAGE_ADDR_MASK) /
            FMC_FLASH_PAGE_SIZE;

    if (is_aprom)
        FMC_ENABLE_AP_UPDATE();
    else
        FMC_ENABLE_LD_UPDATE();

    for (i = 0; i < pages; i++)
    {
        ret = FMC_Erase(addr);
        if (ret != 0)
            break;
        addr += FMC_FLASH_PAGE_SIZE;
    }

    if (is_aprom)
        FMC_DISABLE_AP_UPDATE();
    else
        FMC_DISABLE_LD_UPDATE();

    if (ret)
        return -1;

    return 0;
}

int flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (len == 0)
        return 0;

    if (!addr_is_apromns(addr, len) && !addr_is_ldrom(addr, len))
        return -1;

    uint32_t src = addr & 0x00FFFFFF;
    uint8_t *dst = buf;
    uint32_t rem = len;
    uint32_t tmp;

    if (src % 4)
    {
        uint32_t len2 = 4 - (src % 4);
        if (len2 > len)
            len2 = len;
        tmp = FMC_Read(src);
        memcpy(dst, ((uint8_t *)&tmp) + (src % 4), len2);
        src += len2;
        dst += len2;
        rem -= len2;
    }

    while (rem >= sizeof(uint32_t))
    {
        tmp = FMC_Read(src);
        memcpy(dst, &tmp, sizeof(uint32_t));
        src += sizeof(uint32_t);
        dst += sizeof(uint32_t);
        rem -= sizeof(uint32_t);
    }

    if (rem > 0)
    {
        tmp = FMC_Read(src);
        memcpy(dst, &tmp, rem);
    }

    return 0;
}

int flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    int is_aprom = 0;
    if (len == 0)
        return 0;
    if (addr_is_apromns(addr, len))
        is_aprom = 1;
    else if (addr_is_ldrom(addr, len))
        is_aprom = 0;
    else
        return -1;

    {
        FMC_Open();
        FMC_ENABLE_ISP();
    }

    uint8_t *src = (uint8_t *)buf;
    uint32_t dst = (addr & 0x00FFFFFF);
    uint32_t rem = len;
    uint32_t tmp;

    if (is_aprom)
        FMC_ENABLE_AP_UPDATE();
    else
        FMC_ENABLE_LD_UPDATE();

    if (dst % 4)
    {
        uint32_t len2 = 4 - (dst % 4);
        if (len2 > len)
            len2 = len;
        tmp = FMC_Read(dst);
        memcpy(((uint8_t *)(&tmp)) + (dst % 4), src, len2);
        FMC_Write(dst, tmp);

        src += len2;
        dst += len2;
        rem -= len2;
    }

    while (rem >= sizeof(uint32_t))
    {
        memcpy(&tmp, src, sizeof(uint32_t));
        FMC_Write(dst, tmp);

        src += sizeof(uint32_t);
        dst += sizeof(uint32_t);
        rem -= sizeof(uint32_t);
    }

    if (rem > 0)
    {
        tmp = 0xffffffff;
        memcpy(&tmp, src, rem);
        FMC_Write(dst, tmp);
    }

    if (is_aprom)
        FMC_DISABLE_AP_UPDATE();
    else
        FMC_DISABLE_LD_UPDATE();

    return 0;
}

int flash_otp_write(uint32_t block, const uint8_t *buf, uint32_t len)
{
    uint32_t data[2];

    if (len != OTP_BLOCK_SIZE)
        return -1;

    memcpy(data, buf, OTP_BLOCK_SIZE);

    if (FMC_WriteOTP(block, data[0], data[1]) != 0)
        return -1;

    return 0;
}

int flash_otp_read(uint32_t block, uint8_t *buf, uint32_t len)
{
    uint32_t data[2];

    if (len != OTP_BLOCK_SIZE)
        return -1;

    if (FMC_ReadOTP(block, &data[0], &data[1]) != 0)
        return -1;

    memcpy(buf, data, OTP_BLOCK_SIZE);

    return 0;
}

int flash_otp_lock(uint32_t block)
{
    if (FMC_LockOTP(block) != 0)
        return -1;
    return 0;
}

int flash_otp_is_locked(uint32_t block)
{
    int ret;
    ret = FMC_IsOTPLocked(block);
    switch (ret)
    {
    case 1:
        return 1;  // locked
    case 0:
        return 0;  // unlocked
    default:
        return -1;  // error
    }
}

static uint32_t eadc_channel_bitmap = 0;
static int eadc_adint[4] = {EADC_STATUS2_ADIF0_Msk, EADC_STATUS2_ADIF1_Msk,
                            EADC_STATUS2_ADIF2_Msk, EADC_STATUS2_ADIF3_Msk};
static IRQn_Type eadc_irqn[4] = {EADC0_IRQn, EADC1_IRQn, EADC2_IRQn,
                                 EADC3_IRQn};
#if defined(OS_FREERTOS)
static SemaphoreHandle_t eadc_lock[4] = {
    NULL,
};
static SemaphoreHandle_t eadc_wait[4] = {
    NULL,
};
#else
static volatile int eadc_wait[4];
#endif

int adc_init(adc_id_t idx)
{
    if (EXCEED_ADC_IDX(idx))
        return -1;

    const adc_conf_t *conf = &adc_config[idx];

    if (conf->adint > 3)
        return -1;

#if defined(OS_FREERTOS)
    eadc_lock[conf->adint] = xSemaphoreCreateMutex();
    eadc_wait[conf->adint] = xSemaphoreCreateBinary();
    if (eadc_lock[conf->adint] == NULL || eadc_wait[conf->adint] == NULL)
    {
        if (eadc_lock[conf->adint])
            vSemaphoreDelete(eadc_lock[conf->adint]);
        if (eadc_wait[conf->adint])
            vSemaphoreDelete(eadc_wait[conf->adint]);
        return -1;
    }
#else
    eadc_wait[conf->adint] = 0;
#endif

    GPIO_T *port = (GPIO_T *)PERIPH_GPIO_PORT(conf->pin);
    uint32_t pin = PERIPH_GPIO_PIN(conf->pin);
    uint32_t mod = pin;
    uint32_t channel = pin;

    CLK_SetModuleClock_S(EADC_MODULE, 0, CLK_CLKDIV0_EADC(12));
    CLK_EnableModuleClock_S(EADC_MODULE);
    SYS_ResetModule_S(EADC_RST);

    pin_function_s(PERIPH_GPIO_IDX(conf->pin), pin,
                   find_pin_af(conf->dev, conf->pin, pin_af_eadc));
    port->DINOFF |= (1 << pin);

    EADC_Open(conf->dev, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_ConfigSampleModule(conf->dev, mod, EADC_SOFTWARE_TRIGGER, channel);

    eadc_channel_bitmap |= (1 << pin);

    return 0;
}

void adc_fini(adc_id_t idx)
{
    if (EXCEED_ADC_IDX(idx))
        return;

    const adc_conf_t *conf = &adc_config[idx];
    GPIO_T *port = (GPIO_T *)PERIPH_GPIO_PORT(conf->pin);
    uint32_t pin = PERIPH_GPIO_PIN(conf->pin);

    eadc_channel_bitmap &= ~(1 << pin);

    if (!eadc_channel_bitmap)
    {
        EADC_Close(conf->dev);
        CLK_DisableModuleClock_S(0);
    }

    port->DINOFF &= ~(1 << pin);
    pin_function_s(PERIPH_GPIO_IDX(conf->pin), pin, 0);

#if defined(OS_FREERTOS)
    vSemaphoreDelete(eadc_lock[conf->adint]);
    vSemaphoreDelete(eadc_wait[conf->adint]);
#endif
}

int adc_irq_setup(adc_id_t idx, int priority, void (*isr)(void))
{
    if (EXCEED_ADC_IDX(idx))
        return -1;

    (void)priority;

    const adc_conf_t *conf = &adc_config[idx];
    uint32_t channel = PERIPH_GPIO_PIN(conf->pin);
    uint32_t mod = channel;  // to simplify, set mod = channel

    // EADC_CLR_INT_FLAG(EADC, eadc_adint[conf->adint]);
    conf->dev->STATUS2 |= eadc_adint[conf->adint];

    EADC_ENABLE_INT(conf->dev, 1 << conf->adint);

    EADC_ENABLE_SAMPLE_MODULE_INT(conf->dev, conf->adint, 1 << mod);

    NVIC_SetVector(eadc_irqn[conf->adint], (uint32_t)isr);
    NVIC_EnableIRQ(eadc_irqn[conf->adint]);

    return 0;
}

static void eadc_irq_handler(void)
{
    uint32_t status =
        (EADC->STATUS2 & (EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADIF1_Msk |
                          EADC_STATUS2_ADIF2_Msk | EADC_STATUS2_ADIF3_Msk));
#if defined(OS_FREERTOS)
#define ADC_COMPLETE(i) xSemaphoreGiveFromISR(eadc_wait[i], &yield)
    BaseType_t yield = pdFALSE;
#else
#define ADC_COMPLETE(i) eadc_wait[i] = 0
#endif

    EADC_CLR_INT_FLAG(EADC, status);

    if (status & EADC_STATUS2_ADIF0_Msk)
    {
        ADC_COMPLETE(0);
    }
    if (status & EADC_STATUS2_ADIF1_Msk)
    {
        ADC_COMPLETE(1);
    }
    if (status & EADC_STATUS2_ADIF2_Msk)
    {
        ADC_COMPLETE(2);
    }
    if (status & EADC_STATUS2_ADIF3_Msk)
    {
        ADC_COMPLETE(3);
    }
}

int adc_read(adc_id_t idx)
{
    if (EXCEED_ADC_IDX(idx))
        return -1;

    const adc_conf_t *conf = &adc_config[idx];

#if defined(OS_FREERTOS)
    if (xSemaphoreTake(eadc_lock[conf->adint], portMAX_DELAY) != pdPASS)
        return -1;
#endif

    uint32_t channel = PERIPH_GPIO_PIN(conf->pin);
    uint32_t mod = channel;  // to simplify, set mod = channel

    adc_irq_setup(idx, 0, eadc_irq_handler);
#if defined(OS_FREERTOS)
#else
    eadc_wait[conf->adint] = 0;
#endif

    // EADC_START_CONV(conf->dev, 1<<mod);
    conf->dev->SWTRG |= (1 << mod);

#if defined(OS_FREERTOS)
    xSemaphoreTake(eadc_wait[conf->adint], portMAX_DELAY);
#else
    while (eadc_wait[conf->adint] == 0)
        ;
#endif

    uint32_t dat = EADC_GET_CONV_DATA(conf->dev, mod);

#if defined(OS_FREERTOS)
    xSemaphoreGive(eadc_lock[conf->adint]);
#endif

    return (int)dat;
}

int sc_irq_setup(sc_id_t idx, int priority, void (*isr)(void))
{
    const sc_conf_t *conf = &sc_config[idx];
    SC_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_sc);
    if (!mod)
    {
        return -1;
    }

    (void)priority;
    NVIC_SetVector(mod->irqn, (uint32_t)isr);
    NVIC_EnableIRQ(mod->irqn);

    return 0;
}

/*
int sc_init(sc_id_t idx)
{
    if (EXCEED_SC_IDX(idx))
        return -1;

    const sc_conf_t *conf = &sc_config[idx];
    SC_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_sc);
    if (!mod)
    {
        return -1;
    }

    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);
    CLK_EnableModuleClock_S(mod->clkidx);
    // SYS_ResetModule_S(mod->rstidx);

    pin_function_s(PERIPH_GPIO_IDX(conf->pwr), PERIPH_GPIO_PIN(conf->pwr),
                   find_pin_af(dev, conf->pwr, pin_af_sc_pwr));

    pin_function_s(PERIPH_GPIO_IDX(conf->rst), PERIPH_GPIO_PIN(conf->rst),
                   find_pin_af(dev, conf->rst, pin_af_sc_rst));

    pin_function_s(PERIPH_GPIO_IDX(conf->dat), PERIPH_GPIO_PIN(conf->dat),
                   find_pin_af(dev, conf->dat, pin_af_sc_dat));

    pin_function_s(PERIPH_GPIO_IDX(conf->clk), PERIPH_GPIO_PIN(conf->clk),
                   find_pin_af(dev, conf->clk, pin_af_sc_clk));

    SC_Open(dev, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);

    return 0;
}*/

int sc_close(sc_id_t idx)
{
    if (EXCEED_SC_IDX(idx))
        return -1;

    const mod_desc_t *mod = find_mod_desc(sc_config[idx].dev, modtbl_sc);
    if (!mod)
        return -1;

    SC_Close(mod->device);
    NVIC_DisableIRQ(mod->irqn);
    CLK_DisableModuleClock_S(mod->clkidx);

    return 0;
}

#ifdef MODULE_LCD
void lcd_IRQHandler(void)
{
    //	printf("%s++\n", __func__);

    if (1 == LCD_GET_FRAME_END_FLAG())
        LCD_CLEAR_FRAME_END_FLAG();

    if (1 == LCD_GET_FRAME_COUNTING_END_FLAG())
        LCD_CLEAR_FRAME_COUNTING_END_FLAG();
}

int lcd_irq_setup(int priority, void (*isr)(void))
{
    const lcd_conf_t *conf = &lcd_config;
    LCD_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_lcd);
    if (NULL == mod)
        return -1;

    (void)priority;

    NVIC_SetVector(mod->irqn, (uint32_t)isr);
    NVIC_EnableIRQ(mod->irqn);

    return 0;
}

int lcd_init(void)
{
    const lcd_conf_t *conf = &lcd_config;
    LCD_T *dev = conf->dev;
    if (NULL == dev)
        return -1;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_lcd);
    if (NULL == mod)
        return -1;

    // Config LCD pins
    pin_function_s(PERIPH_GPIO_IDX(conf->com0), PERIPH_GPIO_PIN(conf->com0),
                   find_pin_af(mod->device, conf->com0, pin_af_lcd_com0));

    pin_function_s(PERIPH_GPIO_IDX(conf->com1), PERIPH_GPIO_PIN(conf->com1),
                   find_pin_af(mod->device, conf->com1, pin_af_lcd_com1));

    pin_function_s(PERIPH_GPIO_IDX(conf->com2), PERIPH_GPIO_PIN(conf->com2),
                   find_pin_af(mod->device, conf->com2, pin_af_lcd_com2));

    pin_function_s(PERIPH_GPIO_IDX(conf->com3), PERIPH_GPIO_PIN(conf->com3),
                   find_pin_af(mod->device, conf->com3, pin_af_lcd_com3));

    pin_function_s(PERIPH_GPIO_IDX(conf->s0), PERIPH_GPIO_PIN(conf->s0),
                   find_pin_af(mod->device, conf->s0, pin_af_lcd_seg0));

    pin_function_s(PERIPH_GPIO_IDX(conf->s1), PERIPH_GPIO_PIN(conf->s1),
                   find_pin_af(mod->device, conf->s1, pin_af_lcd_seg1));

    pin_function_s(PERIPH_GPIO_IDX(conf->s2), PERIPH_GPIO_PIN(conf->s2),
                   find_pin_af(mod->device, conf->s2, pin_af_lcd_seg2));

    pin_function_s(PERIPH_GPIO_IDX(conf->s3), PERIPH_GPIO_PIN(conf->s3),
                   find_pin_af(mod->device, conf->s3, pin_af_lcd_seg3));

    pin_function_s(PERIPH_GPIO_IDX(conf->s4), PERIPH_GPIO_PIN(conf->s4),
                   find_pin_af(mod->device, conf->s4, pin_af_lcd_seg4));

    pin_function_s(PERIPH_GPIO_IDX(conf->s5), PERIPH_GPIO_PIN(conf->s5),
                   find_pin_af(mod->device, conf->s5, pin_af_lcd_seg5));

    pin_function_s(PERIPH_GPIO_IDX(conf->s6), PERIPH_GPIO_PIN(conf->s6),
                   find_pin_af(mod->device, conf->s6, pin_af_lcd_seg6));

    pin_function_s(PERIPH_GPIO_IDX(conf->s7), PERIPH_GPIO_PIN(conf->s7),
                   find_pin_af(mod->device, conf->s7, pin_af_lcd_seg7));

    pin_function_s(PERIPH_GPIO_IDX(conf->s8), PERIPH_GPIO_PIN(conf->s8),
                   find_pin_af(mod->device, conf->s8, pin_af_lcd_seg8));

    pin_function_s(PERIPH_GPIO_IDX(conf->s9), PERIPH_GPIO_PIN(conf->s9),
                   find_pin_af(mod->device, conf->s9, pin_af_lcd_seg9));

    pin_function_s(PERIPH_GPIO_IDX(conf->s10), PERIPH_GPIO_PIN(conf->s10),
                   find_pin_af(mod->device, conf->s10, pin_af_lcd_seg10));

    pin_function_s(PERIPH_GPIO_IDX(conf->s11), PERIPH_GPIO_PIN(conf->s11),
                   find_pin_af(mod->device, conf->s11, pin_af_lcd_seg11));

    pin_function_s(PERIPH_GPIO_IDX(conf->s12), PERIPH_GPIO_PIN(conf->s12),
                   find_pin_af(mod->device, conf->s12, pin_af_lcd_seg12));

    pin_function_s(PERIPH_GPIO_IDX(conf->s13), PERIPH_GPIO_PIN(conf->s13),
                   find_pin_af(mod->device, conf->s13, pin_af_lcd_seg13));

    pin_function_s(PERIPH_GPIO_IDX(conf->s14), PERIPH_GPIO_PIN(conf->s14),
                   find_pin_af(mod->device, conf->s14, pin_af_lcd_seg14));

    pin_function_s(PERIPH_GPIO_IDX(conf->s15), PERIPH_GPIO_PIN(conf->s15),
                   find_pin_af(mod->device, conf->s15, pin_af_lcd_seg15));

    pin_function_s(PERIPH_GPIO_IDX(conf->s16), PERIPH_GPIO_PIN(conf->s16),
                   find_pin_af(mod->device, conf->s16, pin_af_lcd_seg16));

    pin_function_s(PERIPH_GPIO_IDX(conf->s17), PERIPH_GPIO_PIN(conf->s17),
                   find_pin_af(mod->device, conf->s17, pin_af_lcd_seg17));

    pin_function_s(PERIPH_GPIO_IDX(conf->s18), PERIPH_GPIO_PIN(conf->s18),
                   find_pin_af(mod->device, conf->s18, pin_af_lcd_seg18));

    pin_function_s(PERIPH_GPIO_IDX(conf->s19), PERIPH_GPIO_PIN(conf->s19),
                   find_pin_af(mod->device, conf->s19, pin_af_lcd_seg19));

    pin_function_s(PERIPH_GPIO_IDX(conf->s20), PERIPH_GPIO_PIN(conf->s20),
                   find_pin_af(mod->device, conf->s20, pin_af_lcd_seg20));

    pin_function_s(PERIPH_GPIO_IDX(conf->s21), PERIPH_GPIO_PIN(conf->s21),
                   find_pin_af(mod->device, conf->s21, pin_af_lcd_seg21));

    pin_function_s(PERIPH_GPIO_IDX(conf->s22), PERIPH_GPIO_PIN(conf->s22),
                   find_pin_af(mod->device, conf->s22, pin_af_lcd_seg22));

    pin_function_s(PERIPH_GPIO_IDX(conf->s23), PERIPH_GPIO_PIN(conf->s23),
                   find_pin_af(mod->device, conf->s23, pin_af_lcd_seg23));

    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);
    CLK_EnableModuleClock_S(mod->clkidx);
    SYS_ResetModule_S(mod->rstidx);

    lcd_irq_setup(0, lcd_IRQHandler);

    LCD_Open((S_LCD_CFG_T *)&conf->cfg);
    LCD_ENABLE_DISPLAY();

    /* Enable charge pump clock MIRC and output voltage level 2 for 3.0V */
    // CLK_EnableXtalRC(CLK_PWRCTL_MIRCEN_Msk);
    CLK_EnableModuleClock_S(LCDCP_MODULE);
    CLK_SetModuleClock_S(LCDCP_MODULE, CLK_CLKSEL1_LCDCPSEL_MIRC, 0);
    LCD_SET_CP_VOLTAGE(LCD_CP_VOLTAGE_LV_2);

    return 0;
}

void lcd_putchar(uint32_t zone, uint32_t idx, uint8_t ch)
{
    LCDLIB_PutChar(zone, idx, ch);
}

void lcd_set_symbol(uint32_t zone, uint32_t idx, uint32_t op)
{
    (void)zone;
    // LCDLIB_SetSymbol(zone, idx, op);
    LCDLIB_SetSymbol(idx, op);
}

int lcd_printf(uint32_t cfg, const char *msg, ...)
{
    int ret = -1;
    va_list arg;
    char tmp[12];
    char tmp_c;
    char buf[12];
    uint32_t msg_max_len;
    uint32_t buf_max_digit;
    char *buf_p = buf;
    uint32_t buf_p_off = 0;
    char sym_buf[6] = {
        0,
    };

    uint32_t i;
    uint32_t msg_len;
    uint8_t zone = LCD_7SEG_F2V(LCD_7SEG_ZONE, cfg);
    uint8_t align = LCD_7SEG_F2V(LCD_7SEG_ALIGN, cfg);
    uint16_t offset = LCD_7SEG_F2V(LCD_7SEG_OFFSET, cfg);

    extern const uint16_t *Zone_TextDisplay[];
    extern const int numof_Zone0_TextDisplay;
    extern const int numof_Zone1_TextDisplay;
    int numof_textdisplay;

#define ASCII_SPACE 0x20
#define ASCII_DOT 0x2e
#define ASCII_COLON 0x3a

    // printf("7seg(cfg: 0x%04lx), zone: 0x%x, align: 0x%x, offset: 0x%x\n",
    // 	cfg, zone, align, offset);

    va_start(arg, msg);
    msg_len = vsnprintf(tmp, sizeof(tmp), msg, arg);

    // printf("%s, %s, %ld\n", __func__, tmp, msg_len);

    // msg length can't over than max length of each 7 segment zone.
    //   zone 0: two digits, zone 1: six digits
    if (LCD_7SEG_ZONE_UPPER == zone)
    {
        msg_max_len = LCD_7SEG_ZONE_UPPER_MAX_STR;
        buf_max_digit = LCD_7SEG_ZONE_UPPER_MAX_DIGIT;
        numof_textdisplay = numof_Zone0_TextDisplay;
    }
    else if (LCD_7SEG_ZONE_LOWER == zone)
    {
        msg_max_len = LCD_7SEG_ZONE_LOWER_MAX_STR;
        buf_max_digit = LCD_7SEG_ZONE_LOWER_MAX_DIGIT;
        numof_textdisplay = numof_Zone1_TextDisplay;
    }
    else
        goto exit;

    // zone 0 can't print more than two digits.
    // zone 1 can't print more than ten digits include two colons and four dots.
    if (msg_max_len < msg_len)
    {
        printf("msg too long: %s\n", tmp);
        goto exit;
    }

    // msg length + offset have to be smaller/equal than max string length of
    // each zone.
    if (msg_max_len < (msg_len + (offset >> LCD_7SEG_OFFSET_POS)))
    {
        printf("msg(%" PRId32 ")+offset(%" PRId16 ") too long: %s\n", msg_len,
               offset >> LCD_7SEG_OFFSET_POS, tmp);
        goto exit;
    }

    // init buffer.
    memset(buf, 0x0, sizeof(buf));

    // Check given msg contain available characters only or not.
    // available characters.
    // z0, z1: 0 ~ 9, a ~ z, A ~ Z, -
    // z1 only: ".", ":"
    for (i = 0; i < msg_len; i++)
    {
        tmp_c = tmp[i];
        // printf("tmp_c %d, %ld\n", tmp_c, i);
        // character should be larger/equal than space(0x20).
        if (ASCII_SPACE > tmp_c)
        {
            printf("invalid character1 %" PRIx8 "\n, %" PRId32, tmp_c, i);
            goto exit;
        }

        // character can't over than available character count.
        //   Zone_TextDisplay[zone] array start from ASCII_SPACE.
        if ((numof_textdisplay + ASCII_SPACE) <= tmp_c)
        {
            printf("invalid character2 %" PRIx8 ", %" PRId32 ", %" PRIx16 "\n",
                   tmp_c, i, (sizeof(*Zone_TextDisplay[zone]) + ASCII_SPACE));
            goto exit;
        }

        if (LCD_7SEG_ZONE_UPPER == zone)
        {
            // zone 0 doesn't need to change character to symbol, just print it.
            buf_p[buf_p_off] = tmp_c;
            buf_p_off++;
        }
        else if (LCD_7SEG_ZONE_LOWER == zone)
        {
            // zone 1 need to change character "." and ":" to symbol.
            // only 2, 4, 6, 8 digits can display symbol.

            // 2nd and 4th digits can display colon or dot.
            //   And 4th and 8th digits can display dot.
            if (ASCII_DOT == tmp_c)
            {
                if (LCD_7SEG_ALIGN_RIGHT == align)
                {
                    printf("can't use right align with symbol\n");
                    goto exit;
                }

                if (2 == buf_p_off)
                {
                    sym_buf[1] = 1;  // 24
                    // lcd_set_symbol(2, 24, 1);
                    // printf("lower %ld continue %d\n", buf_p_off, tmp_c);
                    continue;
                }
                else if (3 == buf_p_off)
                {
                    sym_buf[2] = 1;  // 25
                    // lcd_set_symbol(2, 25, 1);
                    // printf("lower %ld continue %d\n", buf_p_off, tmp_c);
                    continue;
                }
                else if (4 == buf_p_off)
                {
                    sym_buf[4] = 1;  // 27
                    // lcd_set_symbol(2, 27, 1);
                    // printf("lower %ld continue %d\n", buf_p_off, tmp_c);
                    continue;
                }
                else if (5 == buf_p_off)
                {
                    sym_buf[5] = 1;  // 28
                    // lcd_set_symbol(2, 28, 1);
                    // printf("lower %ld continue %d\n", buf_p_off, tmp_c);
                    continue;
                }
            }
            else if (ASCII_COLON == tmp_c)
            {
                if (LCD_7SEG_ALIGN_RIGHT == align)
                {
                    printf("can't use right align with symbol.\n");
                    goto exit;
                }

                if (2 == buf_p_off)
                {
                    sym_buf[0] = 1;  // 23
                    // lcd_set_symbol(2, 23, 1);
                    // printf("lower %ld continue %d\n", buf_p_off, tmp_c);
                    continue;
                }
                else if (4 == buf_p_off)
                {
                    sym_buf[3] = 1;  // 26
                    // lcd_set_symbol(2, 26, 1);
                    // printf("lower %ld continue %d\n", buf_p_off, tmp_c);
                    continue;
                }
            }

            buf_p[buf_p_off] = tmp_c;
            buf_p_off++;
            // printf("buf: %s\n", buf);
        }
    }

    // If align is right side then add padding(space) to front of buffer until
    //  reach to the max len of zone.
    if (LCD_7SEG_ALIGN_RIGHT == align)
    {
        uint32_t a_len = strlen(buf);
        if (buf_max_digit > a_len)
        {
            memmove(&buf[buf_max_digit - a_len], buf, a_len);
            for (i = 0; i < buf_max_digit - a_len; i++) buf[i] = ' ';
        }
    }

    // printf("buf: %s\n", buf);
    // print on 7 segment
    LCDLIB_Printf(zone, buf);

    // update symbols in 7 segment zone1.
    if (LCD_7SEG_ZONE_LOWER == zone)
    {
        for (i = 0; i < sizeof(sym_buf); i++)
            lcd_set_symbol(2, i + 23, sym_buf[i]);
    }

    ret = 0;

exit:
    va_end(arg);

    return ret;
}

void lcd_print_number(uint32_t zone, long long num)
{
    LCDLIB_PrintNumber(zone, num);
}

#endif

int can_init(can_id_t idx, uint32_t baudrate)
{
    if (EXCEED_CAN_IDX(idx))
        return -1;

    const can_conf_t *conf = &can_config[idx];
    CAN_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_can);
    if (!mod)
    {
        return -1;
    }

    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);
    CLK_EnableModuleClock_S(mod->clkidx);

    pin_function_s(PERIPH_GPIO_IDX(conf->rx), PERIPH_GPIO_PIN(conf->rx),
                   find_pin_af(dev, conf->rx, pin_af_can_rx));

    pin_function_s(PERIPH_GPIO_IDX(conf->tx), PERIPH_GPIO_PIN(conf->tx),
                   find_pin_af(dev, conf->tx, pin_af_can_tx));

    CLK_EnableModuleClock_S(mod->clkidx);
    SYS_ResetModule_S(mod->rstidx);

    if (CAN_Open(dev, baudrate, CAN_NORMAL_MODE) != baudrate)
        return -1;

    return 0;
}

int can_deinit(can_id_t idx)
{
    if (EXCEED_CAN_IDX(idx))
        return -1;

    const can_conf_t *conf = &can_config[idx];
    CAN_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_can);
    if (!mod)
    {
        return -1;
    }

    CAN_Close(dev);
    NVIC_DisableIRQ(mod->irqn);
    CLK_DisableModuleClock_S(mod->clkidx);

    return 0;
}

int can_irq_setup(can_id_t idx, can_irq_type_t type, int priority,
                  void (*isr)(void))
{
    const can_conf_t *conf = &can_config[idx];
    CAN_T *dev = conf->dev;
    uint32_t mask = 0;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_can);
    if (!mod)
    {
        return -1;
    }

    (void)priority;
    NVIC_SetVector(mod->irqn, (uint32_t)isr);
    NVIC_SetPriority(mod->irqn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(mod->irqn);

    if ((type & CAN_IRQ_INTERRUPT) == CAN_IRQ_INTERRUPT)
        mask |= CAN_CON_IE_Msk;

    // if((type & CAN_IRQ_STATUS) == CAN_IRQ_STATUS)
    // 	mask |= CAN_CON_SIE_Msk;

    // if(type & CAN_IRQ_ERROR == CAN_IRQ_ERROR)
    // 	mask |= CAN_CON_EIE_Msk;

    if (type > 7)
        return -1;

    CAN_EnableInt(dev, mask);

    return 0;
}

int can_write(can_id_t idx, can_message_t *msg)
{
    STR_CANMSG_T t_msg = {0};

    if (EXCEED_CAN_IDX(idx))
    {
        return -1;
    }

    if (msg == NULL)
    {
        return -1;
    }

    if (msg->len > 8 || msg->len < 0)
    {
        return -1;
    }

    CAN_T *dev = can_config[idx].dev;

    t_msg.FrameType = CAN_DATA_FRAME;

    if (msg->type == CAN_STANDARD)
    {
        t_msg.IdType = CAN_STD_ID;
    }
    else if (msg->type == CAN_EXTENDED)
    {
        t_msg.IdType = CAN_EXT_ID;
    }
    else
    {
        return -1;
    }

    t_msg.Id = msg->id;
    t_msg.DLC = msg->len;

    memcpy(&t_msg.Data[0], &msg->data[0], 8);

    if (CAN_Transmit(dev, MSG(31), &t_msg) == FALSE)
    {
        return -1;
    }

    return 0;
}

int can_read(can_id_t idx, can_message_t *msg, int handle)
{
    STR_CANMSG_T r_msg = {0};

    if (EXCEED_CAN_IDX(idx))
        return -1;

    if (msg == NULL)
        return -1;

    CAN_T *dev = can_config[idx].dev;

    if (CAN_Receive(dev, handle, &r_msg) == FALSE)
    {
        return -1;
    }

    if (r_msg.IdType == CAN_STD_ID)
    {
        msg->type = CAN_STANDARD;
    }
    else if (r_msg.IdType == CAN_EXT_ID)
    {
        msg->type = CAN_EXTENDED;
    }
    else
    {
        return -1;
    }

    msg->len = r_msg.DLC;
    msg->id = r_msg.Id;

    memcpy(&msg->data[0], &r_msg.Data[0], 8);

    return 0;
}

int can_set_filter(can_id_t idx, uint32_t id, uint32_t id_mask, int handle,
                   can_type_t type)
{
    (void)id_mask;
    uint32_t can_type;

    if (EXCEED_CAN_IDX(idx))
        return -1;

    CAN_T *dev = can_config[idx].dev;

    if (type == CAN_STANDARD)
    {
        can_type = CAN_STD_ID;
    }
    else if (type == CAN_EXTENDED)
    {
        can_type = CAN_EXT_ID;
    }
    else
    {
        return -1;
    }

    if (handle < 0 || handle > 31)
        return -1;

#ifdef CAN_M2355_TEST_CHIP

    if (handle > 9)
        return -1;

    if (CAN_SetRxMsg(dev, MSG(handle * 3), can_type, id) == FALSE)
    {
        printf("1\n\r");
        return -1;
    }
    if (CAN_SetRxMsg(dev, MSG(handle * 3 + 1), can_type, id) == FALSE)
    {
        printf("2\n\r");
        return -1;
    }
    if (CAN_SetRxMsg(dev, MSG(handle * 3 + 2), can_type, id) == FALSE)
    {
        printf("3\n\r");
        return -1;
    }
#else
    if (CAN_SetRxMsg(dev, MSG(handle), can_type, id) == FALSE)
        return -1;
#endif

    return 0;
}

int can_get_readable(can_id_t idx)
{
    uint32_t iidr;
    if (EXCEED_CAN_IDX(idx))
        return -1;

    CAN_T *dev = can_config[idx].dev;

    iidr = (dev->IIDR & 0xFF);

    if (!CAN_IsNewDataReceived(dev, iidr - 1))
    {
        return -1;
    }

    return iidr - 1;
}

int can_readable(can_id_t idx, int handle)
{
    (void)handle;

    if (EXCEED_CAN_IDX(idx))
        return -1;

    CAN_T *dev = can_config[idx].dev;

    if (!CAN_IsNewDataReceived(dev, handle))
    {
        return -1;
    }

    return 0;
}

int can_clear_isr(can_id_t idx)
{
    if (EXCEED_CAN_IDX(idx))
        return -1;

    CAN_T *dev = can_config[idx].dev;

    CAN_CLR_INT_PENDING_BIT(dev, ((dev->IIDR) - 1));

    return 0;
}

#if defined(USBD_VCOM)
extern const usbd_conf_t usbd_config[];

static int usbd_initialized = 0;

int usbd_is_init() { return usbd_initialized; }

int usbd_init(usbd_id_t idx)
{
    if (EXCEED_USBD_IDX(idx))
        return -1;

    const usbd_conf_t *conf = &usbd_config[idx];
    USBD_T *dev = conf->dev;

    const mod_desc_t *mod = find_mod_desc(dev, modtbl_usbd_pll);
    if (!mod)
    {
        return -1;
    }

    if (usbd_initialized)
        return 0;

    /* Select USB clock source as PLL and USB clock divider as 3 */
    CLK_SetModuleClock_S(mod->clkidx, mod->clksrc, mod->clkdiv);

    /* Select USBD */
    usb_select_mode_s(0);

    /* Enable USBD module clock */
    CLK_EnableModuleClock_S(mod->clkidx);

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    pin_function_s(PERIPH_GPIO_IDX(conf->vbus), PERIPH_GPIO_PIN(conf->vbus),
                   find_pin_af(dev, conf->vbus, pin_af_usbd));

    pin_function_s(PERIPH_GPIO_IDX(conf->d_n), PERIPH_GPIO_PIN(conf->d_n),
                   find_pin_af(dev, conf->d_n, pin_af_usbd));

    pin_function_s(PERIPH_GPIO_IDX(conf->d_p), PERIPH_GPIO_PIN(conf->d_p),
                   find_pin_af(dev, conf->d_p, pin_af_usbd));

    pin_function_s(PERIPH_GPIO_IDX(conf->otg_id), PERIPH_GPIO_PIN(conf->otg_id),
                   find_pin_af(dev, conf->otg_id, pin_af_usbd));

    usbd_initialized = 1;
    return 0;
}
#endif

#if defined(TRUSTZONE_NONSECURE)

uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t *sp = (uint32_t *)psp;

    /* It is casued by hardfault. Just process the hard fault */
    /* TODO: Implement your hardfault handle code here */

    /* Check the used stack */
    if (lr & 0x40UL)
    {
        /* Secure stack used */
        if (lr & 4UL)
        {
            sp = (uint32_t *)psp;
        }
        else
        {
            sp = (uint32_t *)msp;
        }
    }
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
    else
    {
        /* Non-secure stack used */
        if (lr & 4)
            sp = (uint32_t *)__TZ_get_PSP_NS();
        else
            sp = (uint32_t *)__TZ_get_MSP_NS();
    }
#endif

    printf("  HardFault!\n\n");
    printf("r0  = 0x%" PRIx32 "\n", sp[0]);
    printf("r1  = 0x%" PRIx32 "\n", sp[1]);
    printf("r2  = 0x%" PRIx32 "\n", sp[2]);
    printf("r3  = 0x%" PRIx32 "\n", sp[3]);
    printf("r12 = 0x%" PRIx32 "\n", sp[4]);
    printf("lr  = 0x%" PRIx32 "\n", sp[5]);
    printf("pc  = 0x%" PRIx32 "\n", sp[6]);
    printf("psr = 0x%" PRIx32 "\n", sp[7]);

    /* Or *sp to remove compiler warning */
    while (1U | *sp)
    {
    }

    return lr;
}
#endif

