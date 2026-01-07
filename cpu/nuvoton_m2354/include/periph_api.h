/**
 * @file periph_api.h
 * @brief Peripheral APIs
 *
 * @copyright Copyright (c) 2019 Security Platform Inc. All Rights Reserved.
 */
#ifndef _PERIPH_API_H_
#define _PERIPH_API_H_

#include "periph_def.h"
#include "nsc_api.h"
#include <time.h>

/** @defgroup uart UART
 * @brief UART library
 * @{
 */

/**
 * @param[in]   idx     uart_id_t
 * @retval  0   Success
 * @retval  -1  Failure
 * @brief   @~English Initialization function for UART
 *          @~korean 해당 UART를 초기화 한다
 */
int uart_init(uart_id_t idx, int baudrate);

void uart_fini(uart_id_t idx);

/**
 * @param[in]   idx     uart_id_t
 * @param[in]   buf     buffer
 * @param[in]   length  buffer of length
 * @retval  >0  Read length
 * @retval  -1  Failure
 * @brief   @~English Read data from UART
 *          @~korean 해당 UART에서 data를 읽는다
 */
int uart_read(uart_id_t idx, unsigned char *buf, unsigned int length);

__STATIC_INLINE int uart_irq_is_rx(uart_id_t idx)
{
    UART_T *dev = uart_config[idx].dev;

    if (dev->INTSTS & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))
        return 1;

    return 0;
}

__STATIC_INLINE int uart_irq_is_tx(uart_id_t idx)
{
    UART_T *dev = uart_config[idx].dev;

    if (dev->INTSTS & UART_INTSTS_THREINT_Msk)
        return 1;

    return 0;
}

__STATIC_INLINE void uart_irq_clear(uart_id_t idx)
{
    // FIXME: Ignore all other interrupt flags. Clear them. Otherwise, program
    // will get stuck in interrupt.
    UART_T *dev = uart_config[idx].dev;

    dev->INTSTS = dev->INTSTS;
    dev->FIFOSTS = dev->FIFOSTS;
}

int uart_irq_setup(uart_id_t idx, uart_irq_type_t type, int priority,
                   void (*isr)(void));

/**
 * @param[in]   idx     uart_id_t
 * @param[in]   buf     Buffer storing data to write
 * @param[in]   length  buffer length
 * @retval  >0  Writen length
 * @retval  -1  Failure
 * @brief   @~English Write data to UART
 *          @~Korean 해당 UART에 data를 Write한다
 */
int uart_write(uart_id_t idx, const unsigned char *buf, unsigned int length);

/**
 * @param[in]   idx uart_id_t
 * @retval  0   Success
 * @retval  -1  Failure
 * @brief   Deinitialization UART
 */
int uart_close(uart_id_t idx);

/**
 * @param[in]   idx uart_id_t
 * @retval  1   Data redable
 * @retval  0   No data
 * @retval  -1  Failure
 * @brief   @~korean 해당 UART에 읽을 Data가 있는지 여부를 return한다
 */
int uart_readable(uart_id_t idx);

/**
 * @param[in]   idx         uart_id_t
 * @param[in]   data_bits   data_bits(5~8)
 * @param[in]   parity      serial_parity_t
 * @param[in]   stop_bits   1 or 2
 * @retval  0   Success
 * @retval  -1  Failure
 * @brief   @~English Configuration a UART
 *          @~korean 해당 UART의 Data bits, Parity, Stop bits를 설정한다
 */
int uart_line_config(uart_id_t idx, int data_bits, int parity, int stop_bits);

/**
 * @param[in]   idx         uart_id_t
 * @retval  1   Success
 * @retval  0  Failure
 *          @~korean 해당 UART의 write 가능 여부를 return 한다.
 */
int uart_writable(uart_id_t idx);

/**
 * @param[in]   idx         uart_id_t
 * @retval  0   Success
 * @retval  -1  Failure
 *          @~korean 해당 UART의 auto flow control 기능을 활성화한다. cts, rts
 * 핀 설정되어 있어야 한다.
 */
int uart_set_flowcontrol(uart_id_t idx);

/**
 * @param[in]   idx         uart_id_t
 * @param[in]   val   		0 : low, 1 : high
 * @retval  0   Success
 * @retval  -1  Failure
 *          @~korean 해당 UART의 rts핀의 값을 직접 설정한다.(auto flow control을
 * 사용하지 않을 경우)
 */
int uart_set_rts(uart_id_t idx, int val);

/**
 * @param[in]   idx     uart_id_t
 * @param[in]   addr    address
 * @param[in]   buf     Buffer storing data to write
 * @param[in]   length  @~korean write할 length
 * @retval  >0  Writen length
 * @retval  -1  Failure
 * @brief   @~English Write data to RS485
 *          @~korean 해당 rs485에 data를 write한다
 */
int uart_rs485_write(uart_id_t idx, uint8_t *addr, const unsigned char *buf,
                     unsigned int length);

/**@}*/

/// @cond DO_NOT_DOCUMENT

int timer_init(timer_id_t idx);
void timer_fini(timer_id_t idx);
int timer_irq_setup(timer_id_t idx, int priority, void (*isr)(void));
int timer_start(timer_id_t idx, timer_mode_t mode, uint32_t freq);
int timer_stop(timer_id_t idx);

__STATIC_INLINE void timer_irq_clear(timer_id_t idx)
{
    TIMER_ClearIntFlag(timer_config[idx].dev);
}

int timer_read_counter(timer_id_t idx, uint32_t *count);

/// @endcond DO_NOT_DOCUMENT

/** @defgroup gpio GPIO
 * @brief GPIO library
 * @{
 */

/**
 * @param[in]   idx     gpio_id_t
 * @param[in]   vector  Callback function for ISR mode
 * @retval  0   Success
 * @retval  -1  Failure
 * @brief   Initialization GPIO
 */
int gpio_init(gpio_id_t idx);

/**
 * @param[in]   idx     gpio_id_t
 * @param[in]   value   1 or 0
 * @retval  0   Success
 * @retval  -1  Failure
 * @brief   Write data to GPIO
 */
int gpio_write(gpio_id_t idx, int value);

/**
 * @param[in]   idx gpio_id_t
 * @retval  0   Success
 * @retval  -1  Failure
 * @brief   Toggle GPIO
 */
int gpio_toggle(gpio_id_t idx);

/**
 * @param[in]   idx gpio_id_t
 * @return  int value
 * @brief   Read data from GPIO
 */
int gpio_read(gpio_id_t idx);


void gpio_set(gpio_id_t idx);
void gpio_clear(gpio_id_t idx);

/**@}*/

/** @defgroup i2c I2C Peripheral
 * @brief I2C library
 * @{
 */

/**
 * @param   idx I2C master index
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Initialization I2C Periphreral
 *          @~Korean I2C 장치를 초기화 한다
 */
int i2c_init(i2c_id_t idx);

/**
 * @param   idx I2C master index
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Initialization I2C Periphreral
 *          @~Korean I2C 장치를 종료 한다
 */
int i2c_deinit(i2c_id_t idx);

/**
 * @param   i2c I2C slave device context
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Open I2C Master of slave device
 *          @~Korean I2C Slave device의 Master를 연다
 */
int i2c_acquire(const i2c_dev_t *i2c);

/**
 * @param   i2c I2C slave device context
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Close I2C Master of slave device
 *          @~Korean 열었던 I2c Slave device의 Master를 닫는다
 */
int i2c_release(const i2c_dev_t *i2c);

/**
 * @param[in]   *i2c    Point to I2C peripheral
 * @param[in]   ctx     Point to I2C input/output context structure
 * @retval  -1  Failure
 * @retval  >0  A length of how many bytes have been transmitted
 * @brief   @~English Write multi bytes to EEPROM slave device
 *          @~Korean EEPROM slave device에 데이터를 전송한다
 */
int i2c_tx_bytes(const i2c_dev_t *i2c, const i2c_io_ctx_t *ctx);

/**
 * @param[in]   *i2c    Point to I2C peripheral
 * @param[in]   ctx     Point to I2C input/output context structure
 * @retval  -1  Failure
 * @retval  >0  A length of how many bytes have been received
 * @brief   @~English Read multi bytes from Slave device
 *          @~Korean Slave device에서 데이터를 읽는다
 */
int i2c_rx_bytes(const i2c_dev_t *i2c, const i2c_io_ctx_t *ctx);

/**
 * @param[in]   *i2c    Point to I2C peripheral
 * @param[in]   ctx     Point to I2C input/output context structure
 * @retval  -1  Failure
 * @retval  >0  A length of how many bytes have been transmitted
 * @brief   @~English Write data of register address from Slave device
 *          @~Korean Slave device에서 레지스터에 데이터를 전송한다.
 */
int i2c_write_regs(const i2c_dev_t *i2c, const i2c_io_ctx_t *ctx);

/**
 * @param[in]   *i2c    Point to I2C peripheral
 * @param[in]   ctx     Point to I2C input/output context structure
 * @retval  -1  Failure
 * @retval  >0  A length of how many bytes have been received
 * @brief   @~English Read data of register address from Slave device
 *          @~Korean Slave device에서 레지스터의 데이터를 읽는다
 */
int i2c_read_regs(const i2c_dev_t *i2c, const i2c_io_ctx_t *ctx);
/** @}*/

int i2c_read_byte(const i2c_dev_t *i2c_dev, uint8_t *data, uint32_t dlen);
int i2c_write_byte(const i2c_dev_t *i2c_dev, uint8_t *data, uint32_t dlen);

int i2c_read_reg(const i2c_dev_t *i2c_dev, uint16_t reg, uint16_t rlen,
                 uint8_t *data, uint32_t dlen);
int i2c_write_reg(const i2c_dev_t *i2c_dev, uint16_t reg, uint16_t rlen,
                  uint8_t *data, uint32_t dlen);

/** @defgroup spi SPI Peripheral
 * @brief SPI library
 * @{
 */

/**
 * @param   idx SPI master index
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Initialization SPI Periphreral
 *          @~Korean SPI 장치를 초기화 한다
 */
int spi_init(spi_id_t idx);

void spi_fini(spi_id_t idx);

/**
 * @param   idx SPI index
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Deinitialize SPI Periphreral
 *          @~Korean SPI 장치를 종료 한다
 */
void spi_deinit(spi_id_t idx);

/**
 * @param[in]   spi     SPI device structure
 * @param[in]   cont    transaction type
 * @param[out]  out     data which is transfered out via MOSI
 * @param[in]   in      data which is received from MISO
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Initialization SPI Periphreral
 *          @~Korean SPI 장치를 초기화 한다
 */
int spi_transfer_bytes(const spi_dev_t *spi, spi_io_transaction_t cont,
                       spi_stream_t *out, spi_stream_t *in);

/**
 * @param   spi SPI device structure
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English lock SPI device
 *          @~Korean SPI 장치의 배타적 접근권한을 획득한다
 */
int spi_acquire(const spi_dev_t *spi);

/**
 * @param   spi SPI device structure
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English unlock SPI device
 *          @~Korean SPI 장치의 배타적 접근권한을 반환한다
 */
int spi_release(const spi_dev_t *spi);

/** @}*/

/// @cond DO_NOT_DOCUMENT

int get_cpu_uid(unsigned char *buf, unsigned int buf_len,
                unsigned int *uid_len);

int flash_erase(uint32_t addr, uint32_t len);
int flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);

int flash_otp_write(uint32_t block, const uint8_t *buf, uint32_t len);
int flash_otp_read(uint32_t block, uint8_t *buf, uint32_t len);
int flash_otp_lock(uint32_t block);
int flash_otp_is_locked(uint32_t block);

// int sc_in it(sc_id_t idx); /* sung min, 2023-11-06 sc.h -> sc_init
// duplication, don't use smart card */
int sc_irq_setup(sc_id_t idx, int priority, void (*isr)(void));
int sc_close(uart_id_t idx);

/// @endcond DO_NOT_DOCUMENT

/** @defgroup lcd_peripheral LCD
 * @brief LCD library
 *
 * @~Korean LCD driver API는 BSP/Library/LCDLib/Source/LCDLIB.c에서 찾을 수
 * 있습니다
 * @{
 */

/**
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief   @~English Initialize LCD peripheral
 *          @~Korean LCD peripheral을 초기화 한다
 */
int lcd_init(void);

/**
 * @param[in]   cfg display options
 * @param[in]   msg The text format to show on the display
 * @param[in]   ... The text to show on the display
 * @retval  -1  Failure
 * @retval  0   Success
 * @brief @~English Display text on LCD as follow options
 *        @~Korean 옵션에 따라 글자를 LCD에 출력합니다.
 * @sa lcd_7seg_zone
 * @sa lcd_7seg_align
 * @sa lcd_7seg_offset
 */
int lcd_printf(uint32_t cfg, const char *msg, ...);

/**
 * @param[in]   zone    The assigned number of display area
 * @param[in]   num     The number to show on display
 * @return  none
 * @brief @~English Display number on LCD
 *        @~Korean 숫자를 LCD에 출력합니다
 */
void lcd_print_number(uint32_t zone, long long num);

/**
 * @param[in]   zone    The assigned number of display area
 * @param[in]   idx     The requsted display position in zone
 * @param[in]   ch      The character to show on display\
 * @return  none
 * @brief @~English Display character on LCD
 *        @~Korean 글자를 LCD에 출력합니다
 */
void lcd_putchar(uint32_t zone, uint32_t idx, uint8_t ch);

/**
 * @param[in]   zone    The assigned number of display area
 * @param[in]   idx     The requsted display position in zone
 * @param[in]   op      0: Not display symbol
 *                      1: Display symbol
 * @return  none
 * @brief @~English Display symbol on LCD
 *        @~Korean LCD의 심볼을 켜거나 끕니다
 */
void lcd_set_symbol(uint32_t zone, uint32_t idx, uint32_t op);

/** @}*/

int lcd_irq_setup(int priority, void (*isr)(void));
void lcd_IRQHandler(void);

/** @defgroup kse KSE
 * @brief Keypair Secure Element library
 *
 * @~Korean keypair API는 동봉된 별도의 파일로 제공됩니다
 * @{
 */

/** @}*/

/** @defgroup fatfs FatFs
 * @brief Generic FAT Filesystem Module
 *
 * @~Korean FatFs API는 다음의 링크에서 찾을 수 있습니다.<br>
 *          http://elm-chan.org/fsw/ff/00index_e.html<br>
 *          - 지원하지 않는 API 목록
 *              -# f_fdisk
 *              -# f_getlabel
 *              -# f_setlabel
 *              -# f_setcp
 *              -# f_getcwd
 * @{
 */
/** @}*/

/** @defgroup lfs lfs
 * @brief Little File System Module
 *
 * @~Korean little file system API는 다음의 링크에서 찾을 수 있습니다.<br>
 *          https://github.com/ARMmbed/littlefs<br>
 * 			tag : v2.2.1
 * @{
 */

/** @}*/

int can_init(can_id_t idx, uint32_t baudrate);
int can_irq_setup(can_id_t idx, can_irq_type_t type, int priority,
                  void (*isr)(void));
int can_deinit(can_id_t idx);
int can_set_filter(can_id_t idx, uint32_t id, uint32_t id_mask, int handle,
                   can_type_t type);
int can_read(can_id_t idx, can_message_t *msg, int handle);
int can_write(can_id_t idx, can_message_t *msg);
int can_get_readable(can_id_t idx);
int can_readable(can_id_t idx, int handle);
int can_clear_isr(can_id_t idx);

int adc_init(adc_id_t idx);
void adc_fini(adc_id_t idx);
int adc_read(adc_id_t idx);

int usbd_init(usbd_id_t idx);
int usbd_is_init();

#endif


