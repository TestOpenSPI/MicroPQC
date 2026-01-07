/**
 * @file periph_def.h
 * @brief The definitions for Peripheral APIs
 * @copyright Copyright (c) 2019 Security Platform Inc. All Rights Reserved.
 */
#ifndef _PERIPH_DEF_H_
#define _PERIPH_DEF_H_

#include "NuMicro.h"
#include "pin_def.h"
#include "cpu.h"

typedef int gpio_id_t;
typedef int uart_id_t;
typedef int spi_id_t;
typedef int i2c_id_t;
typedef int sc_id_t;
typedef int timer_id_t;
typedef int lcd_id_t;
typedef int can_id_t;
typedef int adc_id_t;
typedef int usbd_id_t;

typedef enum {
	GPIO_INTR_TYPE_RISING = 0,
	GPIO_INTR_TYPE_FALLING,
	GPIO_INTR_TYPE_BOTH
} gpio_intr_type_t;

/**
 * @struct gpio_conf_t
 * @brief Store the data for GPIO
 */
typedef struct {
    gpio_t gpio;    /*!< gpio index */
    int mode;       /*!< gpio mode : 0=input, 1=output, 2=open-drain, 3=quasi-bidirectional */
    int invert;
    int value;
} gpio_conf_t;

extern const gpio_conf_t gpio_config[];

/**
 * @brief uart_type_t
 *
 */
typedef enum {
    UART_TYPE_ASYNC         = 0,
    UART_TYPE_SYNC,
	UART_TYPE_485TRANCEIVER,
    UART_TYPE_485MASTER     = 0x10,    /*!< RS485 Master Mode */
    UART_TYPE_485SLAVE_AAD,            /*!< RS485 Slave AAD Mode */
    UART_TYPE_485SLAVE_NMM,            /*!< RS485 Slave NMM Mode */
} uart_type_t;

/**
 * @brief Store the data for UART
 *
 */
typedef struct {
    UART_T *dev;            /*!< UART Pointer */
    int baudrate;           /*!< UART baudrate */
    gpio_t rx_pin;          /*!< RX Pin */
    gpio_t tx_pin;          /*!< TX Pin */
    gpio_t rts_pin;         /*!< RTS Pin */
    gpio_t cts_pin;         /*!< CTS Pin */
	int gpio_dir_id;         /*!< DE/RE Pin for RS485 tranceiver, Receiver Output Enable Active LOW, Driver Output Enable Active HIGH */

    uart_type_t type;
    int slaveaddr;           /*!< address for RS485 */
} uart_conf_t;

typedef enum {
	UART_IRQ_RX,
	UART_IRQ_TX,
} uart_irq_type_t;

extern  const uart_conf_t uart_config[];

/**
 * @brief spi_stream_t
 *
 */
typedef struct _spi_stream {
    void *buf;                    /*!< transaction buffer pointer */
    uint32_t len;                 /*!< transaction buffer length */
} spi_stream_t;

/**
 * @brief spi_dev_t
 *
 */
typedef struct spi_dev {
    spi_id_t id;                 /*!< SPI master index */
    int slave_idx;               /*!< SPI slave index */
    int mode;                    /*!< SPI mode */
    uint32_t width;              /*!< SPI I/O width */
    uint32_t clk;                /*!< SPI clock */
} spi_dev_t;

/**
 * @brief spi_io_transaction_t
 *
 */
typedef enum {
    SPI_IO_TRANSACTION_FINISH,    /*!< for an one-shot transaction or finishing a continued transaction */
    SPI_IO_TRANSACTION_CONTINUE   /*!< a transaction will be continued */
} spi_io_transaction_t;

typedef enum{
	SPI_TYPE_SINGLE,
	SPI_TYPE_QUAD,
}spi_type;

/**
 * @struct spi_conf_t
 * @brief Store the data for SPI
 */
typedef struct {
    void *dev;                   /*!< SPI pointer */
	spi_type type;				  /*!< SPI type */
    gpio_t mosi;                  /*!< MOSI pin */
    gpio_t miso;                  /*!< MISO pin */
	gpio_t mosi1;                  /*!< MOSI1 pin of QSPI*/
    gpio_t miso1;                  /*!< MISO1 pin of QSPI*/
    gpio_t sclk;                  /*!< SCLK pin */
    const gpio_t *ssel;           /*!< array of slave pin */
    int ssel_cnt;                 /*!< the number of slaves */
    int is_master;                /*!< currently support SPI master mode */

    /* these are slave configuration */
    int mode;                     /*!< for SPI slave mode (not supported) */
    int width;                    /*!< for SPI slave mode (not supported) */
} spi_conf_t;

extern const spi_conf_t spi_config[];

/**
 * @struct i2c_io_ctx_t
 * @brief Store the data for I2C IO
 */
typedef struct {
    uint8_t slave_addr;
  	
	  struct {
		uint16_t address;
		uint8_t len;
	} reg;

    struct {
        uint16_t len;
        uint8_t *buf;
    } tx;
    struct {
        uint16_t len;
        uint8_t *buf;
    } rx;
} i2c_io_ctx_t;

/**
 * @struct i2c_dev_t
 * @brief Store I2C slave device information
 */
typedef struct {
    i2c_id_t id;
    int32_t slave_idx;
    uint8_t slave_addr;
    uint32_t clk;
} i2c_dev_t;

/**
 * @struct i2c_conf_t
 * @brief Store I2c Peripheral information
 */
typedef struct {
    I2C_T *dev;
    gpio_t scl;
    gpio_t sda;
    int is_master;
} i2c_conf_t;

extern const i2c_conf_t i2c_config[];

/**
 * @struct sc_conf_t
 * @brief Store smartcard Peripheral information
 */
typedef struct {
    SC_T *dev;
    gpio_t pwr;
    gpio_t rst;
    gpio_t dat;
    gpio_t clk;

} sc_conf_t;

extern const sc_conf_t sc_config[];

typedef struct {
	EADC_T *dev;
	gpio_t pin;
	int adint;
} adc_conf_t;

extern const adc_conf_t adc_config[];

typedef struct {
	USBD_T *dev;
	gpio_t vbus;
	gpio_t d_n;
	gpio_t d_p;
	gpio_t otg_id;
} usbd_conf_t;

#if defined(USBD_VCOM)
extern const usbd_conf_t usbd_config[];
#endif
// LCD_7SEG_ZONE field
#define LCD_7SEG_ZONE_MSK           0x000F
#define LCD_7SEG_ZONE_POS           0UL

/**
 * @enum lcd_7seg_zone
 * @brief LCD zone
 */
typedef enum {
	LCD_7SEG_ZONE_UPPER = 0x0000,   /*!< The upper side LCD */
	LCD_7SEG_ZONE_LOWER = 0x0001,   /*!< The lower side LCD */
} lcd_7seg_zone;

// LCD_7SEG_ALIGN field
#define LCD_7SEG_ALIGN_MSK          0x00F0
#define LCD_7SEG_ALIGN_POS          4UL

/**
 * @enum lcd_7seg_align
 * @brief @~English Text Alignment option
 *        @~Korean 텍스트 정렬 옵션
 */
typedef enum {
	LCD_7SEG_ALIGN_LEFT  = 0x0000,  /*!< Align text on left side */
	LCD_7SEG_ALIGN_RIGHT = 0x0010,  /*!< Align text on right side */
} lcd_7seg_align;

// LCD_7SEG_OFFSET field
#define LCD_7SEG_OFFSET_MSK         0x0F00
#define LCD_7SEG_OFFSET_POS         8UL

/**
 * @enum lcd_7seg_offset
 * @brief @~English Add empty space in front of text
 *                  Can not use this option with LCD_7SEG_ALIGN_RIGHT
 *        @~Korean 텍스트 앞에 공백 문자를 추가 합니다
 *                 LCD_7SEG_ALIGN_RIGHT 옵션과 함께 사용할 수 없습니다
 */
typedef enum {
	LCD_7SEG_OFFSET_0 = 0x0000, /*!< Zero empty space */
	LCD_7SEG_OFFSET_1 = 0x0100, /*!< One empty Space */
	LCD_7SEG_OFFSET_2 = 0x0200, /*!< Two empty Spaces */
	LCD_7SEG_OFFSET_3 = 0x0300, /*!< Three empty Spaces */
	LCD_7SEG_OFFSET_4 = 0x0400, /*!< Four empty Spaces */
	LCD_7SEG_OFFSET_5 = 0x0500, /*!< Five empty Spaces */
} lcd_7seg_offset;

#define LCD_7SEG_ZONE_UPPER_MAX_STR   2
#define LCD_7SEG_ZONE_LOWER_MAX_STR   10

#define LCD_7SEG_ZONE_UPPER_MAX_DIGIT 2
#define LCD_7SEG_ZONE_LOWER_MAX_DIGIT 6

/**
  \brief   Mask and shift a bit field value for use in the 7 segment bit range.
  \param[in] field  Name of the 7 segment bit field.
  \param[in] value  Value of the bit field. This parameter is interpreted as an uint32_t type.
  \return           Masked and shifted value.
*/
// #define LCD_7SEG_V2F(field, value)    (((uint32_t)(value) << field ## _POS) & field ## _MSK)
#define LCD_7SEG_V2F(field, value)    ((uint32_t)(value) & field ## _MSK)
/**
  \brief     Mask and shift the 7 segment value to extract a bit field value.
  \param[in] field  Name of the 7 segment bit field.
  \param[in] value  Value of the 7 segment. This parameter is interpreted as an uint32_t type.
  \return           Masked and shifted bit field value.
*/
// #define LCD_7SEG_F2V(field, value)    (((uint32_t)(value) & field ## _MSK) >> field ## _POS)
#define LCD_7SEG_F2V(field, value)    ((uint32_t)(value) & field ## _MSK)

typedef struct {
	LCD_T *dev;
	S_LCD_CFG_T cfg;
	gpio_t com0;
	gpio_t com1;
	gpio_t com2;
	gpio_t com3;
	gpio_t com4;
	gpio_t com5;
	gpio_t com6;
	gpio_t com7;
	gpio_t s0;
	gpio_t s1;
	gpio_t s2;
	gpio_t s3;
	gpio_t s4;
	gpio_t s5;
	gpio_t s6;
	gpio_t s7;
	gpio_t s8;
	gpio_t s9;
	gpio_t s10;
	gpio_t s11;
	gpio_t s12;
	gpio_t s13;
	gpio_t s14;
	gpio_t s15;
	gpio_t s16;
	gpio_t s17;
	gpio_t s18;
	gpio_t s19;
	gpio_t s20;
	gpio_t s21;
	gpio_t s22;
	gpio_t s23;
	gpio_t s24;
	gpio_t s25;
	gpio_t s26;
	gpio_t s27;
	gpio_t s28;
	gpio_t s29;
	gpio_t s30;
	gpio_t s31;
	gpio_t s32;
	gpio_t s33;
	gpio_t s34;
	gpio_t s35;
	gpio_t s36;
	gpio_t s37;
	gpio_t s38;
	gpio_t s39;
} lcd_conf_t;

extern const lcd_conf_t lcd_config;

/// @cond DO_NOT_DOCUMENT

#define FLASH_PAGE_SIZE     0x800
#define FLASH_BLOCK_SIZE   0x2000
#define FLASH_BANK_SIZE   0x20000
#define FLASH_CHIP_SIZE   0x40000

#define OTP_BLOCK_SIZE 8
/// @endcond DO_NOT_DOCUMENT

/**
 * @enum I2C EEPROM model types
 *
 * @brief Possible model type for I2C EEPROM
 */
typedef enum {
    I2C_EEPROM_TYPE_UNKNOWN,
    I2C_EEPROM_TYPE_AT24CM01, /*!< AT24CM01, 1MB model ID */
    I2C_EEPROM_TYPE_AT24CM02, /*!< AT24CM02, 2MB model ID */
} i2c_eeprom_types;

typedef enum {
	TIMER_MODE_ONESHOT,
	TIMER_MODE_PERIODIC,
	TIMER_MODE_CONTINUOUS,
} timer_mode_t;

typedef struct {
	TIMER_T *dev;
	uint32_t mode;
	uint32_t freq;
} timer_conf_t;

extern const timer_conf_t timer_config[];


typedef enum{
	CAN_IRQ_INTERRUPT = 0x01,		// CAN_CON_IE_Msk Module interrupt enable.
	// CAN_IRQ_STATUS = 0x02,			// CAN_CON_SIE_Msk Status change interrupt enable.
	// CAN_IRQ_ERROR = 0x04,			// CAN_CON_EIE_Msk Error interrupt enable.
}can_irq_type_t;

typedef struct {
    CAN_T *dev;         /*!< CAN Pointer */
    gpio_t rx;          /*!< RX Pin */
    gpio_t tx;          /*!< TX Pin */
}can_conf_t;

extern const can_conf_t can_config[];

typedef enum{
	CAN_STANDARD,	
	CAN_EXTENDED,
}can_type_t;

typedef enum{
	CAN_STATUS_RXOK,
	CAN_STATUS_TXOK,
	CAN_STATUS_WARN,
	CAN_STATUS_OFF,
	CAN_STATUS_RX_MSG,
	CAN_STATUS_WAKEUP,
}can_status_t;

typedef struct{
	uint32_t id;
	uint8_t data[8];
	int len;
	can_type_t type;
}can_message_t;

typedef enum{
	CAN_SUCCESS = 0,
	CAN_FAILURE,
	CAN_EMPTY_MSG,
}can_result_t;


int gpio_irq_setup(gpio_id_t idx, gpio_intr_type_t type, int priority, void (*isr)(void));
// int gpio_irq_setup(GPIO_T *port, uint32_t pin, int type, int priority, void (*isr)(void));

// void helloworldnn(void);
// #define CAN_M2355_TEST_CHIP		
#endif

