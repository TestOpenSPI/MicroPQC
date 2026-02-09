#include "periph_def.h"
#include "periph_conf.h"

__WEAK
const uart_conf_t uart_config[] =
{
	[UART_STDIO_DEV] =
	{
		.dev      = UART0,
		.baudrate = 115200,
		.rx_pin   = PERIPH_GPIO_DECLARE(PB, 12),
		.tx_pin   = PERIPH_GPIO_DECLARE(PB, 13),
		.rts_pin  = PERIPH_GPIO_UNDEF,
		.cts_pin  = PERIPH_GPIO_UNDEF,
		.type     = UART_TYPE_ASYNC,
		
	},
};

/* To Use stdio function, set this value */
__WEAK const int uart_stdio_dev = UART_STDIO_DEV;

__WEAK
const gpio_conf_t gpio_config[] =
{

};

#define DECLARE_SPI_SSEL(ss) .ssel = ss, .ssel_cnt = sizeof(ss)/sizeof(ss[0])

static const gpio_t spi0_slave[] = {
	PERIPH_GPIO_DECLARE(PA, 3),
};

const spi_conf_t spi_config[] =
{
	[SPI_MASTER_0] =
	{
		.dev = SPI0,
		.miso = PERIPH_GPIO_DECLARE(PA, 1),
		.mosi = PERIPH_GPIO_DECLARE(PA, 0),
		.sclk = PERIPH_GPIO_DECLARE(PA, 2),
		DECLARE_SPI_SSEL(spi0_slave),
		.is_master = 1,
	},
};

const spi_dev_t spi_nor_dev =
{
	.id        = SPI_MASTER_0,
	.slave_idx = SPI_SLAVE_0_NOR_FLASH,
	.mode      = 0,
	.width     = 8,
	.clk       = 2000000
};

const timer_conf_t timer_config[] =
{
	[TIMER_WATCHDOG] =
	{
		.dev = TIMER1,
		.mode = TIMER_MODE_PERIODIC, // one shot, periodic, ...
		.freq = 1
	}
};


/* Don't touch these value */
__WEAK const int numof_UART = sizeof(uart_config)/sizeof(uart_config[0]);
__WEAK const int numof_GPIO = sizeof(gpio_config)/sizeof(gpio_config[0]);
const int numof_SPI = sizeof(spi_config)/sizeof(spi_config[0]);
// const int numof_I2C = sizeof(i2c_config)/sizeof(i2c_config[0]);
// const int numof_I2C_EEPROM = sizeof(i2c_eeprom_dev)/sizeof(i2c_eeprom_dev[0]);
// const int numof_SC = sizeof(sc_config)/sizeof(sc_config[0]);
const int numof_TIMER = sizeof(timer_config)/sizeof(timer_config[0]);
// const int numof_CAN = sizeof(can_config)/sizeof(can_config[0]);
