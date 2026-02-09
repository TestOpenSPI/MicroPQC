#include "pin_def.h"
#include "cpu.h"

__WEAK
const pin_af_t pin_af_uart_rx[] = {
	_MFP(UART0, PA, 0, RXD),
	_MFP(UART0, PA, 6, RXD),
	_MFP(UART0, PA,15, RXD),
	_MFP(UART0, PB, 8, RXD),
	_MFP(UART0, PB,12, RXD),
	_MFP(UART0, PC,11, RXD),
	_MFP(UART0, PD, 2, RXD),
	_MFP(UART0, PF, 2, RXD),
	_MFP(UART0, PH,11, RXD),

	_MFP(UART1, PA, 2, RXD),
	_MFP(UART1, PA, 8, RXD),
	_MFP(UART1, PB, 2, RXD),
	_MFP(UART1, PB, 6, RXD),
	_MFP(UART1, PC, 8, RXD),
	_MFP(UART1, PD, 6, RXD),
	_MFP(UART1, PD,10, RXD),
	_MFP(UART1, PF, 1, RXD),
	_MFP(UART1, PH, 9, RXD),

	_MFP(UART2, PB, 0, RXD),
	_MFP(UART2, PC, 0, RXD),
	_MFP(UART2, PC, 4, RXD),
	_MFP(UART2, PD,12, RXD),
	_MFP(UART2, PE, 9, RXD),
	_MFP(UART2, PE,15, RXD),
	_MFP(UART2, PF, 5, RXD),

	_MFP(UART3, PB,14, RXD),
	_MFP(UART3, PC, 2, RXD),
	_MFP(UART3, PC, 9, RXD),
	_MFP(UART3, PD, 0, RXD),
	_MFP(UART3, PE, 0, RXD),
	_MFP(UART3, PE,11, RXD),

	_MFP(UART4, PA, 2, RXD),
	_MFP(UART4, PA,13, RXD),
	_MFP(UART4, PB,10, RXD),
	_MFP(UART4, PC, 4, RXD),
	_MFP(UART4, PC, 6, RXD),
	_MFP(UART4, PF, 6, RXD),
	_MFP(UART4, PH,11, RXD),

	_MFP(UART5, PA, 4, RXD),
	_MFP(UART5, PB, 4, RXD),
	_MFP(UART5, PE, 6, RXD),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_uart_tx[] = {
	_MFP(UART0, PA, 1, TXD),
	_MFP(UART0, PA, 7, TXD),
	_MFP(UART0, PA,14, TXD),
	_MFP(UART0, PB, 9, TXD),
	_MFP(UART0, PB,13, TXD),
	_MFP(UART0, PC,12, TXD),
	_MFP(UART0, PD, 3, TXD),
	_MFP(UART0, PF, 3, TXD),
	_MFP(UART0, PH,10, TXD),

	_MFP(UART1, PA, 3, TXD),
	_MFP(UART1, PA, 9, TXD),
	_MFP(UART1, PB, 3, TXD),
	_MFP(UART1, PB, 7, TXD),
	_MFP(UART1, PD, 7, TXD),
	_MFP(UART1, PD,11, TXD),
	_MFP(UART1, PE,13, TXD),
	_MFP(UART1, PF, 0, TXD),
	_MFP(UART1, PH, 8, TXD),

	_MFP(UART2, PB, 1, TXD),
	_MFP(UART2, PB, 5, TXD),
	_MFP(UART2, PC, 1, TXD),
	_MFP(UART2, PC, 5, TXD),
	_MFP(UART2, PC,13, TXD),
	_MFP(UART2, PE, 8, TXD),
	_MFP(UART2, PE,14, TXD),
	_MFP(UART2, PF, 4, TXD),
	_MFP(UART2, PB, 5, TXD),

	_MFP(UART3, PB,15, TXD),
	_MFP(UART3, PC, 3, TXD),
	_MFP(UART3, PC,10, TXD),
	_MFP(UART3, PD, 1, TXD),
	_MFP(UART3, PE, 1, TXD),
	_MFP(UART3, PE,10, TXD),

	_MFP(UART4, PA, 3, TXD),
	_MFP(UART4, PA,12, TXD),
	_MFP(UART4, PB,11, TXD),
	_MFP(UART4, PC, 5, TXD),
	_MFP(UART4, PC, 7, TXD),
	_MFP(UART4, PF, 7, TXD),
	_MFP(UART4, PH,10, TXD),

	_MFP(UART5, PA, 5, TXD),
	_MFP(UART5, PB, 5, TXD),
	_MFP(UART5, PE, 7, TXD),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_uart_rts[] = {
	_MFP(UART0, PA, 4, nRTS),
	_MFP(UART0, PB,10, nRTS),
	_MFP(UART0, PB,14, nRTS),
	_MFP(UART0, PC, 6, nRTS),

	_MFP(UART1, PA, 0, nRTS),
	_MFP(UART1, PB, 8, nRTS),
	_MFP(UART1, PE,12, nRTS),

	_MFP(UART2, PC, 3, nRTS),
	_MFP(UART2, PD, 8, nRTS),
	_MFP(UART2, PF, 4, nRTS),

	_MFP(UART3, PB,13, nRTS),
	_MFP(UART3, PD, 3, nRTS),
	_MFP(UART3, PH, 8, nRTS),

	_MFP(UART4, PE, 0, nRTS),
	_MFP(UART4, PE,13, nRTS),

	_MFP(UART5, PB, 3, nRTS),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_uart_cts[] = {
	_MFP(UART0, PA, 5, nCTS),
	_MFP(UART0, PB,11, nCTS),
	_MFP(UART0, PB,15, nCTS),
	_MFP(UART0, PC, 7, nCTS),

	_MFP(UART1, PA, 1, nCTS),
	_MFP(UART1, PB, 9, nCTS),
	_MFP(UART1, PE,11, nCTS),

	_MFP(UART2, PC, 2, nCTS),
	_MFP(UART2, PD, 9, nCTS),
	_MFP(UART2, PF, 5, nCTS),

	_MFP(UART3, PB,12, nCTS),
	_MFP(UART3, PD, 2, nCTS),
	_MFP(UART3, PH, 9, nCTS),

	_MFP(UART4, PC, 8, nCTS),
	_MFP(UART4, PE, 1, nCTS),

	_MFP(UART5, PB, 2, nCTS),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_spi_mosi[] = {
	_MFP(SPI0, PA, 0, MOSI),
	_MFP(SPI0, PB,12, MOSI),
	_MFP(SPI0, PD, 0, MOSI),
	_MFP(SPI0, PF, 6, MOSI),

	_MFP(SPI1, PB, 4, MOSI),
	_MFP(SPI1, PC, 2, MOSI),
	_MFP(SPI1, PC, 6, MOSI),
	_MFP(SPI1, PD, 6, MOSI),
	_MFP(SPI1, PE, 0, MOSI),
	_MFP(SPI1, PH, 5, MOSI),

	_MFP(SPI2, PA, 8, MOSI),
	_MFP(SPI2, PA,15, MOSI),
	_MFP(SPI2, PE,10, MOSI),
	_MFP(SPI2, PF,11, MOSI),

	_MFP(SPI3, PB, 8, MOSI),
	_MFP(SPI3, PC,11, MOSI),
	_MFP(SPI3, PE, 2, MOSI),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_spi_miso[] = {
	_MFP(SPI0, PA, 1, MISO),
	_MFP(SPI0, PB,13, MISO),
	_MFP(SPI0, PD, 1, MISO),
	_MFP(SPI0, PF, 7, MISO),

	_MFP(SPI1, PB, 5, MISO),
	_MFP(SPI1, PC, 3, MISO),
	_MFP(SPI1, PC, 7, MISO),
	_MFP(SPI1, PD, 7, MISO),
	_MFP(SPI1, PE, 1, MISO),
	_MFP(SPI1, PH, 4, MISO),

	_MFP(SPI2, PA, 9, MISO),
	_MFP(SPI2, PA,14, MISO),
	_MFP(SPI2, PE, 9, MISO),
	_MFP(SPI2, PG, 4, MISO),

	_MFP(SPI3, PB, 9, MISO),
	_MFP(SPI3, PC,12, MISO),
	_MFP(SPI3, PE, 3, MISO),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_spi_sclk[] = {
	_MFP(SPI0, PA, 2, CLK    ),
	_MFP(SPI0, PA, 4, I2SMCLK),
	_MFP(SPI0, PB, 0, I2SMCLK),
	_MFP(SPI0, PB,11, I2SMCLK),
	_MFP(SPI0, PB,14, CLK    ),
	_MFP(SPI0, PD, 2, CLK    ),
	//_MFP(SPI0, PD,13, I2SMCLK),
	_MFP(SPI0, PD,14, I2SMCLK),
	_MFP(SPI0, PF, 8, CLK    ),
	_MFP(SPI0, PF,10, I2SMCLK),

	_MFP(SPI1, PA, 5, I2SMCLK),
	_MFP(SPI1, PA, 7, CLK    ),
	_MFP(SPI1, PB, 1, I2SMCLK),
	_MFP(SPI1, PB, 3, CLK    ),
	_MFP(SPI1, PC, 1, CLK    ),
	_MFP(SPI1, PC, 4, I2SMCLK),
	_MFP(SPI1, PD, 5, CLK    ),
	//_MFP(SPI1, PD,13, I2SMCLK),
	_MFP(SPI1, PH, 6, CLK    ),
	_MFP(SPI1, PH, 8, CLK    ),
	_MFP(SPI1, PH,10, I2SMCLK),

	_MFP(SPI2, PA,10, CLK    ),
	_MFP(SPI2, PA,13, CLK    ),
	_MFP(SPI2, PC,13, I2SMCLK),
	_MFP(SPI2, PE, 8, CLK    ),
	_MFP(SPI2, PE,12, I2SMCLK),
	_MFP(SPI2, PG, 3, CLK    ),

	_MFP(SPI3, PB, 1, I2SMCLK),
	_MFP(SPI3, PB,11, CLK    ),
	_MFP(SPI3, PC,10, CLK    ),
	_MFP(SPI3, PD,14, I2SMCLK),
	_MFP(SPI3, PE, 4, CLK    ),
	_MFP(SPI3, PE, 6, I2SMCLK),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_spi_ssel[] = {
	_MFP(SPI0, PA, 3, SS),
	_MFP(SPI0, PB,15, SS),
	_MFP(SPI0, PD, 3, SS),
	_MFP(SPI0, PF, 9, SS),

	_MFP(SPI1, PA, 6, SS),
	_MFP(SPI1, PB, 2, SS),
	_MFP(SPI1, PC, 0, SS),
	_MFP(SPI1, PD, 4, SS),
	_MFP(SPI1, PH, 7, SS),
	_MFP(SPI1, PH, 9, SS),

	_MFP(SPI2, PA,11, SS),
	_MFP(SPI2, PA,12, SS),
	_MFP(SPI2, PE,11, SS),
	_MFP(SPI2, PG, 2, SS),

	_MFP(SPI3, PB,10, SS),
	_MFP(SPI3, PC, 9, SS),
	_MFP(SPI3, PE, 5, SS),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_i2c_sda[] = {
	_MFP(I2C0, PA, 4, SDA),
	_MFP(I2C0, PB, 4, SDA),
	_MFP(I2C0, PC, 0, SDA),
	_MFP(I2C0, PC, 8, SDA),
	_MFP(I2C0, PC,11, SDA),
	_MFP(I2C0, PD, 6, SDA),
	_MFP(I2C0, PF, 2, SDA),

	_MFP(I2C1, PA, 2, SDA),
	_MFP(I2C1, PA, 6, SDA),
	_MFP(I2C1, PA,13, SDA),
	_MFP(I2C1, PB, 0, SDA),
	_MFP(I2C1, PB,10, SDA),
	_MFP(I2C1, PC, 4, SDA),
	_MFP(I2C1, PD, 4, SDA),
	_MFP(I2C1, PE, 0, SDA),
	_MFP(I2C1, PF, 1, SDA),
	_MFP(I2C1, PG, 3, SDA),

	_MFP(I2C2, PA, 0, SDA),
	_MFP(I2C2, PA,10, SDA),
	_MFP(I2C2, PA,15, SDA),
	_MFP(I2C2, PB,12, SDA),
	_MFP(I2C2, PD, 0, SDA),
	_MFP(I2C2, PD, 8, SDA),
	_MFP(I2C2, PH, 9, SDA),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_i2c_scl[] = {
	_MFP(I2C0, PA, 5, SCL),
	_MFP(I2C0, PB, 5, SCL),
	_MFP(I2C0, PC, 1, SCL),
	_MFP(I2C0, PC,12, SCL),
	_MFP(I2C0, PD, 7, SCL),
	_MFP(I2C0, PE,13, SCL),
	_MFP(I2C0, PF, 3, SCL),

	_MFP(I2C1, PA, 3, SCL),
	_MFP(I2C1, PA, 7, SCL),
	_MFP(I2C1, PA,12, SCL),
	_MFP(I2C1, PB, 1, SCL),
	_MFP(I2C1, PB,11, SCL),
	_MFP(I2C1, PC, 5, SCL),
	_MFP(I2C1, PD, 5, SCL),
	_MFP(I2C1, PE, 1, SCL),
	_MFP(I2C1, PF, 0, SCL),
	_MFP(I2C1, PG, 2, SCL),

	_MFP(I2C2, PA, 1, SCL),
	_MFP(I2C2, PA,11, SCL),
	_MFP(I2C2, PA,14, SCL),
	_MFP(I2C2, PB,13, SCL),
	_MFP(I2C2, PD, 1, SCL),
	_MFP(I2C2, PD, 9, SCL),
	_MFP(I2C2, PH, 8, SCL),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_sc_clk[] = {
	_MFP(SC0, PA, 0, CLK),
	_MFP(SC0, PB, 5, CLK),
	_MFP(SC0, PE, 2, CLK),
	_MFP(SC0, PF, 6, CLK),

	_MFP(SC1, PB,12, CLK),
	_MFP(SC1, PC, 0, CLK),
	_MFP(SC1, PD, 4, CLK),

	_MFP(SC2, PA, 6, CLK),
	_MFP(SC2, PA, 8, CLK),
	_MFP(SC2, PA,15, CLK),
	_MFP(SC2, PD, 0, CLK),
	_MFP(SC2, PE, 0, CLK),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_sc_dat[] = {
	_MFP(SC0, PA, 1, DAT),
	_MFP(SC0, PB, 4, DAT),
	_MFP(SC0, PE, 3, DAT),
	_MFP(SC0, PF, 7, DAT),

	_MFP(SC1, PB,13, DAT),
	_MFP(SC1, PC, 1, DAT),
	_MFP(SC1, PD, 5, DAT),

	_MFP(SC2, PA, 7, DAT),
	_MFP(SC2, PA, 9, DAT),
	_MFP(SC2, PA,14, DAT),
	_MFP(SC2, PD, 1, DAT),
	_MFP(SC2, PE, 1, DAT),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_sc_pwr[] = {
	_MFP(SC0, PA, 3, PWR),
	_MFP(SC0, PB, 2, PWR),
	_MFP(SC0, PE, 5, PWR),
	_MFP(SC0, PF, 9, PWR),

	_MFP(SC1, PB,15, PWR),
	_MFP(SC1, PC, 3, PWR),
	_MFP(SC1, PD, 7, PWR),

	_MFP(SC2, PA,11, PWR),
	_MFP(SC2, PA,12, PWR),
	_MFP(SC2, PC, 7, PWR),
	_MFP(SC2, PD, 3, PWR),
	_MFP(SC2, PH, 8, PWR),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_sc_rst[] = {
	_MFP(SC0, PA, 2, RST),
	_MFP(SC0, PB, 3, RST),
	_MFP(SC0, PE, 4, RST),
	_MFP(SC0, PF, 8, RST),

	_MFP(SC1, PB,14, RST),
	_MFP(SC1, PC, 2, RST),
	_MFP(SC1, PD, 6, RST),

	_MFP(SC2, PA,10, RST),
	_MFP(SC2, PA,13, RST),
	_MFP(SC2, PC, 6, RST),
	_MFP(SC2, PD, 2, RST),
	_MFP(SC2, PH, 9, RST),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_sc_ncd[] = {
	_MFP(SC0, PA, 4, nCD),
	_MFP(SC0, PC,12, nCD),
	_MFP(SC0, PE, 6, nCD),
	_MFP(SC0, PF,10, nCD),

	_MFP(SC1, PC, 4, nCD),
	_MFP(SC1, PD, 3, nCD),
	_MFP(SC1, PD,14, nCD),

	_MFP(SC2, PA, 5, nCD),
	_MFP(SC2, PC,13, nCD),
	//_MFP(SC2, PD,13, nCD),
	_MFP(SC2, PH,10, nCD),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_qspi_clk[] = {
	_MFP(QSPI0, PA, 2, CLK),
	_MFP(QSPI0, PC, 2, CLK),
	_MFP(QSPI0, PF, 2, CLK),
	_MFP(QSPI0, PH, 8, CLK),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_qspi_mosi0[] = {
	_MFP(QSPI0, PA, 0, MOSI0),
	_MFP(QSPI0, PC, 0, MOSI0),
	_MFP(QSPI0, PE, 0, MOSI0),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_qspi_mosi1[] = {
	_MFP(QSPI0, PA, 4, MOSI1),
	_MFP(QSPI0, PC, 4, MOSI1),
	_MFP(QSPI0, PH,11, MOSI1),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_qspi_miso0[] = {
	_MFP(QSPI0, PA, 1, MISO0),
	_MFP(QSPI0, PC, 1, MISO0),
	_MFP(QSPI0, PE, 1, MISO0),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_qspi_miso1[] = {
	_MFP(QSPI0, PA, 5, MISO1),
	_MFP(QSPI0, PC, 5, MISO1),
	_MFP(QSPI0, PH,10, MISO1),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_qspi_ssel[] = {
	_MFP(QSPI0, PA, 3, SS),
	_MFP(QSPI0, PC, 3, SS),
	_MFP(QSPI0, PH, 9, SS),

	_MFP_NONE
};

__WEAK
const pin_af_t pin_af_qspi_sclk[] = {
	_MFP(QSPI0, PA, 2, CLK),
	_MFP(QSPI0, PC, 2, CLK),
	_MFP(QSPI0, PH, 8, CLK),
	_MFP(QSPI0, PF, 2, CLK),

	_MFP_NONE
};

#ifdef MODULE_LCD
const pin_af_t pin_af_lcd_com0[] = {
	_MFP(LCD, PC, 0, COM0),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_com1[] = {
	_MFP(LCD, PC, 1, COM1),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_com2[] = {
	_MFP(LCD, PC, 2, COM2),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_com3[] = {
	_MFP(LCD, PC, 3, COM3),

	_MFP_NONE
};

// const pin_af_t pin_af_lcd_com4[] = {
// 	_MFP(LCD, PC, 4, COM4),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_com5[] = {
// 	_MFP(LCD, PC, 5, COM5),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_com6[] = {
// 	_MFP(LCD, PD, 8, COM6),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_com7[] = {
// 	_MFP(LCD, PD, 9, COM7),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_dh1[] = {
// 	_MFP(LCD, PD, 1, DH1),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_dh2[] = {
// 	_MFP(LCD, PD, 0, DH2),

// 	_MFP_NONE
// };

const pin_af_t pin_af_lcd_seg0[] = {
	_MFP(LCD, PD, 14, SEG0),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg1[] = {
	_MFP(LCD, PH, 11, SEG1),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg2[] = {
	_MFP(LCD, PH, 10, SEG2),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg3[] = {
	_MFP(LCD, PH, 9, SEG3),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg4[] = {
	_MFP(LCD, PH, 8, SEG4),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg5[] = {
	_MFP(LCD, PE, 0, SEG5),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg6[] = {
	_MFP(LCD, PE, 1, SEG6),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg7[] = {
	_MFP(LCD, PE, 2, SEG7),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg8[] = {
	_MFP(LCD, PE, 3, SEG8),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg9[] = {
	_MFP(LCD, PE, 4, SEG9),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg10[] = {
	_MFP(LCD, PE, 5, SEG10),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg11[] = {
	_MFP(LCD, PE, 6, SEG11),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg12[] = {
	_MFP(LCD, PE, 7, SEG12),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg13[] = {
	_MFP(LCD, PD, 6, SEG13),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg14[] = {
	_MFP(LCD, PD, 7, SEG14),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg15[] = {
	_MFP(LCD, PG, 15, SEG15),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg16[] = {
	_MFP(LCD, PG, 14, SEG16),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg17[] = {
	_MFP(LCD, PG, 13, SEG17),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg18[] = {
	_MFP(LCD, PG, 12, SEG18),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg19[] = {
	_MFP(LCD, PG, 11, SEG19),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg20[] = {
	_MFP(LCD, PG, 10, SEG20),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg21[] = {
	_MFP(LCD, PG, 9, SEG21),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg22[] = {
	_MFP(LCD, PE, 15, SEG22),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg23[] = {
	_MFP(LCD, PE, 14, SEG23),

	_MFP_NONE
};

const pin_af_t pin_af_lcd_seg24[] = {
	_MFP(LCD, PA, 0, SEG24),

	_MFP_NONE
};

// const pin_af_t pin_af_lcd_seg25[] = {
// 	_MFP(LCD, PA, 1, SEG25),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg26[] = {
// 	_MFP(LCD, PA, 2, SEG26),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg27[] = {
// 	_MFP(LCD, PA, 3, SEG27),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg28[] = {
// 	_MFP(LCD, PA, 4, SEG28),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg29[] = {
// 	_MFP(LCD, PA, 5, SEG29),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg30[] = {
// 	_MFP(LCD, PE, 10, SEG30),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg31[] = {
// 	_MFP(LCD, PE, 9, SEG31),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg32[] = {
// 	_MFP(LCD, PE, 8, SEG32),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg33[] = {
// 	_MFP(LCD, PH, 7, SEG33),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg34[] = {
// 	_MFP(LCD, PH, 6, SEG34),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg35[] = {
// 	_MFP(LCD, PH, 5, SEG35),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg36[] = {
// 	_MFP(LCD, PH, 4, SEG36),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg37[] = {
// 	_MFP(LCD, PG, 4, SEG37),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg38[] = {
// 	_MFP(LCD, PG, 3, SEG38),

// 	_MFP_NONE
// };

// const pin_af_t pin_af_lcd_seg39[] = {
// 	_MFP(LCD, PG, 2, SEG39),

// 	_MFP_NONE
// };


#endif
__WEAK
const pin_af_t pin_af_can_rx[] = {
	_MFP(CAN0, PA,  4, RXD),
	_MFP(CAN0, PA, 13, RXD),
	_MFP(CAN0, PB, 10, RXD),
	_MFP(CAN0, PC,  4, RXD),
	_MFP(CAN0, PD, 10, RXD),
	_MFP(CAN0, PE, 15, RXD),
	
	_MFP_NONE
};
__WEAK
const pin_af_t pin_af_can_tx[] = {
	_MFP(CAN0, PA,  5, TXD),	
	_MFP(CAN0, PA, 12, TXD),	
	_MFP(CAN0, PB, 11, TXD),	
	_MFP(CAN0, PC,  5, TXD),
	_MFP(CAN0, PD, 11, TXD),
	_MFP(CAN0, PE, 14, TXD),	

	_MFP_NONE
};

#define EADC0 EADC
const pin_af_t pin_af_eadc[] = {
	_MFP(EADC0, PB,  0, CH0),
	_MFP(EADC0, PB,  1, CH1),
	_MFP(EADC0, PB,  2, CH2),
	_MFP(EADC0, PB,  3, CH3),
	_MFP(EADC0, PB,  4, CH4),
	_MFP(EADC0, PB,  5, CH5),
	_MFP(EADC0, PB,  6, CH6),
	_MFP(EADC0, PB,  7, CH7),
	_MFP(EADC0, PB,  8, CH8),
	_MFP(EADC0, PB,  9, CH9),
	_MFP(EADC0, PB, 10, CH10),
	_MFP(EADC0, PB, 11, CH11),
	_MFP(EADC0, PB, 12, CH12),
	_MFP(EADC0, PB, 13, CH13),
	_MFP(EADC0, PB, 14, CH14),
	_MFP(EADC0, PB, 15, CH15),
	_MFP_NONE,
};

#define USB USBD
const pin_af_t pin_af_usbd[] = {
	_MFP(USB, PA, 12, VBUS),
	_MFP(USB, PA, 13, D_N),
	_MFP(USB, PA, 14, D_P),
	_MFP(USB, PA, 15, OTG_ID),
	_MFP_NONE,
};

const mod_desc_t modtbl_usbd_pll[MAX_MOD_USBD+1] = {
	{USBD, USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(2), USBD_RST, USBD_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_usbd_hirc[MAX_MOD_USBD+1] = {
	{USBD, USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(1), USBD_RST, USBD_IRQn},
	{NULL, }
};



const mod_desc_t modtbl_uart[MAX_MOD_UART+1] = {
	{UART0, UART0_MODULE, CLK_CLKSEL2_UART0SEL_PLL, CLK_CLKDIV0_UART0(1), UART0_RST, UART0_IRQn},
	{UART1, UART1_MODULE, CLK_CLKSEL2_UART1SEL_PLL, CLK_CLKDIV0_UART1(1), UART1_RST, UART1_IRQn},
	{UART2, UART2_MODULE, CLK_CLKSEL2_UART2SEL_PLL, CLK_CLKDIV4_UART2(1), UART2_RST, UART2_IRQn},
	{UART3, UART3_MODULE, CLK_CLKSEL2_UART3SEL_PLL, CLK_CLKDIV4_UART3(1), UART3_RST, UART3_IRQn},
	{UART4, UART4_MODULE, CLK_CLKSEL3_UART4SEL_PLL, CLK_CLKDIV4_UART4(1), UART4_RST, UART4_IRQn},
	{UART5, UART5_MODULE, CLK_CLKSEL3_UART5SEL_PLL, CLK_CLKDIV4_UART5(1), UART5_RST, UART5_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_spi[MAX_MOD_SPI+1] = {
	{SPI0, SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PLL, MODULE_NoMsk, SPI0_RST, SPI0_IRQn},
	{SPI1, SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PLL, MODULE_NoMsk, SPI1_RST, SPI1_IRQn},
	{SPI2, SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PLL, MODULE_NoMsk, SPI2_RST, SPI2_IRQn},
	{SPI3, SPI3_MODULE, CLK_CLKSEL2_SPI3SEL_PLL, MODULE_NoMsk, SPI3_RST, SPI3_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_qspi[MAX_MOD_QSPI+1] = {
	{QSPI0, QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk, QSPI0_RST, QSPI0_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_i2c[MAX_MOD_I2C+1] = {
	{I2C0, I2C0_MODULE, 0, 0, I2C0_RST, I2C0_IRQn},
	{I2C1, I2C1_MODULE, 0, 0, I2C1_RST, I2C1_IRQn},
	{I2C2, I2C2_MODULE, 0, 0, I2C2_RST, I2C2_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_sc[MAX_MOD_SC+1] = {
	{SC0, SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(3), SC0_RST, SC0_IRQn},
	{SC1, SC1_MODULE, CLK_CLKSEL3_SC1SEL_HIRC, CLK_CLKDIV1_SC1(3), SC1_RST, SC1_IRQn},
	{SC2, SC2_MODULE, CLK_CLKSEL3_SC2SEL_HIRC, CLK_CLKDIV1_SC2(3), SC2_RST, SC2_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_timer[MAX_MOD_TIMER+1] = {
	{TIMER0, TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0 , 0, TMR0_RST, TMR0_IRQn},
	{TIMER1, TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0 , 0, TMR1_RST, TMR1_IRQn},
	{TIMER2, TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1 , 0, TMR2_RST, TMR2_IRQn},
	{TIMER3, TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1 , 0, TMR3_RST, TMR3_IRQn},
	{TIMER4, TMR4_MODULE, CLK_CLKSEL3_TMR4SEL_PCLK0 , 0, TMR4_RST, TMR4_IRQn},
	{TIMER5, TMR5_MODULE, CLK_CLKSEL3_TMR5SEL_PCLK0 , 0, TMR5_RST, TMR5_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_lcd[MAX_MOD_LCD + 1] = {
	{LCD, LCD_MODULE, CLK_CLKSEL1_LCDSEL_LIRC, MODULE_NoMsk, LCD_RST, LCD_IRQn},
	{NULL, }
};

const mod_desc_t modtbl_can[MAX_MOD_CAN + 1] = {
	{CAN0, CAN0_MODULE, CLK_CLKSEL2_UART1SEL_PLL, MODULE_NoMsk, CAN0_RST, CAN0_IRQn},
	{NULL, }
};

int find_pin_af(void *dev, gpio_t gpio, const pin_af_t tb[])
{
	const pin_af_t *p;

	if (gpio == 0)
		return 0;

	for (p=tb; p->device; p++)
	{
		if (p->device == dev && p->gpio == gpio)
			return p->af;
	}

	return 0;
}

#if 0
int set_pin_af(void *dev, gpio_t gpio, int af)
{
	if (af == 0)
		return -1;

	volatile uint32_t ctl = (uint32_t)(&SYS->GPA_MFPL);
	ctl += ((PERIPH_GPIO_IDX(gpio) * 2) * 4);
	ctl += ((PERIPH_GPIO_PIN(gpio) / 8) * 4);

	uint32_t mfp = *(volatile uint32_t *)ctl;

	mfp &= ~(0xf << ((PERIPH_GPIO_PIN(gpio) % 8) * 4));
	mfp |= af;

	*(volatile uint32_t *)ctl = mfp;

	return 0;
}
#endif

const mod_desc_t *find_mod_desc(void *dev, const mod_desc_t tb[])
{
	const mod_desc_t *m;
	for (m=tb; m->device; m++)
	{
		if (m->device == dev)
			return m;
	}
	return NULL;
}

