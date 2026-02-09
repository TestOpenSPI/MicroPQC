/**
 * @file periph_conf.h
 * @brief The definitions for AXIO Smart meter
 * @copyright Copyright (c) 2019 Security Platform Inc. All Rights Reserved.
 */
#ifndef _PERIPH_CONF_H_
#define _PERIPH_CONF_H_

#include "periph_def.h"

/**
 * @enum uart_id_t
 *
 * @brief Possible index for UART
 *
 */
#define UART_STDIO_DEV          0

extern const int uart_stdio_dev;

#define SPI_MASTER_0            0
#define SPI_SLAVE_0_NOR_FLASH   0
extern const spi_dev_t spi_nor_dev;

#define TIMER_ENTROPY           0

extern const i2c_dev_t i2c_kepcocf_dev;

#define TIMER_WATCHDOG        1

#endif

