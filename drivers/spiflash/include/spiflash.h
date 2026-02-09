#ifndef _SPIFLASH_H_
#define _SPIFLASH_H_

#if defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#elif defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#include "periph_api.h"
#include "mtd.h"

struct spi_nor
{
//	struct mtd_info mtd;

#if defined(OS_CMSIS_RTX)
	osMutexId_t lock;
#elif defined(OS_FREERTOS)
	SemaphoreHandle_t lock;
#else
	int lock;
#endif
	const spi_dev_t *spi;
	const struct flash_info *info;
	uint32_t flags;
	uint8_t erase_opcode;
	uint8_t read_opcode;
	uint8_t pp_opcode;

#if 0
	int (*prepare)(struct spi_nor *nor, enum spi_nor_ops ops);
	void (*unprepare)(struct spi_nor *nor, enum spi_nor_ops ops);
#endif

	int (*read_reg)(struct spi_nor *nor, uint8_t opcode, uint8_t *buf, uint32_t len);
	int (*write_reg)(struct spi_nor *nor, uint8_t opcode, uint8_t *buf, uint32_t len);

	int (*read)(struct spi_nor *nor, uint32_t addr, uint32_t len, uint8_t *buf);
	int (*write)(struct spi_nor *nor, uint32_t addr, uint32_t len, const uint8_t *buf);
	int (*erase)(struct spi_nor *nor, uint32_t addr);

	int (*flash_lock)(struct spi_nor *nor, uint32_t addr, uint32_t len);
	int (*flash_unlock)(struct spi_nor *nor, uint32_t addr, uint32_t len);
	int (*flash_is_locked)(struct spi_nor *nor, uint32_t addr, uint32_t len);
};

extern const struct mtd_operation spi_nor_mtd_operation;

#endif

