#include <string.h>
#include "spiflash.h"
#if defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#elif defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#elif defined(MODULE_SYSTICK)
#include "systick.h"
#else
#error Need time keeping method
#endif

#define BIT(n) (1<<(n))

#define SNOR_MFR_ATMEL      0x001F
#define SNOR_MFR_GIGADEVICE 0xc8
#define SNOR_MFR_INTEL      0x0089
#define SNOR_MFR_MICRON     0x0020 /* ST Micro <--> Micron */
#define SNOR_MFR_MACRONIX   0x00C2
#define SNOR_MFR_SPANSION   0x0001
#define SNOR_MFR_SST        0x00BF
#define SNOR_MFR_WINBOND    0xef /* Also used by some Spansion */

struct flash_opcode {
	uint8_t rdid;
	uint8_t wren;
	uint8_t rdsr;
	uint8_t rdfsr;
	uint8_t wrsr;
	uint8_t wrdi;
	uint8_t clsr;
	uint8_t clfsr;
	uint8_t read;
	uint8_t read_fast;
	uint8_t page_program;
	uint8_t erase_4k;
	uint8_t erase_32k;
	uint8_t erase_64k;
	uint8_t erase_chip;
};

static struct flash_opcode opcode_default = {
	.rdid = 0x9f,
	.wren = 0x06,
	.rdsr = 0x05,
	.rdfsr = 0x70,
	.wrsr = 0x01,
	.wrdi = 0x04,
	.clsr = 0x30,
	.clfsr = 0x50,
	.read = 0x03,
	.read_fast = 0x0b,
	.page_program = 0x02,
	.erase_4k = 0x20,
	.erase_32k = 0x52,
	.erase_64k = 0xd8,
	.erase_chip = 0xc7,
};

struct flash_info {
	char *name;
	uint8_t id[7];
	uint8_t id_len;
	uint32_t sector_size;
	uint16_t n_sectors;
	uint16_t page_size;
	uint16_t addr_width;
	uint16_t flags;
	struct flash_opcode *opcode;
};

#define SECT_4K             BIT(0)  /* SPINOR_OP_BE_4K works uniformly */
#define SPI_NOR_NO_ERASE    BIT(1)  /* No erase command needed */
#define SST_WRITE           BIT(2)  /* use SST byte programming */
#define SPI_NOR_NO_FR       BIT(3)  /* Can't do fastread */
#define SECT_4K_PMC         BIT(4)  /* SPINOR_OP_BE_4K_PMC works uniformly */
#define SPI_NOR_DUAL_READ   BIT(5)  /* Flash supports Dual Read */
#define SPI_NOR_QUAD_READ   BIT(6)  /* Flash supports Quad Read */
#define USE_FSR             BIT(7)  /* use flag status register */
#define SPI_NOR_HAS_LOCK    BIT(8)  /* Flash supports lock/unlock via SR */
#define SPI_NOR_HAS_TB      BIT(9)  /*
									 * Flash SR has Top/Bottom (TB) protect
									 * bit. Must be used with
									 * SPI_NOR_HAS_LOCK.
									 */
#define SPI_S3AN            BIT(10) /*
						    		 * Xilinx Spartan 3AN In-System Flash
						    		 * (MFR cannot be used for probing
						    		 * because it has the same value as
						    		 * ATMEL flashes)
						    		 */
#define SPI_NOR_4B_OPCODES  BIT(11) /*
									 * Use dedicated 4byte address op codes
									 * to support memory size above 128Mib.
									 */
#define NO_CHIP_ERASE       BIT(12) /* Chip does not support chip erase */
#define SPI_NOR_SKIP_SFDP   BIT(13) /* Skip parsing of SFDP tables */
#define USE_CLSR            BIT(14) /* use CLSR command */

#define JEDEC_MFR(info) ((info)->id[0])

/* Used when the "_ext_id" is two bytes at most */
#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags, _addr_width, _opcode)  \
	.id = {                         \
		((_jedec_id) >> 16) & 0xff, \
		((_jedec_id) >> 8) & 0xff,  \
		(_jedec_id) & 0xff,         \
		((_ext_id) >> 8) & 0xff,    \
		(_ext_id) & 0xff,           \
	},                              \
	.id_len = (!(_jedec_id) ? 0 : (3 + ((_ext_id) ? 2 : 0))), \
	.sector_size = (_sector_size),  \
	.n_sectors = (_n_sectors),      \
	.page_size = 256,               \
	.flags = (_flags),              \
	.addr_width = _addr_width,      \
	.opcode = _opcode,

#define INFO6(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags) \
	.id = {                         \
		((_jedec_id) >> 16) & 0xff, \
		((_jedec_id) >> 8) & 0xff,  \
		(_jedec_id) & 0xff,         \
		((_ext_id) >> 16) & 0xff,   \
		((_ext_id) >> 8) & 0xff,    \
		(_ext_id) & 0xff,           \
	},                      \
	.id_len = 6,                    \
	.sector_size = (_sector_size),  \
	.n_sectors = (_n_sectors),      \
	.page_size = 256,               \
	.flags = (_flags),

#define CAT25_INFO(_sector_size, _n_sectors, _page_size, _addr_width, _flags)   \
	.sector_size = (_sector_size),  \
	.n_sectors = (_n_sectors),      \
	.page_size = (_page_size),      \
	.addr_width = (_addr_width),    \
	.flags = (_flags),

#define S3AN_INFO(_jedec_id, _n_sectors, _page_size)            \
	.id = {                         \
		((_jedec_id) >> 16) & 0xff, \
		((_jedec_id) >> 8) & 0xff,  \
		(_jedec_id) & 0xff          \
	},                      \
	.id_len = 3,                    \
	.sector_size = (8*_page_size),  \
	.n_sectors = (_n_sectors),      \
	.page_size = _page_size,        \
	.addr_width = 3,                \
	.flags = SPI_NOR_NO_FR | SPI_S3AN,

static const struct flash_info spi_nor_ids[] = {
	{ "n25q128a13"   , INFO(0x20ba18, 0, 64 * 1024,  256, SECT_4K | SPI_NOR_QUAD_READ, 3, &opcode_default) },
	{ "mx25l12805d"  , INFO(0xc22018, 0,      4096, 4096, SECT_4K                    , 3, &opcode_default) },
	{ "gd25q128"     , INFO(0xc84018, 0,      4096, 4096, SECT_4K                    , 3, &opcode_default) },
	{ "s25fl064k"    , INFO(0xef4017, 0, 64 * 1024,  128, SECT_4K                    , 3, &opcode_default) },
	{ "w25q128"      , INFO(0xef4018, 0, 64 * 1024,  256, SECT_4K                    , 3, &opcode_default) },
	{ "w25q128jv"    , INFO(0xef7018, 0, 64 * 1024,  256, SECT_4K                    , 3, &opcode_default) },
	{ "w25q16jv"    ,  INFO(0xef4015, 0,      4096,  512, SECT_4K                    , 3, &opcode_default) },
	{ "XT25F128B"    , INFO(0x0b4018, 0, 64 * 1024,  256, SECT_4K                    , 3, &opcode_default) },
	{ "MX25L6433F"   , INFO(0xc22017, 0,      4096, 2048, SECT_4K                    , 3, &opcode_default) },
	{ NULL, }
};

#define FLASHADDR_0(addr) ((uint8_t)((addr >> 24) & 0xff))
#define FLASHADDR_1(addr) ((uint8_t)((addr >> 16) & 0xff))
#define FLASHADDR_2(addr) ((uint8_t)((addr >> 8) & 0xff))
#define FLASHADDR_3(addr) ((uint8_t)((addr >> 0) & 0xff))

/* Status Register bits. */
#define SR_WIP          BIT(0)  /* Write in progress */
#define SR_WEL          BIT(1)  /* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define SR_BP0          BIT(2)  /* Block protect 0 */
#define SR_BP1          BIT(3)  /* Block protect 1 */
#define SR_BP2          BIT(4)  /* Block protect 2 */
#define SR_TB           BIT(5)  /* Top/Bottom protect */
#define SR_SRWD         BIT(7)  /* SR write protect */
/* Spansion/Cypress specific status bits */
#define SR_E_ERR        BIT(5)
#define SR_P_ERR        BIT(6)

/* Flag Status Register bits */
#define FSR_READY       BIT(7)  /* Device status, 0 = Busy, 1 = Ready */
#define FSR_E_ERR       BIT(5)  /* Erase operation status */
#define FSR_P_ERR       BIT(4)  /* Program operation status */
#define FSR_PT_ERR      BIT(1)  /* Protection error bit */

#define SPI_NOR_DEFAULT_TIMEOUT (1*1000)

enum spi_nor_ops {
    SPI_NOR_OPS_READ = 0,
    SPI_NOR_OPS_WRITE,
    SPI_NOR_OPS_ERASE,
    SPI_NOR_OPS_LOCK,
    SPI_NOR_OPS_UNLOCK,
};

enum spi_nor_option_flags {
	SNOR_F_USE_FSR      = BIT(0),
	SNOR_F_HAS_SR_TB    = BIT(1),
	SNOR_F_NO_OP_CHIP_ERASE = BIT(2),
	SNOR_F_S3AN_ADDR_DEFAULT = BIT(3),
	SNOR_F_READY_XSR_RDY    = BIT(4),
	SNOR_F_USE_CLSR     = BIT(5),
	SNOR_F_BROKEN_RESET = BIT(6),
};

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct spi_nor *nor)
{
	uint8_t val;

	if (nor->read_reg(nor, nor->info->opcode->rdsr, &val, 1) != 0)
	{
		return -1;
	}

	return (int)val;
}

/*
 * Read the flag status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_fsr(struct spi_nor *nor)
{
	uint8_t val;

	if (nor->read_reg(nor, nor->info->opcode->rdfsr, &val, 1) != 0)
	{
		return -1;
	}

	return (int)val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
__STATIC_INLINE int write_sr(struct spi_nor *nor, uint8_t val)
{
	return nor->write_reg(nor, nor->info->opcode->wrsr, &val, 1);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
__STATIC_INLINE int write_enable(struct spi_nor *nor)
{
	return nor->write_reg(nor, nor->info->opcode->wren, NULL, 0);
}

/*
 * Send write disable instruction to the chip.
 */
__STATIC_INLINE int write_disable(struct spi_nor *nor)
{
	return nor->write_reg(nor, nor->info->opcode->wrdi, NULL, 0);
}

__STATIC_INLINE int spi_nor_sr_ready(struct spi_nor *nor)
{
	int sr = read_sr(nor);
	if (sr < 0)
		return sr;

	if (nor->flags & SNOR_F_USE_CLSR && sr & (SR_E_ERR | SR_P_ERR)) {
#if 0
		if (sr & SR_E_ERR)
			dev_err(nor->dev, "Erase Error occurred\r\n");
		else
			dev_err(nor->dev, "Programming Error occurred\r\n");
#endif

		nor->write_reg(nor, nor->info->opcode->clsr, NULL, 0);
		return -1;
	}

	return !(sr & SR_WIP);
}

__STATIC_INLINE int spi_nor_fsr_ready(struct spi_nor *nor)
{
	int fsr = read_fsr(nor);
	if (fsr < 0)
		return fsr;

	if (fsr & (FSR_E_ERR | FSR_P_ERR)) {
#if 0
		if (fsr & FSR_E_ERR)
			dev_err(nor->dev, "Erase operation failed.\r\n");
		else
			dev_err(nor->dev, "Program operation failed.\r\n");

		if (fsr & FSR_PT_ERR)
			dev_err(nor->dev,
					"Attempted to modify a protected sector.\r\n");
#endif
		nor->write_reg(nor, nor->info->opcode->clfsr, NULL, 0);
		return -1;
	}

	return fsr & FSR_READY;
}

static int spi_nor_ready(struct spi_nor *nor)
{
	int sr, fsr;

#if 0
	if (nor->flags & SNOR_F_READY_XSR_RDY)
		sr = s3an_sr_ready(nor);
	else
#endif
		sr = spi_nor_sr_ready(nor);
	if (sr < 0)
		return sr;

	fsr = nor->flags & SNOR_F_USE_FSR ? spi_nor_fsr_ready(nor) : 1;
	if (fsr < 0)
		return fsr;

	return sr && fsr;
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int spi_nor_wait_till_ready_with_timeout(struct spi_nor *nor,
                        uint32_t timeout)
{
#if defined(OS_CMSIS_RTX)
	volatile uint32_t start = osKernelGetTickCount();
#elif defined(OS_FREERTOS)
	volatile uint32_t start = (uint32_t)xTaskGetTickCount();
#elif defined(MODULE_SYSTICK)
	volatile uint32_t start = get_systick();
#endif
	volatile uint32_t current;

	int flag = 0, ret;

	while (!flag) {
#if defined(OS_CMSIS_RTX) || defined(OS_FREERTOS)
#if defined(OS_CMSIS_RTX)
		current = osKernelGetTickCount();
#else
		current = (uint32_t)xTaskGetTickCount();
#endif

		if (current >= start)
		{
			if (current - start > timeout)
			{
				flag = 1;
			}
		}
		else
		{
			if ((current + (0xffffffff - start)) > timeout)
			{
				flag = 1;
			}
		}
#elif defined(MODULE_SYSTICK)
		current = get_systick();
		if (systick_diff(start, current) > timeout)
		{
			flag = 1;
		}
#endif
		ret = spi_nor_ready(nor);
		if (ret < 0)
			return ret;
		if (ret)
			return 0;

#if defined(OS_CMSIS_RTX)
		osThreadYield();
#elif defined(OS_FREERTOS)
		vPortYield();
#elif defined(MODULE_SYSTICK)
		systick_delay(1);
#endif
	}

	/* timeout */
	return -1;
}

static int spi_nor_wait_till_ready(struct spi_nor *nor)
{
	return spi_nor_wait_till_ready_with_timeout(nor, SPI_NOR_DEFAULT_TIMEOUT);
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct spi_nor *nor)
{
	return nor->write_reg(nor, nor->info->opcode->erase_chip, NULL, 0);
}

static const struct flash_info *spi_nor_read_id(struct spi_nor *nor)
{
	uint8_t id[6]; /* max id length */

	// nor->info is not present yet.
	if (nor->read_reg(nor, 0x9F, id, 6) != 0)
	{
		return NULL;
	}

	unsigned int i;
	const struct flash_info *info;

	for (i=0; i<(sizeof(spi_nor_ids)/sizeof(spi_nor_ids[0]) - 1); i++)
	{
		info = &spi_nor_ids[i];
		if (info->id_len)
		{
			if (!memcmp(info->id, id, info->id_len))
			{
				return info;
			}
		}
	}

	return NULL;
}

static int spi_nor_lock_and_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	int ret = 0;
#if 0
	mutex_lock(&nor->lock);

	if (nor->prepare) {
		ret = nor->prepare(nor, ops);
		if (ret) {
			dev_err(nor->dev, "failed in the preparation.\r\n");
			mutex_unlock(&nor->lock);
			return ret;
		}
	}
#endif
	(void)ops;
#if defined(OS_CMSIS_RTX)
	osMutexAcquire(nor->lock, 0);
#elif defined(OS_FREERTOS)
	xSemaphoreTake(nor->lock, portMAX_DELAY);
#else
	while(nor->lock);
	nor->lock = 1;
#endif

	return ret;
}

static void spi_nor_unlock_and_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
#if 0
	if (nor->unprepare)
		nor->unprepare(nor, ops);
#endif
	(void)ops;
#if defined(OS_CMSIS_RTX)
	osMutexRelease(nor->lock);
#elif defined(OS_FREERTOS)
	xSemaphoreGive(nor->lock);
#else
	nor->lock = 0;
#endif
}

static int spi_nor_lock(mtd_t *mtd, uint32_t ofs, uint32_t len)
{
	struct spi_nor *nor = (struct spi_nor *)mtd->dev;
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_LOCK);
	if (ret)
		return ret;

	ret = nor->flash_lock(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_UNLOCK);
	return ret;
}

static int spi_nor_unlock(mtd_t *mtd, uint32_t ofs, uint32_t len)
{
	struct spi_nor *nor = (struct spi_nor *)mtd->dev;
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_UNLOCK);
	if (ret)
		return ret;

	ret = nor->flash_unlock(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_LOCK);
	return ret;
}

static int spi_nor_is_locked(mtd_t *mtd, uint32_t ofs, uint32_t len)
{
	struct spi_nor *nor = (struct spi_nor *)mtd->dev;
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_UNLOCK);
	if (ret)
		return ret;

	ret = nor->flash_is_locked(nor, ofs, len);

	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_LOCK);
	return ret;
}

/*
 * Initiate the erasure of a single sector
 */
static int spi_nor_erase_sector(struct spi_nor *nor, uint32_t addr)
{
	uint8_t buf[4];
	int i;

#if 0
	if (nor->flags & SNOR_F_S3AN_ADDR_DEFAULT)
		addr = spi_nor_s3an_addr_convert(nor, addr);
#endif

	if (nor->erase)
		return nor->erase(nor, addr);

	/*
	 * Default implementation, if driver doesn't have a specialized HW
	 * control
	 */
	for (i = nor->info->addr_width - 1; i >= 0; i--) {
		buf[i] = addr & 0xff;
		addr >>= 8;
	}

	return nor->write_reg(nor, nor->erase_opcode, buf, nor->info->addr_width);
}

/*
 * Erase an address range on the nor chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int spi_nor_erase(mtd_t *mtd, uint32_t addr, uint32_t len)
{
	struct spi_nor *nor = (struct spi_nor *)mtd->dev;
	uint32_t rem;
	int ret;

#if 0
	dev_dbg(nor->dev, "at 0x%llx, len %lld\r\n", (long long)instr->addr,
			(long long)instr->len);

	div_u64_rem(instr->len, mtd->erasesize, &rem);
#else
	rem = len % mtd->erasesize;
#endif
	if (rem)
		return -1;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_ERASE);
	if (ret)
		return ret;

	/* whole-chip erase? */
	if (len == mtd->size && !(nor->flags & SNOR_F_NO_OP_CHIP_ERASE)) {
		unsigned long timeout;

		write_enable(nor);

		if (erase_chip(nor)) {
			ret = -1;
			goto erase_err;
		}

		/*
		 * Scale the timeout linearly with the size of the flash, with
		 * a minimum calibrated to an old 2MB flash. We could try to
		 * pull these from CFI/SFDP, but these values should be good
		 * enough for now.
		 */
#if 0
		timeout = max(CHIP_ERASE_2MB_READY_WAIT_JIFFIES,
				CHIP_ERASE_2MB_READY_WAIT_JIFFIES *
				(unsigned long)(mtd->size / SZ_2M));
#else
		timeout = SPI_NOR_DEFAULT_TIMEOUT;
#endif
		ret = spi_nor_wait_till_ready_with_timeout(nor, timeout);
		if (ret)
			goto erase_err;

		/* REVISIT in some cases we could speed up erasing large regions
		 * by using SPINOR_OP_SE instead of SPINOR_OP_BE_4K.  We may have set up
		 * to use "small sector erase", but that's not always optimal.
		 */

		/* "sector"-at-a-time erase */
	} else {
		while (len) {
			write_enable(nor);

			ret = spi_nor_erase_sector(nor, addr);
			if (ret)
				goto erase_err;

			addr += mtd->erasesize;
			len -= mtd->erasesize;

			ret = spi_nor_wait_till_ready(nor);
			if (ret)
				goto erase_err;
		}
	}

	write_disable(nor);

erase_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_ERASE);

	return ret;
}

static int spi_nor_read(mtd_t *mtd, uint32_t from, uint32_t len,
		uint8_t *buf, uint32_t *retlen)
{
	struct spi_nor *nor = (struct spi_nor *)mtd->dev;
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_READ);
	if (ret)
		return ret;

	while (len) {
		uint32_t addr = from;

#if 0
		if (nor->flags & SNOR_F_S3AN_ADDR_DEFAULT)
			addr = spi_nor_s3an_addr_convert(nor, addr);
#endif

		ret = nor->read(nor, addr, len, buf);
		if (ret == 0) {
			/* We shouldn't see 0-length reads */
			ret = -1;
			goto read_err;
		}
		if (ret < 0)
			goto read_err;

#if 0
		WARN_ON(ret > len);
#endif
		*retlen += ret;
		buf += ret;
		from += ret;
		len -= ret;
	}
	ret = 0;

read_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_READ);
	return ret;
}

static int spi_nor_write(mtd_t *mtd, uint32_t to, uint32_t len,
		const uint8_t *buf, uint32_t *retlen)
{
	struct spi_nor *nor = (struct spi_nor *)mtd->dev;
	size_t page_offset, page_remain, i;
	int ret;

	ret = spi_nor_lock_and_prep(nor, SPI_NOR_OPS_WRITE);
	if (ret)
		return ret;

	for (i = 0; i < len; ) {
		size_t written;
		uint32_t addr = to + i;

		/*
		 * If page_size is a power of two, the offset can be quickly
		 * calculated with an AND operation. On the other cases we
		 * need to do a modulus operation (more expensive).
		 * Power of two numbers have only one bit set and we can use
		 * the instruction hweight32 to detect if we need to do a
		 * modulus (do_div()) or not.
		 */
#if 0
		if (hweight32(nor->page_size) == 1) {
			page_offset = addr & (nor->page_size - 1);
		} else {
			uint64_t aux = addr;

			page_offset = do_div(aux, nor->page_size);
		}
		/* the size of data remaining on the first page */
		page_remain = min_t(size_t,
				nor->page_size - page_offset, len - i);
#else
		page_offset = addr & (nor->info->page_size - 1);
		if (nor->info->page_size - page_offset < len - i)
			page_remain = nor->info->page_size - page_offset;
		else
			page_remain = len - i;
#endif

#if 0
		if (nor->flags & SNOR_F_S3AN_ADDR_DEFAULT)
			addr = spi_nor_s3an_addr_convert(nor, addr);
#endif

		write_enable(nor);
		ret = nor->write(nor, addr, page_remain, buf + i);
		if (ret < 0)
			goto write_err;
		written = ret;

		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			goto write_err;
		*retlen += written;
		i += written;
		if (written != page_remain) {
			ret = -1;
			goto write_err;
		}
	}

write_err:
	spi_nor_unlock_and_unprep(nor, SPI_NOR_OPS_WRITE);
	return ret;
}

static int _spi_nor_read_reg(struct spi_nor *nor, uint8_t opcode, uint8_t *buf, uint32_t len)
{
	if (spi_acquire(nor->spi) != 0)
		return -1;

	int ret = 0;

	spi_stream_t send = { .buf = &opcode, .len = 1 };

	ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_CONTINUE, &send, NULL);
	if (ret != 0)
		goto exit;

	spi_stream_t recv = { .buf = buf, .len = len };

	ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_FINISH, NULL, &recv);

exit:
	spi_release(nor->spi);
	return ret;
}

static int _spi_nor_write_reg(struct spi_nor *nor, uint8_t opcode, uint8_t *buf, uint32_t len)
{
	if (spi_acquire(nor->spi) != 0)
		return -1;

	int ret = 0;

	spi_stream_t send = { .buf = &opcode, .len = 1 };

	if (buf)
	{
		ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_CONTINUE, &send, NULL);
		if (ret != 0)
			goto exit;

		send.buf = buf;
		send.len = len;

		ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_FINISH, &send, NULL);
		if (!ret)
			goto exit;
	}
	else
	{
		ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_FINISH, &send, NULL);
	}

exit:
	spi_release(nor->spi);
	return ret;
}

static int _spi_nor_read(struct spi_nor *nor, uint32_t addr, uint32_t len, uint8_t *buf)
{
	if (spi_acquire(nor->spi) != 0)
		return -1;

	int ret = 0;
	int l = 0;
	uint8_t cmd[5];

	cmd[l++] = nor->read_opcode;
	if (nor->info->addr_width == 4)
	{
		cmd[l++] = FLASHADDR_0(addr);
	}
	cmd[l++] = FLASHADDR_1(addr);
	cmd[l++] = FLASHADDR_2(addr);
	cmd[l++] = FLASHADDR_3(addr);

	spi_stream_t send = { .buf = cmd, .len = l };

	ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_CONTINUE, &send, NULL);
	if (ret != 0)
		goto exit;

	spi_stream_t recv = { .buf = buf, .len = len };

	ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_FINISH, NULL, &recv);

exit:
	spi_release(nor->spi);

	if (ret == 0)
		return len;
	return ret;
}

static int _spi_nor_write(struct spi_nor *nor, uint32_t addr, uint32_t len, const uint8_t *buf)
{
	if (spi_acquire(nor->spi) != 0)
		return -1;

	int ret = 0;
	int l = 0;
	uint8_t cmd[5];

	cmd[l++] = nor->pp_opcode;

	if (nor->info->addr_width == 4)
	{
		cmd[l++] = FLASHADDR_0(addr);
	}
	cmd[l++] = FLASHADDR_1(addr);
	cmd[l++] = FLASHADDR_2(addr);
	cmd[l++] = FLASHADDR_3(addr);


	spi_stream_t send = { .buf = cmd, .len = l };

	ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_CONTINUE, &send, NULL);
	if (ret != 0)
		goto exit;

	send.buf = (uint8_t *)buf;
	send.len = len;

	ret = spi_transfer_bytes(nor->spi, SPI_IO_TRANSACTION_FINISH, &send, NULL);

exit:
	spi_release(nor->spi);

	if (ret == 0)
		return len;
	return ret;
}

static int spi_nor_init(mtd_t *mtd)
{
	const struct flash_info *info;
	struct spi_nor *nor = mtd->dev;

	nor->read_reg        = _spi_nor_read_reg;
	nor->write_reg       = _spi_nor_write_reg;
	nor->read            = _spi_nor_read;
	nor->write           = _spi_nor_write;
	nor->erase           = NULL; // device specific erase function or default erase function
#if 0
	nor->flash_lock      = NULL;
	nor->flash_unlock    = NULL;
	nor->flash_is_locked = NULL;
#endif

	if (1)
	{
		info = spi_nor_read_id(nor);
	}
	else
	{
		info = &spi_nor_ids[0];
	}

	if (!info)
	{
		//printf("spi_nor: flash ID not found\r\n");
		return -1;
	}

	//printf("found spi nor flash %s\r\n", info->name);

	nor->info = info;

#if defined(OS_CMSIS_RTX)
	nor->lock = osMutexNew(NULL);
#elif defined(OS_FREERTOS)
	nor->lock = xSemaphoreCreateMutex();
#else
	nor->lock = 0;
#endif

	if (info->flags & SPI_S3AN)
		nor->flags |= SNOR_F_READY_XSR_RDY;

	mtd->size = nor->info->sector_size * nor->info->n_sectors;
	
#if 0
	// check use_size
	if(mtd->use_size == 0){
		mtd->use_size = mtd->size - mtd->base_addr;
	}
	else{
		if(mtd->use_size + mtd->base_addr > mtd->size){
			return -1;
		}
	}
#endif

	if (info->flags & SECT_4K)
	{
		mtd->erasesize = 4096;
		nor->erase_opcode = nor->info->opcode->erase_4k;
	}
	else
	{
		mtd->erasesize = nor->info->sector_size;
		nor->erase_opcode = nor->info->opcode->erase_64k;
	}

	mtd->page_size = nor->info->page_size;

	// TODO: select read opcode
	nor->read_opcode = nor->info->opcode->read;

	// TODO: select program opcode
	nor->pp_opcode = nor->info->opcode->page_program;

	// TODO: flash_lock, flash_unlock
	if (JEDEC_MFR(info) == SNOR_MFR_MICRON || info->flags & SPI_NOR_HAS_LOCK)
	{
#if 0
		//nor->flash_lock =
		//nor->flash_unlock =
		//nor->flash_is_locked =
#endif
	}

	if (nor->flash_lock && nor->flash_unlock && nor->flash_is_locked)
	{
		mtd->_lock = spi_nor_lock;
		mtd->_unlock = spi_nor_unlock;
		mtd->_is_locked = spi_nor_is_locked;
	}

	if (info->flags & USE_FSR)          nor->flags |= SNOR_F_USE_FSR;
	if (info->flags & SPI_NOR_HAS_TB)   nor->flags |= SNOR_F_HAS_SR_TB;
	if (info->flags & NO_CHIP_ERASE)    nor->flags |= SNOR_F_NO_OP_CHIP_ERASE;
	if (info->flags & USE_CLSR)         nor->flags |= SNOR_F_USE_CLSR;
	//if (info->flags & SPI_NOR_NO_ERASE) mtd->flags |= MTD_NO_ERASE;

	// TODO: 4byte address
	if (info->addr_width == 4)
	{
	}

	if (JEDEC_MFR(info) == SNOR_MFR_ATMEL ||
		JEDEC_MFR(info) == SNOR_MFR_INTEL ||
		JEDEC_MFR(info) == SNOR_MFR_SST ||
		info->flags & SPI_NOR_HAS_LOCK)
	{
		write_enable(nor);
		write_sr(nor, 0);
		spi_nor_wait_till_ready(nor);
	}

	// TODO
	//mtd->flags = convert_to_mtd_flag(nor->info->flags);

#if 0
	printf("%s info: size = 0x%" PRIx32 " (%" PRId32 "MiB), use area = 0x%" PRIx32 " ~ 0x%" PRIx32 " (%" PRId32 "MiB), erasesize = 0x%" PRIx32 "(%" PRIu32 "KiB)\r\n",
			nor->info->name, mtd->size, mtd->size>>20, mtd->base_addr, mtd->base_addr + mtd->use_size, mtd->use_size>>20,
			mtd->erasesize, mtd->erasesize >> 10);
#endif

	return 0;
}

const struct mtd_operation spi_nor_mtd_operation =
{
	.init  = spi_nor_init,
	.erase = spi_nor_erase,
	.read  = spi_nor_read,
	.write = spi_nor_write
};

