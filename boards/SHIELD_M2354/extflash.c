#include "periph_conf.h"
#include "spiflash.h"
#include "board.h"
#include "mtd.h"
#if defined(MODULE_FATFS)
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"
#endif

static struct spi_nor spi_nor_flash =
{
	.spi = &spi_nor_dev,
};

#ifdef MODULE_MTD
mtd_t mtd_map[] = {
	// for application
	[MTD_DEVICE_APP] = 
	{
		.use_size = 0,	// use remains.
		.base_addr = 0,
		.dev = &spi_nor_flash,
		.op = &spi_nor_mtd_operation,
	}
};


mtd_t *board_get_mtd_device(int mtd_index)
{
	if (mtd_index < 0)
		return NULL;

	if((size_t)(mtd_index + 1) > (sizeof(mtd_map) / sizeof(mtd_t)))
		return NULL;

	return &mtd_map[mtd_index];
}
#endif

#if defined(MODULE_FATFS)
static const struct fatfs_disk_map _fatfs_disk_map[FF_VOLUMES] =
{
	/* volume 0 */
	{
		.disk = &mtd_device,
		.diskio = &fatfs_mtd_diskio
	},
};

const struct fatfs_disk_map *find_fatfs_diskio(BYTE pdrv)
{
	if (pdrv <FF_VOLUMES)
		return &_fatfs_disk_map[pdrv];
	return NULL;
}

FATFS FatFs[FF_VOLUMES];
#endif

