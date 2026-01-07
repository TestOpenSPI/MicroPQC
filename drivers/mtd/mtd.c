#if defined(MODULE_PRINTF)
#include "printf.h"
#else
#include <stdio.h>
#endif

#include "mtd.h"
#include <inttypes.h>

#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#elif defined(OS_CMSIS_RTX)
#include "cmsis_os2.h"
#elif defined(OS_LINUX)
#include <semaphore.h>
#endif

#define LOCK(m) (&m->lock[0])
static inline int _lock(mtd_t *mtd)
{
#if defined(OS_FREERTOS)
	SemaphoreHandle_t *s = (SemaphoreHandle_t *)LOCK(mtd);
	if (xSemaphoreTake(*s, portMAX_DELAY) != pdPASS)
		return -1;
#elif defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s = (osSemaphoreId_t *)LOCK(mtd);
	if (osSemaphoreAcquire(*s, osWaitForever) != osOK)
		return -1;
#elif defined(OS_LINUX)
    sem_t *s = (sem_t *)LOCK(mtd);
	sem_wait(s);
#else
	int *s = (int *)LOCK(mtd);
	while((*s) != 0);
	*s = 1;
#endif
	return 0;
}

static inline int _unlock(mtd_t *mtd)
{
#if defined(OS_FREERTOS)
	SemaphoreHandle_t *s = (SemaphoreHandle_t *)LOCK(mtd);
	xSemaphoreGive(*s);
#elif defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s = (osSemaphoreId_t *)LOCK(mtd);
	osSemaphoreRelease(*s);
#elif defined(OS_LINUX)
    sem_t *s = (sem_t *)LOCK(mtd);
	sem_post(s);
#else
	int *s = (int *)LOCK(mtd);
	*s = 0;
#endif
	return 0;
}

int mtd_init(mtd_t *mtd)
{
	if (!mtd->op)
		return -1;

#if defined(OS_FREERTOS)
	SemaphoreHandle_t *s = (SemaphoreHandle_t *)LOCK(mtd);
	if ((*s = xSemaphoreCreateMutex()) == NULL)
	{
		return -2;
	}
#elif defined(OS_CMSIS_RTX)
	osSemaphoreId_t *s = (osSemaphoreId_t *)LOCK(mtd);
    if ((*s = osSemaphoreNew(1, 0, NULL)) == NULL)
    {
        return -2;
    }
#elif defined(OS_LINUX)
    sem_t *s = (sem_t *)LOCK(mtd);
    sem_init(s, 0, 1);
#else
	int *s;
	s = (int *)LOCK(mtd);
	*s = 0;
#endif

	if (mtd->op->init(mtd) != 0)
		return -3;

	mtd->init_done = 1;

	return 0;
}

int mtd_erase(mtd_t *mtd, uint32_t addr, uint32_t len)
{
	int ret;

	if (!mtd)
	{
		return -1;
	}

#if 0
	if (addr >= mtd->use_size || len > mtd->use_size - addr)
	{
		return -1;
	}
	addr += mtd->base_addr;
#endif

	if (!mtd->erasesize || !mtd->op || !mtd->op->erase)
	{
        printf("1\n");
        return -1;
    }

	if (addr >= mtd->size || len > mtd->size - addr)
	{
		printf("2\n");
		return -1;
	}

	if (!len)
	{
		return 0;
	}

	_lock(mtd);
	ret = mtd->op->erase(mtd, addr, len);
	_unlock(mtd);

	// printf(" mtd->op->erase %d %ld %ld\n\r", ret, addr, len);

	return ret;
}

int mtd_read(mtd_t *mtd, uint32_t from, uint32_t len, uint8_t *buf, uint32_t *retlen)
{
	int ret;

	*retlen = 0;

	if (!mtd)
	{
		return -1;
	}

#if 0
	if (from >= mtd->use_size || len > mtd->use_size - from)
	{
		return -1;
	}
	from += mtd->base_addr;
#endif

	if (from >= mtd->size || len > mtd->size - from)
	{
		return -1;
	}

	if (!len)
		return 0;

	if (mtd->op && mtd->op->read)
	{
		_lock(mtd);
		ret = mtd->op->read(mtd, from, len, buf, retlen);
		_unlock(mtd);
		// printf(" mtd->op->read %d %ld %ld\n\r", ret, from, len);
	}
#if 0
	else if (mtd->_read_oob)
	{
		ret = mtd->_read_oob(mtd, from, ...);
		*retlen =
		return ret;
	}
#endif
	else
	{
		return -1;
	}

	return ret;
}

int mtd_write(mtd_t *mtd, uint32_t to, uint32_t len, const uint8_t *buf, uint32_t *retlen)
{
	int ret;

	*retlen = 0;

	if (!mtd)
	{
		return -1;
	}

#if 0
	if (to >= mtd->use_size || len > mtd->use_size - to)
	{
		return -1;
	}
	to += mtd->base_addr;
#endif

	if (to >= mtd->size || len > mtd->size - to)
	{
		return -1;
	}

	if (!mtd->op)
	{
		return -1;
	}

	if (!mtd->op->write /* && !mtd->op->write_oob */)
	{
		return -1;
	}

	if (!len)
	{
		return -1;
	}

	if (!mtd->op->write)
	{
#if 0
		ret = mtd->_write_oob(mtd, to, ...);
		*retlen =
		return ret;
#endif
	}
	_lock(mtd);
	ret =  mtd->op->write(mtd, to, len, buf, retlen);
	_unlock(mtd);
	// printf(" mtd->op->write %d %ld %ld\n\r", ret, to, len); /* sungmin 2023-12-01 ulock print */

	return ret;
}

int mtd_get_info(mtd_t *mtd, char *s)
{
	if (mtd && mtd->init_done && s) {
		sprintf(s, "mtd info: size = 0x%" PRIx32 " (%" PRId32 "MiB), erasesize = 0x%" PRIx32 "(%" PRIu32 "KiB)\r\n",
				mtd->size, mtd->size>>20,
				mtd->erasesize, mtd->erasesize >> 10);
		return 0;
	}
	return -1;
}

#define MTD_PART_CHECK() \
	if (!p) return -1; \
	if (addr >= p->limit) return -1; \
	if (addr + len > p->limit) return -1;

int mtd_part_init(mtd_part_t *p)
{
	if (!p) return -1;
	if (!p->mtd) return -1;
	if (p->mtd->init_done == 1) return 0;
	return mtd_init(p->mtd);
}

int mtd_part_read(mtd_part_t *p, uint32_t addr, uint32_t len, uint8_t *buf, uint32_t *retlen)
{
	MTD_PART_CHECK();
	return mtd_read(p->mtd, addr + p->base, len, buf, retlen);
}

int mtd_part_write(mtd_part_t *p, uint32_t addr, uint32_t len, const uint8_t *buf, uint32_t *retlen)
{
	MTD_PART_CHECK();
	return mtd_write(p->mtd, addr + p->base, len, buf, retlen);
}

int mtd_part_erase(mtd_part_t *p, uint32_t addr, uint32_t len)
{
	MTD_PART_CHECK();
	return mtd_erase(p->mtd, addr + p->base, len);
}

#if defined(MODULE_FATFS)

#include "ff.h"
#include "ffconf.h"
#include "diskio.h"

static DSTATUS mtd_disk_status(void *dev)
{
	mtd_part_t *p = dev;
	if (p && p->mtd && p->mtd->op)
		return 0;
	return STA_NOINIT;
}

static DSTATUS mtd_disk_initialize(void *dev)
{
	mtd_part_t *p = dev;
	if (mtd_part_init(p) == 0)
		return 0;
	return STA_NOINIT;
}

static DRESULT mtd_disk_read(void *dev, BYTE *buff, DWORD sector, UINT count)
{
	mtd_part_t *p = dev;
	mtd_t *mtd = dev;
	uint32_t retlen;

	if (mtd_read(mtd, sector * mtd->erasesize, count * mtd->erasesize, buff, &retlen) == 0)
	{
		if (retlen > 0)
		{
			if (retlen / mtd->erasesize == count)
			{
				return RES_OK;
			}
		}
	}
	return RES_ERROR;
}

static DRESULT mtd_disk_write(void *dev, const BYTE *buff, DWORD sector, UINT count)
{
	mtd_part_t *p = dev;
	mtd_t *mtd = p->mtd;
	uint32_t retlen;

	if (mtd_part_erase(p, sector * mtd->erasesize, count * mtd->erasesize) != 0)
	{
		return RES_ERROR;
	}

	if (mtd_part_write(p, sector * mtd->erasesize, count * mtd->erasesize, buff, &retlen) == 0)
	{
		if (retlen > 0)
		{
			if (retlen / mtd->erasesize == count)
			{
				return RES_OK;
			}
		}
	}
	return RES_ERROR;
}

static DRESULT mtd_disk_ioctl(void *dev, BYTE cmd, void *buff)
{
	mtd_part_t *p = dev;
	mtd_t *mtd = p->mtd;
	switch (cmd)
	{
#if (FF_FS_READONLY == 0)
		case CTRL_SYNC:
			return RES_OK;
#endif
#if (FF_USE_MKFS == 1)
		case GET_SECTOR_COUNT:
			*(DWORD *)buff = p->limit / mtd->erasesize;
			return RES_OK;
		case GET_BLOCK_SIZE:
			*(DWORD *)buff = 1;
			return RES_OK;
#endif
#if (FF_MAX_SS != FF_MIN_SS)
		case GET_SECTOR_SIZE:
			*(WORD *)buff = mtd->erasesize;
			return RES_OK;
#endif
#if (FF_USE_TRIM == 1)
		case CTRL_TRIM:
			return RES_OK;
#endif
	}

	return RES_ERROR;
}

const struct fatfs_diskio fatfs_mtd_diskio =
{
	.status     = mtd_disk_status,
	.initialize = mtd_disk_initialize,
	.read       = mtd_disk_read,
	.write      = mtd_disk_write,
	.ioctl      = mtd_disk_ioctl,
};

#endif
