#ifndef _MTD_H_
#define _MTD_H_

#include <stdint.h>

#define MTD_ABSENT      0
#define MTD_RAM         1
#define MTD_ROM         2
#define MTD_NORFLASH        3
#define MTD_NANDFLASH       4   /* SLC NAND */
#define MTD_DATAFLASH       6
#define MTD_UBIVOLUME       7
#define MTD_MLCNANDFLASH    8   /* MLC NAND (including TLC) */

typedef struct mtd_info
{
	void *dev;
	const struct mtd_operation *op;

	uint32_t erasesize;
	uint32_t size;
	uint32_t page_size;

	int type;
	uint32_t flags;

#if 1
	uint32_t base_addr;
	uint32_t use_size;
#endif
#if 0
	_read_oob
	_write_oob
#endif

	int (*_lock) (struct mtd_info *mtd, uint32_t ofs, uint32_t len);
	int (*_unlock) (struct mtd_info *mtd, uint32_t ofs, uint32_t len);
	int (*_is_locked) (struct mtd_info *mtd, uint32_t ofs, uint32_t len);

	uint32_t lock[8]; // opaque data to hide OS type. 32byte is large enough
	int init_done;
} mtd_t;

typedef struct mtd_part {
	mtd_t *mtd;
	uint32_t base;
	uint32_t limit;
} mtd_part_t;

struct mtd_operation
{
	int (*init) (mtd_t *mtd);

    int (*erase) (mtd_t *mtd, uint32_t addr, uint32_t len);
    int (*read) (mtd_t *mtd, uint32_t from, uint32_t len,
			uint8_t *buf, uint32_t *retlen);
    int (*write) (mtd_t *mtd, uint32_t to, uint32_t len,
			const uint8_t *buf, uint32_t *retlen);
};

int mtd_init(mtd_t *mtd);
int mtd_erase(mtd_t *mtd, uint32_t addr, uint32_t len);
int mtd_read(mtd_t *mtd, uint32_t from, uint32_t len, uint8_t *buf, uint32_t *retlen);
int mtd_write(mtd_t *mtd, uint32_t to, uint32_t len, const uint8_t *buf, uint32_t *retlen);
int mtd_get_info(mtd_t *mtd, char *s);

int mtd_part_init(mtd_part_t *p);
int mtd_part_read(mtd_part_t *p, uint32_t addr, uint32_t len, uint8_t *buf, uint32_t *retlen);
int mtd_part_write(mtd_part_t *p, uint32_t addr, uint32_t len, const uint8_t *buf, uint32_t *retlen);
int mtd_part_erase(mtd_part_t *p, uint32_t addr, uint32_t len);
#if defined(MODULE_FATFS)

extern const struct fatfs_diskio fatfs_mtd_diskio;

#endif

#endif

