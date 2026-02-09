#ifndef __USER__BOARD_H__
#define __USER__BOARD_H__

#include "cpu.h"

#if defined(MODULE_MTD)
#include "mtd.h"

#define MTD_DEVICE_ABM (-1)
#define MTD_DEVICE_APP 0
mtd_t* board_get_mtd_device(int mtd_index);
#endif

void board_init(void);

void board_fini(void);

#endif
