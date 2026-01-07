#ifndef __LAYOUT_H
#define __LAYOUT_H

#include "abm_board.h"

#if ABM_SUPPORT_BANKSWAP == 0
#define GET_BANK1_ADDR(addr)   ( addr )
#else 
#define GET_BANK1_ADDR(addr)   ( addr | 0x80000)
#endif

#define LAYOUT_BOOTLOADER_ADDR      0x00000000
#define LAYOUT_BOOTLOADER_SIZE      0x0001A000

#define LAYOUT_SECURE_ADDR          0x0001B000
#define LAYOUT_SECURE_SIZE          0x00034000

#define LAYOUT_SECURE_RAM_SIZE      0x00030000

#define LAYOUT_CONFIG_ADDR          0x0004F000
#define LAYOUT_CONFIG_SIZE          0x00001000

#define LAYOUT_NONSECURE_ADDR       0x00050000

#if ABM_SUPPORT_BANKSWAP == 0
#define LAYOUT_NONSECURE_SIZE       0x000D0000
#else
#define LAYOUT_NONSECURE_SIZE       0x0004F000
#endif

#endif
