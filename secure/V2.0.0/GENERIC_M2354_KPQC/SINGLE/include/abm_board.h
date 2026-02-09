#ifndef ABM_BOARD_H
#define ABM_BOARD_H

/*
 * if 1, exec_bank and swap_bank must be implemented suitably
 * if 0, exec_bank should return 0
 */

#if ABM_SUPPORT_BANKSWAP == 1
#define ABM_SUPPORT_BANKSWAP              1
#else
#define ABM_SUPPORT_BANKSWAP              0
#endif

/*
 * if 1, the header of axsign is not required
 * Also, this imples ABM_ALLOW_ROLLBACK_IMAGE = 1
 */
#define ABM_NSPE_HEADER_TYPE              HEADER_TYPE_DEVELOPMENT
/*
 * if 1, image version rollback can be done
 * if 0, image version cannot be rolled back
 */
#define ABM_ALLOW_ROLLBACK_IMAGE          1

#if defined(ABM_PRODUCT_MODE) && ABM_PRODUCT_MODE == 1
#define ABM_PRODUCT_MODE_STRING  "MP"
#define ABM_NO_VERIFY_DEVICE_CERT
#else 
#define ABM_PRODUCT_MODE_STRING  ""
// #define ABM_NO_VERIFY_DEVICE_CERT
#endif

/*
 * Least Common Multiple of erase size of 2 banks
 * Bank0: 2K (m2354 internal flash)
 * Bank1: 4K (spiflash)
 * LCM(2048, 4096) = 4096
 */
#define _FLASH_ERASE_SIZE          4096

#define OTP_KEY_BLOCK_START	       16
#define FLASH_CFG1_ADDR            (FMC_LDROM_BASE)
#define FLASH_CFG2_ADDR            (FMC_LDROM_BASE+FMC_FLASH_PAGE_SIZE)

#define ABM_VER_ADDR             0x00000020
#define ABM_CDI_ADDR             __StackLimit

#define CID0                       0x1122
#define CID1                       0x3344
#define CID2                       0x5566
#define CID3                       0x7788
#define CID4                       0x1122
#define CID5                       0x3344
#define CID6                       0x5566
#define CID7                       0x7788
#define CID8                       0x1122
#define CID9                       0x3344
#define CID10                      0x5566
#define CID11                      0x7788
#define CID12                      0x1122
#define CID13                      0x3344
#define CID14                      0x5566
#define CID15                      0x7788

#include "partition_M2354_tz.h"

#endif
