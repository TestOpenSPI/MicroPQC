#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "spiflash.h"
#include "hexdump.h"
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

#define SPIFLASH_PAGE_SIZE (256)

#if 0 /* bccho, 2023-10-24, 8M */
#define SPIFLASH_TOTAL_SIZE (SPIFLASH_PAGE_SIZE * 4096)
#else
#define SPIFLASH_TOTAL_SIZE (SPIFLASH_PAGE_SIZE * 32768 * 2)
#endif

static int file_init(mtd_t *mtd)
{
    struct stat st;
    mtd->erasesize = SPIFLASH_PAGE_SIZE;
    mtd->page_size = SPIFLASH_PAGE_SIZE;
    mtd->size = SPIFLASH_TOTAL_SIZE;
    mtd->dev = (void *)SPIFLASH_FILENAME;

    FILE *fp = NULL;
    uint8_t *buf = NULL;
    buf = (uint8_t *)malloc(mtd->size);

    if(!(fp = fopen((char *)mtd->dev, "r+"))){
        fp = fopen((char *)mtd->dev, "w+");
        memset(buf, 0xFF, mtd->size);
        fwrite(buf, 1, mtd->size, fp);
        free(buf);
    }

    fstat(fileno(fp), &st);

    if (st.st_size != mtd->size)
    {
        remove((char *)mtd->dev);
        fp = fopen((char *)mtd->dev, "w+");

        memset(buf, 0xFF, mtd->size);
        fwrite(buf, 1, mtd->size, fp);
        free(buf);
    }

    if (fp != NULL)
    {
        fclose(fp);
        return 0;
    }
    return 0;
}

static int file_erase(mtd_t *mtd, uint32_t addr, uint32_t len)
{
    int ret = -1;
    uint8_t *buf = NULL;
    uint32_t rem;
    FILE *fp = NULL;

    fp = fopen((char *)mtd->dev, "r+");

    rem = len % mtd->erasesize;
    if (rem)
        goto end;

    buf = malloc(len);
    if (buf == NULL)
        goto end;

    memset(buf, 0xFF, len);

    fseek(fp, addr, SEEK_SET);
    ret = fwrite(buf, 1, len, fp);
    ret = 0;

end:
    if (buf != NULL)
    {
        free(buf);
    }

    if (fp != NULL)
        fclose(fp);

    return ret;
}

static int file_write(mtd_t *mtd, uint32_t to, uint32_t len, const uint8_t *buf,
                      uint32_t *retlen)
{
    int ret = -1;
    uint8_t *rbuf = NULL;
    FILE *fp = NULL;

    fp = fopen((char *)mtd->dev, "r+");

    rbuf = malloc(len);

    fseek(fp, to, SEEK_SET);
    if (fread(rbuf, 1, len, fp) != len)
    {
        goto end;
    }
    for (uint32_t i = 0; i < len; i++)
    {
        rbuf[i] = rbuf[i] & buf[i];
    }

    fseek(fp, to, SEEK_SET);
    *retlen = fwrite(rbuf, 1, len, fp);
    ret = 0;
end:
    if (rbuf != NULL)
    {
        free(rbuf);
    }

    if (fp != NULL)
    {
        fclose(fp);
    }

    return ret;
}

static int file_read(mtd_t *mtd, uint32_t from, uint32_t len, uint8_t *buf,
                     uint32_t *retlen)
{
    (void)mtd;
    int ret = -1;
    FILE *fp = NULL;

    fp = fopen(mtd->dev, "rb");
    if (fp == NULL)
    {
        goto end;
    }

    fseek(fp, from, SEEK_SET);
    *retlen = fread(buf, 1, len, fp);
    ret = 0;
end:
    if (fp != NULL)
        fclose(fp);

    return ret;
}

const struct mtd_operation file_mtd_operation = {.init = file_init,
                                                 .erase = file_erase,
                                                 .read = file_read,
                                                 .write = file_write};
