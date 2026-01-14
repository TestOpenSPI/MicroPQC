#include "hexdump.h"
#include <string.h>
#include <inttypes.h>
#if defined(MODULE_PRINTF)
#include "printf.h"
#endif

#undef isprint
#define isprint(c) ( ( (c&0xff)>=' ' && (c&0xff)<='~' ) ? 1 : 0 )

static int bitcount(uint32_t v)
{
    uint32_t c;
    c = v - ((v >> 1) & 0x55555555);
    c = ((c >> 2) & 0x33333333) + (c & 0x33333333);
    c = ((c >> 4) + c) & 0x0F0F0F0F;
    c = ((c >> 8) + c) & 0x00FF00FF;
    c = ((c >> 16) + c) & 0x0000FFFF;
    return c;
}

static void dump_1(uint32_t * target, int len, uint32_t offset, int attr)
{
    uint8_t *c;
    uint32_t *addr = (uint32_t *)target;
    char *ascii = NULL, buffer[128];
    int i, blen, ascii_idx;

    if (len <= 0)
        return;

    memset(buffer, ' ', sizeof(buffer));
    if (attr & HEXDUMP_ATTR_ASCII) {
        if (attr & HEXDUMP_ATTR_DEFAULT) {
            ascii_idx = 55;
        } else if (attr & (HEXDUMP_ATTR_XCOMMA)) {
            ascii_idx = 88;
        } else if (attr & HEXDUMP_ATTR_PRESS) {
            ascii_idx = 40;
        } else if (attr & HEXDUMP_ATTR_SWAP32) {
            ascii_idx = 43;
        } else {
            return;
        }
        buffer[ascii_idx] = '|';
        ascii = &buffer[ascii_idx+1];
    }

    blen = sprintf(buffer, "%04" PRIx32 "  ", offset);

    if (attr & (HEXDUMP_ATTR_DEFAULT | HEXDUMP_ATTR_PRESS | HEXDUMP_ATTR_XCOMMA)) {
        for (i = 0; i < (len / 4); i++) {
            volatile uint32_t val = addr[i];
            c = (uint8_t *)&val;
            if (attr & HEXDUMP_ATTR_DEFAULT) {
                blen += sprintf(buffer + blen, "%02x %02x %02x %02x ", c[0], c[1], c[2], c[3]);
            } else if (attr & HEXDUMP_ATTR_PRESS) {
                blen += sprintf(buffer + blen, "%02x%02x%02x%02x", c[0], c[1], c[2], c[3]);
            } else if (attr & HEXDUMP_ATTR_XCOMMA) {
                blen += sprintf(buffer + blen, "0x%02x,0x%02x,0x%02x,0x%02x,", c[0], c[1], c[2], c[3]);
            }
            if (attr & HEXDUMP_ATTR_ASCII) {
                *ascii++ = isprint(c[0])?c[0]:'.';
                *ascii++ = isprint(c[1])?c[1]:'.';
                *ascii++ = isprint(c[2])?c[2]:'.';
                *ascii++ = isprint(c[3])?c[3]:'.';
            }
        }
    }
    else if (attr & HEXDUMP_ATTR_SWAP32) {
        for (i = 0; i < (len / 4); i++) {
            c = (uint8_t *)&addr[i];
            blen += sprintf(buffer + blen, "%08" PRIx32 " ", addr[i]);
            if (attr & HEXDUMP_ATTR_ASCII) {
                *ascii++ = isprint(c[3])?c[3]:'.';
                *ascii++ = isprint(c[2])?c[2]:'.';
                *ascii++ = isprint(c[1])?c[1]:'.';
                *ascii++ = isprint(c[0])?c[0]:'.';
            }
        }
    }
    else {
        return;
    }

    c = (uint8_t *)&addr[i];
    if (attr & (HEXDUMP_ATTR_DEFAULT|HEXDUMP_ATTR_PRESS|HEXDUMP_ATTR_XCOMMA)) {
        for (i = 0; i < (len % 4); i++) {
            if (attr&HEXDUMP_ATTR_DEFAULT) {
                blen += sprintf(buffer + blen, "%02x ", c[i]);
            } else if (attr&HEXDUMP_ATTR_PRESS) {
                blen += sprintf(buffer + blen, "%02x", c[i]);
            } else if (attr&HEXDUMP_ATTR_XCOMMA) {
                blen += sprintf(buffer + blen, "0x%02x,", c[i]);
            }
            if (attr & HEXDUMP_ATTR_ASCII) {
                *ascii++ = isprint(c[i])?c[i]:'.';
            }
        }
        buffer[blen] = (attr & HEXDUMP_ATTR_ASCII)?' ':'\0';
    }
    else if (attr & HEXDUMP_ATTR_SWAP32) {
        if (len % 4) {
            buffer[blen] = ' ';
            blen += (2 * (4 - (len % 4)));
            if (attr & HEXDUMP_ATTR_ASCII) {
                ascii += (4 - (len %4));
            }
            for (i = (len % 4) - 1; i >= 0; i--) {
                blen += sprintf(buffer + blen, "%02x", c[i]);
                if (attr & HEXDUMP_ATTR_ASCII) {
                    *ascii++ = isprint(c[i])?c[i]:'.';
                }
            }
        }
        if (attr & HEXDUMP_ATTR_ASCII) {
            buffer[blen] = ' ';
        }
    }

    if (attr & HEXDUMP_ATTR_ASCII) {
        *ascii++ = '|';
        *ascii++ = '\0';
    }

    printf("%s\n", buffer);
}

void hexlog(char *str, void *data, size_t data_len)
{
    if(str == NULL)
        printf("NULL (%p %" PRId32 ")\n\r", data, (uint32_t)data_len);
    else
        printf("%s (%p %" PRId32 ")\n\r", str, data, (uint32_t)data_len);
    
    hexdump(data, data_len, 0);
}

void hexdump(void *target, size_t len, uint32_t attr)
{
	uint8_t *ptr = (uint8_t*)target;
	uint32_t offset = 0;
	int _attr = 0;
	uint32_t aligned_buf[4]; /* 16 bytes */

	if (!attr) {
		attr = HEXDUMP_ATTR_DEFAULT | HEXDUMP_ATTR_ASCII;
	} else {
		_attr = (attr & HEXDUMP_ATTR_ASCII);
		attr &= ~HEXDUMP_ATTR_ASCII;

		if (bitcount(attr) != 1) {
			printf("hexdump: invalid attr\n");
			return;
		}

		attr |= _attr;
	}

	if (len > 0x10000) {
		printf("hexdump: too long length\n");
		return;
	}

	while (len >= 16) {
                memcpy(aligned_buf, ptr, 16);           // in case target is not 32bit aligned
                dump_1(aligned_buf, 16, offset, attr);

		ptr += 16;
		offset += 16;
		len -= 16;
	}

	if (len > 0) {
                memcpy(aligned_buf, ptr, len);
                dump_1(aligned_buf, len, offset, attr);
	}
}

