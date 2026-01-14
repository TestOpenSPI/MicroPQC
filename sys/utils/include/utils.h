#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdio.h>

int hexTobin(const char *hex, uint8_t *bin, int bin_len);
int hex2bin(const char* hex, int hexSize, uint8_t* bin);
int bin2hex(const uint8_t* bin, int binSize, char *hex, int hex_size);

#endif
