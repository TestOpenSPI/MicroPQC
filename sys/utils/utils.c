#include <string.h>
#include <stdlib.h>
#include <stdio.h>
// #include <unistd.h>
#include <stdint.h>

#include "utils.h"

int _nibble(const char c)
{
    int n = 0;

    if ('0' <= c && c <= '9')
        n = c-'0';
    else if ('a' <= c && c <= 'f')
        n = 10 + c-'a';
    else if ('A'<= c && c <= 'F')
        n = 10 + c - 'A';
    else
        n = -1;

    return n;
}

int _h2b(const char *s)
{
    int ret = 0;
    int check_ret = 0;
    int i = 0;

    for (i = 0; i < 2; i++){
        char c = *s++;

        check_ret = _nibble(c);
        if (check_ret == -1) {
            ret = check_ret;
            break;
        }
        ret = check_ret + ret * 16;
    }

    return ret;
}

int hexTobin(const char *hex, uint8_t *bin, int bin_len)
{
    int len;

    if (hex == NULL || bin == NULL || bin_len <= 0)
        return -1;
    
    len = strlen(hex);
    
    if (len % 2 != 0 || len / 2 > bin_len)
        return -1;

    if (hex2bin(hex, len, bin) != 0)
        return 0;

    return len / 2;
}

int hex2bin(const char* hex, int hexSize, uint8_t* bin)
{
    int ret = 0;
    int check_ret = 0;
    int i = 0;
    int rest = 0;

    if ((hexSize % 2) != 0) {
        rest = 1;
        hexSize += 1;
    }

    for (i = 0; i < (hexSize / 2); i++) {
        if ((rest == 1) && (i == 0)) {
            check_ret = _nibble(hex[0]);
            if (check_ret == -1) {
                ret = check_ret;
                break;
            }
            bin[i] = check_ret;
            hex += 1;
        } else {
            check_ret = _h2b(hex);
            if (check_ret == -1) {
                ret = check_ret;
                break;
            }
            bin[i] = check_ret;
            hex += 2;
        }
    }

    return ret;
}

int bin2hex(const uint8_t* bin, int binSize, char *hex, int hex_size) 
{
    if (bin == NULL || hex == NULL || binSize <= 0) {
        return -1;
    }

    if (hex_size < binSize * 2 + 1) {
        return -1;
    }
    
    for (int i = 0; i < binSize; i++) {
        sprintf(hex + (i * 2), "%02X", bin[i]);
    }
    hex[binSize * 2] = '\0';
    
    return binSize * 2;
}