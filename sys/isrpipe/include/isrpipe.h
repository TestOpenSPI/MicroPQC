/*
 * Copyright (C) 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup isr_pipe ISR Pipe
 * @ingroup sys
 * @brief ISR -> userspace pipe
 *
 * @{
 * @file
 * @brief       isrpipe Interface
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 */

#ifndef ISRPIPE_H
#define ISRPIPE_H

#include <stdint.h>
#include "tsrb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Context structure for isrpipe
 */

struct _sema {
	uint32_t opaque[8]; // to hide OS type. 32byte is large enough
};

typedef struct {
    tsrb_t tsrb;        /**< isrpipe thread safe ringbuffer */
	struct _sema sema;
} isrpipe_t;

/**
 * @brief   Initialisation function for isrpipe
 *
 * @param[in]   isrpipe     isrpipe object to initialize
 * @param[in]   buf         buffer to use as ringbuffer (must be power of two sized!)
 * @param[in]   bufsize     size of @p buf
 */
int isrpipe_init(isrpipe_t *isrpipe, char *buf, size_t bufsize);

void isrpipe_free(isrpipe_t *isrpipe);



/**
 * @brief   Put one character into the isrpipe's buffer
 *
 * @param[in]   isrpipe     isrpipe object to initialize
 * @param[in]   c           character to add to isrpipe buffer
 *
 * @returns     0 if character could be added
 * @returns     -1 if buffer was full
 */
int isrpipe_write_one(isrpipe_t *isrpipe, char c);
int isrpipe_write(isrpipe_t *isrpipe, const char *src, size_t n);

/**
 * @brief   Read data from isrpipe (blocking)
 *
 * @param[in]   isrpipe    isrpipe object to operate on
 * @param[in]   buf        buffer to write to
 * @param[in]   count      number of bytes to read
 *
 * @returns     number of bytes read
 */
int isrpipe_read(isrpipe_t *isrpipe, char *buf, size_t count, uint32_t timeout);

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* ISRPIPE_H */
