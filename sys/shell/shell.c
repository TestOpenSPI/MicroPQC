/*
 * Copyright (C) 2009, Freie Universitaet Berlin (FUB).
 * Copyright (C) 2013, INRIA.
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell
 * @{
 *
 * @file
 * @brief       Implementation of a very simple command interpreter.
 *              For each command (i.e. "echo"), a handler can be specified.
 *              If the first word of a user-entered command line matches the
 *              name of a handler, the handler will be called with the whole
 *              command line as parameter.
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      René Kijewski <rene.kijewski@fu-berlin.de>
 *
 * @}
 */
// clang-format off
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "cpu.h"
#include "shell.h"

#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif

#if !defined(SHELL_NO_ECHO) || !defined(SHELL_NO_PROMPT)
#ifdef MODULE_NEWLIB
/* use local copy of putchar, as it seems to be inlined,
 * enlarging code by 50% */
static void _putchar(int c) {
    putchar(c);
}
#else
#define _putchar putchar
#endif
#endif

static void flush_if_needed(void)
{
#if defined(MODULE_NEWLIB)
    fflush(stdout);
#endif
}

#if defined( __ICCARM__ )
#pragma section = "__shell_command_start__"
static const shell_command_t *__shell_command_start__ = (shell_command_t *)__section_begin("__shell_command_start__");
#elif defined( __ARMCC_VERSION )
extern unsigned int Image$$ER_IROM_SHELLCMD$$Base;
static const shell_command_t *__shell_command_start__ = (shell_command_t *)&Image$$ER_IROM_SHELLCMD$$Base;
#else
extern const shell_command_t __shell_command_start__[];
#endif

#ifndef __LP64__
static shell_command_handler_t find_handler(const shell_command_t *command_list, char *command)
{
    const shell_command_t *command_lists[] = {
        __shell_command_start__,
        command_list,
    };

    const shell_command_t *entry;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                if (strcmp(entry->name, command) == 0) {
                    return entry->handler;
                }
                else {
                    entry ++;
                }
            }
        }
    }

    return NULL;
}

static void print_help(const shell_command_t *command_list)
{
    printf("%-20s %s\r\n", "Command", "Description");
    puts("---------------------------------------\r\n");

    const shell_command_t *command_lists[] = {
        __shell_command_start__,
        command_list,
    };

    const shell_command_t *entry;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                printf("%-20s %s\r\n", entry->name, entry->desc);
                entry++;
            }
        }
    }
}
#else
static shell_command_handler_t find_handler(const shell_command_t *command_list, char *command)
{
    const shell_command_t *command_lists[] = {
        __shell_command_start__,
        command_list,
    };

    const shell_command_t *entry;
    void *p;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                if (strcmp(entry->name, command) == 0) {
                    return entry->handler;
                }
                else {
                    p = (void *)entry;
                    p += 32;
                    entry = (const shell_command_t *)p;
                }
            }
        }
    }

    return NULL;
}

static void print_help(const shell_command_t *command_list)
{
    printf("%-20s %s\r\n", "Command", "Description");
    puts("---------------------------------------\r\n");

    const shell_command_t *command_lists[] = {
        __shell_command_start__,
        command_list,
    };

    const shell_command_t *entry;
    void *p;

    /* iterating over command_lists */
    for (unsigned int i = 0; i < sizeof(command_lists) / sizeof(entry); i++) {
        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                printf("%-20s %s\r\n", entry->name, entry->desc);
                p = (void *)entry;
                p += 32;
                entry = (const shell_command_t *)p;
            }
        }
    }
}
#endif
static void handle_input_line(const shell_command_t *command_list, char *line)
{
    static const char *INCORRECT_QUOTING = "shell: incorrect quoting";

    /* first we need to calculate the number of arguments */
    unsigned argc = 0;
    char *pos = line;
    int contains_esc_seq = 0;
    while (1) {
        if ((unsigned char) *pos > ' ') {
            /* found an argument */
            if (*pos == '"' || *pos == '\'') {
                /* it's a quoted argument */
                const char quote_char = *pos;
                do {
                    ++pos;
                    if (!*pos) {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                    else if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                        continue;
                    }
                } while (*pos != quote_char);
                if ((unsigned char) pos[1] > ' ') {
                    puts(INCORRECT_QUOTING);
                    return;
                }
            }
            else {
                /* it's an unquoted argument */
                do {
                    if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                    }
                    ++pos;
                    if (*pos == '"') {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                } while ((unsigned char) *pos > ' ');
            }

            /* count the number of arguments we got */
            ++argc;
        }

        /* zero out the current position (space or quotation mark) and advance */
        if (*pos > 0) {
            *pos = 0;
            ++pos;
        }
        else {
            break;
        }
    }
    if (!argc) {
        return;
    }

    /* then we fill the argv array */
//    char *argv[argc + 1];
    #define ARG_MAX 20
    if (ARG_MAX < argc)
    {
        printf("shell: too many argc(max:current %d:%d)\n", ARG_MAX, argc);
        return;
    }
    char *argv[ARG_MAX];
    argv[argc] = NULL;
    pos = line;
    for (unsigned i = 0; i < argc; ++i) {
        while (!*pos) {
            ++pos;
        }
        if (*pos == '"' || *pos == '\'') {
            ++pos;
        }
        argv[i] = pos;
        while (*pos) {
            ++pos;
        }
    }
    for (char **arg = argv; contains_esc_seq && *arg; ++arg) {
        for (char *c = *arg; *c; ++c) {
            if (*c != '\\') {
                continue;
            }
            for (char *d = c; *d; ++d) {
                *d = d[1];
            }
            if (--contains_esc_seq == 0) {
                break;
            }
        }
    }

    /* then we call the appropriate handler */
    shell_command_handler_t handler = find_handler(command_list, argv[0]);
    if (handler != NULL) {
        handler(argc, argv);
    }
    else {
        if (strcmp("help", argv[0]) == 0) {
            print_help(command_list);
        }
        else {
            printf("shell: command not found: %s\n", argv[0]);
        }
    }
}

static int readline(char *buf, size_t size)
{
    char *line_buf_ptr = buf;

    while (1) {
        if ((line_buf_ptr - buf) >= ((int) size) - 1) {
            return -1;
        }

        int c = getchar();
        if (c < 0)
        {
#ifndef SHELL_NO_ECHO
            return EOF;
#else       
#if defined(CPU_X86_64)
            /* bccho, 2023-10-27, shell Task보다 낮은 우선순위 Task 동작하도록 */
            vTaskDelay(10); 
#endif
            continue;
#endif
        }

        /* We allow Unix linebreaks (\n), DOS linebreaks (\r\n), and Mac linebreaks (\r). */
        /* QEMU transmits only a single '\r' == 13 on hitting enter ("-serial stdio"). */
        /* DOS newlines are handled like hitting enter twice, but empty lines are ignored. */
        if (c == '\r' || c == '\n') {
            *line_buf_ptr = '\0';
#ifndef SHELL_NO_ECHO
            _putchar('\r');
            _putchar('\n');
#endif

            /* return 1 if line is empty, 0 otherwise */
            return line_buf_ptr == buf;
        }
        /* QEMU uses 0x7f (DEL) as backspace, while 0x08 (BS) is for most terminals */
        else if (c == 0x08 || c == 0x7f) {
            if (line_buf_ptr == buf) {
                /* The line is empty. */
                continue;
            }

            *--line_buf_ptr = '\0';
            /* white-tape the character */
#ifndef SHELL_NO_ECHO
            _putchar('\b');
            _putchar(' ');
            _putchar('\b');
#endif
        }
        else {
            *line_buf_ptr++ = c;
#ifndef SHELL_NO_ECHO
            _putchar(c);
#endif
        }
        flush_if_needed();
    }
}

__STATIC_INLINE void print_prompt(void)
{
#ifndef SHELL_NO_PROMPT
    _putchar('>');
    _putchar(' ');
#endif

    flush_if_needed();
}

static void shell_run(const shell_command_t *shell_commands, char *line_buf, int len)
{
    print_prompt();

    while (1) {
        int res = readline(line_buf, len);

        if (res == EOF) {
            break;
        }

        if (!res) {
            handle_input_line(shell_commands, line_buf);
        }

        print_prompt();
    }
}

#if defined( __ICCARM__ )
#pragma location = "shell_command_end"
const shell_command_t __shell_cmd_end = { .name = NULL, .desc = NULL, .handler = NULL };
#else
const shell_command_t __shell_cmd_end __attribute__((used,aligned(4),section("shell_command_end"))) = { .name = NULL, .desc = NULL, .handler = NULL };
#endif

#ifdef MODULE_AXIOCRYPTO_SHELLCMD
#define SZ_SHELL_LINEBUF 1024
#else
#define SZ_SHELL_LINEBUF 128
#endif
static char line_buf[SZ_SHELL_LINEBUF];

void shell_thread(void *arg)
{
#if defined(OS_FREERTOS)
#if !defined(MODULE_NSCBROKER)
	portALLOCATE_SECURE_CONTEXT( configMINIMAL_SECURE_STACK_SIZE );
#endif
#endif

    while(1)
	{
        shell_run((const shell_command_t *)arg, line_buf, sizeof(line_buf));
    }
}

