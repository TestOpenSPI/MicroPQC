#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"

#if defined(MODULE_VERSION)
#include "version.h"
#endif

#if defined(MODULE_UART_STDOUT)
#include "uart_stdout.h"
#endif

#include "periph_conf.h"
#include "periph_api.h"

#if defined(MODULE_SHELL)
#include "uart_stdin.h"
#include "shell.h"
extern const shell_command_t _app_command_list[];
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "axiocrypto.h"

static int banner()
{
	printf("\r\n\r\n---------------------------------\r\n");
    printf("|  %-28s |\r\n", __PROJECT_NAME);
	printf("|  %-28s |\r\n", version_get_string());
    printf("|  SecurityPlatform Inc.        |\r\n");
	printf("---------------------------------\r\n");
	return 0;
}

int main(void)
{
    board_init();

#if defined(MODULE_UART_STDOUT)

    uart_stdout_init();

#if defined(MODULE_VERSION)
    banner();
#endif
#endif

    CRYPTO_STATUS status = axiocrypto_init(NULL, 0);
    if (status == CRYPTO_SUCCESS || status == CRYPTO_ERR_ALREADY_INIT) {
        char info[36] = {0};
        operation_mode_t opmode;
        if (axiocrypto_info(info, sizeof(info), &opmode) != CRYPTO_SUCCESS) {
            printf("AxioCrypto info failed\n");
            goto exit;
        }
        printf("AxioCrypto Version : %s\n", info);
        printf("AxioCrypto Operation Mode : %s\n", opmode == OP_MODE_APPROVED_KCMVP ? "KCMVP Mode" : "Non-Approved Mode");
        if (opmode != OP_MODE_APPROVED_KCMVP) {
            printf("Axiocrypto change to KCMVP mode and reset \n");
            if (axiocrypto_set_mode(OP_MODE_APPROVED_KCMVP) != CRYPTO_SUCCESS) {
                printf("AxioCrypto set mode failed\n");
                goto exit;
            }
        }
    }

#if defined(MODULE_SHELL)
    uart_stdin_init();
    if (xTaskCreate(shell_thread, "shell", 8192, (void *)_app_command_list,
                    configMAX_PRIORITIES - 1, NULL) != pdTRUE)  // 2048, -1
    {
        printf("shell thread could not be launched\n");
        goto exit;
    }
#endif

    vTaskStartScheduler();

exit:
    //  unreachable
    while (1) {
    }

    return 0;
}
