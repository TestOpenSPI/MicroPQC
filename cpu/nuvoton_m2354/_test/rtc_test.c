#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "shell.h"
#include "periph_api.h"
#include "NuMicro.h"

#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif
/*
Command list(argv[])
argv[0] argv[1] argv[2] argv[3] argv[4] argv[5] argv[6] argv[7]
rtc     g
rtc     s       year    month   day     hour    min     sec
*/
#if defined(TRUSTZONE_NONSECURE) && !defined(BOARD_NATIVE)

static int _example_rtc_help()
{
	printf("RTC Test : rtc [option] \n\r");
	printf("Options : \n\r");
	printf("  setclock  [internal or external]                         set RTC clock\n\r");
	printf("  settime   [year]  [month]  [day]  [hour]  [min]  [sec]   set test time\n\r");
	printf("  gettime                                                  get test time \n\r");
	return 0;
}

static int _rtc_handler(int argc, char *argv[])
{
	struct tm t;

	if (argc > 1) {
		if (strcmp(argv[1], "settime") == 0 && argc == 8) {
			t.tm_year = strtoul(argv[2], NULL, 10) - 1900,
			t.tm_mon = strtoul(argv[3], NULL, 10) - 1;
			t.tm_mday = strtoul(argv[4], NULL, 10);
			t.tm_hour = strtoul(argv[5], NULL, 10);
			t.tm_min = strtoul(argv[6], NULL, 10);
			t.tm_sec = strtoul(argv[7], NULL, 10);

			printf("rtc_set: %u-%u-%u %u:%u:%u\n",
				t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
				t.tm_hour, t.tm_min, t.tm_sec);

			rtc_set(&t);
			printf("rtc time set success\n\r");
		} else if (strcmp(argv[1], "setclock") == 0 && argc == 3){
			if (strcmp(argv[2], "external") == 0) {
				rtc_set_clock(RTC_CLOCK_EXTERNAL); // RTC SET EXTERNAL Crystal
				printf("set external clock\n\r");
			} else if(strcmp(argv[2], "internal") == 0 && argc == 3) {
				rtc_set_clock(RTC_CLOCK_INTERNAL);
				printf("set internal clock\n\r");
			} else {
				printf("select internal or external\n\r");
				return 1;	
			}
		} else if(strcmp(argv[1], "gettime") == 0) {
			rtc_get(&t);
			printf("rtc_get: %d-%d-%d %d:%d:%d\n",
				t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
				t.tm_hour, t.tm_min, t.tm_sec);
		} else { 
			_example_rtc_help();
		}
	} else {
		_example_rtc_help();
	}
	return 0;
}

DEFINE_SHELL_CMD(rtc, "rtc test", _rtc_handler);
#endif
