#include <stdio.h>
#if defined(MODULE_PRINTF)
#include "printf.h"
#endif
#include "version.h"

#define xstr(s) str(s)
#define str(s) #s

#define __VERSION_STRING  xstr(__VERSION_MAJOR) "." xstr(__VERSION_MINOR) "." xstr(__VERSION_PATCH)
#define __VERSION_STRING_FULL  "v" __VERSION_STRING "(" __BUILD_TIMESTAMP ")"

char *version_get_string()
{
	return __VERSION_STRING_FULL;
}
