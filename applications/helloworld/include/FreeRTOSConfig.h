#ifndef _SECURITYPLATFORM_FREERTOS_CONFIG_H_
#define _SECURITYPLATFORM_FREERTOS_CONFIG_H_

#if defined(TRUSTZONE_NONSECURE)
#include "FreeRTOSConfig_tz.h"
#else
#error Need TRUSTZONE configuration
#endif

#endif

