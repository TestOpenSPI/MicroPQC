# 사용자(Non-Secure) 보드 설정
BOARD = SHIELD_M2354

# RTOS 설정
OS = freertos

# trustzone 설정
TRUSTZONE = nonsecure

# 사용할 Secure 펌웨어, 버전, BANK 모드(DUAL or SINGLE) 설정
SECURE_FW = GENERIC_M2354_KPQC
SECURE_VER = V2.0.0
SECURE_BANK = SINGLE

# 사용할 모듈 설정
USEMODULE += shell hexdump 
USEMODULE += axiocrypto_nonsecure_sdk
USEMODULE += axiocrypto_pqc_example
USEMODULE += spiflash mtd
USEMODULE += utils
USEMODULE += printf

# 테스트 라이브러리 포함 여부 (0: 미포함, 1: 포함)
INCLUDE_TESTLIB = 0

# 빌드 VERBOSE 설정 (1: 조용히, 0: 자세히)
QUIET = 1

# 최적화 옵션 설정 (예: -O0, -O1, -O2, -O3, -Os)
CFLAG_OPT = -Oz

# 이하 자동 설정(변경하지 마십시오)
APPDIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

APPLICATION = $(notdir $(APPDIR))

BASE := $(abspath $(APPDIR)/../..)

include $(BASE)/Makefile.include

