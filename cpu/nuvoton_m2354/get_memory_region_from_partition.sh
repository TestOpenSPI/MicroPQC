#!/bin/bash

# $1: TRUSTZONE
# $2: TRUSTZONE_APPDIR or APPDIR

f=$2/include/partition_M2354_tz.h
b=$2/include/abm_board.h

securerom=$(sed -n 's/^#define FMC_SECURE_ROM_SIZE[ \t]\+\(0x.*\)/\1/p' $f)
securerom=$((securerom+0x1000))
secureram=$(sed -n 's/^#define SCU_SECURE_SRAM_SIZE[ \t]\+\(0x.*\)/\1/p' $f)

flash_length=0x00100000
if grep -q '^[ \t]*#define[ \t]*ABM_SUPPORT_BANKSWAP[ \t]*1[\t]*$' $b; then
	flash_length=0x00080000
fi

if [ "$1" == "secure" ]; then
	echo "ld_flash_start=$(printf "0x%lx"  $((0x0000d000)))"
	echo "ld_flash_length=$(printf "0x%lx" $((0x00022000)))"
	echo "ld_ram_length=$(printf "0x%lx"   $((secureram)))"
elif [ "$1" == "nonsecure" ]; then
	echo "ld_flash_start=$(printf "0x%lx"  $((0x10000000+securerom)))"
	echo "ld_flash_length=$(printf "0x%lx" $((flash_length-securerom)))"
	echo "ld_ram_start=$(printf "0x%lx"    $((0x30000000+secureram)))"
	echo "ld_ram_length=$(printf "0x%lx"   $((0x00040000-secureram)))"
fi

