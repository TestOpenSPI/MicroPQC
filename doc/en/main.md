<p align="right">
  <a href="main.md">English</a> | <a href="../main.md">한국어</a>
</p>

# MicroPQC

This project is built on Arm TrustZone.

Arm TrustZone splits the system into a Secure zone and a Non-Secure zone.

- **Secure Zone** → Secure firmware
- **Non-Secure Zone** → User firmware

The Secure firmware includes the Bootloader and Secure world, while the user firmware refers to this SDK project.
The Secure firmware is provided as a binary, and the user firmware is provided as an SDK.

## Overview

- **Secure firmware**: Includes Bootloader + Secure region. Provided as a binary (`sprom_GENERIC_M2354_KPQC.bin`) under `secure/`.
- **User firmware**: The region where application code runs in the Non-Secure world (this SDK).
- **RTOS**: FreeRTOS
- **Crypto module**: AxioCrypto
- **MCU**: Nuvoton M2354

## Table of Contents

- [Development environment](01.development-environment.md)
- [Setup](02.setup.md)
- [Build](03.build.md)
- [Firmware download](04.firmware-download.md)
- [Runtime](05.runtime.md)
- [Examples](06.examples.md)
- [Modules](07.modules.md)
- [Memory layout](08.memory-layout.md)
- [Project structure](09.project-structure.md)
