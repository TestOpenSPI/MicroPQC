<p align="right">
  <a href="en/main.md">English</a> | <a href="main.md">한국어</a>
</p>

# MicroPQC

이 프로젝트는 Arm TrustZone 기반으로 작성되어 있습니다.

Arm TrustZone은 Secure Zone과 Non-Secure Zone으로 구성됩니다.
- **Secure Zone** → 시큐어 펌웨어
- **Non-Secure Zone** → 유저 펌웨어

시큐어 펌웨어는 Bootloader와 Secure를 포함하며, 유저 펌웨어는 이 SDK 프로젝트를 의미합니다.
시큐어 펌웨어는 바이너리로, 유저 펌웨어는 SDK 형태로 제공됩니다.

## 개요

- **시큐어 펌웨어**: Bootloader + Secure 영역을 포함하며, `secure/` 폴더의 바이너리(`sprom_GENERIC_M2354_KPQC.bin`)입니다.
- **유저 펌웨어**: Non-Secure 영역의 애플리케이션 코드가 실행되는 영역입니다.
- **RTOS**: FreeRTOS를 사용합니다.
- **암호 모듈**: AxioCrypto를 사용합니다.
- **MCU** : Nuvoton M2354

## 목차

- [개발환경](01.development-environment.md)
- [환경설정](02.setup.md)
- [빌드](03.build.md)
- [펌웨어 다운로드](04.firmware-download.md)
- [실행화면](05.runtime.md)
- [예제](06.examples.md)
- [모듈 설명](07.modules.md)
- [메모리 레이아웃](08.memory-layout.md)
- [프로젝트 구조](09.project-structure.md)
