<p align="right">
  <a href="README.md">English</a> | <a href="README.ko.md">한국어</a>
</p>

# MicroPQC

MicroPQC는 Arm TrustZone 기반 보안 솔루션 **AxioMicroOS**의 **Secure 영역**에서  
양자내성암호(**PQC**)와 한국형 양자내성암호(**KPQC**)를 구현하고,  
**Nuvoton M2354** 환경에서 **KAT(정합성 검증)** 및 **성능 벤치마크**를 수행한 프로젝트입니다.

- 대상: **WSL(리눅스) + Arm GNU Toolchain 크로스컴파일**
- [제품/플랫폼 상세 문서 바로가기](doc/main.md)

---

## 환경설정

### 필수 요구사항
- WSL2(Ubuntu 등)
- `make`
- Arm GNU Toolchain (arm-none-eabi)

### Toolchain 경로 설정
크로스컴파일러 경로는 아래 파일에서 설정합니다.

- `scripts/ARM_EABI_TOOLCHAIN.mk`

예시:
```makefile
TOOLCHAIN_PREFIX = /opt/arm-gnu-toolchain-*/bin/arm-none-eabi-
```

자세한 내용은 [개발환경](doc/02.setup.md) 문서를 참고하세요.

---

## 빌드

```bash
make -C applications/helloworld
```

클린:
```bash
make -C applications/helloworld clean
```

빌드 산출물은 `applications/helloworld/bin/` 하위에 생성됩니다.

자세한 내용은 [빌드](doc/03.build.md) 문서를 참고하세요.

---

## 모듈 적용

1. 보드에 펌웨어를 다운로드(프로그램)합니다.  
   - Secure 펌웨어(부트/시큐어)는 `secure/` 하위 제공 바이너리를 사용
   - Non-Secure 펌웨어는 위 빌드 결과물을 사용
2. UART로 콘솔에 접속합니다(보드 설정에 따라 포트/baudrate는 상이할 수 있음).
3. 부팅 후 쉘에서 `help`로 명령 목록을 확인합니다.

자세한 내용은 [펌웨어 다운로드](doc/04.firmware-download.md) 문서를 참고하세요.

---

## PQC 테스트 방법

UART 쉘에서 KAT(Known Answer Test)를 실행합니다.

```text
pqc kat all
```

UART 쉘에서 Benchmark를 실행합니다.

```text
pqc bench all
```

KAT, Benchmark에 대한 자세한 내용은 [예제](doc/06.examples.md) 문서를 참고하세요.

## 성능 측정 결과

### Signature (전자서명)

| 알고리즘 | 기능 | Stack/RAM (KB) | 소요시간 (ms) | 개인키 (bytes) | 공개키 (bytes) | 전자서명 (bytes) |
|---|---|---:|---:|---:|---:|---:|
| AIMer128F | 키생성<br>전자서명 생성<br>전자서명 검증 | 8<br>13<br>15 | 28<br>1,817<br>1,150 | 48 | 32 | 5,888 |
| HAETAE2 | 키생성<br>전자서명 생성<br>전자서명 검증 | 23<br>81<br>29 | 312<br>3,066<br>47 | 1,408 | 992 | 1,474 |
| ML-DSA-44 (Dilithium2) | 키생성<br>전자서명 생성<br>전자서명 검증 | 37<br>51<br>36 | 71<br>334<br>80 | 2,560 | 1,312 | 2,420 |
| SLH-DSA-SHAKE128F (Spincs128F) | 키생성<br>전자서명 생성<br>전자서명 검증 | 3<br>3<br>3 | 3,138<br>72,736<br>4,583 | 64 | 32 | 17,088 |
| FN-DSA-512 (FALCON) | 키생성<br>전자서명 생성<br>전자서명 검증 | 17<br>42<br>4 | 9,807<br>2,245<br>13 | 1,281 | 897 | 690 |

### Key Exchange (KEM / 키교환)

| 알고리즘 | 기능 | Stack/RAM (KB) | 소요시간 (ms) | 개인키 (bytes) | 공개키 (bytes) | 암호문 (bytes) |
|---|---|---:|---:|---:|---:|---:|
| ML-KEM-512 (KYBER512) | 키생성<br>캡슐화<br>역캡슐화 | 6<br>8<br>9 | 23<br>24<br>19 | 1,632 | 800 | 768 |
| NTRU+768 | 키생성<br>캡슐화<br>역캡슐화 | 9<br>9<br>15 | 14<br>18<br>13 | 2,337 | 1,152 | 1,152 |
| SMAUG-T1 | 키생성<br>캡슐화<br>역캡슐화 | 10<br>12<br>13 | 39<br>38<br>44 | 832 | 672 | 672 |

---

## 권장 알고리즘

마이크로 컨트롤러 환경에서 실용적인 사용을 위해 메모리 효율과 실행 속도를 고려하여 다음 알고리즘을 권장합니다:

| 용도 | NIST PQC 권장 | Korea PQC 권장 | 선택 이유 |
|---|---|---|---|
| 전자서명 | ML-DSA-44 | AIMer128f | 낮은 메모리 사용량과 실용적인 처리 속도 |
| 키교환 | ML-KEM-512 | SMAUG-T1 | 최소 스택 메모리와 작은 키 크기 |

---

## 문서

- 시작점: `doc/` 하위 `.md` 문서
- 제품/플랫폼 구성, 다운로드(플래싱), 메모리 레이아웃, 모듈 설명 등은 문서를 기준으로 확인하세요.

---

## 라이선스

The codes and the specifications are under the MIT license.
