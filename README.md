<p align="right">
  <a href="README.md">English</a> | <a href="README.ko.md">한국어</a>
</p>

# MicroPQC

MicroPQC is a project that implements **Post-Quantum Cryptography (PQC)** and **Korean PQC (KPQC)** in the **Secure world** of **AxioMicroOS**, our Arm TrustZone–based security solution, and runs **KAT (correctness validation)** and **performance benchmarks** on **Nuvoton M2354**.

- Target: **WSL (Linux) + Arm GNU Toolchain cross-compilation**
- [Product / platform documentation](doc/en/main.md)

---

## Environment Setup

### Requirements
- WSL2 (Ubuntu, etc.)
- `make`
- Arm GNU Toolchain (`arm-none-eabi`)

### Toolchain path configuration
Set the cross-compiler path in:

- `scripts/ARM_EABI_TOOLCHAIN.mk`

Example:
```makefile
TOOLCHAIN_PREFIX = /opt/arm-gnu-toolchain-*/bin/arm-none-eabi-
```

For more details on Environment Setup, see: [setup.md](doc/en/02.setup.md)

---

## Build

```bash
make -C applications/helloworld
```

Clean:
```bash
make -C applications/helloworld clean
```

Build artifacts are generated under `applications/helloworld/bin/`

For more details on Build, see: [build.md](doc/en/03.build.md)

---

## Deployment / Integration

1. Program (flash) the firmware to the board.  
   - Secure firmware (boot/secure) uses the prebuilt binaries under `secure/`
   - Non-Secure firmware uses the build output from this repository
2. Connect to the UART console (port/baudrate may vary depending on board configuration).
3. After boot, run `help` in the shell to list available commands.

For more details on Deployment / Integration, see: [firmware-download.md](doc/en/04.firmware-download.md)

---

## How to Run PQC Tests

Run KAT(Known Answer Test) from the UART shell:

```text
pqc kat all
```

Run benchmarks from the UART shell:

```text
pqc bench all
```

For more details on KAT and benchmark, see: [examples.md](doc/en/06.examples.md)

## Performance Results

### Signature

| Algorithm | Operation | Stack/RAM (KB) | Time (ms) | Secret Key (bytes) | Public Key (bytes) | Signature (bytes) |
|---|---|---:|---:|---:|---:|---:|
| AIMer128F | KeyGen<br>Sign<br>Verify | 8<br>13<br>15 | 28<br>1,817<br>1,150 | 48 | 32 | 5,888 |
| HAETAE2 | KeyGen<br>Sign<br>Verify | 23<br>81<br>29 | 312<br>3,066<br>47 | 1,408 | 992 | 1,474 |
| ML-DSA-44 (Dilithium2) | KeyGen<br>Sign<br>Verify | 37<br>51<br>36 | 71<br>334<br>80 | 2,560 | 1,312 | 2,420 |
| SLH-DSA-SHAKE128F (Spincs128F) | KeyGen<br>Sign<br>Verify | 3<br>3<br>3 | 3,138<br>72,736<br>4,583 | 64 | 32 | 17,088 |
| FN-DSA-512 (FALCON) | KeyGen<br>Sign<br>Verify | 17<br>42<br>4 | 9,807<br>2,245<br>13 | 1,281 | 897 | 690 |

### Key Exchange (KEM)

| Algorithm | Operation | Stack/RAM (KB) | Time (ms) | Secret Key (bytes) | Public Key (bytes) | Ciphertext (bytes) |
|---|---|---:|---:|---:|---:|---:|
| ML-KEM-512 (KYBER512) | KeyGen<br>Encaps<br>Decaps | 6<br>8<br>9 | 23<br>24<br>19 | 1,632 | 800 | 768 |
| NTRU+768 | KeyGen<br>Encaps<br>Decaps | 9<br>9<br>15 | 14<br>18<br>13 | 2,337 | 1,152 | 1,152 |
| SMAUG-T1 | KeyGen<br>Encaps<br>Decaps | 10<br>12<br>13 | 39<br>38<br>44 | 832 | 672 | 672 |

---

## Recommended Algorithms

For practical use in microcontroller environments, we recommend the following algorithms considering memory efficiency and execution speed:

| Purpose | NIST PQC Recommended | Korea PQC Recommended | Reason |
|---|---|---|---|
| Digital Signature | ML-DSA-44 | AIMer128f | Low memory usage and practical processing speed |
| Key Exchange | ML-KEM-512 | SMAUG-T1 | Minimal stack memory and small key sizes |

---

## Documentation

- Entry point: `.md` files under `doc/en/`
- Refer to the docs for product/platform architecture, flashing steps, memory layout, and module descriptions.

---

## License

The codes and the specifications are under the MIT license.
