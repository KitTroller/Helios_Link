# HeliosLink Codebase Dissection

## A Complete Embedded-Engineering Analysis of the CC1101 Radio Link Firmware

**Target audience:** Electrical Engineering students with foundational knowledge of circuits, digital systems, and C programming.

**Hardware platform:** STM32F446RE Nucleo-64 board + Texas Instruments CC1101 sub-GHz radio transceiver module.

**Purpose of this document:** To walk through every source file in this project at a depth that rivals a university textbook chapter — explaining not just *what* the code does, but *why* each engineering decision was made, what the hardware is doing at the electrical level, and how all the pieces connect from power-on reset to a packet appearing on a serial terminal.

---

## Table of Contents

1. [Project Architecture Overview](#1-project-architecture-overview)
2. [The Hardware Platform](#2-the-hardware-platform)
3. [Boot Sequence: From Power-On to main()](#3-boot-sequence-from-power-on-to-main)
4. [The Linker Script: Memory Layout of a Bare-Metal System](#4-the-linker-script-memory-layout-of-a-bare-metal-system)
5. [The Startup Assembly File: First Instructions After Reset](#5-the-startup-assembly-file-first-instructions-after-reset)
6. [The Clock Tree: SystemClock_Config() Explained](#6-the-clock-tree-systemclock_config-explained)
7. [GPIO Configuration: Electrical Pin Control](#7-gpio-configuration-electrical-pin-control)
8. [SPI: The Communication Bus to the CC1101](#8-spi-the-communication-bus-to-the-cc1101)
9. [UART: The Debug Console to Your Mac](#9-uart-the-debug-console-to-your-mac)
10. [The HAL MSP Layer: Hardware-Software Binding](#10-the-hal-msp-layer-hardware-software-binding)
11. [Interrupt Architecture: stm32f4xx_it.c](#11-interrupt-architecture-stm32f4xx_itc)
12. [CC1101 Register Map: The Radio's Control Surface](#12-cc1101-register-map-the-radios-control-surface)
13. [CC1101 Driver Header: The Public API Contract](#13-cc1101-driver-header-the-public-api-contract)
14. [CC1101 Driver Implementation: The Radio Comes Alive](#14-cc1101-driver-implementation-the-radio-comes-alive)
15. [main.c: Application Logic — Transmit and Receive](#15-mainc-application-logic--transmit-and-receive)
16. [RF Engineering Fundamentals Used in This Code](#16-rf-engineering-fundamentals-used-in-this-code)
17. [The Build System and Debug Artefacts](#17-the-build-system-and-debug-artefacts)
18. [Common Failure Modes and Debugging Guide](#18-common-failure-modes-and-debugging-guide)
19. [Glossary of Key Terms](#19-glossary-of-key-terms)

---

## 1. Project Architecture Overview

### 1.1 What This Project Is

HeliosLink is a point-to-point radio telemetry link operating at 868 MHz in the European ISM band. Two identical STM32F446RE Nucleo development boards — one acting as a satellite transmitter, the other as a ground-station receiver — communicate wirelessly via CC1101 radio transceiver modules. The system measures Packet Error Rate (PER), logs Received Signal Strength Indication (RSSI), and sends structured telemetry packets inspired by the CCSDS (Consultative Committee for Space Data Systems) standard used in real satellite missions.

### 1.2 Software Architecture — Layered Abstraction

The firmware is structured in distinct layers. Understanding this layering is fundamental to embedded engineering:

```
┌───────────────────────────────────────────────────┐
│                 APPLICATION LAYER                  │
│              main.c (your code)                   │
│  Builds telemetry packets, controls TX/RX loop,   │
│  calculates PER, formats UART output              │
├───────────────────────────────────────────────────┤
│                  DRIVER LAYER                      │
│         cc1101.c / cc1101.h / cc1101_registers.h  │
│  Abstracts the radio hardware: init, send packet, │
│  receive packet, register read/write              │
├───────────────────────────────────────────────────┤
│        HARDWARE ABSTRACTION LAYER (HAL)            │
│              STM32F4xx HAL Library                 │
│  HAL_SPI_Transmit, HAL_GPIO_WritePin,             │
│  HAL_UART_Transmit, HAL_Delay, etc.              │
├───────────────────────────────────────────────────┤
│                 CMSIS LAYER                         │
│   ARM Cortex-M4 core definitions, NVIC, SysTick   │
├───────────────────────────────────────────────────┤
│               PHYSICAL HARDWARE                    │
│  STM32F446RE silicon, SPI2, USART2, GPIO ports,   │
│  CC1101 radio chip, antenna, crystal oscillators   │
└───────────────────────────────────────────────────┘
```

**Key principle:** Each layer knows only about the layer directly below it. `main.c` never calls `HAL_SPI_Transmit()` — it calls `CC1101_SendPacket()`, which internally uses HAL SPI functions. This encapsulation means you can change the SPI wiring or even replace the radio chip without modifying `main.c`.

### 1.3 File Map

| File | Role | You edit it? |
|------|------|--------------|
| `Core/Src/main.c` | Application entry point, TX/RX loops | Yes |
| `Core/Inc/main.h` | Pin definitions, HAL includes | CubeMX generates it |
| `Core/Src/cc1101.c` | CC1101 radio driver implementation | Yes |
| `Core/Inc/cc1101.h` | CC1101 driver public API | Yes |
| `Core/Inc/cc1101_registers.h` | CC1101 register address definitions | Yes (lookup table) |
| `Core/Src/stm32f4xx_hal_msp.c` | MCU Support Package — GPIO/clock init for peripherals | CubeMX generates it |
| `Core/Src/stm32f4xx_it.c` | Interrupt Service Routines | Mostly CubeMX |
| `Core/Inc/stm32f4xx_hal_conf.h` | HAL module enable/disable switches | CubeMX generates it |
| `Core/Inc/stm32f4xx_it.h` | ISR prototypes | CubeMX generates it |
| `Core/Src/system_stm32f4xx.c` | System clock initialisation (pre-main) | Do not edit |
| `Core/Src/syscalls.c` | C library system call stubs | Do not edit |
| `Core/Src/sysmem.c` | Heap memory management (`_sbrk`) | Do not edit |
| `Core/Startup/startup_stm32f446retx.s` | ARM assembly — vector table, Reset_Handler | Do not edit |
| `STM32F446RETX_FLASH.ld` | GNU linker script — memory layout | Rarely |
| `GroundStation_Node.ioc` | CubeMX project file — pin/clock configuration | Via CubeMX GUI |

---

## 2. The Hardware Platform

### 2.1 The STM32F446RE Microcontroller

The STM32F446RETx is an ARM Cortex-M4 microcontroller manufactured by STMicroelectronics. Here are the specifications that matter for this project:

| Parameter | Value | Significance |
|-----------|-------|--------------|
| CPU core | ARM Cortex-M4F | 32-bit RISC, hardware FPU, Thumb-2 ISA |
| Max clock | 180 MHz | We run at 84 MHz (voltage scale 3, conservative) |
| Flash | 512 KB | Non-volatile. Your compiled code lives here |
| SRAM | 128 KB | Volatile. Variables, stack, heap at runtime |
| SPI peripherals | 4 | We use SPI2 to talk to the CC1101 |
| USART peripherals | 4 | We use USART2 for the serial debug console |
| GPIO ports | A through H | We use pins on A, B, and C |
| Package | LQFP-64 | 64 pins, 0.5 mm pitch surface-mount |
| Supply voltage | 1.7–3.6 V | Nucleo board regulates to 3.3 V |

**The Nucleo-64 board** adds:
- An ST-Link/V2-1 debug probe (the small top section of the board) connected via USB to your Mac. This simultaneously provides power, programming (SWD), and a virtual COM port (USART2 routed through the ST-Link).
- A green LED on PA5 (LD2).
- A blue push-button on PC13 (active low — pressed = logic 0).
- Arduino-compatible pin headers for connecting external modules.

### 2.2 The CC1101 Radio Transceiver

The CC1101 is a low-power sub-1 GHz radio transceiver IC designed by Texas Instruments. It is not a microcontroller — it has no CPU, no programmable memory. It is a pure RF front-end that you configure and command entirely over SPI.

| Parameter | Value | Significance |
|-----------|-------|--------------|
| Frequency range | 300–348 MHz, 387–464 MHz, 779–928 MHz | We use 868.35 MHz (EU ISM) |
| Modulation | 2-FSK, GFSK, MSK, OOK, ASK | We use GFSK |
| Max data rate | 500 kbps | We use 1.2 kbps for maximum range |
| Sensitivity | −112 dBm at 1.2 kbps | This determines your maximum link distance |
| Max TX power | +12 dBm (~16 mW) | EU limit is 25 mW ERP at 868 MHz |
| Supply voltage | 1.8–3.6 V | **3.3 V only. 5 V will destroy it** |
| SPI speed | Up to 10 MHz | We run at ~2.6 MHz |
| TX/RX FIFO | 64 bytes each | Limits max packet size |
| Crystal | 26 MHz | Reference for all frequency synthesis |

### 2.3 Physical Wiring

```
STM32F446RE (Nucleo)          CC1101 Module
    PB10 (SPI2_SCK)  ────────── SCK
    PC1  (SPI2_MOSI) ────────── MOSI (SI)
    PC2  (SPI2_MISO) ────────── MISO (SO)
    PB12 (GPIO out)   ────────── CSn (Chip Select)
    3.3V              ────────── VCC
    GND               ────────── GND
```

**Why PB12 for CS instead of hardware NSS?** The STM32's hardware NSS (slave select) toggles automatically per byte, but the CC1101 requires CS to stay low for the entire multi-byte SPI transaction. Using a GPIO pin gives us manual control. This is called "software-managed NSS" or "bit-banged chip select," and it is the standard practice for SPI peripherals that need multi-byte transactions.

**Why PC2 for MISO (not the default PB14)?** The STM32F446RE has an alternate function multiplexer — each SPI signal can be routed to several pins. The CubeMX `.ioc` file shows PC2 was selected for SPI2_MISO (using alternate function AF5). This is a design choice, perhaps driven by physical routing convenience on the Nucleo board's Arduino headers.

---

## 3. Boot Sequence: From Power-On to main()

When you plug in the USB cable or press the reset button, the processor executes the following sequence. Understanding this is essential — it explains why your global variables start at zero, how the stack works, and what happens before a single line of your C code runs.

```
Power-on / Reset button
    │
    ▼
┌──────────────────────────────────────────────┐
│ 1. CPU reads address 0x08000000 (FLASH base) │
│    This is the initial Stack Pointer value    │
│    → loaded into SP register                  │
│                                               │
│ 2. CPU reads address 0x08000004              │
│    This is the Reset_Handler address          │
│    → loaded into PC register                  │
│    → execution begins here                    │
└──────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────┐
│ Reset_Handler (startup_stm32f446retx.s)      │
│                                               │
│ 3. Set stack pointer: ldr sp, =_estack       │
│    (_estack = 0x20000000 + 128K = 0x20020000)│
│                                               │
│ 4. Call SystemInit() — minimal clock setup    │
│                                               │
│ 5. Copy .data section from FLASH → RAM       │
│    (initialized global variables)             │
│                                               │
│ 6. Zero-fill .bss section in RAM             │
│    (uninitialized globals → 0)               │
│                                               │
│ 7. Call __libc_init_array()                  │
│    (C++ constructors, C init functions)       │
│                                               │
│ 8. Call main()                                │
└──────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────┐
│ main() — your application code               │
│                                               │
│ 9.  HAL_Init() — SysTick, NVIC, caches      │
│ 10. SystemClock_Config() — PLL to 84 MHz     │
│ 11. MX_GPIO_Init() — configure pins          │
│ 12. MX_USART2_UART_Init() — serial console   │
│ 13. MX_SPI2_Init() — SPI bus to CC1101       │
│ 14. CC1101_Init() — radio configuration      │
│ 15. CC1101_Verify() — check radio responds   │
│ 16. Enter infinite TX or RX loop             │
└──────────────────────────────────────────────┘
```

**Why is this important?** In a desktop application, the OS sets up your process's memory, loads shared libraries, and calls `main()`. On bare metal, there is no OS. Steps 3–8 are the equivalent of what Linux or Windows does before your program starts. If anything goes wrong here (corrupt linker script, wrong memory addresses), your firmware will hard-fault before reaching `main()`.

---

## 4. The Linker Script: Memory Layout of a Bare-Metal System

**File:** `STM32F446RETX_FLASH.ld`

The linker script is one of the most misunderstood files in embedded development. It tells the GNU linker (`ld`) exactly where in the microcontroller's address space to place each category of data. There is no virtual memory, no MMU, no paging. Every address is a physical, electrical address on the silicon die.

### 4.1 Memory Regions

```ld
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH  (rx)     : ORIGIN = 0x8000000,    LENGTH = 512K
}
```

| Region | Start Address | Size | Attributes | Physical Nature |
|--------|--------------|------|------------|-----------------|
| FLASH | `0x08000000` | 512 KB | `rx` (read, execute) | Non-volatile NOR flash. Survives power cycles. ~100,000 erase cycles. ~20 ns read time. |
| RAM | `0x20000000` | 128 KB | `xrw` (execute, read, write) | Volatile SRAM. Loses contents on power loss. ~1 ns access time. |

These addresses are defined by the STM32F446RE's memory map in the Reference Manual (RM0390). They are hardwired into the silicon and cannot be changed.

### 4.2 Section Placement

The `SECTIONS` block defines what goes where:

**In FLASH (persistent, read-only at runtime):**

| Section | Content | Why in FLASH? |
|---------|---------|---------------|
| `.isr_vector` | Interrupt vector table (97 entries) | CPU reads this on boot/interrupt — must be at a fixed address |
| `.text` | Machine code (compiled functions) | Code doesn't change at runtime |
| `.rodata` | Constants, string literals (`"CC1101 detected OK"`) | Read-only data |
| `.ARM.extab` / `.ARM.exidx` | C++ exception handling tables | Required by the ABI even if not using exceptions |
| `.init_array` / `.fini_array` | Constructors/destructors | Called by startup code |

**In RAM (volatile, read-write):**

| Section | Content | Initialisation |
|---------|---------|----------------|
| `.data` | Initialised global/static variables (e.g., `uint32_t x = 42;`) | Startup code copies initial values from FLASH to RAM |
| `.bss` | Uninitialised global/static variables (e.g., `uint32_t y;`) | Startup code fills with zeros |
| Stack | Function call frames, local variables | Grows downward from `_estack` (top of RAM) |
| Heap | Dynamic allocation (`malloc`) | Grows upward from end of `.bss` |

### 4.3 The .data Copy Trick

```ld
_sidata = LOADADDR(.data);

.data :
{
    _sdata = .;
    *(.data) *(.data*)
    _edata = .;
} >RAM AT> FLASH
```

This is a critical linker feature. The `>RAM AT> FLASH` directive means: "At runtime, this section lives in RAM (starting at `_sdata` = `0x20000000`). But in the `.elf` file, store the initial values in FLASH (starting at `_sidata`)."

The startup code then copies from `_sidata` to `_sdata..._edata`:

```assembly
CopyDataInit:
  ldr r4, [r2, r3]    ; Read from FLASH (source)
  str r4, [r0, r3]    ; Write to RAM (destination)
  adds r3, r3, #4     ; Next word
```

**Without this, any global variable you initialise to a non-zero value would read as garbage after power-on.** The C standard guarantees that `int x = 42;` starts at 42, and this mechanism is how bare-metal systems deliver that guarantee.

### 4.4 Stack and Heap

```ld
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* 0x20020000 */
_Min_Heap_Size = 0x200;                  /* 512 bytes */
_Min_Stack_Size = 0x400;                 /* 1024 bytes */
```

The stack starts at the top of RAM (`0x20020000`) and grows downward. The heap starts after `.bss` and grows upward. The `._user_heap_stack` section is a linker check — if `.bss` + heap + stack exceed 128 KB, the link fails with an error. This is your compile-time guard against stack overflow.

```
RAM layout (128 KB: 0x20000000 – 0x2001FFFF):

0x20020000  ┌─────────────┐ ← _estack (initial SP)
            │   STACK      │   ↓ grows downward
            │   (1 KB min) │
            ├─────────────┤
            │   (free)     │
            ├─────────────┤
            │   HEAP       │   ↑ grows upward
            │   (512B min) │
            ├─────────────┤ ← _ebss
            │   .bss       │   Zero-initialised globals
            ├─────────────┤ ← _sbss
            │   .data      │   Initialised globals (copied from FLASH)
0x20000000  └─────────────┘ ← _sdata
```

---

## 5. The Startup Assembly File: First Instructions After Reset

**File:** `Core/Startup/startup_stm32f446retx.s`

This file is written in ARM Thumb assembly. It runs before any C code and does three things:

### 5.1 The Vector Table

```assembly
.section .isr_vector,"a",%progbits

g_pfnVectors:
  .word  _estack           ; Initial stack pointer value
  .word  Reset_Handler     ; Reset vector — entry point
  .word  NMI_Handler       ; Non-Maskable Interrupt
  .word  HardFault_Handler ; Hard fault
  .word  MemManage_Handler ; Memory management fault
  ...
  .word  SPI2_IRQHandler   ; SPI2 global interrupt
  .word  USART2_IRQHandler ; USART2 global interrupt
  ...
```

The vector table is an array of 32-bit words placed at the very start of FLASH (`0x08000000`). The ARM Cortex-M architecture defines this layout:

- **Word 0** (`0x08000000`): Initial Main Stack Pointer (MSP) value. The CPU loads this into SP before executing any code.
- **Word 1** (`0x08000004`): Address of `Reset_Handler`. The CPU loads this into PC and starts executing.
- **Words 2–15**: ARM-defined exception handlers (NMI, faults, SVC, PendSV, SysTick).
- **Words 16+**: Vendor-specific peripheral interrupt handlers (SPI, UART, DMA, timers, etc.).

Each `.word` is a function pointer. When an interrupt fires, the CPU hardware:
1. Pushes registers (R0–R3, R12, LR, PC, xPSR) onto the stack.
2. Reads the corresponding vector table entry.
3. Jumps to that address.
4. On return, pops the registers and resumes the interrupted code.

**Weak aliases:** At the bottom of the file, each handler is declared as a `.weak` alias for `Default_Handler` (an infinite loop). This means: if you don't write a handler for SPI2, the linker uses `Default_Handler`. If you do define `SPI2_IRQHandler` in your C code, the linker uses yours instead. This is a compile-time override mechanism.

### 5.2 Reset_Handler — The True Entry Point

```assembly
Reset_Handler:
  ldr   sp, =_estack          ; (1) Set stack pointer
  bl    SystemInit             ; (2) Minimal clock init (FPU enable, vector table offset)
  ; Copy .data from FLASH to RAM
  ldr r0, =_sdata             ; (3) RAM destination start
  ldr r1, =_edata             ;     RAM destination end
  ldr r2, =_sidata            ;     FLASH source start
  movs r3, #0                 ;     Offset = 0
  b LoopCopyDataInit

LoopCopyDataInit:
  adds r4, r0, r3             ; Current RAM address = _sdata + offset
  cmp r4, r1                  ; Reached _edata?
  bcc CopyDataInit            ; No → copy next word

  ; Zero-fill .bss
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss             ; Store 0, advance pointer

  bl __libc_init_array        ; (4) C runtime init (constructors)
  bl main                     ; (5) YOUR CODE STARTS HERE
  bx lr                       ; (6) If main() returns — should never happen
```

**Step by step:**
1. **Stack pointer setup:** `_estack` = `0x20020000`. The stack grows downward, so the first push will write to `0x2001FFFC`.
2. **SystemInit:** Defined in `system_stm32f4xx.c`. Enables the FPU, sets the vector table offset register (VTOR) so the CPU knows where the vector table is.
3. **Data copy loop:** Iterates through every initialised global variable and copies its initial value from FLASH to its runtime RAM address. Uses 32-bit word-aligned copies.
4. **BSS zeroing loop:** Every uninitialised global variable gets set to zero. This is why `uint32_t x;` at file scope starts at 0 in C — it's not magic, it's this loop.
5. **Call main():** Finally, your code runs.
6. **If main returns:** `bx lr` would return to whatever called Reset_Handler — but nothing did. In practice, `main()` contains `while(1){}` and never returns.

---

## 6. The Clock Tree: SystemClock_Config() Explained

**File:** `main.c`, lines 315–338 (Phase 2 section) / 541–582 (CubeMX section)

The clock tree is perhaps the single most important piece of configuration on any microcontroller. Every peripheral's timing — SPI bit rate, UART baud rate, timer resolution, ADC sampling rate — derives from this tree. Get it wrong and nothing works.

### 6.1 The STM32F446RE Clock Architecture

```
                                    ┌──────────┐
HSI (16 MHz internal RC) ──────────┤          │     ┌─────────┐
                                    │   PLL    │────→│ SYSCLK  │── 84 MHz
HSE (8 MHz external xtal) ─────────┤ (×336÷4) │     │         │
                                    └──────────┘     └────┬────┘
                                                          │
                                    ┌─────────────────────┼───────────┐
                                    │                     │           │
                                    ▼                     ▼           ▼
                              ┌──────────┐         ┌──────────┐ ┌──────────┐
                              │ AHB ÷1   │         │ APB1 ÷2  │ │ APB2 ÷1  │
                              │ = 84 MHz │         │ = 42 MHz │ │ = 84 MHz │
                              └──────────┘         └──────────┘ └──────────┘
                                    │                     │           │
                              FLASH, SRAM,          SPI2, USART2,   SPI1,
                              DMA, GPIO             I2C, TIM2-7     USART1,
                              core                                   TIM1,8
```

### 6.2 PLL Configuration — The Maths

The code configures the Phase-Locked Loop to multiply the internal 16 MHz oscillator up to 84 MHz:

```c
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;  // Input = 16 MHz
RCC_OscInitStruct.PLL.PLLM = 16;   // VCO input = 16 MHz ÷ 16 = 1 MHz
RCC_OscInitStruct.PLL.PLLN = 336;  // VCO output = 1 MHz × 336 = 336 MHz
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;  // SYSCLK = 336 MHz ÷ 4 = 84 MHz
```

**The PLL formula:**

```
f_SYSCLK = (f_HSI / PLLM) × PLLN / PLLP
         = (16 MHz / 16) × 336 / 4
         = 1 MHz × 336 / 4
         = 84 MHz
```

**Constraints (from the datasheet):**
- VCO input (after PLLM): must be 1–2 MHz. We get 1 MHz. ✓
- VCO output (after PLLN): must be 100–432 MHz. We get 336 MHz. ✓
- SYSCLK (after PLLP): must not exceed 180 MHz (or 84 MHz for voltage scale 3). We get 84 MHz. ✓

### 6.3 Bus Dividers and Peripheral Clocks

```c
RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   // HCLK = 84 MHz
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      // PCLK1 = 42 MHz
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      // PCLK2 = 84 MHz
```

**Why this matters for SPI:** SPI2 sits on the APB1 bus. Its input clock is **42 MHz**. The SPI prescaler is set to 16:

```
f_SPI2 = 42 MHz ÷ 16 = 2.625 MHz
```

The CC1101 supports up to 10 MHz SPI, so 2.625 MHz is conservative and reliable. Lower SPI speeds are more tolerant of long jumper wires, noisy breadboards, and signal integrity issues.

**Why this matters for UART:** USART2 also sits on APB1. Its input clock is 42 MHz. The baud rate generator divides this to produce 115200 bps. The HAL calculates the USARTDIV register automatically.

### 6.4 Flash Latency

```c
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
```

NOR flash memory has a read access time of roughly 30 ns. At 84 MHz, one clock cycle is 11.9 ns. The CPU needs data in one cycle, but flash can't deliver it that fast. The solution: **wait states**. `FLASH_LATENCY_2` means the CPU inserts 2 wait states (3 cycles total) for each flash read. The STM32 also uses prefetch buffers and instruction/data caches to hide this latency for sequential code execution.

| SYSCLK | Supply Voltage | Required Wait States |
|--------|---------------|---------------------|
| ≤ 30 MHz | 2.7–3.6 V | 0 |
| ≤ 60 MHz | 2.7–3.6 V | 1 |
| ≤ 84 MHz | 2.7–3.6 V | 2 |
| ≤ 180 MHz | 2.7–3.6 V | 5 |

### 6.5 Voltage Scaling

```c
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
```

The STM32F446 has an internal voltage regulator with three scale modes. Scale 3 is the lowest performance but lowest power. It limits the maximum clock to 84 MHz. For this project, 84 MHz is more than enough — the bottleneck is the 1.2 kbps radio link, not the CPU.

---

## 7. GPIO Configuration: Electrical Pin Control

**File:** `main.c` → `MX_GPIO_Init()`

### 7.1 How GPIO Works Electrically

Each GPIO pin on the STM32 connects to a configurable output driver and input buffer:

```
                    VDD (3.3V)
                      │
                    ┌─┤ P-MOSFET (pull-up, optional)
                    │ │
    Pad ────────────┼─┤──── Output data register bit
    (physical pin)  │ │
                    │ ┌─┤ N-MOSFET (pull-down, optional)
                    │ │ │
                    │ │ GND
                    │ │
                    │ └── To input data register (Schmitt trigger)
                    │
                    └── To alternate function mux (SPI, UART, etc.)
```

### 7.2 Pin Configurations in This Project

**PB12 — CC1101 Chip Select (CS):**
```c
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // Start HIGH (deselected)
GPIO_InitStruct.Pin   = GPIO_PIN_12;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  // Push-pull output
GPIO_InitStruct.Pull  = GPIO_NOPULL;          // No internal pull resistor
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // ~2 MHz slew rate
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

- **Push-pull** (`GPIO_MODE_OUTPUT_PP`): The pin actively drives both high (via P-MOSFET to VDD) and low (via N-MOSFET to GND). This gives clean, fast transitions.
- **No pull** (`GPIO_NOPULL`): No internal pull-up or pull-down resistor. Not needed because we always drive this pin actively.
- **Low speed** (`GPIO_SPEED_FREQ_LOW`): Controls the output driver's slew rate (how fast the voltage transition is). Lower slew rate = less EMI, less ringing on the wire. CS doesn't need to toggle at MHz speeds.
- **Initial state HIGH**: CS is active-low. We start with the CC1101 deselected.

**PA5 — Green LED (LD2):**
```c
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Start OFF
GPIO_InitStruct.Pin   = GPIO_PIN_5;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
```

Same configuration. Push-pull output, starts low (LED off). The Nucleo board has the LED connected between PA5 and ground through a current-limiting resistor, so driving PA5 high turns it on.

**PC13 — Blue Button (B1):**
```c
GPIO_InitStruct.Pin  = GPIO_PIN_13;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

- **`GPIO_MODE_IT_FALLING`**: Configures this pin as an input that triggers an interrupt on a falling edge (1→0 transition). The button connects PC13 to GND when pressed, so pressing it creates a falling edge.
- The interrupt is configured but not actually handled in this codebase (no `EXTI15_10_IRQHandler` implementation uses it). It's there from the CubeMX Nucleo template.

### 7.3 Clock Gating — Why __HAL_RCC_GPIOx_CLK_ENABLE()?

```c
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
```

On STM32 microcontrollers, every peripheral's clock is gated off by default at power-on. This is a power-saving measure. Writing to a peripheral's registers without first enabling its clock has no effect — the writes are silently dropped. You must enable the clock for each GPIO port you use.

This is a common embedded "gotcha." If your GPIO pin does nothing, the first thing to check is whether you called `__HAL_RCC_GPIOx_CLK_ENABLE()`.

---

## 8. SPI: The Communication Bus to the CC1101

**File:** `main.c` → `MX_SPI2_Init()`

### 8.1 What SPI Is

SPI (Serial Peripheral Interface) is a synchronous, full-duplex, master-slave serial bus. It uses four wires:

| Wire | Direction | Purpose |
|------|-----------|---------|
| SCK (Serial Clock) | Master → Slave | Master generates clock. Data is sampled on clock edges. |
| MOSI (Master Out Slave In) | Master → Slave | Data from STM32 to CC1101 |
| MISO (Master In Slave Out) | Slave → Master | Data from CC1101 to STM32 |
| CS (Chip Select) | Master → Slave | Active low. Selects which slave to talk to. |

**Key properties:**
- **Synchronous:** Data is clocked, not free-running like UART. No baud rate mismatch possible.
- **Full-duplex:** MOSI and MISO transfer simultaneously. Every byte sent on MOSI produces a byte on MISO (even if it's garbage).
- **No addressing:** Unlike I2C, SPI has no device address. You select the target by pulling its CS pin low.

### 8.2 SPI Modes: Clock Polarity and Phase

The CC1101 datasheet specifies: **CPOL = 0, CPHA = 0** (SPI Mode 0).

```c
hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;   // CPOL=0: clock idles LOW
hspi2.Init.CLKPhase    = SPI_PHASE_1EDGE;    // CPHA=0: sample on rising edge
```

This means:
- When no transfer is happening, SCK sits at 0 V.
- Data is sampled (read) on the **rising** edge of SCK.
- Data is shifted out (changed) on the **falling** edge of SCK.

```
     CS  ───┐                                      ┌───
             └──────────────────────────────────────┘
    SCK       ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐
          ────┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──
   MOSI   ──X═══D7══X═══D6══X═══D5══X═══D4══X═══D3══X═══D2══X═══D1══X═══D0══
             ↑       ↑       ↑       ↑       ↑       ↑       ↑       ↑
           sample  sample  sample  sample  sample  sample  sample  sample
```

Getting the SPI mode wrong is a classic embedded bug. The CC1101 would see shifted or inverted data, and every register read would return garbage. If `CC1101_Verify()` fails, SPI mode is one of the first things to check.

### 8.3 SPI Configuration Registers

```c
hspi2.Instance               = SPI2;                    // Which SPI peripheral
hspi2.Init.Mode              = SPI_MODE_MASTER;         // STM32 generates the clock
hspi2.Init.Direction         = SPI_DIRECTION_2LINES;    // Full duplex (MOSI + MISO)
hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;       // 8-bit transfers
hspi2.Init.NSS               = SPI_NSS_SOFT;            // We control CS via GPIO
hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;// 42 MHz ÷ 16 = 2.625 MHz
hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;        // CC1101 expects MSB first
hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;      // Standard SPI, not TI SSP
hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; // No SPI-level CRC
```

**Why prescaler 16?** Available prescalers are 2, 4, 8, 16, 32, 64, 128, 256. At prescaler 16: 42 MHz ÷ 16 = 2.625 MHz. This is well under the CC1101's 10 MHz limit, providing plenty of margin for signal integrity with jumper wires.

**Why MSB first?** The CC1101 datasheet specifies MSB-first bit order. The address byte's bit 7 (the most significant bit) is the R/W flag. If you configured LSB-first, bit 0 would arrive first, and the CC1101 would interpret the address byte completely wrong.

---

## 9. UART: The Debug Console to Your Mac

**File:** `main.c` → `MX_USART2_UART_Init()`

### 9.1 How UART Differs from SPI

UART (Universal Asynchronous Receiver-Transmitter) is fundamentally different from SPI:

| Property | SPI | UART |
|----------|-----|------|
| Clocking | Synchronous (master provides clock) | Asynchronous (both sides agree on baud rate) |
| Wires | 4 (SCK, MOSI, MISO, CS) | 2 (TX, RX) |
| Topology | Master + slaves | Peer-to-peer |
| Speed | Up to 42 MHz on STM32 | Typically 115200 bps |
| Error detection | None built-in | Parity bit (optional) |

### 9.2 UART Frame Format

At 115200 bps, 8N1 (8 data bits, no parity, 1 stop bit):

```
Idle ────┐    ┌──┬──┬──┬──┬──┬──┬──┬──┬──┐
(HIGH)    └────┤D0│D1│D2│D3│D4│D5│D6│D7│SP├──── Idle
          Start│  │  │  │  │  │  │  │  │  │     (HIGH)
           Bit │        8 Data Bits       │Stop
                                           Bit

Duration of each bit = 1/115200 = 8.68 µs
Total frame time = 10 bits × 8.68 µs = 86.8 µs per byte
```

### 9.3 Configuration

```c
huart2.Init.BaudRate     = 115200;                // Bits per second
huart2.Init.WordLength   = UART_WORDLENGTH_8B;    // 8 data bits
huart2.Init.StopBits     = UART_STOPBITS_1;       // 1 stop bit
huart2.Init.Parity       = UART_PARITY_NONE;      // No parity
huart2.Init.Mode         = UART_MODE_TX_RX;       // Both directions
huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;   // No RTS/CTS
huart2.Init.OverSampling = UART_OVERSAMPLING_16;   // 16x oversampling
```

**Oversampling:** The UART receiver samples the incoming signal 16 times per bit period. The middle samples are used to determine the bit value. This provides noise immunity — a single glitch in one sample doesn't cause a bit error.

### 9.4 printf() Redirect

```c
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
```

On a desktop system, `printf()` writes to stdout, which the OS routes to your terminal. On bare metal, there is no stdout. The C library's `printf()` eventually calls `_write()`, a function you must implement. This implementation redirects all `printf()` output to USART2, which flows through the ST-Link's virtual COM port to your Mac.

`HAL_MAX_DELAY` means the function blocks until all bytes are sent. At 115200 bps, a 100-character string takes about 8.7 ms. This is blocking — the CPU does nothing else during this time. For high-throughput applications, you would use DMA-based UART transmission.

### 9.5 Connecting to the Serial Console

The Nucleo's ST-Link creates a virtual COM port over USB. On macOS:

```bash
screen /dev/tty.usbmodem* 115200
```

Or use STM32CubeIDE's built-in Serial Monitor.

---

## 10. The HAL MSP Layer: Hardware-Software Binding

**File:** `Core/Src/stm32f4xx_hal_msp.c`

"MSP" stands for **MCU Support Package**. This file is the glue between the HAL's hardware-agnostic peripheral drivers and the specific pin/clock configuration of your board.

### 10.1 Why This Layer Exists

The HAL SPI driver knows how to configure the SPI2 peripheral registers, generate clock signals, and transfer data. But it does *not* know which GPIO pins SPI2 is connected to — that depends on your board design. The MSP callbacks provide this information.

When you call `HAL_SPI_Init()`, internally it calls `HAL_SPI_MspInit()`, which you define in this file:

```c
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
    if(hspi->Instance == SPI2) {
        __HAL_RCC_SPI2_CLK_ENABLE();     // Enable the SPI2 peripheral clock
        __HAL_RCC_GPIOC_CLK_ENABLE();    // Enable GPIOC clock (for MOSI, MISO)
        __HAL_RCC_GPIOB_CLK_ENABLE();    // Enable GPIOB clock (for SCK)

        // Configure PC1 as SPI2_MOSI (alternate function 7)
        GPIO_InitStruct.Pin       = GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;          // Alternate function, push-pull
        GPIO_InitStruct.Alternate = GPIO_AF7_SPI2;             // Mux select: SPI2
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        // Configure PC2 as SPI2_MISO (alternate function 5)
        GPIO_InitStruct.Pin       = GPIO_PIN_2;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;             // Note: different AF number!
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        // Configure PB10 as SPI2_SCK (alternate function 5)
        GPIO_InitStruct.Pin       = GPIO_PIN_10;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}
```

### 10.2 Alternate Functions — The Pin Mux

Each STM32 GPIO pin can serve multiple purposes. The "alternate function" (AF) selector determines which internal peripheral drives the pin. For example, PC1 can be:
- AF5: SPI2_MOSI (not used here)
- AF7: SPI2_MOSI (used here, via a different mux path)
- AF8: UART5_TX
- And others

The correct AF number comes from the STM32F446 datasheet, Table 12 ("Alternate function mapping"). Getting this wrong means SPI2 is configured correctly in its registers but the signals never reach the physical pins.

**Note the asymmetry:** PC1 (MOSI) uses AF7, while PC2 (MISO) and PB10 (SCK) use AF5. This is not an error — different pins have different alternate function assignments even for the same peripheral. This is one of the most confusing aspects of STM32 development, and CubeMX exists largely to handle this complexity.

### 10.3 UART MSP

Similarly for USART2:

```c
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    if(huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // PA2 = USART2_TX, PA3 = USART2_RX, both AF7
        GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}
```

---

## 11. Interrupt Architecture: stm32f4xx_it.c

**File:** `Core/Src/stm32f4xx_it.c`

### 11.1 What Interrupts Are

An interrupt is a hardware signal that causes the CPU to stop executing the current code, save its state, and jump to a specific handler function. When the handler returns, the CPU restores its state and resumes where it left off.

On the Cortex-M4, interrupts are managed by the NVIC (Nested Vectored Interrupt Controller), which supports:
- 240 external interrupt lines
- 16 priority levels (with sub-priorities)
- Tail-chaining (fast back-to-back interrupt handling)
- Automatic stacking (hardware saves/restores registers)

### 11.2 Interrupts Used in This Project

The project uses only one active interrupt:

**SysTick (System Tick Timer):**
```c
void SysTick_Handler(void) {
    HAL_IncTick();
}
```

SysTick is a 24-bit down-counting timer built into the Cortex-M4 core (not an STM32 peripheral). HAL configures it to fire every 1 ms. Each time it fires, `HAL_IncTick()` increments a global counter. This counter is used by:
- `HAL_Delay(ms)` — busy-waits until the tick counter advances by the specified amount.
- `HAL_GetTick()` — returns the current tick count (milliseconds since boot).

Without SysTick, `HAL_Delay()` would hang forever and `HAL_GetTick()` would always return 0.

### 11.3 Fault Handlers

The fault handlers are "should-never-happen" safety nets:

```c
void HardFault_Handler(void) {
    while (1) {}  // Hang forever — connect a debugger to find out why
}
```

| Fault | Cause | Example |
|-------|-------|---------|
| HardFault | Escalated fault, or catch-all | Dereferencing NULL, executing from invalid address |
| MemManage | MPU violation | Writing to FLASH without unlocking it |
| BusFault | Invalid bus access | Reading from a non-existent peripheral address |
| UsageFault | Illegal instruction | Executing a Thumb instruction with bit 0 = 0 |

**Debugging tip:** When your firmware hangs and the LED doesn't blink, it's often trapped in one of these handlers. Connect a debugger, pause execution, and check which handler you're in. Then examine the stacked PC register to find which instruction caused the fault.

---

## 12. CC1101 Register Map: The Radio's Control Surface

**File:** `Core/Inc/cc1101_registers.h`

### 12.1 The CC1101 SPI Address Byte

Every SPI transaction with the CC1101 begins with an **address byte**. This single byte encodes three pieces of information:

```
Bit:    7     6     5  4  3  2  1  0
      ┌─────┬─────┬────────────────────┐
      │ R/W │Burst│    Address (6-bit)  │
      └─────┴─────┴────────────────────┘
```

| Bit 7 (R/W) | Bit 6 (Burst) | Meaning |
|-------------|--------------|---------|
| 0 | 0 | Write single config register |
| 0 | 1 | Write burst (multiple registers or TX FIFO) |
| 1 | 0 | Read single config register **OR** execute command strobe |
| 1 | 1 | Read burst (multiple registers) **OR** read status register |

The SPI flags defined in the code:
```c
#define CC1101_WRITE_SINGLE 0x00  // 0b00000000
#define CC1101_WRITE_BURST  0x40  // 0b01000000
#define CC1101_READ_SINGLE  0x80  // 0b10000000
#define CC1101_READ_BURST   0xC0  // 0b11000000
```

### 12.2 The Three Address Spaces

The CC1101 has three overlapping address spaces in the range 0x00–0x3F:

**1. Configuration Registers (0x00–0x2E, read/write):**

These are the registers you write during `CC1101_Init()` to configure the radio's operating parameters. Key groups:

| Address Range | Group | Purpose |
|--------------|-------|---------|
| 0x00–0x02 | IOCFG | GDO pin function select (not used in this project) |
| 0x04–0x05 | SYNC | Sync word (default 0xD391 — unique bit pattern the receiver scans for) |
| 0x06–0x08 | PKTCTRL/PKTLEN | Packet format: variable length, CRC enable, whitening |
| 0x0B–0x0C | FSCTRL | Intermediate frequency and frequency offset |
| 0x0D–0x0F | FREQ | Carrier frequency (24-bit word) |
| 0x10–0x14 | MDMCFG | Modem config: modulation, data rate, bandwidth, preamble |
| 0x15 | DEVIATN | FSK frequency deviation |
| 0x17–0x18 | MCSM | State machine: what happens after TX/RX |
| 0x19–0x1D | FOCCFG/BSCFG/AGC | Receiver tuning: offset compensation, bit sync, AGC |
| 0x21–0x22 | FREND | Front-end config: LNA/mixer for RX, PATABLE for TX |
| 0x23–0x26 | FSCAL | Frequency synthesiser calibration |
| 0x2C–0x2E | TEST | Internal test registers (TI-recommended magic values) |

**2. Command Strobes (0x30–0x3D, write only):**

These are not registers — they are single-byte commands. Writing to these addresses triggers immediate hardware actions:

| Address | Mnemonic | Action |
|---------|----------|--------|
| 0x30 | SRES | Software reset — restore all registers to defaults |
| 0x34 | SRX | Enter receive mode |
| 0x35 | STX | Enter transmit mode |
| 0x36 | SIDLE | Go to idle mode (stop TX/RX) |
| 0x3A | SFRX | Flush the receive FIFO |
| 0x3B | SFTX | Flush the transmit FIFO |
| 0x3D | SNOP | Do nothing — just return the status byte |

**3. Status Registers (0x30–0x3D, read only):**

These occupy the same address range as the command strobes. The CC1101 distinguishes between them using bit 6 (the Burst bit):

| Address | Mnemonic | Content |
|---------|----------|---------|
| 0x30 | PARTNUM | Always 0x00 for CC1101 |
| 0x31 | VERSION | Silicon revision — always 0x14 for CC1101 |
| 0x34 | RSSI | Raw received signal strength |
| 0x35 | MARCSTATE | Current state machine state |
| 0x3A | TXBYTES | Bytes in TX FIFO |
| 0x3B | RXBYTES | Bytes in RX FIFO |

### 12.3 The Critical Overlap: Why Two Read Functions?

This is the single most confusing aspect of the CC1101 SPI protocol and deserves special attention.

Addresses 0x30–0x3D serve **double duty**:
- **Write** to them → command strobe (e.g., writing to 0x35 triggers STX)
- **Read** from them → status register (e.g., reading 0x35 returns MARCSTATE)

But the CC1101 uses bit 6 to decide which you want:
- `READ_SINGLE` (bit 7 = 1, bit 6 = 0) → The chip sees bit 6 = 0 and interprets the access as a **command strobe**, not a read
- `READ_BURST` (bit 7 = 1, bit 6 = 1) → The chip sees bit 6 = 1 and interprets it as a **status register read**

**Concrete example with VERSION (0x31):**

```
CC1101_ReadReg(0x31)   →  sends 0x31 | 0x80 = 0xB1  →  bit 6 = 0
  → CC1101 sees: "Write to address 0x31? No, bit 7 = read. But bit 6 = 0, so...
     it's a command strobe at 0x31 = SFSTXON!"
  → The synthesiser fires up. You get garbage back. The radio is now in the wrong state.

CC1101_ReadStatus(0x31) →  sends 0x31 | 0xC0 = 0xF1  →  bit 6 = 1
  → CC1101 sees: "Read, burst, address 0x31 → status register read."
  → Returns 0x14 (the correct version number). No side effects.
```

This is why the driver has two separate read functions, and why using the wrong one can cause mysterious, intermittent failures.

---

## 13. CC1101 Driver Header: The Public API Contract

**File:** `Core/Inc/cc1101.h`

### 13.1 Return Codes

```c
typedef enum {
    CC1101_OK             = 0,
    CC1101_ERR_NO_CHIP    = 1,
    CC1101_ERR_TIMEOUT    = 2,
    CC1101_ERR_CRC        = 3,
    CC1101_ERR_OVERFLOW   = 4,
} CC1101_Status_t;
```

Every function that can fail returns this enum. The caller (main.c) checks the return value and handles each case. This is the embedded equivalent of exceptions — but explicit and zero-cost at runtime (no stack unwinding, no RTTI).

### 13.2 Packet Command Types

```c
typedef enum {
    CMD_PING        = 0x01,
    CMD_BEACON      = 0x02,
    CMD_TELEMETRY   = 0x03,
    CMD_ACK         = 0x04,
} CC1101_Cmd_t;
```

This is the application-layer protocol. Each packet carries a command byte that tells the receiver what the payload contains. This is analogous to:
- The EtherType field in Ethernet (0x0800 = IPv4, 0x0806 = ARP)
- The APID (Application Process Identifier) in CCSDS space packets

### 13.3 Packet Structure

```c
#define CC1101_MAX_PAYLOAD  58

typedef struct {
    uint8_t packet_id;
    CC1101_Cmd_t command;
    uint8_t payload[CC1101_MAX_PAYLOAD];
    uint8_t payload_len;
} CC1101_Packet_t;
```

This struct represents one logical packet. Note that it is **not** what gets written to the CC1101's FIFO. The driver adds a length byte and strips it on receive. The struct is a clean application-layer abstraction.

**Why 58 bytes max?** The CC1101 has a 64-byte FIFO. After accounting for:
- 1 byte: length byte (required by variable-length mode)
- 1 byte: packet_id
- 1 byte: command
- 2 bytes: CRC-16 (appended by hardware)

That leaves 64 − 1 − 1 − 1 − 2 = 59 bytes. The code uses 58 to provide a 1-byte safety margin, rounding to a cleaner number.

### 13.4 Packet Mode vs Direct Mode

The header comment explains an important architectural choice:

**Direct mode:** You clock raw bits in and out of the CC1101's GDO pins. You are responsible for preamble generation, sync word insertion, CRC calculation, timing — everything. The CC1101 is just a frequency modulator/demodulator.

**Packet mode (used here):** The CC1101 handles the entire lower MAC layer:
- Generates 4 bytes of 0xAA preamble (for receiver AGC lock and bit synchronisation)
- Inserts the sync word (0xD391 by default)
- Reads the length byte and transmits that many data bytes from the FIFO
- Calculates and appends a CRC-16
- On receive: scans for preamble + sync, reads length, fills RX FIFO, checks CRC, signals completion

This is analogous to using a UART peripheral vs bit-banging RS-232 with GPIO toggles. Packet mode offloads timing-critical work to dedicated hardware.

---

## 14. CC1101 Driver Implementation: The Radio Comes Alive

**File:** `Core/Src/cc1101.c`

This is the heart of the project. Every function is analysed below.

### 14.1 File-Scope Setup

```c
extern SPI_HandleTypeDef hspi2;

#define CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define MISO_PIN() HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
```

- **`extern`**: The SPI handle is created in `main.c`. This file needs access to it but doesn't own it. The `extern` keyword tells the compiler "this variable exists somewhere else; the linker will resolve it."
- **CS macros**: Wrapping GPIO calls in macros makes the driver code more readable. `CS_LOW()` means "begin talking to the CC1101." `CS_HIGH()` means "stop talking."
- **MISO_PIN()**: Used only during the reset sequence to detect when the CC1101's crystal oscillator has stabilised.

### 14.2 Low-Level Register Access

**Writing a configuration register:**
```c
void CC1101_WriteReg(uint8_t regAddr, uint8_t data) {
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &regAddr, 1, 100);  // Send address byte
    HAL_SPI_Transmit(&hspi2, &data, 1, 100);     // Send data byte
    CS_HIGH();
}
```

SPI timing on the wire:
```
CS  ───┐                                              ┌───
       └──────────────────────────────────────────────┘
SCK      │ 8 clocks for address │ 8 clocks for data │
MOSI     │ [R/W=0|B=0|addr]    │ [data byte]        │
MISO     │ (status byte)        │ (don't care)       │
```

The CC1101 simultaneously returns a status byte on MISO during the address byte transmission. This status byte contains the current state and FIFO occupancy. We discard it here (HAL_SPI_Transmit ignores MISO), but advanced implementations can use it for flow control.

**Reading a configuration register (0x00–0x2E):**
```c
uint8_t CC1101_ReadReg(uint8_t regAddr) {
    uint8_t received = 0;
    uint8_t read_addr = regAddr | CC1101_READ_SINGLE;  // Set bit 7
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &read_addr, 1, 100);     // Send address
    HAL_SPI_Receive(&hspi2, &received, 1, 100);        // Clock in response
    CS_HIGH();
    return received;
}
```

**Reading a status register (0x30–0x3D):**
```c
uint8_t CC1101_ReadStatus(uint8_t regAddr) {
    uint8_t received = 0;
    uint8_t read_addr = regAddr | CC1101_READ_BURST;   // Set bits 7 AND 6
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &read_addr, 1, 100);
    HAL_SPI_Receive(&hspi2, &received, 1, 100);
    CS_HIGH();
    return received;
}
```

The only difference is `READ_SINGLE` (0x80) vs `READ_BURST` (0xC0). This single bit prevents the catastrophic misinterpretation described in Section 12.3.

**Sending a command strobe:**
```c
void CC1101_SendCmd(uint8_t cmd) {
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);  // One byte, no data follows
    CS_HIGH();
}
```

Command strobes are the simplest SPI transactions: one byte, no data. The CC1101 executes the command immediately upon receiving the byte.

### 14.3 FIFO Access

**Burst-writing the TX FIFO:**
```c
static void CC1101_WriteFIFO(uint8_t *data, uint8_t len) {
    uint8_t addr = CC1101_FIFO | CC1101_WRITE_BURST;  // 0x3F | 0x40 = 0x7F
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &addr, 1, 100);          // Address: FIFO, burst write
    HAL_SPI_Transmit(&hspi2, data, len, 100);          // All data bytes in one burst
    CS_HIGH();
}
```

**Why burst mode?** The CC1101's FIFO is a special register that acts as a queue. In burst mode, each subsequent byte you clock in gets appended to the FIFO. In single-byte mode, only the first byte enters the FIFO — subsequent bytes would be interpreted as new address bytes.

Burst-read works identically for the RX FIFO:
```c
static void CC1101_ReadFIFO(uint8_t *buf, uint8_t len) {
    uint8_t addr = CC1101_FIFO | CC1101_READ_BURST;   // 0x3F | 0xC0 = 0xFF
    CS_LOW();
    HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
    HAL_SPI_Receive(&hspi2, buf, len, 100);
    CS_HIGH();
}
```

**Important hardware detail:** The FIFO address `0x3F` is shared between TX and RX. The CC1101 automatically routes writes to the TX FIFO and reads from the RX FIFO. They are separate physical memories despite sharing an address.

### 14.4 Initialisation — CC1101_Init()

This function configures 30+ registers to set up the radio. Each register value is carefully chosen. Here is the complete RF parameter derivation:

#### Carrier Frequency: 868.35 MHz

```c
CC1101_WriteReg(CC1101_FREQ2, 0x21);  // bits [23:16]
CC1101_WriteReg(CC1101_FREQ1, 0x62);  // bits [15:8]
CC1101_WriteReg(CC1101_FREQ0, 0x76);  // bits [7:0]
```

**Formula from the CC1101 datasheet (Section 21):**

```
f_carrier = (f_xosc / 2^16) × FREQ[23:0]

Where:
  f_xosc = 26 MHz (the CC1101 module's crystal oscillator)
  FREQ = 0x216276 = 2,187,894

f_carrier = (26,000,000 / 65,536) × 2,187,894
          = 396.7285... × 2,187,894
          = 868,349,914 Hz
          ≈ 868.35 MHz ✓
```

**Why 868.35 MHz?** This falls within the EU ISM band (868.0–868.6 MHz) for short-range devices. This sub-band allows up to 25 mW ERP with a 1% duty cycle. No licence is needed for this power level.

#### Data Rate: 1.2 kbps

```c
CC1101_WriteReg(CC1101_MDMCFG4, 0xF5);  // DRATE_E = 5 (bits [3:0])
CC1101_WriteReg(CC1101_MDMCFG3, 0x83);  // DRATE_M = 131 (bits [7:0])
```

**Formula:**

```
R_data = (f_xosc / 2^28) × (256 + DRATE_M) × 2^DRATE_E
       = (26,000,000 / 268,435,456) × (256 + 131) × 2^5
       = 0.096858... × 387 × 32
       = 1,199.1 bps
       ≈ 1.2 kbps ✓
```

**Why 1.2 kbps?** This is the minimum data rate the CC1101 supports. Lower data rate = narrower receiver bandwidth = better sensitivity = longer range. At 1.2 kbps, the CC1101 achieves approximately −112 dBm sensitivity, which is close to the theoretical noise floor for this bandwidth.

**Trade-off:** The cost of 1.2 kbps is latency. A 10-byte payload takes:

```
Over-air bytes: 4 (preamble) + 2 (sync) + 1 (length) + 12 (data) + 2 (CRC) = 21 bytes = 168 bits
Time = 168 / 1200 = 140 ms
```

This is fine for 1-second telemetry beacons but would be impractical for real-time control.

#### Modulation: GFSK

```c
CC1101_WriteReg(CC1101_MDMCFG2, 0x13);  // bits [6:4] = 001 → GFSK
```

GFSK (Gaussian Frequency-Shift Keying) passes the binary data through a Gaussian low-pass filter before frequency modulating. This smooths the instantaneous frequency transitions:

```
2-FSK:  Frequency jumps instantaneously between f_carrier ± deviation
        → Sharp transitions → wide spectrum → more adjacent channel interference

GFSK:   Frequency transitions are smoothed by Gaussian filter
        → Gentler transitions → narrower spectrum → better spectral efficiency
        → Required for EU type approval in the 868 MHz band
```

#### Frequency Deviation: 5.16 kHz

```c
CC1101_WriteReg(CC1101_DEVIATN, 0x15);  // DEVIATION_E = 1, DEVIATION_M = 5
```

**Formula:**

```
f_dev = (f_xosc / 2^17) × (8 + DEVIATION_M) × 2^DEVIATION_E
      = (26,000,000 / 131,072) × (8 + 5) × 2^1
      = 198.36... × 13 × 2
      = 5,157 Hz ≈ 5.16 kHz
```

The modulation index is:
```
h = 2 × f_dev / R_data = 2 × 5,157 / 1,200 = 8.6
```

This is a wide modulation index, which provides robust demodulation at the expense of slightly wider bandwidth.

#### Channel Filter Bandwidth: 58.5 kHz

```c
CC1101_WriteReg(CC1101_MDMCFG4, 0xF5);  // bits [7:6] = 11 → BW_E = 3, BW_M = 3
```

The receiver's IF filter is set to 58.5 kHz. This must be wide enough to pass our signal (carrier ± deviation ± Doppler + crystal tolerance) but narrow enough to reject adjacent-channel interference and noise.

#### Packet Engine Configuration

```c
CC1101_WriteReg(CC1101_PKTCTRL1, 0x04);  // Append status bytes (RSSI + LQI/CRC_OK)
CC1101_WriteReg(CC1101_PKTCTRL0, 0x45);  // Whitening ON, CRC ON, variable-length mode
CC1101_WriteReg(CC1101_PKTLEN,   61);    // Max packet length
```

**Data whitening (bit 6 of PKTCTRL0):** XORs the payload with a PN9 pseudo-random sequence before transmission and after reception. Purpose: prevents long runs of identical bits (e.g., 0x00 0x00 0x00) which would create a constant-frequency signal that the receiver's DC offset cancellation circuitry might suppress. Both sides must have whitening enabled for the XOR to cancel out.

**CRC-16 (bit 2 of PKTCTRL0):** The CC1101 calculates a 16-bit CRC over the packet data and appends it. On receive, it recalculates the CRC and sets the CRC_OK flag. A single-bit error anywhere in the packet is guaranteed to be detected. The probability of an undetected error is 1/2^16 = 0.0015%.

**Variable-length mode (bits [1:0] of PKTCTRL0 = 01):** The first byte in the packet is the length, telling the CC1101 how many more bytes to expect. This allows different-sized packets without wasting airtime on padding.

**Append status (bit 2 of PKTCTRL1):** After each received packet, the CC1101 hardware appends two extra bytes to the RX FIFO:
- Byte 1: Raw RSSI value (received signal strength)
- Byte 2: [CRC_OK (bit 7)] [LQI (bits 6:0)] (link quality indicator)

This gives you free per-packet RF diagnostics with zero additional code.

#### TX Power: +12 dBm

```c
CC1101_WriteReg(CC1101_PATABLE, 0xC0);  // PATABLE[0] = 0xC0 → ~+12 dBm
```

The PATABLE is an 8-entry lookup table of TX power levels. For FSK/GFSK, only entry [0] is used (the FREND0 register points to it). The value `0xC0` corresponds to approximately +12 dBm output power at 868 MHz, as specified in the CC1101 datasheet Table 39.

+12 dBm = ~16 mW. The EU 868 MHz limit is 25 mW ERP, so this is legal.

#### Reset Sequence

```c
CS_HIGH(); HAL_Delay(1);
CS_LOW();  HAL_Delay(1);
CS_HIGH(); HAL_Delay(1);
CS_LOW();

while (MISO_PIN() != GPIO_PIN_RESET) { /* wait for MISO low */ }

HAL_SPI_Transmit(&hspi2, (uint8_t[]){CC1101_SRES}, 1, 100);
CS_HIGH();
HAL_Delay(10);
```

This follows the CC1101 datasheet Section 10.1 exactly:

1. **CS pulse** (HIGH → LOW → HIGH → LOW): Forces the CC1101's SPI interface into a known state, even if the MCU reset without power-cycling the radio.
2. **Wait for MISO low**: The CC1101 signals readiness by pulling MISO low. This is a hardware handshake — much more reliable than a fixed delay.
3. **Send SRES (0x30)**: Software reset. Restores all registers to power-on defaults.
4. **Wait 10 ms**: The datasheet specifies a minimum 41 µs post-reset delay. 10 ms provides generous margin.
5. **Wait for MISO low again**: Confirms the crystal oscillator has relocked after reset.

### 14.5 CC1101_Verify() — SPI Self-Test

```c
CC1101_Status_t CC1101_Verify(void) {
    uint8_t version = CC1101_ReadStatus(CC1101_VERSION);
    if (version != 0x14) {
        return CC1101_ERR_NO_CHIP;
    }
    return CC1101_OK;
}
```

This reads the VERSION register, which should always return `0x14` on a genuine CC1101. If it returns anything else, the SPI bus is not working. Common causes:
- Wrong pin wiring (SCK, MOSI, MISO, CS in the wrong order)
- CS pin not toggling (forgot to initialise GPIO)
- 5V power instead of 3.3V (chip destroyed)
- SPI mode wrong (CPOL/CPHA)
- Missing clock enable for the SPI peripheral

### 14.6 CC1101_SendPacket() — Transmitting

```c
CC1101_Status_t CC1101_SendPacket(CC1101_Packet_t *pkt) {
    if (pkt->payload_len > CC1101_MAX_PAYLOAD) return CC1101_ERR_OVERFLOW;

    CC1101_SendCmd(CC1101_SIDLE);   // Step 1: Go to IDLE
    CC1101_SendCmd(CC1101_SFTX);    // Step 2: Flush TX FIFO

    uint8_t total_data_len = 2 + pkt->payload_len;
    uint8_t tx_buf[CC1101_MAX_PAYLOAD + 3];
    tx_buf[0] = total_data_len;              // Length byte
    tx_buf[1] = pkt->packet_id;             // Sequence number
    tx_buf[2] = (uint8_t)pkt->command;      // Command type
    memcpy(&tx_buf[3], pkt->payload, pkt->payload_len);

    CC1101_WriteFIFO(tx_buf, total_data_len + 1);  // Step 3: Write to FIFO
    CC1101_SendCmd(CC1101_STX);                      // Step 4: Start TX

    // Step 5: Wait for TX complete
    uint32_t timeout = HAL_GetTick() + 500;
    uint8_t state;
    do {
        state = CC1101_ReadStatus(CC1101_MARCSTATE) & 0x1F;
        if (HAL_GetTick() > timeout) {
            CC1101_SendCmd(CC1101_SIDLE);
            CC1101_SendCmd(CC1101_SFTX);
            return CC1101_ERR_TIMEOUT;
        }
    } while (state == CC1101_STATE_TX);

    return CC1101_OK;
}
```

**What the CC1101 does after STX:**

```
Time ──────────────────────────────────────────────────────────►

State:  IDLE → CALIBRATING → TX ─────────────────────────→ IDLE
                (~800 µs)

Over-air signal:
        [nothing]  [preamble: 0xAA×4]  [sync: 0xD391]  [len]  [data...]  [CRC-16]
                   │←── 4 bytes ────→│ │←── 2 bytes ──→│ │←── payload ──→│ │2 bytes│
```

The preamble is a repeating 10101010 pattern that the receiver uses to lock its automatic gain control (AGC) and bit synchronisation PLL. Without it, the receiver might not detect the packet at all.

**The FIFO byte layout:**

```
tx_buf[0] = total_data_len (e.g., 10 for 8-byte telemetry)
tx_buf[1] = packet_id (0x00–0xFF sequence number)
tx_buf[2] = command (0x03 = CMD_TELEMETRY)
tx_buf[3..10] = telemetry struct bytes
```

The length byte (tx_buf[0]) tells the CC1101 how many bytes follow. It does NOT count itself. So `total_data_len = 2 + payload_len` counts the packet_id (1) + command (1) + payload bytes.

But `CC1101_WriteFIFO` receives `total_data_len + 1` because we also need to write the length byte itself to the FIFO.

### 14.7 CC1101_ReceivePacket() — Listening

```c
CC1101_Status_t CC1101_ReceivePacket(CC1101_Packet_t *pkt, int8_t *rssi_out, uint32_t timeout_ms) {

    CC1101_SendCmd(CC1101_SIDLE);   // Force IDLE
    CC1101_SendCmd(CC1101_SFRX);    // Flush RX FIFO (remove stale data)
    CC1101_SendCmd(CC1101_SRX);     // Enter RX mode

    // Wait for bytes in FIFO
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    while (CC1101_ReadStatus(CC1101_RXBYTES) == 0) {
        if (HAL_GetTick() > deadline) {
            CC1101_SendCmd(CC1101_SIDLE);
            return CC1101_ERR_TIMEOUT;
        }
    }

    // Wait for complete packet (radio returns to IDLE)
    deadline = HAL_GetTick() + 100;
    uint8_t state;
    do {
        state = CC1101_ReadStatus(CC1101_MARCSTATE) & 0x1F;
        if (HAL_GetTick() > deadline) break;
    } while (state == CC1101_STATE_RX);

    // Read entire FIFO
    uint8_t rxbytes = CC1101_ReadStatus(CC1101_RXBYTES);
    uint8_t rx_raw[64];
    CC1101_ReadFIFO(rx_raw, rxbytes);

    // Parse frame
    uint8_t frame_len = rx_raw[0];
    pkt->packet_id   = rx_raw[1];
    pkt->command     = (CC1101_Cmd_t)rx_raw[2];
    pkt->payload_len = frame_len - 2;
    memcpy(pkt->payload, &rx_raw[3], pkt->payload_len);

    // Extract hardware-appended status bytes
    uint8_t rssi_raw = rx_raw[rxbytes - 2];
    uint8_t lqi_byte = rx_raw[rxbytes - 1];
    uint8_t crc_ok   = (lqi_byte >> 7) & 0x01;

    // Convert RSSI to dBm
    if (rssi_raw >= 128)
        *rssi_out = (int8_t)(((int16_t)rssi_raw - 256) / 2) - 74;
    else
        *rssi_out = (int8_t)(rssi_raw / 2) - 74;

    if (!crc_ok) return CC1101_ERR_CRC;
    return CC1101_OK;
}
```

**RX FIFO layout after a successful reception:**

```
Index:    0        1          2         3..N      N+1       N+2
        ┌────────┬──────────┬─────────┬─────────┬─────────┬─────────┐
        │ Length │ packet_id│ command │ payload │ RSSI_raw│LQI|CRC  │
        │ byte   │          │         │ bytes   │(appended│(appended│
        │        │          │         │         │by HW)   │by HW)   │
        └────────┴──────────┴─────────┴─────────┴─────────┴─────────┘
                  │←────── Length bytes ────────→│
```

**RSSI conversion (CC1101 datasheet Section 17.3):**

The raw RSSI byte is an 8-bit 2's complement number where 1 LSB = 0.5 dB. The offset of −74 dB accounts for the CC1101's internal gain.

```
If RSSI_raw ≥ 128:
    RSSI_dBm = (RSSI_raw − 256) / 2 − 74

If RSSI_raw < 128:
    RSSI_dBm = RSSI_raw / 2 − 74
```

**Example:** If `RSSI_raw = 0xC0 = 192`:
```
RSSI_dBm = (192 − 256) / 2 − 74 = (−64) / 2 − 74 = −32 − 74 = −106 dBm
```

This means the received signal was extremely weak — only 6 dB above the sensitivity floor. You would expect significant packet loss at this level.

---

## 15. main.c: Application Logic — Transmit and Receive

**File:** `Core/Src/main.c`

This file contains two versions of the code: a **Phase 2 packet link** (at the top, within the first `USER CODE BEGIN Header` block) and the **active CubeMX-generated code** (which includes a mock telemetry simulator for a QML dashboard). Both share the same peripheral init functions.

### 15.1 Role Selection via Preprocessor

```c
#define NODE_TRANSMITTER
// #define NODE_RECEIVER
```

Both boards run the same source code. The `#define` selects which code path is compiled. This is controlled by **conditional compilation** — the `#ifdef NODE_TRANSMITTER ... #endif` blocks cause the preprocessor to include or exclude entire code sections before the compiler ever sees them.

**To flash a receiver:** Comment out `#define NODE_TRANSMITTER` and uncomment `#define NODE_RECEIVER`. Rebuild. Flash the second board.

### 15.2 The Telemetry Struct

```c
typedef struct __attribute__((packed)) {
    uint16_t voltage_mv;   // 2 bytes
    int16_t  temp_c10;     // 2 bytes
    uint32_t uptime_s;     // 4 bytes
} Telemetry_t;             // Total: 8 bytes, guaranteed
```

**`__attribute__((packed))`** is critical for radio communication. Without it, the compiler may insert padding bytes between struct fields to align them to natural boundaries. For example, on a 32-bit ARM platform:

```
Without packed (compiler may add padding):
  voltage_mv:  offset 0, size 2
  [2 bytes padding for alignment]
  temp_c10:    offset 4, size 2
  [2 bytes padding]
  uptime_s:    offset 8, size 4
  Total: 12 bytes — 4 bytes of invisible waste

With packed (guaranteed no padding):
  voltage_mv:  offset 0, size 2
  temp_c10:    offset 2, size 2
  uptime_s:    offset 4, size 4
  Total: 8 bytes — exactly what we expect
```

If the transmitter and receiver structs have different padding (different compilers, different optimisation levels), the receiver would misinterpret the data — temperature would contain part of uptime, etc. Packed structs prevent this. The trade-off is slightly slower access on some architectures (unaligned memory access), but the Cortex-M4 handles unaligned accesses in hardware.

### 15.3 Transmitter Main Loop (Phase 2)

```c
while (1) {
    telemetry.voltage_mv = 3700 + (HAL_GetTick() % 200);
    telemetry.temp_c10   = 235 + (sequence_number % 20);
    telemetry.uptime_s   = (HAL_GetTick() - boot_time) / 1000;

    tx_pkt.packet_id   = sequence_number;
    tx_pkt.command     = CMD_TELEMETRY;
    tx_pkt.payload_len = sizeof(Telemetry_t);
    memcpy(tx_pkt.payload, &telemetry, sizeof(Telemetry_t));

    CC1101_Status_t result = CC1101_SendPacket(&tx_pkt);

    if (result == CC1101_OK) {
        printf("%3d | OK        | %lu s\r\n", sequence_number, telemetry.uptime_s);
        LED_Blink(1, 50, 0);
    } else {
        printf("%3d | TX FAILED | %lu s\r\n", sequence_number, telemetry.uptime_s);
        LED_Blink(3, 500, 200);
    }

    sequence_number++;
    HAL_Delay(1000);
}
```

**Duty cycle calculation:**
- Packet transmission time: ~140 ms (21 bytes at 1.2 kbps)
- Interval: 1000 ms
- Duty cycle: 140/1000 = 14%

Wait — that exceeds the 1% EU limit! The code comment says ~1.7% (based on 17 ms for a 10-byte payload). Let's recalculate with the actual payload:

- Total over-air: 4 (preamble) + 2 (sync) + 1 (length) + 2 (id+cmd) + 8 (telemetry) + 2 (CRC) = 19 bytes = 152 bits
- Time at 1.2 kbps: 152 / 1200 = 126.7 ms
- Duty cycle: 126.7 / 1000 = 12.7%

This is well above 1%. The code comment acknowledges this — it's fine for indoor lab testing but would need a 13-second interval for legal outdoor use.

### 15.4 Receiver Main Loop (Phase 2)

```c
while (1) {
    CC1101_Status_t result = CC1101_ReceivePacket(&rx_pkt, &rssi_dbm, 2000);

    if (result == CC1101_OK) {
        // Sequence gap detection
        uint8_t expected_seq = last_seq + 1;
        if (rx_pkt.packet_id != expected_seq && total_packets > 0) {
            uint8_t gap = (uint8_t)(rx_pkt.packet_id - expected_seq);
            missed_packets += gap;
        }
        last_seq = rx_pkt.packet_id;
        total_packets++;

        // Decode telemetry
        Telemetry_t received_data;
        memcpy(&received_data, rx_pkt.payload, sizeof(Telemetry_t));

        // PER calculation
        uint32_t total_sent_estimate = total_packets + missed_packets;
        uint32_t per_tenths = (missed_packets * 1000 / total_sent_estimate);

        printf("%3d | %4d dBm | %4d mV | %3d.%1d°C | %5lu s | %lu.%lu%%\r\n",
            rx_pkt.packet_id, rssi_dbm,
            received_data.voltage_mv,
            received_data.temp_c10 / 10, received_data.temp_c10 % 10,
            received_data.uptime_s,
            per_tenths / 10, per_tenths % 10);
    }
}
```

**PER (Packet Error Rate) calculation:**

The receiver tracks `packet_id` (sequence number). If it receives packet 5 then packet 8, it knows packets 6 and 7 were lost. This is the simplest form of sequence-gap PER measurement:

```
PER = missed_packets / (received_packets + missed_packets) × 100%
```

This is a standard metric in telecommunications. A PER < 1% is considered excellent for most applications. A PER > 10% indicates significant link degradation.

**Integer-only fixed-point formatting:** The `printf` uses integer division and modulo to display one decimal place without floating point:

```c
per_tenths / 10,   // Integer part
per_tenths % 10    // Decimal part
```

This avoids pulling in the floating-point printf library, which adds ~10 KB to the binary on ARM — significant when you have 512 KB of FLASH.

### 15.5 Active Code: Mock Telemetry for QML Dashboard

The code that actually compiles (the CubeMX `USER CODE` sections) generates simulated telemetry for a QML dashboard:

```c
int rssi = -85 + (rand() % 15);
float ber = (rand() % 50) / 1000.0f;
float snr = 12.0f + (rand() % 10);
float i_val = 0.707f * (rand() % 2 == 0 ? 1 : -1) + ((rand() % 20 - 10) / 100.0f);
float q_val = 0.707f * (rand() % 2 == 0 ? 1 : -1) + ((rand() % 20 - 10) / 100.0f);

sprintf(uart_buffer, "RSSI:%d,BER:%.3f,SNR:%.1f,SEQ:%lu,I:%.3f,Q:%.3f\n",
        rssi, ber, snr, sequence_number, i_val, q_val);
HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
```

**I/Q constellation simulation:** The `i_val` and `q_val` simulate a QPSK (Quadrature Phase-Shift Keying) constellation diagram. In QPSK, there are four ideal constellation points at (±0.707, ±0.707). The noise added (±0.1) simulates real-world signal degradation. This data feeds a QML dashboard that visualises the constellation in real-time.

**Packet loss simulation:** `rand() % 100 < 5` gives a 5% probability of skipping a sequence number, simulating packet drops.

---

## 16. RF Engineering Fundamentals Used in This Code

### 16.1 Link Budget

The link budget determines whether two radios can communicate at a given distance:

```
P_received = P_transmit + G_tx_antenna - L_path + G_rx_antenna

Where:
  P_transmit      = +12 dBm (CC1101 output power)
  G_tx_antenna    = +2 dBi (typical for a helical PCB antenna)
  L_path          = Free-space path loss (distance-dependent)
  G_rx_antenna    = +2 dBi
  P_received      must be > -112 dBm (CC1101 sensitivity at 1.2 kbps)
```

**Free-space path loss:**
```
L_path = 20 × log10(d) + 20 × log10(f) - 147.55    [in dB]

At d = 1 km, f = 868 MHz:
L_path = 20 × log10(1000) + 20 × log10(868×10^6) - 147.55
       = 60 + 178.77 - 147.55
       = 91.22 dB
```

**Link margin at 1 km:**
```
P_received = 12 + 2 - 91.22 + 2 = -75.22 dBm
Margin = -75.22 - (-112) = 36.78 dB
```

A 36 dB margin means you could tolerate significant fading, obstruction, or increased distance before losing the link. Real-world tests (with multipath, Fresnel zone obstruction, foliage, interference) typically show 10–20 dB less margin than free-space predictions.

### 16.2 Sensitivity and Its Relationship to Data Rate

Receiver sensitivity is fundamentally limited by thermal noise:

```
N_floor = -174 + 10 × log10(BW)    [dBm]

At 58.5 kHz bandwidth:
N_floor = -174 + 10 × log10(58500) = -174 + 47.67 = -126.33 dBm
```

The CC1101 achieves −112 dBm sensitivity, which means the receiver needs a signal ~14 dB above the noise floor (this 14 dB is the required SNR for reliable GFSK demodulation at the configured BER).

If we increased the data rate to 250 kbps, the bandwidth would increase to ~540 kHz:
```
N_floor = -174 + 10 × log10(540000) = -174 + 57.33 = -116.67 dBm
```

The sensitivity would degrade to approximately −104 dBm — meaning 8 dB less range margin.

### 16.3 Duty Cycle and EU Regulations

The 868.0–868.6 MHz sub-band (Band 47 in ETSI EN 300 220) has these constraints:

| Parameter | Limit |
|-----------|-------|
| Max ERP | 25 mW (+14 dBm) |
| Duty cycle | ≤ 1% |
| Channel bandwidth | ≤ 600 kHz |
| Spectrum access | Listen-before-talk OR duty-cycle limited |

1% duty cycle means: in any rolling 1-hour window, your transmitter can be "on" for no more than 36 seconds total. At 1.2 kbps with a ~130 ms packet, that's about 277 packets per hour, or one every 13 seconds.

---

## 17. The Build System and Debug Artefacts

### 17.1 Build Toolchain

The project uses the **ARM GCC toolchain** (`arm-none-eabi-gcc`) invoked through STM32CubeIDE's managed build system. Key tools:

| Tool | Purpose |
|------|---------|
| `arm-none-eabi-gcc` | C compiler — generates ARM Thumb-2 machine code |
| `arm-none-eabi-as` | Assembler — assembles the startup `.s` file |
| `arm-none-eabi-ld` | Linker — combines `.o` files using the linker script |
| `arm-none-eabi-objcopy` | Extracts `.bin` or `.hex` from `.elf` for flashing |
| `arm-none-eabi-size` | Reports code/data size (flash and RAM usage) |

### 17.2 Debug Output Files

The `Debug/` directory contains:

| File | Purpose |
|------|---------|
| `GroundStation_Node.elf` | The complete binary with debug symbols. Used by the debugger (GDB). |
| `GroundStation_Node.map` | Linker map — shows exactly where every function and variable lives in memory. |
| `GroundStation_Node.list` | Disassembly listing — shows the machine code next to the C source. |
| `*.o` | Object files — one per `.c` file. Intermediate compilation output. |
| `*.d` | Dependency files — track which headers each `.c` file includes, so make knows what to rebuild. |
| `*.su` | Stack usage files — report the stack frame size of each function. |
| `*.cyclo` | Cyclomatic complexity reports — measure code complexity per function. |

### 17.3 The .ioc File

`GroundStation_Node.ioc` is a CubeMX project file in INI format. It stores your pin assignments, clock configuration, and peripheral settings. When you regenerate code in CubeMX, it reads this file and produces:
- `main.c` (skeleton with `USER CODE` sections preserved)
- `stm32f4xx_hal_msp.c` (MSP init/deinit)
- `stm32f4xx_it.c` (interrupt handlers)
- `main.h` (pin defines)
- `stm32f4xx_hal_conf.h` (HAL module switches)

**Critical rule:** Never edit code outside `USER CODE BEGIN/END` markers in CubeMX-generated files. CubeMX will overwrite anything outside those markers.

---

## 18. Common Failure Modes and Debugging Guide

### 18.1 SPI Communication Failures

| Symptom | Likely Cause | Diagnosis |
|---------|-------------|-----------|
| `CC1101_Verify()` returns `ERR_NO_CHIP` | Wiring error | Check SCK, MOSI, MISO, CS connections with multimeter |
| VERSION reads 0x00 | CS not toggling | Verify PB12 is configured as output, starts HIGH |
| VERSION reads 0xFF | MISO floating | Check MISO wire connection, pull-up/pull-down |
| Intermittent failures | Loose jumper wire | Re-seat all connections, use shorter wires |
| Works at low speed only | Signal integrity | Reduce SPI prescaler (increase divider), add decoupling caps |

### 18.2 RF Communication Failures

| Symptom | Likely Cause | Diagnosis |
|---------|-------------|-----------|
| TX OK but RX always times out | Different frequency settings | Verify both boards have identical `CC1101_Init()` |
| Packets received but CRC fails | Frequency mismatch, interference | Check RSSI — if very low, adjust antenna; if high, check whitening match |
| High PER at short range | FIFO overflow | Ensure RX loop reads FIFO fast enough |
| Works at 1 m, fails at 10 m | Low TX power, no antenna | Verify PATABLE = 0xC0, check antenna connection |

### 18.3 Debugging Tools

- **Serial console (`screen /dev/tty.usbmodem* 115200`):** First line of defence. The firmware prints status on every packet.
- **SWD debugger (STM32CubeIDE):** Set breakpoints, inspect variables, read registers. The ST-Link on the Nucleo board provides this.
- **Logic analyser:** Capture SPI waveforms to verify timing, bit order, and CS behavior. Essential for debugging SPI issues.
- **SDR (Software Defined Radio):** Use an RTL-SDR dongle with GNU Radio to verify the radio is actually transmitting on 868.35 MHz.

---

## 19. Glossary of Key Terms

| Term | Definition |
|------|-----------|
| **AGC** | Automatic Gain Control — adjusts receiver amplification to handle varying signal levels |
| **APB** | Advanced Peripheral Bus — a low-speed bus in the ARM AMBA architecture, connecting peripherals |
| **AHB** | Advanced High-performance Bus — the high-speed bus connecting the CPU core, memory, and DMA |
| **BSS** | Block Started by Symbol — the memory section for uninitialised global variables (zeroed at startup) |
| **CMSIS** | Cortex Microcontroller Software Interface Standard — ARM's hardware abstraction for Cortex-M cores |
| **CPOL/CPHA** | Clock Polarity / Clock Phase — define the four SPI modes (0–3) |
| **CRC** | Cyclic Redundancy Check — an error-detection code computed over packet data |
| **dBm** | Decibels relative to 1 milliwatt — absolute RF power unit. 0 dBm = 1 mW, +10 dBm = 10 mW |
| **DMA** | Direct Memory Access — hardware that moves data between peripherals and memory without CPU involvement |
| **Duty Cycle** | The fraction of time a transmitter is active. EU 868 MHz band limits this to 1% |
| **ERP** | Effective Radiated Power — TX power adjusted for antenna gain, used in regulatory limits |
| **FIFO** | First In First Out — a hardware queue buffer. The CC1101 has separate 64-byte TX and RX FIFOs |
| **FSK** | Frequency-Shift Keying — digital modulation that encodes bits as frequency shifts |
| **GFSK** | Gaussian FSK — FSK with a Gaussian filter that smooths frequency transitions |
| **GPIO** | General-Purpose Input/Output — configurable digital pins |
| **HAL** | Hardware Abstraction Layer — ST's API that wraps register access with portable function calls |
| **HSI** | High-Speed Internal oscillator — the STM32's built-in 16 MHz RC oscillator |
| **ISM Band** | Industrial, Scientific, Medical — unlicensed radio frequency bands (e.g., 868 MHz in EU) |
| **ISR** | Interrupt Service Routine — the function that runs when a hardware interrupt fires |
| **LNA** | Low-Noise Amplifier — the first amplification stage in a receiver, critical for sensitivity |
| **LQI** | Link Quality Indicator — the CC1101's estimate of demodulation quality |
| **MARC** | Main Radio Control state machine — the CC1101's internal state machine |
| **MISO/MOSI** | Master In Slave Out / Master Out Slave In — SPI data lines |
| **MSP** | MCU Support Package — board-specific peripheral pin/clock configuration |
| **NVIC** | Nested Vectored Interrupt Controller — ARM Cortex-M interrupt management hardware |
| **PATABLE** | Power Amplifier Table — the CC1101's lookup table for TX power levels |
| **PER** | Packet Error Rate — the fraction of transmitted packets that fail to arrive correctly |
| **PLL** | Phase-Locked Loop — a circuit that generates a high-frequency clock from a low-frequency reference |
| **PN9** | Pseudo-Noise sequence of period 2^9−1 — used for data whitening |
| **Push-pull** | An output driver that actively drives both high and low (vs open-drain which only drives low) |
| **RSSI** | Received Signal Strength Indication — the power level of the received signal |
| **SCK** | Serial Clock — the SPI clock line generated by the master |
| **Slew rate** | How fast a signal transitions between voltage levels — controlled by GPIO speed setting |
| **SNR** | Signal-to-Noise Ratio — the power ratio between the desired signal and background noise |
| **SPI** | Serial Peripheral Interface — a synchronous full-duplex serial bus |
| **SRAM** | Static Random-Access Memory — volatile, fast, byte-addressable memory |
| **SWD** | Serial Wire Debug — a 2-wire debug interface used by ARM Cortex-M debuggers |
| **SysTick** | System Tick Timer — a 24-bit counter in the Cortex-M core, typically configured for 1 ms ticks |
| **UART** | Universal Asynchronous Receiver-Transmitter — an asynchronous serial protocol |
| **VCO** | Voltage-Controlled Oscillator — the core of the PLL that generates the carrier frequency |
| **Weak symbol** | A linker symbol that can be overridden by a strong symbol of the same name |

---

## Appendix A: Complete SPI Transaction Trace

For a single `CC1101_WriteReg(CC1101_FREQ2, 0x21)` call, this is what happens at the electrical level:

```
Time (µs)  0              0.4           0.8           1.2    ...    3.2
           │               │              │              │              │
CS    ─────┘               │              │              │              └─────
           ↓ LOW           │              │              │              ↑ HIGH
           │               │              │              │              │
SCK   ─────────╥──╥──╥──╥─╥──╥──╥──╥────╥──╥──╥──╥──╥─╥──╥──╥──╥────
               ╨  ╨  ╨  ╨ ╨  ╨  ╨  ╨    ╨  ╨  ╨  ╨  ╨ ╨  ╨  ╨  ╨
               1  2  3  4  5  6  7  8    1  2  3  4  5  6  7  8
               │                         │
MOSI  ─────────│ 0x0D = 00001101        │ 0x21 = 00100001
               │ (FREQ2 register addr)  │ (data: FREQ2 high byte)
               │ Bit7=0(W), Bit6=0(S)   │
               │ Addr=001101=0x0D       │
```

Each clock pulse transfers one bit. 8 pulses = 1 byte. At 2.625 MHz SPI clock, each bit takes 381 ns, and the full 2-byte transaction takes about 6.1 µs plus CS setup/hold time.

---

## Appendix B: State Machine of the CC1101

```
                    ┌──────────┐
     Power-on ────→│   IDLE   │←──── SIDLE command
                    └────┬─────┘
                         │
              ┌──────────┼──────────┐
              │          │          │
              ▼          │          ▼
        ┌──────────┐    │    ┌──────────┐
  SRX → │CALIBRATE │    │    │CALIBRATE │ ← STX
        └────┬─────┘    │    └────┬─────┘
             │          │          │
             ▼          │          ▼
        ┌──────────┐    │    ┌──────────┐
        │    RX    │    │    │    TX    │
        │ (listen) │    │    │ (send)   │
        └────┬─────┘    │    └────┬─────┘
             │          │          │
             ▼          │          ▼
    Packet received     │    Packet sent
     or timeout         │         │
             │          │          │
             └──────────┴──────────┘
                        │
                  Back to IDLE
                 (MCSM1 = 0x30)
```

The MCSM1 register is set to `0x30`, which means: after TX, go to IDLE; after RX, go to IDLE. This gives the software full control — the driver explicitly transitions the radio between states rather than relying on automatic TX↔RX cycling.

---

*This document covers the complete HeliosLink codebase as of April 2026. For the CC1101 datasheet (SWRS061I) and STM32F446RE reference manual (RM0390), consult the respective manufacturer websites.*
