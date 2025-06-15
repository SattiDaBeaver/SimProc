=# Simple Processor (SimProc)

SimProc is an 8-bit processor designed for educational purposes, featuring a simplified instruction set and memory-mapped I/O architecture.

# Instruction Set

The SimProc instruction set consists of basic arithmetic, logical, and control flow operations. Currently, SimProc supports 11 instructions:

## **Arithmetic and Logical Operations**

| **Instruction** | **Description** | **Operation** |
| --- | --- | --- |
| add rA, rB | Add registers rA and rB, and store in rA | rA ← rA + rB |
| sub rA, rB | Subtract registers rA and rB, and store in rA | rA ← rA - rB |
| nand rA, rB | NAND registers rA and rB, and store in rA | rA ← rA NAND rB |
| ori imm5 | OR immediate r0 with imm5 | r0 ← r0 OR ZE(imm5) |
| shift dir rA, imm2 | Shift (dir = l : left, r : right) by imm2 | rA ← rA << ZE(imm2) or rA >> ZE(imm2) |

## **Memory Operations (Load and Store)**

| load rA, (rB) | Load from memory rB into rA | rA ← Mem[rB] |
| --- | --- | --- |
| store rA, (rB) | Store to memory rB from rA | Mem[rB] ← rA |

## Branch Operations

| j imm4 | Jump relatively by imm4 | PC ← PC + SE(imm4) |
| --- | --- | --- |
| bz imm4 | Branch relatively by imm4 if result of previous operation was zero | PC ← PC + SE(imm4)    if operation = 0 |
| bnz imm4 | Branch relatively by imm4 if result of previous operation was not zero | PC ← PC + SE(imm4)    if operation **≠0** |
| bpz imm4  | Branch relatively by imm4 if result of previous operation was positive | PC ← PC + SE(imm4)    if operation ≥ **0** |

**General Notes:** 

- rA, rB can be any of the processor's four general-purpose registers: r0, r1, r2, r3
- imm5 is a 5-bit zero extended integer (eg. imm5(23) → 00010111)
- imm2 is a 2-bit zero extended integer (eg. imm2(2) → 00000010)
- imm4 is a 4-bit sign extended integer (eg. imm4(12) → 11111100, imm4(7) → 00000111)

All arithmetic operations are performed in 8-bit precision with overflow wrapping.

# Memory Mapped Input/Output

The Simple Processor (SimProc) uses memory mapped I/O to interact with external devices. This means certain memory addresses are reserved for I/O operations rather than regular data storage.

## Memory Map Overview

| **Device** | **Memory Address** |
| --- | --- |
| 0x00 - 0xAF | Regular Memory |
| 0xB0  | LEDs |
| 0xC0 | HEX1, HEX0 |
| 0xC1 | HEX3, HEX2 |
| 0xC2 | HEX5, HEX4 |

## Regular Memory (0x00 - 0xAF)

Regular memory addresses from 0x00 to 0xAF are used for both program instructions and data values during processor operation.

This provides 176 bytes of addressable memory space for general-purpose use, which is sufficient for simple educational programs and demonstrations.

## LEDs (0xB0)

Memory Mapped LEDs can be accessed by either writing to, or reading from the memory address 0xB0. Currently, only the first 8 LEDs of the FPGA can be accessed through memory mapped I/O.

**Note:** LEDR_8 and LEDR_9 are inaccessible

| Address |  | Name |
| --- | --- | --- |
| 0xB0 | 7 [LEDR_7]        ….         0 [LEDR_0]  | Data Register |

# Example Codes

## Loading 8-bit Numbers

Since there is only one immediate instruction (ori), we cannot load a 8-bit integer into a register in one instruction. We have to use multiple instructions to be able to load any 8-bit integer into a register.

This can be done by loading the upper 5-bits of the integer to r0, shifting it to the left by 3, and then loading the lower 3 bits. This number can then be transferred to any other register.

```nasm
; loading 79 (01001111) into r1
sub r0, r0      ; set r0 to zero (in case it wasn't zero already)
ori 0b01001     ; load upper 5-bits (r0 <- 00001001)
shift l r0, 3   ; shift r0 left by 3 (r0 << 3: r0 = 01001000)
ori 0b111       ; load lower 3 bits (r0 <- r0 or 111: r0 = 01001111)

; optional: if we want to move the number to r2
sub r2, r2      ; set r2 to zero (in case it wasn't zero already)
add r2, r0      ; r2 <- r2 + r0 (r0 = 0 + 79: r0 = 79)
```

## LED Counter

This code counts up by 1 and stores the count in the LEDs.

```nasm
sub r0, r0
sub r1, r1
sub r2, r2

ori 1
add r2, r0

sub r0, r0

ori 22
shift l r0, 3

store r1, r0
add r1, r2

j -3
```

Note: All I/O operations should be performed using load (load) and store (store) instructions. The processor treats I/O addresses like regular memory addresses.

