/*
    ArduinoX86 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduinoX86

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER   
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include <cstdint>

#define OPCODE_HALT 0xF4
#define OPCODE_DOUBLE_HALT 0xF4F4 // For 386, which only fetches at even addresses
#define OPCODE_NOP 0x90
#define OPCODE_DOUBLENOP 0x9090
#define OPCODE_80NOP 0x00
#define OPCODE_DOUBLE_80NOP 0x0000

#define MODRM_OP(M) (((M & 0b00111000) >> 3) & 0x07)

#define GRP1 105
#define GRP2A 106
#define GRP2B 110
#define GRP3 107
#define GRP4 108
#define GRP5 109
#define IS_GRP_OP(O) ((OPCODE_REFS[O] >= GRP1) && (OPCODE_REFS[O] <= GRP2B))

extern const char * const OPCODE_STRS[];
extern const char * const OPCODE_STRS_GRP1[];
extern const char * const OPCODE_STRS_GRP2A[];  
extern const char * const OPCODE_STRS_GRP2B[];
extern const char * const OPCODE_STRS_GRP3[];
extern const char * const OPCODE_STRS_GRP4[];
extern const char * const OPCODE_STRS_GRP5[];
extern const char * const OPCODE_8080_STRS[];
extern const uint8_t OPCODE_REFS[];
extern const uint8_t OPCODE_8080_REFS[];

// Various opcode definitions for 286 and 386
enum class Opcode : uint8_t {
  Escape0F = 0x0F,
  PopSs = 0x17,
  PrefixEs = 0x26,
  PrefixCs = 0x2E,
  PrefixSs = 0x36,
  PrefixDs = 0x3E,
  IncAx = 0x40,
  IncCx = 0x41,
  IncDx = 0x42,
  IncBx = 0x43,
  IncSp = 0x44,
  IncBp = 0x45,
  IncSi = 0x46,
  IncDi = 0x47,
  DecAx = 0x48,
  DecCx = 0x49,
  DecDx = 0x4A,
  DecBx = 0x4B,
  DecSp = 0x4C,
  DecBp = 0x4D,
  DecSi = 0x4E,
  DecDi = 0x4F,
  // 386 prefixes; these encodings are undefined on the 286.
  PrefixFs = 0x64,
  PrefixGs = 0x65,
  PrefixOperandSize = 0x66,
  PrefixAddressSize = 0x67,
  MovSegmentRm16 = 0x8E,
  Nop = 0x90,
  CallFarImmediate = 0x9A,
  Popf = 0x9D,
  MovAlImmediate = 0xB0,
  MovClImmediate = 0xB1,
  MovDlImmediate = 0xB2,
  MovBlImmediate = 0xB3,
  MovAhImmediate = 0xB4,
  MovChImmediate = 0xB5,
  MovDhImmediate = 0xB6,
  MovBhImmediate = 0xB7,
  MovAxImmediate = 0xB8,
  MovCxImmediate = 0xB9,
  MovDxImmediate = 0xBA,
  MovBxImmediate = 0xBB,
  MovSpImmediate = 0xBC,
  MovBpImmediate = 0xBD,
  MovSiImmediate = 0xBE,
  MovDiImmediate = 0xBF,
  RetFarImmediate = 0xCA,
  RetFar = 0xCB,
  Int3 = 0xCC,
  IntImmediate = 0xCD,
  Into = 0xCE,
  Iret = 0xCF,
  JmpFarImmediate = 0xEA,
  PrefixLock = 0xF0,
  StoreallPrefix286 = 0xF1,
  PrefixRepne = 0xF2,
  PrefixRep = 0xF3,
  Halt = 0xF4,
  Group5 = 0xFF,
};

// Secondary bytes after 0F have their own opcode map on the 286.
enum class Opcode0F286 : uint8_t {
  SystemGroup = 0x01,
  Loadall = 0x05,
};

// ModR/M reg fields are opcode extensions (or segment indices), not primary
// opcode bytes. Keep their names scoped to the instruction which uses them.
enum class Group5Opcode : uint8_t {
  CallFar = 0x03,
  JmpFar = 0x05,
};

enum class SystemGroupOpcode286 : uint8_t {
  Lgdt = 0x02,
  Lidt = 0x03,
  Lmsw = 0x06,
};

enum class SegmentRegisterEncoding : uint8_t {
  Es = 0x00,
  Cs = 0x01,
  Ss = 0x02,
  Ds = 0x03,
};