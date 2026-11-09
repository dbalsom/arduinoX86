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
#ifndef _ARDUINO_X86_H
#define _ARDUINO_X86_H

#include <Arduino.h>
#include <stdint.h>

#include <config.h>
#include <serial_config.h>
#include <ansi_color.h>
#include <BusTypes.h>
#include <Shield.h>
#include <gpio_pins.h>
#include <Cpu.h>
#include <InstructionQueue.h>
#include <InlineProgram.h>
#include <programs.h>
#include <registers.h>

#define SMRAM_FIRST_WRITE 0x03FFFC
#define SMRAM_LAST_WRITE 0x03FF10

// Code segment to use for load program.
const uint16_t LOAD_SEG = 0xD000;
const uint16_t STORE_SEG = 0xE000;
const uint32_t NMI_ADDR = 0x00008;

// Maximum size of the processor instruction queue. For 8088 == 4, 8086 == 6. 
#define QUEUE_SIZE 6

// Processor instruction queue
typedef struct queue {
  uint8_t queue[QUEUE_SIZE];
  uint8_t types[QUEUE_SIZE];
  size_t size;
  uint8_t front;
  uint8_t back;
  uint8_t len;
} Queue;

#define QUEUE_IDLE 0x00
#define QUEUE_FIRST 0x01
#define QUEUE_FLUSHED 0x02
#define QUEUE_SUBSEQUENT 0x03

// Strings for pretty-printing instruction queue status from QS0,QS1
// '.' = Idle  
// 'F' = First byte fetched 
// 'E' = Queue Emptied 
// 'S' = Subsequent byte fetched
const char QUEUE_STATUS_CHARS[] = {
  ' ', 'F', 'E', 'S'
};

typedef struct program_stats {
  uint16_t code_read_xfers;
  uint16_t memory_read_xfers;
  uint16_t memory_write_xfers;
  uint16_t io_read_xfers;
  uint16_t io_write_xfers;
  uint32_t idle_cycles;
  uint32_t program_cycles;
} p_stats;

typedef struct i8288 {
  BusStatus last_status; // S0-S2 of previous cycle
  BusStatus status; // S0-S2 of current cycle
  BusStatus status_latch;
  TCycle tcycle;
  bool ale;
  bool mrdc;
  bool amwc;
  bool mwtc;
  bool iorc;
  bool aiowc;
  bool iowc;
  bool inta;
} Intel8288;

// ----------------------------- CPU FLAGS ----------------------------------//
const uint16_t CPU_FLAG_CARRY      = 0b0000000000000001;
const uint16_t CPU_FLAG_RESERVED1  = 0b0000000000000010;
const uint16_t CPU_FLAG_PARITY     = 0b0000000000000100;
const uint16_t CPU_FLAG_RESERVED3  = 0b0000000000001000;
const uint16_t CPU_FLAG_AUX_CARRY  = 0b0000000000010000;
const uint16_t CPU_FLAG_RESERVED5  = 0b0000000000100000;
const uint16_t CPU_FLAG_ZERO       = 0b0000000001000000;
const uint16_t CPU_FLAG_SIGN       = 0b0000000010000000;
const uint16_t CPU_FLAG_TRAP       = 0b0000000100000000;
const uint16_t CPU_FLAG_INT_ENABLE = 0b0000001000000000;
const uint16_t CPU_FLAG_DIRECTION  = 0b0000010000000000;
const uint16_t CPU_FLAG_OVERFLOW   = 0b0000100000000000;

#define CPU_FLAG_DEFAULT_SET_8086 0xF002
#define CPU_FLAG_DEFAULT_SET_286 0x0002
#define CPU_FLAG_DEFAULT_SET_386 0x0002
#define CPU_FLAG_DEFAULT_CLEAR_8086 0xFFD7
#define CPU_FLAG_DEFAULT_CLEAR_286 0xFFD7
#define CPU_FLAG_DEFAULT_CLEAR_386 0xFFFFFFD7;
// ----------------------------- GPIO PINS ----------------------------------//

#define SPIN_DELAY(count) do { \
    volatile unsigned int _i; \
    for (_i = 0; _i < (count); _i++) { \
        __asm__ __volatile__("nop"); \
    } \
} while(0)

// Time in microseconds to wait after setting clock HIGH or LOW

#if defined(__SAM3X8E__) // If Arduino DUE
  #define CLOCK_PIN_DELAY (void)0
  #define CLOCK_PIN_HIGH_DELAY SPIN_DELAY(1000)
  #define CLOCK_PIN_LOW_DELAY SPIN_DELAY(1000)
#elif defined(ARDUINO_GIGA) 
  #define CLOCK_PIN_DELAY ((void)0)
  #define CLOCK_PIN_HIGH_DELAY ((void)0)
  #define CLOCK_PIN_LOW_DELAY ((void)0)
#endif

// Microseconds to wait after a pin direction change. Without some sort of delay
// a subsequent read/write may fail. You may need to tweak this if you have a 
// different board - some types need longer delays

#if defined(__AVR_ATmega2560__) // Arduino MEGA
  #if BOARD_TYPE == ELEGOO_MEGA 
    #define PIN_CHANGE_DELAY (delayMicroseconds(3))
  #elif BOARD_TYPE == ARDUINO_MEGA
    #define PIN_CHANGE_DELAY (delayMicroseconds(1))
  #endif
#elif defined(__SAM3X8E__) // If Arduino DUE
  #define PIN_CHANGE_DELAY ((void)0)
#elif defined(ARDUINO_GIGA)
  #define PIN_CHANGE_DELAY ((void)0)
#endif

// Bit reverse LUT from http://graphics.stanford.edu/~seander/bithacks.html#BitReverseTable
static const uint8_t BIT_REVERSE_TABLE[256] = 
{
#   define R2(n)    n,     n + 2*64,     n + 1*64,     n + 3*64
#   define R4(n) R2(n), R2(n + 2*16), R2(n + 1*16), R2(n + 3*16)
#   define R6(n) R4(n), R4(n + 2*4 ), R4(n + 1*4 ), R4(n + 3*4 )
    R6(0), R6(2), R6(1), R6(3)
};

// --------------------- Function declarations --------------------------------

// main.cpp
void cycle();
void set_error(const char *msg);
void clear_error();
bool cpu_id();
void patch_load_pgm(InlineProgram *pgm, volatile registers1_t *reg);
void patch_brkem_pgm(InlineProgram *pgm, volatile registers1_t *regs);

void convert_inline_registers(volatile void *inline_regs);
void reverse_stack_buf();
//bool is_transfer_done(); -> inlined
void handle_fetch(uint8_t q);
void handle_cpuid_state(uint8_t q);
void handle_cpu_setup_state();
void handle_jump_vector_state(uint8_t q);
void handle_loadall_286();
void handle_loadall_386();
void handle_load_state(uint8_t q);
void handle_load_done_state();
void handle_emu_enter_state(uint8_t q);
void handle_storeall_286();
void handle_smm_load_386();
void handle_smm_store_386();
void handle_store_state();
void handle_execute_state();
//void handle_execute_automatic();    -> inlined
void handle_execute_finalize_state();
void detect_fpu_type();
void detect_cpu_type(uint32_t cpuid_cycles);
void reset_screen();
bool readParameterBytes(uint8_t *buf, size_t buf_len, size_t len);

uint32_t calc_flat_address(uint16_t seg, uint16_t offset);

void clock_tick();
void data_bus_write(uint16_t data, CpuBusWidth width);
uint16_t data_bus_read();

// TrapExecution.cpp
// Null on success, otherwise a diagnostic for CmdLoad.
const char* prepare_trap_execution();
void handle_execute_trap();
void resume_trap_handler();
void handle_ice_recovery_286();

// cpu.cpp
void read_address(bool peek);
void read_status0();
uint8_t read_status0_raw();
bool cpu_reset();
void cpu_set_width(CpuBusWidth width);
const char *get_opcode_str(uint8_t op1, uint8_t op2, bool modrm);

// bus.cpp
void data_bus_write(uint16_t data, ActiveBusWidth width);
uint16_t data_bus_read(ActiveBusWidth width);
uint16_t data_bus_peek(CpuBusWidth width);
void read_address();
uint32_t peek_address();
bool a0();
uint32_t read_address_pins(bool peek);

// buzzer.cpp
void beep(uint32_t time);
void error_beep();



#endif // _ARDUINO_X86_H
