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
#include <Arduino.h>
#include <CpuTypes.h>
#include <BusTypes.h>
#include <InstructionQueue.h>
#include <programs.h>
#include <registers.h>
#include <TrapExecution.h>

// This class is slowly being converted from a C structure. Pardon the mess.

// Main CPU State
class Cpu {
public:

  CpuType cpu_type; // Detected type of the CPU.
  FpuType fpu_type; // Detected type of FPU (0 if none)
  CpuBusWidth width; // Native bus width of the CPU. Detected on reset from BHE line.
  bool doing_reset;
  bool doing_id;
  bool do_emulation; // Flag that determines if we enter 8080 emulation mode after Load
  bool in_emulation; // Flag set when we have entered 8080 emulation mode and cleared when we have left
  bool do_prefetch; // Flag that determines if we enter Prefetch state and execute a prefetch program.
  uint32_t cpuid_counter; // Cpuid cycle counter. Used to time to identify the CPU type.
  uint32_t cpuid_queue_reads; // Number of queue reads since reset of Cpuid cycle counter.
  uint32_t state_begin_time;
  uint32_t last_address_bus;
  uint32_t address_bus;
  
  BusStatus bus_state_latched; // Bus state latched on T1 and valid for entire bus cycle (immediate bus state goes PASV on T3)
  BusStatus bus_state; // Bus state is current status of S0-S2 at given cycle (may not be valid)
  TCycle last_bus_cycle;
  TCycle bus_cycle;
  ActiveBusWidth data_width; // Current size of data bus. Detected during bus transfer from BHE line.
  uint16_t data_bus;
  bool data_bus_resolved; // Whether we have resolved the data bus this m-cycle or not.
  bool prefetching_store;
  uint8_t reads_during_prefetching_store;
  QueueDataType data_type;
  uint8_t status0; // S0-S5, QS0 & QS1
  uint8_t command_bits; // 8288 command outputs
  uint8_t control_bits; // 8288 control outputs
  uint16_t v_pc; // Virtual program counter
  uint16_t s_pc; // Store program counter
  uint16_t stack_r_op_ct; // Number of stack read operations in current state
  uint16_t stack_w_op_ct; // Number of stack write operations in current state
  uint16_t pre_emu_flags; // Flags pushed to stack by BRKEM
  uint8_t emu_flags; // Flags pushed to stack by PUSH PSW during EmuExit program
  volatile registers1_t load_regs; // Register state set by Load command
  volatile Loadall286 loadall_regs_286; // Register state set by Loadall command on 286
  TrapExecution trap;
  volatile Loadall386 loadall_regs_386; // Register state set by Loadall command on 386
  volatile registers1_t post_regs; // Register state retrieved from Store program
  uint8_t *readback_p;
  bool have_queue_status; // Whether we have access to the queue status lines. Can be detected during RESET.
  InstructionQueue queue; // Instruction queue
  uint8_t opcode; // Currently executing opcode
  const char *mnemonic; // Decoded mnemonic
  uint8_t qb; // Last byte value read from queue
  QueueDataType qt; // Last data type read from queue
  bool q_ff; // Did we fetch a first instruction byte from the queue this cycle?
  uint8_t q_fn; // What # byte of instruction did we fetch?
  bool nmi_terminate; // Whether we are entering ExecuteFinalize via NMI termination.
  bool smi_terminate; // Whether we are entering StoreAll via SMI termination.
  uint8_t nmi_checkpoint; // How many reads we have done at the NMI IVT address.
  uint8_t smi_checkpoint; 
  uint16_t nmi_buf_cursor;
  InlineProgram *program = &JUMP_VECTOR;
  CallStackFrame nmi_stack_frame; // NMI stack frame for 286/386 CPUs
  uint8_t loadall_checkpoint;
  int error_cycle_ct;
  int execute_cycle_ct;
  int wait_states;
  int wait_state_ct;
  bool exception_armed;
  uint32_t predicted_fetch;
  uint32_t exception_stage;

  void reset(CpuResetResult reset_result, bool preserve_bus_state = false, bool reset_registers = false);

  bool use_smm() const { return use_smm_; }
  void set_use_smm(bool use_smm) {
    use_smm_ = use_smm;
  }

  inline uint64_t cycle_ct() const __attribute__((always_inline));
  inline void tick() __attribute__((always_inline));
  inline uint32_t address_latch() const __attribute__((always_inline));
  inline void latch_address(uint32_t address) __attribute__((always_inline));

  bool is_shutdown() const { return is_shutdown_; }
  void set_shutdown() { is_shutdown_ = true; }
  void clear_shutdown() { is_shutdown_ = false; }



  void set_program_bounds(uint32_t start, uint32_t end) {
    program_start_ = start;
    program_end_ = end;
  }

  bool is_address_in_program(uint32_t address) const {
    return (address >= program_start_) && (address < program_end_);
  }

  void set_jump_hint(bool enabled, bool odd_address) {
    jump_hint_enabled_ = enabled;
    jump_hint_odd_address_ = odd_address;
  }

  bool have_jump_hint() const {
    return jump_hint_enabled_;
  }

  bool jump_hint_is_odd() const {
    return jump_hint_odd_address_;
  }

private:

  uint32_t program_start_ = 0;
  uint32_t program_end_ = 0;
  bool use_smm_ = false; // Use SMM for register readout on 386/486 CPUs
  bool is_shutdown_ = false; // Whether the CPU is in a shutdown state.
  uint64_t cycle_ct_ = 0; // Number of cycles executed since reset.
  uint32_t address_latch_ = 0; // Value of address bus as of ALE/ADS.

  bool jump_hint_enabled_ = false; // Whether the jump hint is or not
  bool jump_hint_odd_address_ = false; // If the jump hint is enabled
};

uint64_t Cpu::cycle_ct() const {
  return cycle_ct_;
}

void Cpu::tick() {
  cycle_ct_++;
}

uint32_t Cpu::address_latch() const {
  return address_latch_;
}

void Cpu::latch_address(uint32_t address) {
  // Latch the address bus value on ALE/ADS
  address_latch_ = address;
}
