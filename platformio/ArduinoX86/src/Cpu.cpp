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

#include "Cpu.h"

void Cpu::reset(CpuResetResult reset_result, bool preserve_bus_state, bool reset_registers) {
  trap.reset();

  // Retain detected cpu & fpu type and emulation flags.
  //cpu_type = CpuType::Undetected;
  //fpu_type = FpuType::noFpu;
  if (reset_result.busWidth == BusWidth::Eight) {
    width = CpuBusWidth::Eight;
    queue = InstructionQueue(4, BusWidth::Eight);
  } else {
    width = CpuBusWidth::Sixteen;
    queue = InstructionQueue(6, BusWidth::Sixteen);
  }
  cycle_ct_ = 0;
  doing_reset = false;
  doing_id = false;
  //do_emulation = false;
  in_emulation = false;
  do_prefetch = false;
  cpuid_counter = 0;
  cpuid_queue_reads = 0;
  state_begin_time = 0;
  exception_stage = 0;

  if (!preserve_bus_state) {
    last_address_bus = 0;
    address_bus = 0;
    address_latch_ = 0;
    bus_state_latched = BusStatus::PASV;
    bus_state = BusStatus::PASV;
    last_bus_cycle = TCycle::TI;
    bus_cycle = TCycle::TI;
    data_width = ActiveBusWidth::EightLow;
    data_bus = 0;
    data_type = QueueDataType::Program;
    data_bus_resolved = false;
    prefetching_store = false;
    reads_during_prefetching_store = 0;
    status0 = 0;
    command_bits = 0;
    control_bits = 0;
  }
  
  v_pc = 0;
  s_pc = 0;
  stack_r_op_ct = 0;
  stack_w_op_ct = 0;
  pre_emu_flags = 0;
  emu_flags = 0;
  
  if (reset_registers) {
    memset(const_cast<registers1_t*>(&load_regs), 0, sizeof(load_regs));
    memset(const_cast<registers1_t*>(&post_regs), 0, sizeof(post_regs));
    memset(const_cast<Loadall286*>(&loadall_regs_286), 0, sizeof(loadall_regs_286));
    memset(const_cast<Loadall386*>(&loadall_regs_386), 0, sizeof(loadall_regs_386));
  }
  readback_p = (uint8_t *)&post_regs;

  have_queue_status = reset_result.queueStatus;

  opcode = 0;
  mnemonic = "NONE";
  qb = 0;
  qt = QueueDataType::Program;
  q_ff = false;
  q_fn = 0;
  nmi_terminate = false;
  smi_terminate = false;
  nmi_checkpoint = 0;
  smi_checkpoint = 0;
  nmi_buf_cursor = 0;
  program = &JUMP_VECTOR;
  program->reset();
  memset(&nmi_stack_frame, 0, sizeof(nmi_stack_frame));
  loadall_checkpoint = 0;
  error_cycle_ct = 0;
  execute_cycle_ct = 0;
  wait_states = 0;
  wait_state_ct = 0;
  exception_armed = false;
  predicted_fetch = 0;
}
