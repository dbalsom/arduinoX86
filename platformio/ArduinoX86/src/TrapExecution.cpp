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

#include <arduinoX86.h>
#include <globals.h>
#include <BoardController.h>
#include <CommandServer.h>
#include "opcodes.h"

namespace {
InlineProgram trap_return("Trap IRET", {0xCF}); // IRET
constexpr uint32_t ICE_RESTART_286 = 0xFFFFF0;
constexpr uint8_t ICE_LOADALL_286[] = {0xF1, 0x0F, 0x05}; // F1 LOADALL

void fail_trap(TrapReason reason) {
  CPU.trap.fail(reason);
  ArduinoX86::Server.change_state(ServerState::Error);
}

// Observe only valid T2 data, once per bus transfer; an odd byte is on D8-D15.
void observe_data(bool write) {
  auto& t = CPU.trap;
  const uint32_t address = CPU.address_latch();
  if (CPU.data_width == ActiveBusWidth::EightHigh) {
    if (write) {
      t.write_byte(address, CPU.data_bus >> 8);
    }
    else {
      t.read_byte(address, CPU.data_bus >> 8);
    }
  } else {
    if (write) {
      t.write_byte(address, CPU.data_bus);
    }
    else {
      t.read_byte(address, CPU.data_bus);
    }
    if (CPU.data_width == ActiveBusWidth::Sixteen) {
      if (write) {
        t.write_byte(address + 1, CPU.data_bus >> 8);
      }
      else {
        t.read_byte(address + 1, CPU.data_bus >> 8);
      }
    }
  }
}


}

const char* prepare_trap_execution() {
  if (CPU.cpu_type != CpuType::i80286) {
    return "Trap execution requires an 80286";
  }
  // TF belongs to the execution mode, not to the caller's initial FLAGS image.
  CPU.loadall_regs_286.flags |= TrapExecution::TRAP_FLAG;
  const auto& regs = *const_cast<const Loadall286*>(&CPU.loadall_regs_286);
  auto& bus = *ArduinoX86::Bus;
  const bool pm = regs.msw & 1;
  const uint32_t vector = TrapExecution::base(regs.idt_desc) + (pm ? 8 : 4);
  if (regs.idt_desc.limit < (pm ? 15 : 7) || vector > 0x1000000u - (pm ? 8u : 4u)) {
    return "IDT does not contain a valid vector-1 entry";
  }
  uint32_t handler = CPU.trap.real_mode_handler;
  // Keep the managed handler segment at 0500h, with vector 1 at offset 0010h.
  uint16_t offset = handler & 0xff;
  uint16_t selector = (handler - offset) >> 4;
  if (pm) {
    offset = bus.mem_read_u16(vector, false);
    selector = bus.mem_read_u16(vector + 2, false);
    const uint8_t access = bus.mem_read_u8(vector + 5, false);
    // Initial implementation requires a present 286 interrupt/trap gate to a
    // nonconforming ring-0 GDT code segment. Task gates need a different frame.
    if ((access & 0x9e) != 0x86 || (selector & 7) || selector == 0) {
      return "Vector 1 requires a present 286 gate to a ring-0 GDT code segment";
    }
    if (uint32_t(selector) + 7 > regs.gdt_desc.limit) {
      return "Vector-1 code selector is outside the GDT";
    }
    const uint32_t desc = TrapExecution::base(regs.gdt_desc) + selector;
    if (desc > 0xfffff8) {
      return "Vector-1 code descriptor crosses the address bus limit";
    }
    const uint8_t code_access = bus.mem_read_u8(desc + 5, false);
    if ((code_access & 0xfc) != 0x98
        || uint32_t(offset) + TrapExecution::HANDLER_SPAN - 1 > bus.mem_read_u16(desc, false)) {
      return "Vector-1 handler needs a present ring-0 code segment with room for server dispatch";
    }
    handler = bus.mem_read_u16(desc + 2, false) | (uint32_t(bus.mem_read_u8(desc + 4, false)) << 16);
    handler += offset;
  }
  if (handler > 0x1000000u - TrapExecution::HANDLER_SPAN
      || TrapExecution::overlaps(handler, TrapExecution::HANDLER_SPAN, LOADALL286_ADDRESS, sizeof(Loadall286))) {
    return "Trap handler overlaps STOREALL memory or crosses the address bus limit";
  }
  if (!pm) {
    if (!TrapExecution::valid_real_mode_handler(handler)
        || TrapExecution::overlaps(vector, 4, handler, TrapExecution::HANDLER_SPAN)
        || TrapExecution::overlaps(vector, 4, LOADALL286_ADDRESS, sizeof(Loadall286))) {
      return "Real-mode trap vector/handler overlaps reserved memory; relocate the handler or IDT";
    }
    // Install after uploads and before guest execution. Only vector 1 changes;
    // handler instructions are supplied on the bus, not written to guest RAM.
    // IDTR may be unaligned; byte writes work on both memory backends.
    bus.mem_write_u8(vector, offset & 0xff);
    bus.mem_write_u8(vector + 1, offset >> 8);
    bus.mem_write_u8(vector + 2, selector & 0xff);
    bus.mem_write_u8(vector + 3, selector >> 8);
  }
  CPU.trap.begin(regs, handler, selector);
  CPU.trap.handler_base = handler - offset;
  return nullptr;
}

void resume_trap_handler() {
  auto& t = CPU.trap;
  // Let the CPU perform IRET, including cache reloads and any resulting faults.
  // Trapping is part of this execution environment, not a transparent observer.
  t.dispatch_pending = false;
  t.returning = true;
  // Dispatch begins at this CODE transfer; the next fetch follows its byte(s).
  t.expected_fetch = CPU.address_latch() + ((CPU.address_latch() & 1) ? 1 : 2);
  trap_return.reset();
}

void handle_ice_recovery_286() {
  auto& t = CPU.trap;
  auto& server = ArduinoX86::Server;
  if (!t.capture_ready) {
    server.set_error("ICE recovery requires a completed trap STOREALL image");
    fail_trap(TrapReason::InvalidFrame);
    return;
  }

  // F1 LOADALL restores the unmodified image, including the post-STOREALL IP.
  // Normal bus status returns with the first fetch at that handler address.
  if (t.restore_phase == TrapRestorePhase::Handler
      && CPU.bus_cycle == T1 && CPU.bus_state_latched == CODE) {
    const uint32_t resume = TrapExecution::base(t.capture_image.cs_desc) + t.capture_image.ip;
    if (CPU.address_latch() != resume) {
      server.set_error("ICE LOADALL returned to %06lX, expected %06lX", CPU.address_latch(), resume);
      fail_trap(TrapReason::InvalidFrame);
      return;
    }
    t.restore_phase = TrapRestorePhase::None;
    resume_trap_handler();
    server.change_state(ServerState::Execute);
    return;
  }

  // F1 makes the LOADALL image reads visible on the normal memory bus. Keep
  // serving the trailing STOREALL read too: the pause occurs in that transfer.
  if (CPU.bus_cycle == WRITE_CYCLE && !Controller.readMRDCPin()
      && CPU.bus_state_latched == MEMR) {
    const uint32_t address = CPU.address_latch();
    if (address < LOADALL286_ADDRESS || address >= LOADALL286_ADDRESS + sizeof(Loadall286)) {
      server.set_error("ICE LOADALL read outside its image: %06lX", address);
      fail_trap(TrapReason::InvalidFrame);
      return;
    }
    const auto* bytes = reinterpret_cast<const uint8_t*>(&t.capture_image);
    const uint32_t offset = address - LOADALL286_ADDRESS;
    if (CPU.data_width == ActiveBusWidth::EightHigh) {
      CPU.data_bus = uint16_t(bytes[offset]) << 8;
    }
    else {
      CPU.data_bus = bytes[offset];
      if (CPU.data_width == ActiveBusWidth::Sixteen && offset + 1 < sizeof(Loadall286)) {
        CPU.data_bus |= uint16_t(bytes[offset + 1]) << 8;
      }
    }
    Controller.writeDataBus(CPU.data_bus, CPU.data_width);
    CPU.data_bus_resolved = true;
    if (t.restore_phase == TrapRestorePhase::Loadall
        && offset + (CPU.data_width == ActiveBusWidth::Sixteen ? 2u : 1u) == sizeof(Loadall286)) {
      t.restore_phase = TrapRestorePhase::Handler;
    }
    return;
  }

  // ICE fetches expose live addresses but no normal CODE/ALE indication on
  // this 286. Do not latch a fake bus cycle or sample the unbonded ICE pins.
  // Qualify entry by FFFFF0h; idle all-ones addresses alone must not arm it.
  const uint32_t address = CPU.address_bus;
  if (t.restore_phase == TrapRestorePhase::Restart && address == ICE_RESTART_286) {
    t.restore_phase = TrapRestorePhase::Loadall;
  }
  if (t.restore_phase != TrapRestorePhase::Loadall || address < ICE_RESTART_286 || address > 0xFFFFFF) {
    return;
  }
  const uint32_t offset = (address & ~1u) - ICE_RESTART_286;
  const uint8_t low = offset < sizeof(ICE_LOADALL_286) ? ICE_LOADALL_286[offset] : OPCODE_NOP;
  const uint8_t high = offset + 1 < sizeof(ICE_LOADALL_286) ? ICE_LOADALL_286[offset + 1] : OPCODE_NOP;
  CPU.data_bus = uint16_t(low) | (uint16_t(high) << 8);
  Controller.writeDataBus(CPU.data_bus, ActiveBusWidth::Sixteen);
}

void handle_execute_trap() {
  auto& t = CPU.trap;
  auto& server = ArduinoX86::Server;
  if (t.state.status == TrapStatus::Ready && t.at_endpoint()) {
    // No guest instruction has executed. Return the known LOADALL state.
    t.state.status = TrapStatus::Captured;
    t.state.reason = TrapReason::Endpoint;
    t.state.capture_cycle = CPU.cycle_ct();
    server.change_state(ServerState::StoreDone);
    return;
  }
  if (t.state.status == TrapStatus::Ready && CPU.bus_cycle != T1) {
    t.state.status = TrapStatus::Running;
  }
  if (!t.remaining_cycles) {
    fail_trap(TrapReason::Timeout);
    return;
  }
  --t.remaining_cycles;
  if (CPU.bus_state == HALT) {
    fail_trap(TrapReason::Halt);
    return;
  }

  if (CPU.bus_state_latched == CODE && CPU.bus_cycle == T1) {
    if (t.observe_fetch(CPU.address_latch(), CPU.cycle_ct())) {
      if (t.at_endpoint() || t.step) {
        t.capturing = true;
        t.dispatch_pending = false;
        t.state.reason = t.at_endpoint() ? TrapReason::Endpoint : TrapReason::Step;
        server.change_state(ServerState::StoreAll);
        return;
      }
      resume_trap_handler();
    }
    if (t.state.status == TrapStatus::Failed) {
      server.change_state(ServerState::Error);
      return;
    }
  }
  if (CPU.bus_cycle != WRITE_CYCLE) {
    return;
  }
  auto& bus = *ArduinoX86::Bus;
  const bool bhe = !Controller.readBHEPin();
  if (!Controller.readMWTCPin()) {
    bus.mem_write_bus(CPU.address_latch(), CPU.data_bus, bhe);
    observe_data(true);
  }
  if (!Controller.readMRDCPin()) {
    const bool fetch = CPU.bus_state_latched == CODE;
    if (fetch && t.returning) {
      CPU.data_bus = trap_return.read(CPU.address_latch(), CPU.data_width);
    }
    else if (fetch && t.supply_handler_nops(CPU.address_latch())) {
      // Early handler fetches can precede the complete saved frame. Keep them
      // harmless until dispatch, without overwriting the backing memory.
      CPU.data_bus = OPCODE_DOUBLENOP;
    }
    else {
      CPU.data_bus = bus.mem_read_bus(CPU.address_latch(), bhe, fetch);
    }
    if (!fetch) {
      observe_data(false);
    }
    Controller.writeDataBus(CPU.data_bus, CPU.data_width);
    CPU.data_bus_resolved = true;
  }
  if (!Controller.readIOWCPin()) {
    bus.io_write_bus(CPU.address_latch(), CPU.data_bus, bhe);
  }
  if (!Controller.readIORCPin()) {
    CPU.data_bus = bus.io_read_bus(CPU.address_latch(), bhe);
    Controller.writeDataBus(CPU.data_bus, CPU.data_width);
  }
}
