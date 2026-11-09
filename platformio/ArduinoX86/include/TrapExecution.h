#pragma once

#include <cstdint>
#include <cstring>
#include <registers.h>

// Wire values are shared with arduinox86_client::trap. Keep this record packed.
enum class TrapStatus : uint8_t { Idle, Ready, Running, Paused, Captured, Failed };
enum class TrapReason : uint8_t { None, Step, Endpoint, Exception, Timeout, Shutdown, Halt, InvalidFrame };

enum class TrapRestorePhase : uint8_t { None, Restart, Loadall, Handler };

struct __attribute__((packed)) InstructionState {
  static constexpr uint8_t FRAME_VALID = 0x01;
  static constexpr uint8_t STACK_VALID = 0x02;
  static constexpr uint8_t PROTECTED_MODE = 0x04;
  static constexpr uint8_t STACK_SWITCHED = 0x08;
  uint8_t version = 1;
  TrapStatus status = TrapStatus::Idle;
  TrapReason reason = TrapReason::None;
  uint8_t vector = 0xff;
  uint32_t run_id = 0;
  uint32_t sequence = 0;
  uint16_t start_cs = 0, start_ip = 0;
  uint16_t resume_cs = 0, resume_ip = 0;
  uint16_t flags = 0, ss = 0, sp = 0;
  uint8_t frame_bytes = 0;
  uint8_t validity = 0; // Mask of the flags defined above.
  uint32_t frame_address = 0;
  uint32_t trap_cycle = 0;
  uint32_t resume_cycle = 0;
  uint32_t handler_address = 0;
  uint32_t capture_cycle = 0;
};
static_assert(sizeof(InstructionState) == 48, "InstructionState wire layout");

class TrapExecution {
public:
  static constexpr uint16_t TRAP_FLAG = 0x0100;
  // Managed layout, mirrored in arduinox86_client::trap. Raw LOADALL callers
  // still supply their own IDTR and protected-mode gate/descriptor addresses.
  static constexpr uint32_t DEFAULT_REAL_MODE_HANDLER = 0x005010;
  static constexpr uint32_t HANDLER_SPAN = 16;
  static constexpr uint32_t MAX_DISPATCH_OFFSET = 8;
  uint32_t real_mode_handler = DEFAULT_REAL_MODE_HANDLER;
  InstructionState state;
  bool enabled = false;
  bool endpoint_enabled = false;
  uint16_t endpoint_cs = 0, endpoint_ip = 0;
  bool step = false;
  bool returning = false;
  bool dispatch_pending = false;
  bool capturing = false;
  bool capture_ready = false;
  Loadall286 capture_image = {};
  TrapRestorePhase restore_phase = TrapRestorePhase::None;
  uint32_t handler_address = 0, handler_base = 0, vector_address = 0;
  uint32_t idt_base = 0;
  uint16_t idt_limit = 0;
  uint32_t expected_fetch = 0;
  uint32_t remaining_cycles = 0;
  uint16_t handler_cs = 0;
  bool protected_mode = false;

  void reset() {
    const uint32_t run = state.run_id;
    state = InstructionState{};
    state.run_id = run;
    enabled = false;
    step = false;
    remaining_cycles = 0;
    returning = dispatch_pending = capturing = false;
    capture_ready = false;
    restore_phase = TrapRestorePhase::None;
    clear_entry();
  }

  void begin(const Loadall286& regs, uint32_t handler, uint16_t selector) {
    const uint32_t run = state.run_id + 1;
    reset();
    enabled = true;
    protected_mode = (regs.msw & 1) != 0;
    idt_base = base(regs.idt_desc);
    idt_limit = regs.idt_desc.limit;
    vector_address = idt_base + (protected_mode ? 8 : 4);
    handler_address = handler;
    handler_cs = selector;
    state.run_id = run;
    state.status = TrapStatus::Ready;
    state.start_cs = state.resume_cs = regs.cs;
    state.start_ip = state.resume_ip = regs.ip;
    state.flags = regs.flags;
    state.ss = regs.ss;
    state.sp = regs.sp;
    state.handler_address = handler;
    state.validity = InstructionState::STACK_VALID | (protected_mode ? InstructionState::PROTECTED_MODE : 0);
    remaining_cycles = 1000000;
  }

  static uint32_t base(const SegmentDescriptor286& d) {
    return uint32_t(d.addr_lo) | (uint32_t(d.addr_hi) << 16);
  }

  static bool overlaps(uint32_t address, uint32_t size, uint32_t other, uint32_t other_size) {
    return address < other + other_size && other < address + size;
  }

  static bool valid_real_mode_handler(uint32_t address) {
    return address <= 0x100000 - HANDLER_SPAN
      && !overlaps(address, HANDLER_SPAN, LOADALL286_ADDRESS, sizeof(Loadall286));
  }

  bool supply_handler_nops(uint32_t address) const {
    return enabled && !returning && handler_seen_ && candidate_vector_ == 1
      && address >= handler_address && address - handler_address <= MAX_DISPATCH_OFFSET;
  }

  void start(bool single_step, uint32_t budget, uint32_t cycle) {
    step = single_step;
    remaining_cycles = budget;
    state.status = TrapStatus::Running;
    state.reason = TrapReason::None;
    state.resume_cycle = cycle;
  }

  void fail(TrapReason reason, uint8_t vector = 0xff) {
    state.status = TrapStatus::Failed;
    state.reason = reason;
    state.vector = vector;
  }

  void complete_capture(const Loadall286& image, uint32_t cycle) {
    capture_image = image;
    capture_ready = true;
    capturing = false;
    state.capture_cycle = cycle;
    state.status = state.reason == TrapReason::Endpoint ? TrapStatus::Captured : TrapStatus::Paused;
  }

  void clear_entry() {
    vector_mask_ = 0;
    write_count_ = 0;
    candidate_vector_ = 0xff;
    handler_seen_ = false;
  }

  void read_byte(uint32_t address, uint8_t value) {
    if (returning) {
      return;
    }
    if (address < idt_base || address - idt_base > idt_limit) {
      return;
    }
    const uint32_t entry_size = protected_mode ? 8 : 4;
    const uint32_t offset = address - idt_base;
    const uint32_t vector = offset / entry_size;
    if (vector > 255) {
      return;
    }
    if (candidate_vector_ != vector) {
      candidate_vector_ = vector;
      vector_mask_ = 0;
    }
    const uint8_t index = offset % entry_size;
    const uint8_t old_mask = vector_mask_;
    vector_bytes_[index] = value;
    vector_mask_ |= uint8_t(1u << index);
    // Protected entry reads the gate before pushing the frame. Discard earlier
    // guest stores so data resembling a frame cannot trigger at the first fetch.
    if (protected_mode && (old_mask & 0x3f) != 0x3f && (vector_mask_ & 0x3f) == 0x3f) {
      write_count_ = 0;
    }
  }

  void write_byte(uint32_t address, uint8_t value) {
    if (returning) {
      return;
    }
    if (write_count_ == 16) {
      memmove(writes_, writes_ + 1, 15 * sizeof(writes_[0]));
      write_count_--;
    }
    writes_[write_count_++] = {address, value};
  }

  // Called at the beginning of a handler fetch. The first fetch can precede
  // the stack pushes on a same-ring 286 trap, so wait for the complete frame.
  bool observe_fetch(uint32_t address, uint32_t cycle) {
    if (returning) {
      if (address != expected_fetch) {
        returning = false;
        clear_entry();
        state.start_cs = state.resume_cs;
        state.start_ip = state.resume_ip;
        state.resume_cycle = cycle;
      }
      expected_fetch = address + ((address & 1) ? 1 : 2);
      return false;
    }
    const uint8_t required = protected_mode ? 0x3f : 0x0f;
    if ((vector_mask_ & required) != required) {
      return false;
    }
    const uint16_t selector = uint16_t(vector_bytes_[2]) | (uint16_t(vector_bytes_[3]) << 8);
    if (selector != handler_cs) {
      return false;
    }
    if (candidate_vector_ != 1) {
      const uint16_t offset = uint16_t(vector_bytes_[0]) | (uint16_t(vector_bytes_[1]) << 8);
      if (address == handler_base + offset) {
        fail(TrapReason::Exception, candidate_vector_);
      }
      return false;
    }
    if (address == handler_address) {
      handler_seen_ = true;
    }
    if (!handler_seen_ || address < handler_address) {
      return false;
    }
    if (address - handler_address > MAX_DISPATCH_OFFSET) {
      fail(TrapReason::InvalidFrame);
      return false;
    }
    if (write_count_ < 6) {
      return false;
    }
    const uint16_t ip = word_back(0), cs = word_back(2), flags = word_back(4);
    if ((flags & 0x0102) != 0x0102) {
      return false;
    }
    const bool switched = protected_mode && ((cs & 3) != (handler_cs & 3));
    const uint8_t bytes = switched ? 10 : 6;
    if (write_count_ < bytes || !valid_frame(bytes)) {
      return false;
    }
    state.sequence++;
    capture_ready = false;
    state.capture_cycle = 0;
    state.vector = 1;
    state.resume_cs = cs;
    state.resume_ip = ip;
    state.flags = flags;
    state.frame_bytes = bytes;
    state.frame_address = writes_[write_count_ - 2].address;
    state.trap_cycle = cycle;
    state.validity = InstructionState::FRAME_VALID
      | (protected_mode ? InstructionState::PROTECTED_MODE : 0);
    if (switched) {
      state.sp = word_back(6);
      state.ss = word_back(8);
      state.validity |= InstructionState::STACK_VALID | InstructionState::STACK_SWITCHED;
    } else {
      // A same-ring frame contains no SS:SP. The original LOADALL cache may
      // no longer describe the guest stack, so do not infer it from an address.
      state.ss = 0;
      state.sp = 0;
    }
    dispatch_pending = true;
    return true;
  }

  bool at_endpoint() const {
    return endpoint_enabled && state.resume_cs == endpoint_cs && state.resume_ip == endpoint_ip;
  }

private:
  struct ByteWrite { uint32_t address; uint8_t value; };
  ByteWrite writes_[16] = {};
  uint8_t write_count_ = 0;
  uint8_t vector_bytes_[8] = {};
  uint8_t vector_mask_ = 0;
  uint8_t candidate_vector_ = 0xff;
  bool handler_seen_ = false;

  uint16_t word_back(uint8_t offset) const {
    return uint16_t(writes_[write_count_ - offset - 2].value)
      | (uint16_t(writes_[write_count_ - offset - 1].value) << 8);
  }
  bool valid_frame(uint8_t count) const {
    const uint32_t ip_address = writes_[write_count_ - 2].address;
    for (uint8_t offset = 0; offset < count; offset += 2) {
      const auto& lo = writes_[write_count_ - offset - 2];
      const auto& hi = writes_[write_count_ - offset - 1];
      if (lo.address != ip_address + offset || hi.address != lo.address + 1) {
        return false;
      }
    }
    return true;
  }
};
