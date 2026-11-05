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

#include <cstdarg>

#include <Arduino.h>
#include <config.h>

#include <Shield.h>
#include <Board.h>
#include <globals.h>
#include <BoardController.h>
#include <CommandServer.h>
#include <DebugFilter.h>
#include <programs.h>
#include <bus_emulator/IBusBackend.h>

#if defined (ARDUINO_GIGA)
#define MAX_BUFFER_LEN 4096ul
#else
#define MAX_BUFFER_LEN 512ul
#endif

template<typename BoardType, typename ShieldType>
CommandServer<BoardType,ShieldType>::CommandServer(BoardController<BoardType,ShieldType>& controller_)
  : controller_(controller_)
{
  flags_ |= FLAG_LOG_CYCLES;

  useSmm_ = USE_SMI;
  if (useSmm_) {
    flags_ |= FLAG_USE_SMM;
    CPU.set_use_smm(useSmm_);
  }
  
  if (controller_.getBoard().isDebugEnabled()) {
    flags_ |= FLAG_DEBUG_ENABLED;
  }
}


template<typename BoardType, typename ShieldType>
void CommandServer<BoardType,ShieldType>::reset() 
{
  ArduinoX86::Bus->reset_logging();
  ArduinoX86::Bus->disable_logging();
  change_state(ServerState::Done);
  commandState_ = CommandState::WaitingForCommand;
}


/// @brief Returns the name of the command based on the ServerCommand enum.
/// This is useful for debugging and logging purposes.
/// @tparam BoardType 
/// @tparam ShieldType 
/// @param cmd The command to get the name of.
/// @return Name of the command as a constant C string.
template<typename BoardType, typename ShieldType>
const char* CommandServer<BoardType, ShieldType>::get_command_name(ServerCommand cmd) {
  switch(cmd) {
      case ServerCommand::CmdNone: return "CmdNone";
      case ServerCommand::CmdVersion: return "CmdVersion";
      case ServerCommand::CmdResetCpu: return "CmdResetCpu";
      case ServerCommand::CmdLoad: return "CmdLoad";
      case ServerCommand::CmdCycle: return "CmdCycle";
      case ServerCommand::CmdReadAddressLatch: return "CmdReadAddressLatch";
      case ServerCommand::CmdReadStatus: return "CmdReadStatus";
      case ServerCommand::CmdRead8288Command: return "CmdRead8288Command";
      case ServerCommand::CmdRead8288Control: return "CmdRead8288Control";
      case ServerCommand::CmdReadDataBus: return "CmdReadDataBus";
      case ServerCommand::CmdWriteDataBus: return "CmdWriteDataBus";
      case ServerCommand::CmdFinalize: return "CmdFinalize";
      case ServerCommand::CmdBeginStore: return "CmdBeginStore";
      case ServerCommand::CmdStore: return "CmdStore";
      case ServerCommand::CmdQueueLen: return "CmdQueueLen";
      case ServerCommand::CmdQueueBytes: return "CmdQueueBytes";
      case ServerCommand::CmdWritePin: return "CmdWritePin";
      case ServerCommand::CmdReadPin: return "CmdReadPin";
      case ServerCommand::CmdGetProgramState: return "CmdGetProgramState";
      case ServerCommand::CmdLastError: return "CmdLastError";
      case ServerCommand::CmdGetCycleState: return "CmdGetCycleState";
      case ServerCommand::CmdAvailable00: return "CmdAvailable00";
      case ServerCommand::CmdPrefetchStore: return "CmdPrefetchStore";
      case ServerCommand::CmdReadAddress: return "CmdReadAddress";
      case ServerCommand::CmdCpuType: return "CmdCpuType";
      case ServerCommand::CmdSetFlags: return "CmdSetFlags";
      case ServerCommand::CmdPrefetch: return "CmdPrefetch";
      case ServerCommand::CmdInitScreen: return "CmdInitScreen";
      case ServerCommand::CmdStoreAll: return "CmdStoreAll";
      case ServerCommand::CmdSetRandomSeed: return "CmdSetRandomSeed";
      case ServerCommand::CmdRandomizeMem: return "CmdRandomizeMem";
      case ServerCommand::CmdSetMemory: return "CmdSetMemory";
      case ServerCommand::CmdGetCycleStates: return "CmdGetCycleStates";
      case ServerCommand::CmdEnableDebug: return "CmdEnableDebug";
      case ServerCommand::CmdSetMemoryStrategy: return "CmdSetMemoryStrategy";
      case ServerCommand::CmdGetFlags: return "CmdGetFlags";
      case ServerCommand::CmdReadMemory: return "CmdReadMemory";
      case ServerCommand::CmdEraseMemory: return "CmdEraseMemory";
      case ServerCommand::CmdServerStatus: return "CmdServerStatus";
      case ServerCommand::CmdClearCycleLog: return "CmdClearCycleLog";
      case ServerCommand::CmdSetProgramBounds: return "CmdSetProgramBounds";
      case ServerCommand::CmdSetJumpHint: return "CmdSetJumpHint";
      case ServerCommand::CmdInvalid: return "CmdInvalid";
      default: return "Unknown";
  }
}

template<typename BoardType, typename ShieldType>
const char* CommandServer<BoardType, ShieldType>::get_state_string(ServerState state) {
  switch(state) {
      case ServerState::Reset: return "Reset";
      case ServerState::CpuId: return "CpuId";
      case ServerState::CpuSetup: return "CpuSetup";
      case ServerState::JumpVector: return "JumpVector";
      case ServerState::Load: return "Load";
      case ServerState::LoadSmm: return "LoadSmm";
      case ServerState::LoadDone: return "LoadDone";
      case ServerState::EmuEnter: return "EmuEnter";
      case ServerState::Prefetch: return "Prefetch";
      case ServerState::Execute: return "Execute";
      case ServerState::ExecuteFinalize: return "ExecuteFinalize";
      case ServerState::ExecuteDone: return "ExecuteDone";
      case ServerState::EmuExit: return "EmuExit";
      case ServerState::Store: return "Store";
      case ServerState::StoreDone: return "StoreDone";
      case ServerState::StoreDoneSmm: return "StoreDoneSmm";
      case ServerState::Done: return "Done";
      case ServerState::StoreAll: return "StoreAll";
      case ServerState::Shutdown: return "Shutdown";
      case ServerState::Error: return "Error";
      default: return "Invalid";
  }
}

template<typename BoardType, typename ShieldType>
char CommandServer<BoardType, ShieldType>::get_state_char(ServerState state) {
  switch(state) {
      case ServerState::Reset: return 'R';
      case ServerState::CpuId: return 'I';
      case ServerState::CpuSetup: return 'C';
      case ServerState::JumpVector: return 'J';
      case ServerState::Load: return 'L';
      case ServerState::LoadSmm: return 'L';
      case ServerState::LoadDone: return 'M';
      case ServerState::EmuEnter: return '8';
      case ServerState::Prefetch: return 'P';
      case ServerState::Execute: return 'E';
      case ServerState::ExecuteFinalize: return 'F';
      case ServerState::ExecuteDone: return 'X';
      case ServerState::EmuExit: return '9';
      case ServerState::Store: return 'S';
      case ServerState::StoreDone: return 'T';
      case ServerState::StoreDoneSmm: return 'U';
      case ServerState::StoreAll: return 'A';
      case ServerState::Done: return 'D';
      case ServerState::Error: return '!';
      case ServerState::Shutdown: return 'H';
      default: return '?';
  }
}

/// @brief Dispatches a command based on the command byte received.
/// @tparam BoardType
/// @tparam ShieldType
/// @param cmd_byte The command byte to dispatch.
/// @return True if the command was successfully dispatched, false otherwise.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::dispatch_command(ServerCommand cmd) {
  switch(cmd) {
    case ServerCommand::CmdNone:
        return cmd_null();
    case ServerCommand::CmdVersion:
        return cmd_version();
    case ServerCommand::CmdResetCpu:
        return cmd_reset_cpu();
    case ServerCommand::CmdLoad:
        return cmd_load();
    case ServerCommand::CmdCycle:
        return cmd_cycle();
    case ServerCommand::CmdReadAddressLatch:
        return cmd_read_address_latch();
    case ServerCommand::CmdReadStatus:
        return cmd_read_status();
    case ServerCommand::CmdRead8288Command:
        return cmd_read_8288_command();
    case ServerCommand::CmdRead8288Control:
        return cmd_read_8288_control();
    case ServerCommand::CmdReadDataBus:
        return cmd_read_data_bus();
    case ServerCommand::CmdWriteDataBus:
        return cmd_write_data_bus();
    case ServerCommand::CmdFinalize:
        return cmd_finalize();
    case ServerCommand::CmdBeginStore:
        return cmd_begin_store();
    case ServerCommand::CmdStore:
        return cmd_store();
    case ServerCommand::CmdQueueLen:
        return cmd_queue_len();
    case ServerCommand::CmdQueueBytes:
        return cmd_queue_bytes();
    case ServerCommand::CmdWritePin:
        return cmd_write_pin();
    case ServerCommand::CmdReadPin:
        return cmd_read_pin();
    case ServerCommand::CmdGetProgramState:
        return cmd_get_program_state();
    case ServerCommand::CmdLastError:
        return cmd_get_last_error();
    case ServerCommand::CmdGetCycleState:
        return cmd_get_cycle_state();
    case ServerCommand::CmdAvailable00:
        return cmd_null();
    case ServerCommand::CmdPrefetchStore:
        return cmd_prefetch_store();
    case ServerCommand::CmdReadAddress:
        return cmd_read_address();
    case ServerCommand::CmdCpuType:
        return cmd_cpu_type();
    case ServerCommand::CmdSetFlags:
        return cmd_set_flags();
    case ServerCommand::CmdPrefetch:
        return cmd_prefetch();
    case ServerCommand::CmdInitScreen:
        return cmd_init_screen();
    case ServerCommand::CmdStoreAll:
        return cmd_storeall();        
    case ServerCommand::CmdSetRandomSeed:
        return cmd_set_random_seed();
    case ServerCommand::CmdRandomizeMem:
        return cmd_randomize_mem();
    case ServerCommand::CmdSetMemory:
        return cmd_set_memory();
    case ServerCommand::CmdGetCycleStates:
        return cmd_get_cycle_states();
    case ServerCommand::CmdEnableDebug:
        return cmd_enable_debug();
    case ServerCommand::CmdSetMemoryStrategy:
        return cmd_set_memory_strategy();
    case ServerCommand::CmdGetFlags:
        return cmd_get_flags();
    case ServerCommand::CmdReadMemory:
        return cmd_read_memory();
    case ServerCommand::CmdEraseMemory:
        return cmd_erase_memory();
    case ServerCommand::CmdServerStatus:
        return cmd_server_status();
    case ServerCommand::CmdClearCycleLog:
        return cmd_clear_cycle_log();
    case ServerCommand::CmdSetProgramBounds:
        return cmd_set_program_bounds();      
    case ServerCommand::CmdSetJumpHint:
        return cmd_set_jump_hint();  
    case ServerCommand::CmdInvalid:
    default:
        return cmd_invalid();
  }
}

/// @brief Handle the version command. This is the first command sent upon opening a serial port to discover an ArduinoX86 server.
/// It sends an identification string, and protocol version number.
/// @tparam BoardType 
/// @tparam ShieldType 
/// @return Always returns true.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_version() {
  debug_cmd("In cmd");

  const char msg[] = "ardx86 ";
  proto_write((const uint8_t *)msg, sizeof(msg) - 1);
  //INBAND_SERIAL.write((uint8_t *)VERSION_DAT, VERSION_DAT_LEN);
  proto_write(VERSION_NUM);
  proto_flush();
  delay(10);  // let USB complete the transaction
  
  controller_.getBoard().debugPrintln(DebugType::CMD, "Got version query!");
  return true;
}

/// @brief Get the number of input bytes expected from the client for a given command
/// @tparam BoardType 
/// @tparam ShieldType 
/// @param cmd 
/// @return The number of input bytes expected for the command
template<typename BoardType, typename ShieldType>
uint8_t CommandServer<BoardType, ShieldType>::get_command_input_bytes(ServerCommand cmd) {
    switch(cmd) {
        case ServerCommand::CmdNone: return 0;
        case ServerCommand::CmdVersion: return 0;
        case ServerCommand::CmdResetCpu: return 0;
        case ServerCommand::CmdLoad: return 1;  // Parameter: Type of register file to load
        case ServerCommand::CmdCycle: return 1; // Parameter: Number of cycles to execute
        case ServerCommand::CmdReadAddressLatch: return 0;
        case ServerCommand::CmdReadStatus: return 0;
        case ServerCommand::CmdRead8288Command: return 0;
        case ServerCommand::CmdRead8288Control: return 0;
        case ServerCommand::CmdReadDataBus: return 0;
        case ServerCommand::CmdWriteDataBus: return 2; // Parameter: 16-bit value to write
        case ServerCommand::CmdFinalize: return 0;
        case ServerCommand::CmdBeginStore: return 0;
        case ServerCommand::CmdStore: return 0;
        case ServerCommand::CmdQueueLen: return 0;
        case ServerCommand::CmdQueueBytes: return 0;
        case ServerCommand::CmdWritePin: return 2; // Parameters: Pin to read, value to write
        case ServerCommand::CmdReadPin: return 1;  // Parameter: Pin to read
        case ServerCommand::CmdGetProgramState: return 0;
        case ServerCommand::CmdLastError: return 0;
        case ServerCommand::CmdGetCycleState: return 1; // Parameter: Flags. Bit 0 set to 1 will cycle CPU first
        case ServerCommand::CmdAvailable00: return 0;  // Null
        case ServerCommand::CmdPrefetchStore: return 0;
        case ServerCommand::CmdReadAddress: return 0;
        case ServerCommand::CmdCpuType: return 0;
        case ServerCommand::CmdSetFlags: return 4; // Parameter: uint32_t flags to set
        case ServerCommand::CmdPrefetch: return 0;
        case ServerCommand::CmdInitScreen: return 0;
        case ServerCommand::CmdStoreAll: return 0;
        case ServerCommand::CmdSetRandomSeed: return 4; // Parameter: uint32_t seed for randomization
        case ServerCommand::CmdRandomizeMem: return 4; // Parameter: uint32_t seed for randomization
        case ServerCommand::CmdSetMemory: return 8; // Parameters: address (4 bytes) and size (4 bytes).
        case ServerCommand::CmdGetCycleStates: return 0; 
        case ServerCommand::CmdEnableDebug: return 1; // Parameter: 0 to disable debug, 1 to enable debug
        case ServerCommand::CmdSetMemoryStrategy: return 9; // Parameters: Strategy (1 byte), start_addr (4 bytes), end_addr (4 bytes).
        case ServerCommand::CmdGetFlags: return 0;
        case ServerCommand::CmdReadMemory: return 8; // Parameters: address (4 bytes) and size (4 bytes).
        case ServerCommand::CmdEraseMemory: return 0;
        case ServerCommand::CmdServerStatus: return 0;
        case ServerCommand::CmdClearCycleLog: return 0; // No parameters needed to clear cycle log
        case ServerCommand::CmdSetProgramBounds: return 8; // Parameters: start_addr (4 bytes), end_addr (4 bytes).
        case ServerCommand::CmdSetJumpHint: return 2; // Parameter. First byte: 0 (clear hint) (1) set hint. 2nd byte: 0 (even address), 1 (odd address)
        case ServerCommand::CmdInvalid: return 0;
        default: return 0;
    }
}

template<typename BoardType, typename ShieldType>
void CommandServer<BoardType, ShieldType>::change_state(ServerState new_state) {

  // Leave current state.
  switch (state_) {
    case ServerState::CpuId: // FALLTHROUGH
    case ServerState::JumpVector:
      CPU.program->reset();
      break;
    case ServerState::ExecuteFinalize:
      NMI_VECTOR.reset();
      CPU.nmi_checkpoint = 0;
      CPU.nmi_buf_cursor = 0;
      break;
    case ServerState::Load:
      CPU.program->reset();
      CPU.loadall_checkpoint = 0;
      break;
    default:
      break;
  }

  executing_ = false;

  // Enter new state.
  switch (new_state) {
    case ServerState::Reset:
      CPU.doing_reset = true;
      CPU.cpuid_counter = 0;
      CPU.cpuid_queue_reads = 0;
      break;
    case ServerState::CpuSetup:
      CPU.program = &SETUP_PROGRAM;
      CPU.program->reset();
      CPU.v_pc = 0;
      break;
    case ServerState::CpuId:
      CPU.program = &CPUID_PROGRAM;
      CPU.program->reset();
      CPU.doing_reset = false;
      CPU.doing_id = true;
      CPU.cpuid_counter = 0;
      CPU.cpuid_queue_reads = 0;
      break;
    case ServerState::JumpVector:
      CPU.program = &JUMP_VECTOR;
      CPU.program->reset();
      CPU.doing_reset = false;
      break;
    case ServerState::Load:
      CPU.wait_states = 1;
      CPU.wait_state_ct = 0;
      CPU.loadall_checkpoint = 0;
      if (CPU.cpu_type == CpuType::i80286) {
        // Use LOADALL instead of load program on 286.
        CPU.program = &LOAD_PROGRAM_286;
        CPU.program->reset();
      }
      else if (CPU.cpu_type == CpuType::i80386) {
        // Use LOADALL instead of load program on 386.
        CPU.program = &LOAD_PROGRAM_386;
        CPU.program->reset();
      }
      else {
        CPU.program = &LOAD_PROGRAM;
        CPU.program->set_pc(2); // Set pc to 2 to skip flag bytes
      }
      break;
    case ServerState::LoadSmm:
      CPU.loadall_checkpoint = 0;
      if (CPU.cpu_type == CpuType::i80386) {
        // Use LOADALL instead of load program on 386.
        CPU.program = &LOAD_PROGRAM_SMM_386;
        CPU.program->reset();
      }
      else {
        controller_.getBoard().debugPrintln(DebugType::ERROR, "LoadSmm state invalid for this CPU.");
      }
      break;
    case ServerState::LoadDone:
      break;
    case ServerState::EmuEnter:
      CPU.stack_r_op_ct = 0;
      CPU.stack_w_op_ct = 0;
      CPU.program = &EMU_ENTER_PROGRAM;
      CPU.program->set_pc(4); // Set v_pc to 4 to skip IVT segment:offset
      break;
    case ServerState::Execute:
      // Reset cycle logger.
      ArduinoX86::Bus->reset_logging();
      ArduinoX86::CycleLogger->reset();
      ArduinoX86::CycleLogger->enable_logging();
      ArduinoX86::Bus->enable_logging();
      CPU.predicted_fetch = 0;
      CPU.exception_stage = 0;
      CPU.exception_armed = false;
      CPU.execute_cycle_ct = 0;
      CPU.smi_checkpoint = 0;
      CPU.nmi_checkpoint = 0;
      CPU.program->reset();
      if (CPU.do_emulation) {
        // Set v_pc to 4 to skip IVT segment:offset
        CPU.program->set_pc(4);
      }
      executing_ = true;
      break;
    case ServerState::ExecuteFinalize:
      NMI_VECTOR.reset();
      CPU.smi_checkpoint = 0;
      CPU.nmi_checkpoint = 0;
      CPU.nmi_buf_cursor = 0;  // Reset cursor for NMI stack buffer storage

      if (CPU.in_emulation) {
        CPU.program = &EMU_EXIT_PROGRAM;
      } 
      else if (CPU.nmi_terminate) {
        if (CPU.cpu_type == CpuType::i80386) {
          CPU.program = &STORE_PROGRAM_NMI_386;
        } 
        else {
          // Use STORE_PROGRAM_NMI
          CPU.program = &STORE_PROGRAM_NMI;
        } 
      }
      else {
        CPU.program = &STORE_PROGRAM_INLINE;
      }
      CPU.program->reset();
      executing_ = true;
      break;
    case ServerState::ExecuteDone:
      executing_ = true;
      break;
    case ServerState::EmuExit:
      CPU.stack_r_op_ct = 0;
      CPU.stack_w_op_ct = 0;
      CPU.program->reset();
      break;
    case ServerState::Store:
      reverse_stack_buf();
      CPU.nmi_buf_cursor = 0;  // Reset cursor for NMI stack buffer storage
      // Take a raw uint8_t pointer to the register struct. Both x86 and Arduino are little-endian,
      // so we can write raw incoming data over the struct. Faster than logic required to set
      // specific members.
      CPU.readback_p = (uint8_t *)&CPU.post_regs;
      executing_ = true;
      break;
    case ServerState::StoreAll:
      CPU.wait_states = 2;
      if (CPU.cpu_type == CpuType::i80386) {
        // Use STOREALL for 386.
        CPU.program = &STOREALL_PROGRAM_386;
      }
      else {
        CPU.program = &STOREALL_PROGRAM;
      }
      CPU.program->reset();
      executing_ = true;
      break;
    case ServerState::StoreDone:
      executing_ = false;
      break;  
    case ServerState::StoreDoneSmm:
      executing_ = false;
      break;
    case ServerState::Done:
      executing_ = false;
      break;
    case ServerState::Shutdown:
      CPU.error_cycle_ct = 0;
      controller_.getBoard().debugPrintln(DebugType::ERROR, "Entering shutdown state. Please reset the CPU.");
      executing_ = false;
      break;
    case ServerState::Error:
      CPU.error_cycle_ct = 0;
      controller_.getBoard().debugPrintln(DebugType::ERROR, "Entering error state. Please reset the CPU.");
      executing_ = false;
      break;
    default:
      controller_.getBoard().debugPrint(DebugType::ERROR, "Unhandled state change to: ");
      controller_.getBoard().debugPrintln(DebugType::ERROR, get_state_string(new_state));
      executing_ = false;
      // Unhandled state.
      break;
  }

  uint32_t state_end_time = micros();

  // Report time we spent in the previous state.
  if (stateBeginTime_ != 0) {
    uint32_t elapsed = state_end_time - stateBeginTime_;
    controller_.getBoard().debugPrintf(DebugType::STATE, false, 
      "## Changing to state: %s. Spent %lu us in previous state. ##\n\r", 
      get_state_string(new_state), elapsed);
  }
  else {
    controller_.getBoard().debugPrintf(DebugType::STATE, false, 
      "## Changing to state: %s.\n\r", 
      get_state_string(new_state));
  }

  stateBeginTime_ = micros();
  state_ = new_state;
}


// Server command - Reset
// Attempt to reset the CPU and report status.
// This will be rarely used by itself as the register state is not set up. The Load
// command will reset the CPU and set register state.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_reset_cpu() {
  debug_cmd("In cmd_reset()");
  CpuResetResult result;
  clear_error();

  
  result = controller_.resetCpu();
  CPU.reset(result, true);
  if (result.success) {
    CPU.have_queue_status = result.queueStatus;
    change_state(ServerState::Execute);
  }
  return result.success;
}

// Server command - Cpu type
// Return the detected CPU type and the queue status availability bit in MSB
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_cpu_type() {
  debug_cmd("In cmd_cpu_type()");
  clear_error();

  uint8_t byte = (uint8_t)CPU.cpu_type;
  // Set queue status available bit
  if (CPU.have_queue_status) {
    byte |= 0x80;
  }

  // Set FPU present bit
  if (CPU.fpu_type != FpuType::noFpu) {
    byte |= 0x40;
  }

  proto_write(byte);
  return true;
}

// Server command - Cycle
// Execute the specified number of CPU cycles.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_cycle() {
  uint8_t cycle_ct = commandBuffer_[0];
  for (uint8_t i = 0; i < cycle_ct; i++) {
    cycle();
  }
  return true;
}

// Server command - Load
// Load the specified register state into the CPU.
// 
// This command takes one byte first, which indicates the type of registers to load.
// On 8088-80186, this command takes 28 bytes, which correspond to the word values of each of the 14
// CPU registers.
// On 80286, this command takes 102 bytes, which correspond to the LOADALL structure.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_load() {

  clear_error();
  volatile uint8_t *read_p = nullptr;
  uint8_t reg_type = commandBuffer_[0];
  bool read_result = false;
  bool reset_cpu = true;
  switch (reg_type) {
    case 0:
      Controller.getBoard().debugPrintln(DebugType::LOAD, "## cmd_load(): Reading register struct type: 8088-80186");
      // 8088-80186 register load
      // This is the default register load type.
      read_result = readParameterBytes(commandBuffer_, sizeof(commandBuffer_), sizeof(registers1_t));

      if (!read_result) {
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## cmd_load(): Timed out reading parameter bytes");
        set_error("Failed to read parameter bytes");
        return false;
      }

      // Write raw command bytes over register struct.
      // All possible bit representations are valid.
      read_p = reinterpret_cast<volatile uint8_t*>(&CPU.load_regs);
      for (size_t i = 0; i < sizeof(registers1_t); i++) {
        *read_p++ = commandBuffer_[i];
      }

      patch_load_pgm(&LOAD_PROGRAM, &CPU.load_regs);
      patch_brkem_pgm(&EMU_ENTER_PROGRAM, &CPU.load_regs);

      CPU.load_regs.flags &= CPU_FLAG_DEFAULT_CLEAR_8086;
      CPU.load_regs.flags |= CPU_FLAG_DEFAULT_SET_8086;
      break;

    case 1:
      Controller.getBoard().debugPrintln(DebugType::LOAD, "## cmd_load(): Reading register struct type: 80286 (LOADALL)");
      read_result = readParameterBytes(commandBuffer_, sizeof(commandBuffer_), sizeof(Loadall286));
      if (!read_result) {
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## cmd_load(): Timed out reading parameter bytes");
        set_error("Failed to read parameter bytes");
        return false;
      }

      // Write raw command bytes over register struct.
      read_p = reinterpret_cast<volatile uint8_t*>(&CPU.loadall_regs_286);
      for (size_t i = 0; i < sizeof(Loadall286); i++) {
        *read_p++ = commandBuffer_[i];
      }

      CPU.loadall_regs_286.flags &= CPU_FLAG_DEFAULT_CLEAR_286;
      CPU.loadall_regs_286.flags |= CPU_FLAG_DEFAULT_SET_286;
      break;

    case 2:
      Controller.getBoard().debugPrintln(DebugType::LOAD, "## cmd_load(): Reading register struct type: 80386 (LOADALL)");
      read_result = readParameterBytes(commandBuffer_, sizeof(commandBuffer_), sizeof(Loadall386));
      if (!read_result) {
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## cmd_load(): Timed out reading parameter bytes");
        set_error("Failed to read parameter bytes");
        return false;
      }

      // Write raw command bytes over register struct.
      read_p = reinterpret_cast<volatile uint8_t*>(&CPU.loadall_regs_386);
      for (size_t i = 0; i < sizeof(Loadall386); i++) {
        *read_p++ = commandBuffer_[i];
      }

      CPU.loadall_regs_386.eflags &= CPU_FLAG_DEFAULT_CLEAR_386;
      CPU.loadall_regs_386.eflags |= CPU_FLAG_DEFAULT_SET_386;
      break;

    case 3:
      {
        // SMM register load for 386. We must be in StoreDoneSmm state!
        if (state_ != ServerState::StoreDoneSmm) {
          ArduinoX86::Server.change_state(ServerState::Error);
          set_error("SMM register load requires StoreDoneSmm state");
          return false;
        }

        Controller.getBoard().debugPrintf(
          DebugType::LOAD, 
          false, 
          "## cmd_load(): Reading register struct type: 80386 (SMM), size: %ld\n\r", 
          sizeof(SmmDump386)
        );
        read_result = readParameterBytes(commandBuffer_, sizeof(commandBuffer_), sizeof(SmmDump386));
        if (!read_result) {
          Controller.getBoard().debugPrintln(DebugType::ERROR, "## cmd_load(): Timed out reading parameter bytes");
          set_error("Failed to read parameter bytes");
          return false;
        }

        SmmDump386* smm_dump = &ArduinoX86::Bus->smm_dump386_regs();

        // Write raw command bytes over register struct.
        memcpy((void *)smm_dump, (void*)commandBuffer_, sizeof(SmmDump386));

        // Unlike other register loads, we do not reset the CPU when leaving SMM.
        reset_cpu = false;
        ArduinoX86::Server.change_state(ServerState::LoadSmm);
      }
      break;
      
    default:
      set_error("Invalid register type");
      return false;
      break;
  }

  if (reset_cpu) {
    ArduinoX86::Server.change_state(ServerState::Reset);
    CpuResetResult result = Controller.resetCpu();
    CPU.reset(result, true);
    if (!result.success) {
      //set_error("Failed to reset CPU");
      Controller.getBoard().debugPrintln(DebugType::ERROR, "Failed to reset CPU!");
      return false;
    }
    Controller.getBoard().debugPrintln(DebugType::LOAD, "## cmd_load(): Successfully reset CPU...");
    CPU.have_queue_status = result.queueStatus;

#if USE_SETUP_PROGRAM
    change_state(ServerState::CpuSetup);
#else
    change_state(ServerState::JumpVector);
#endif
  }

  // Run CPU and wait for load to finish
  int load_timeout = 0;
  while ((state_ != ServerState::Execute) && (state_ != ServerState::Error)) {
    cycle();
    load_timeout++;

    if (load_timeout > LOAD_TIMEOUT) {
      // Something went wrong in load program
      Controller.getBoard().debugPrintf(
        DebugType::ERROR, 
        false, 
        "## cmd_load(): Load timeout after %d cycles!  Address latch: %08X\n\r", 
        LOAD_TIMEOUT, 
        CPU.address_latch()
      );
      change_state(ServerState::Error);
      set_error("Load timeout");
      return false;
    }
  }

#if LOAD_INDICATOR
  DEBUG_SERIAL.print(".");
#endif

  Controller.getBoard().debugPrintf(DebugType::LOAD, false, "## cmd_load(): Load done after %d cycles!\n\r", load_timeout);
  debug_proto("LOAD DONE");
  return true;
}

// Server command - ReadAddressLatch
// Read back the contents of the address latch as a sequence of 3 bytes (little-endian)
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_address_latch() {
  INBAND_SERIAL.write((uint8_t)(CPU.address_latch() & 0xFF));
  INBAND_SERIAL.write((uint8_t)((CPU.address_latch() >> 8) & 0xFF));
  INBAND_SERIAL.write((uint8_t)((CPU.address_latch() >> 16) & 0xFF));
  return true;
}

// Server command - ReadAddress
// Read back the contents of the address bus as a sequence of 3 bytes (little-endian)
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_address() {
  //read_address_pins(true);
  CPU.address_bus = Controller.readAddressBus(true);
  INBAND_SERIAL.write((uint8_t)(CPU.address_bus & 0xFF));
  INBAND_SERIAL.write((uint8_t)((CPU.address_bus >> 8) & 0xFF));
  INBAND_SERIAL.write((uint8_t)((CPU.address_bus >> 16) & 0xFF));
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_invalid() {
  DEBUG_SERIAL.println("Called cmd_invalid!");
  return false;
}

// Server command - ReadStatus
// Return the value of the CPU status lines S0-S5 and QS0-QS1
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_status() {
  CPU.status0 = Controller.readCpuStatusLines();
  INBAND_SERIAL.write(CPU.status0);
  return true;
}

// Server command - Read8288Command
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_8288_command() {
  CPU.command_bits = Controller.readBusControllerCommandLines();
  //read_8288_command_bits();
  INBAND_SERIAL.write(CPU.command_bits);
  return true;
}

// Server command - Read8288Control
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_8288_control() {
  Controller.readBusControllerControlLines();
  //read_8288_control_bits();
  INBAND_SERIAL.write(CPU.control_bits);
  return true;
}

// Server command - ReadDataBus
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_data_bus() {
  INBAND_SERIAL.write((uint8_t)CPU.data_bus);
  INBAND_SERIAL.write((uint8_t)(CPU.data_bus >> 8));
  return true;
}

// Server command - WriteDataBus
// Takes an argument of 2 bytes.
// Sets the data bus to the provided value. On 8-bit CPUs the upper byte is ignored.
// This should not be called for CODE fetches after we have called cmd_prefetch_store(),
// unless a flow control operation occurs that flushes the queue and returns us to
// within original program boundaries.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_write_data_bus() {
  if (CPU.bus_state_latched == CODE) {
    // We've just been instructed to write a normal fetch byte to the bus.
    // If we were prefetching the store program, reset this status as a queue
    // flush must have executed (or we goofed up...)
    CPU.prefetching_store = false;
    CPU.s_pc = 0;
  }

  CPU.data_bus = (uint16_t)commandBuffer_[0];
  CPU.data_bus |= ((uint16_t)commandBuffer_[1] << 8);
  CPU.data_type = QueueDataType::Program;

  Controller.getBoard().debugPrintf(DebugType::CMD, false, "## cmd_write_data_bus(): Writing to data bus: %04X\n\r", CPU.data_bus);
  Controller.writeDataBus(CPU.data_bus, ActiveBusWidth::Sixteen);
  CPU.data_bus_resolved = true;
  return true;
}

// Server command - PrefetchStore
// Instructs the CPU server to load the next byte of the Store (or EmuExit) program early
// Should be called in place of cmd_write_data_bus() by host on T3/TwLast when
// program bytes have been exhausted.
// (When we are prefetching past execution boundaries during main program execution)
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_prefetch_store() {

  if (CPU.in_emulation) {
    // Prefetch the EmuExit program
    if (CPU.s_pc >= sizeof EMU_EXIT_PROGRAM) {
      set_error("EmuExit program underflow");
      return false;
    }

#if DEBUG_STORE
    debugPrintColor(ansi::yellow, "## PREFETCH_EMU_EXIT: s_pc: ");
#endif

    CPU.prefetching_store = true;
    CPU.data_bus = EMU_EXIT_PROGRAM.read(CPU.address_latch(), CPU.data_width);
    CPU.data_type = QueueDataType::ProgramEnd;
  } else {
    // Prefetch the Store program
    if (!STORE_PROGRAM_INLINE.has_remaining()) {
      set_error("## Store program underflow!");
      return false;
    }

#if DEBUG_STORE
    debugPrintColor(ansi::yellow, "## PREFETCH_STORE: s_pc: ");
#endif

    CPU.prefetching_store = true;
    CPU.data_bus = STORE_PROGRAM_INLINE.read(CPU.address_latch(), CPU.data_width);
    CPU.data_type = QueueDataType::ProgramEnd;
  }

#if DEBUG_STORE
  debugPrintColor(ansi::yellow, CPU.s_pc);
  debugPrintColor(ansi::yellow, " addr: ");
  debugPrintColor(ansi::yellow, CPU.address_latch(), 16);
  debugPrintColor(ansi::yellow, " data: ");
  debugPrintlnColor(ansi::yellow, CPU.data_bus, 16);
#endif

  return true;
}

// Server command - Finalize
// Sets the data bus flag to DATA_PROGRAM_END, so that the Execute state can terminate
// on the next instruction queue fetch
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_finalize() {
  if (state_ == ServerState::Execute) {
    change_state(ServerState::ExecuteFinalize);

    // Wait for execute done state
    int execute_ct = 0;
    int timeout = FINALIZE_TIMEOUT;
    if (CPU.in_emulation) {
      // We need more time to exit emulation mode
      timeout = FINALIZE_EMU_TIMEOUT;
    }
    while (state_ != ServerState::ExecuteDone) {
      cycle();
      execute_ct++;

      if (execute_ct > timeout) {
        set_error("cmd_finalize(): state timeout");
        return false;
      }
    }
    return true;
  } else {
    error_beep();
    set_error("cmd_finalize(): wrong state: %s", get_state_string(state_));
    controller_.getBoard().debugPrint(DebugType::ERROR, "cmd_finalize(): wrong state: ");
    controller_.getBoard().debugPrintln(DebugType::ERROR, get_state_string(state_));
    return false;
  }
}

// Server command - BeginStore
// Execute state must be in ExecuteDone before intiating BeginStore command
//
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_begin_store(void) {
  /*
  char err_msg[30];

  // Command only valid in ExecuteDone state
  if(state_ != ExecuteDone) {
    snprintf(err_msg, 30, "BeginStore: Wrong state: %d ", state_);
    set_error(err_msg);
    return false;
  }

  change_state(Store);
  */
  return true;
}

// Server command - Store
//
// Returns values of registers in the following order, little-endian
// AX, BX, CX, DX, SS, SP, FLAGS, IP, CS, DS, ES, BP, SI, DI
// Execute state must be in ExecuteDone before executing Store command
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_store(void) {

  size_t write_len = 0;

  if (flags_ & FLAG_EXECUTE_AUTOMATIC) {
    // In automatic mode, Store command is only valid in StoreDone state.
    if ((state_ != ServerState::StoreDone) && (state_ != ServerState::StoreDoneSmm)) {
      controller_.getBoard().debugPrintf(DebugType::ERROR, false, "## STORE: Wrong state: %s", get_state_string(state_));
      set_error("## STORE: Wrong state: %s", get_state_string(state_));
      return false;
    }

    switch (CPU.cpu_type) {
      case CpuType::i80286:
        {
          // If we are in automatic mode, we can just return the stored registers
          // without executing the Store program.
          INBAND_SERIAL.write((uint8_t)0x01); // Send 0x01 to indicate V2 register format.
          Loadall286 regs = ArduinoX86::Bus->loadall286_regs();
          // Patch the registers with the call stack frame from NMI.
          if (CPU.nmi_terminate) {
            // If we terminated with NMI, we need to patch the registers with the NMI call stack frame.
            controller_.getBoard().debugPrintln(DebugType::STORE, "## STORE: Patching registers with NMI call stack frame...");
            regs.patch_stack_frame(CPU.nmi_stack_frame);
          }
          // Dump the raw byte representation of the registers to the serial port.
          uint8_t *reg_p = (uint8_t *)&regs;
          INBAND_SERIAL.write(reg_p, sizeof(Loadall286));
        }
        return true;
        break;
      case CpuType::i80386:
        {
          if (useSmm_) {
            // Send 3 to indicate V3B register format.
            INBAND_SERIAL.write((uint8_t)3);
            // Write the registers in the V3B format.
            SmmDump386 smm386 = ArduinoX86::Bus->smm_dump386_regs();
            smm386.normalize_flags();
            write_len = INBAND_SERIAL.write((uint8_t *)&smm386, sizeof(SmmDump386));
            controller_.getBoard().debugPrintf(
              DebugType::STORE, 
              false, 
              "## STORE: Wrote %d bytes of registers in V3B format.\n\r", 
              write_len
            );
          }
          else {
            // Send 2 to indicate V3A register format.
            INBAND_SERIAL.write((uint8_t)2);
            // Write the registers in the V3A format.
            Loadall386 regs368 = ArduinoX86::Bus->loadall386_regs();
            write_len = INBAND_SERIAL.write((uint8_t *)&regs368, sizeof(Loadall386));
            controller_.getBoard().debugPrintf(
              DebugType::STORE, 
              false, 
              "## STORE: Wrote %d bytes of registers in V3A format.\n\r", 
              write_len
            );
          }

        }
        return true;
        break;
      default:
        controller_.getBoard().debugPrintf(DebugType::ERROR, false, "## STORE: Unsupported CPU model for automatic mode");
        return false;
    }
    return true;
  }
  else {
    // In non-automatic mode, Store Command is only valid in ExecuteDone state
    if (state_ != ServerState::ExecuteDone) {
      controller_.getBoard().debugPrintf(DebugType::ERROR, false, "## STORE: Wrong state: %s", get_state_string(state_));
      set_error("## STORE: Wrong state: %s", get_state_string(state_));
      return false;
    }
  }

  change_state(ServerState::Store);

  int store_timeout = 0;

  // Cycle CPU until Store complete
  while (state_ != ServerState::StoreDone) {
    cycle();
    store_timeout++;

    if (store_timeout > STORE_TIMEOUT) {
      controller_.getBoard().debugPrintf(DebugType::ERROR, false, "## STORE: Timeout! ##");
      set_error("StoreDone timeout.");
      error_beep();
      return false;
    }
  }

  controller_.getBoard().debugPrintf(DebugType::STORE, false, "## STORE: Flags are: %04X\n\r", CPU.post_regs.flags);

  if (!CPU.nmi_terminate) {
    // We didn't use NMI to terminate code. which means we used the inline STORE routine,
    // which means we need to convert the registers struct.
    controller_.getBoard().debugPrintln(DebugType::STORE, "## STORE: Converting registers from STORE INLINE format...");
    convert_inline_registers(&CPU.post_regs);
  }

  Loadall386 regs368;

  switch (CPU.cpu_type) {
    case CpuType::i8088:
    case CpuType::i80186:
    case CpuType::i80286:
      // Send 0 to indicate V1 register format.
      INBAND_SERIAL.write((uint8_t)0);
      // Write the registers in the V1 format.
      INBAND_SERIAL.write((uint8_t *)&CPU.post_regs, sizeof(registers1_t));
      break;

    case CpuType::i80386:
      // Send 2 to indicate V3 register format.
      INBAND_SERIAL.write((uint8_t)2);
      // Write the registers in the V2 format.
      regs368 = ArduinoX86::Bus->loadall386_regs();
      INBAND_SERIAL.write((uint8_t *)&regs368, sizeof(Loadall386));
      controller_.getBoard().debugPrintf(DebugType::STORE, false, "## STORE: Wrote %d bytes of registers in V3 format.\n\r", sizeof(Loadall386));
      break;

    default:
      // Unknown CPU type?
      change_state(ServerState::Error);
      controller_.getBoard().debugPrintln(DebugType::ERROR, "## STORE: Unknown CPU type!");
      set_error("Unknown CPU type: %d", (int)CPU.cpu_type);
      break;
  }

#if STORE_INDICATOR
  DEBUG_SERIAL.print("?");
#endif
  change_state(ServerState::StoreDone);
  return true;
}


// Server command - QueueLen
// Return the length of the instruction queue in bytes
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_queue_len(void) {
  INBAND_SERIAL.write(static_cast<uint8_t>(CPU.queue.len()));
  return true;
}

// Server command - QueueBytes
// Return the contents of the instruction queue, from 0-6 bytes.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_queue_bytes(void) {
  for (size_t i = 0; i < CPU.queue.len(); i++) {
    INBAND_SERIAL.write(static_cast<uint8_t>(CPU.queue.read_byte(i)));
  }
  return true;
}

// Server command - Write pin
// Sets the value of the specified CPU input pin
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_write_pin(void) {

  uint8_t pin_idx = commandBuffer_[0];
  uint8_t pin_val = commandBuffer_[1] & 0x01;

#if defined(SHIELD_286_5V_V1)
  if (pin_idx == 4) {
    Controller.writePin(OutputPin::Error, pin_val);
    return true;
  }
#endif

  if (pin_idx < 4) {
    //uint8_t pin_no = WRITE_PINS[pin_idx];

    switch (pin_idx) {
      case 0: // READY pin
      {
        controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_write_pin(): Setting READY pin to: %d\n\r", pin_val);
        Controller.writePin(OutputPin::Ready, pin_val);

        bool new_value = Controller.readPin(OutputPin::Ready);
        controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_write_pin(): READY pin now reads as: %d\n\r", new_value ? 1 : 0);
        if (new_value != (pin_val != 0)) {
          controller_.getBoard().debugPrintln(DebugType::ERROR, "cmd_write_pin(): WARNING: READY pin readback does not match written value!");
        }
        break;
      }
      case 1: // TEST pin
        controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_write_pin(): Setting TEST pin to: %d\n\r", pin_val);
        Controller.writePin(OutputPin::Test, pin_val);
        break;

      case 2: // INTR pin
        controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_write_pin(): Setting INTR pin to: %d\n\r", pin_val);
        Controller.writePin(OutputPin::Intr, pin_val);
        break;

      case 3: // NMI pin
        controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_write_pin(): Setting NMI pin to: %d\n\r", pin_val);
        Controller.writePin(OutputPin::Nmi, pin_val);
        break;

      default:
        error_beep();
        return false;
    }
    return true;
  } 
  else {
    // Invalid pin
    Controller.getBoard().debugPrintln(DebugType::ERROR, "cmd_write_pin(): Invalid pin index");
    error_beep();
    return false;
  }
}

// Server command - Read pin
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_pin(void) {
  uint8_t pin_idx = commandBuffer_[0];
#if defined(SHIELD_286_5V_V1)
  if (pin_idx == 4) {
    proto_write(uint8_t(Controller.readPin(OutputPin::Error) ? 1 : 0));
    return true;
  }
#endif
  bool value = false;
  bool state = true;

  switch (pin_idx) {
    case 0: // READY pin
      value = Controller.readPin(OutputPin::Ready);
      Controller.getBoard().debugPrintf(DebugType::CMD, false, "cmd_read_pin(): Reading READY pin state: %d\n\r", value ? 1 : 0);
      break;

    case 1: // TEST pin
      value = Controller.readPin(OutputPin::Test);
      break;

    case 2: // INTR pin
      value = Controller.readPin(OutputPin::Intr);
      break;

    case 3: // NMI pin
      value = Controller.readPin(OutputPin::Nmi);
      break;

    default:
      Controller.getBoard().debugPrintln(DebugType::ERROR, "cmd_read_pin(): Invalid pin index");
      value = false;
      state = false;
  }

  INBAND_SERIAL.write((uint8_t)(value ? 1 : 0));
  return state;
}

// Server command - Get program state
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_get_program_state(void) {
  controller_.getBoard().debugPrintf(
    DebugType::CMD, 
    false,
    "## cmd_get_program_state(): State: %s Raw: %02X\n\r", 
    get_state_string(state_), 
    state_);
  INBAND_SERIAL.write((uint8_t)state_);
  return true;
}

// Server command - Get last error
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_get_last_error(void) {
  INBAND_SERIAL.write(errorBuffer_);
  return true;
}

// Server command - Get Cycle State
// Returns all the state info typically needed for a single cycle
// Parameter: One byte, flags. Bit 0 set to 1 will cycle CPU first before 
//            returning the state.
// Returns 11 bytes
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_get_cycle_state(void) {
  uint8_t send_buf[11] = { 0 };
  bool do_cycle = (bool)(commandBuffer_[0] & 0x01);
  if (do_cycle) {
    // Perform a cycle if requested
    cycle();
  }

  //CPU.status0 = Controller.readCpuStatusLines();
  CPU.command_bits = Controller.readBusControllerCommandLines();
  CPU.control_bits = Controller.readBusControllerControlLines();

  //read_8288_command_bits();
  //read_8288_control_bits();
  uint8_t server_state = ((uint8_t)state_) & 0x3F;
  uint8_t cpu_state_byte = 0;

  cpu_state_byte |= ((uint8_t)(CPU.last_bus_cycle)) & 0x07; // Bits 0-2: Bus cycle

  send_buf[0] = server_state; // Byte 0
  send_buf[1] = cpu_state_byte; // Byte 1
  send_buf[2] = CPU.status0; // Byte 2
  send_buf[3] = CPU.control_bits; // Byte 3
  send_buf[4] = CPU.command_bits; // Byte 4
  send_buf[5] = (uint8_t)(CPU.address_bus & 0xFF); // Bytes 5-8
  send_buf[6] = (uint8_t)((CPU.address_bus >> 8) & 0xFF);
  send_buf[7] = (uint8_t)((CPU.address_bus >> 16) & 0xFF);
  send_buf[8] = (uint8_t)((CPU.address_bus >> 24) & 0xFF);
  send_buf[9] = (uint8_t)(CPU.data_bus & 0xFF); // Bytes 9-10
  send_buf[10] = (uint8_t)(CPU.data_bus >> 8);
  // Send the state bytes
  INBAND_SERIAL.write(send_buf, sizeof(send_buf));
  return true;
}

// Server command - Set flags
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_set_flags(void) {

  uint32_t new_flags = commandBuffer_[0] | 
            (static_cast<uint32_t>(commandBuffer_[1]) << 8) |
            (static_cast<uint32_t>(commandBuffer_[2]) << 16) |
            (static_cast<uint32_t>(commandBuffer_[3]) << 24);

  if (new_flags & CommandServer::FLAG_EMU_8080) {
      // Enable 8080 emulation mode
    if ((CPU.cpu_type == CpuType::necV20) || (CPU.cpu_type == CpuType::necV30)) {
      // Simply toggle the emulation flag
      CPU.do_emulation = true;
      controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling 8080 emulation mode! ##");
    }
    else {
      // Unsupported CPU!
      controller_.getBoard().debugPrintln(DebugType::ERROR, "## cmd_set_flags(): Bad CPU type for emulation flag ## ");
      return false;
    }
  }

  if (new_flags & CommandServer::FLAG_EXECUTE_AUTOMATIC) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling automatic execution ##");
  }
  if (new_flags & CommandServer::FLAG_HALT_AFTER_JUMP) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling halt after jump ##");
  }

  if ((new_flags & CommandServer::FLAG_USE_SDRAM_BACKEND) && !(flags_ & CommandServer::FLAG_USE_SDRAM_BACKEND)) {
    // SDRAM backend is requested, but not currently enabled. Replace backend.
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling SDRAM memory backend ##");
    ArduinoX86::Bus->replace_backend(new SdramBackend(MEMORY_SIZE, ADDRESS_SPACE_MASK));
  }
  else if (!(new_flags & CommandServer::FLAG_USE_SDRAM_BACKEND) && (flags_ & CommandServer::FLAG_USE_SDRAM_BACKEND)) {
    // SDRAM backend is disabled, but currently enabled. Replace backend.
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling HashTable memory backend ##");
    ArduinoX86::Bus->replace_backend(new HashBackend());
  }

  if ((new_flags & CommandServer::FLAG_USE_SMM) && !(flags_ & CommandServer::FLAG_USE_SMM)) {
    // SMM is requested, but not currently enabled. Enable SMM.
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling SMM ##");
    useSmm_ = true;
    CPU.set_use_smm(true);
  }
  else if (!(new_flags & CommandServer::FLAG_USE_SMM) && (flags_ & CommandServer::FLAG_USE_SMM)) {
    // SMM is disabled, but currently enabled. Disable SMM.
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Disabling SMM ##");
    useSmm_ = false;
    CPU.set_use_smm(false);
  }

  if ((new_flags & CommandServer::FLAG_DEBUG_ENABLED) && !(flags_ & CommandServer::FLAG_DEBUG_ENABLED)) {
    controller_.getBoard().setDebugEnabled(true);
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling debug mode");
  } 
  else if (!(new_flags & CommandServer::FLAG_DEBUG_ENABLED) && (flags_ & CommandServer::FLAG_DEBUG_ENABLED)) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Disabling debug mode");
    controller_.getBoard().setDebugEnabled(false);
  }

  if ((new_flags & CommandServer::FLAG_LOG_CYCLES) && !(flags_ & CommandServer::FLAG_LOG_CYCLES)) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling cycle logging ##");
  } 
  else if (!(new_flags & CommandServer::FLAG_LOG_CYCLES) && (flags_ & CommandServer::FLAG_LOG_CYCLES)) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Disabling cycle logging ##");
  }

  if ((new_flags & CommandServer::FLAG_ALE_INTERRUPT) && !(flags_ & CommandServer::FLAG_ALE_INTERRUPT)) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling ALE interrupt ##");
    controller_.setAleInterrupt(true);
    ale_interrupt_enabled_ = true;
  } 
  else if (!(new_flags & CommandServer::FLAG_ALE_INTERRUPT) && (flags_ & CommandServer::FLAG_ALE_INTERRUPT)) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Disabling ALE interrupt ##");
    controller_.setAleInterrupt(false);
    ale_interrupt_enabled_ = false;
  }

  if ((new_flags & CommandServer::FLAG_RESOLVE_BUS_STEP) && !(flags_ & CommandServer::FLAG_RESOLVE_BUS_STEP)) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Enabling bus resolution during step ##");
  } 
  else if (!(new_flags & CommandServer::FLAG_RESOLVE_BUS_STEP) && (flags_ & CommandServer::FLAG_RESOLVE_BUS_STEP)) {
    controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_set_flags(): Disabling bus resolution during step ##");
  }

  flags_ = new_flags;
  return true;
}

// Server command - prefetch
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_prefetch(void) {
  if ((CPU.cpu_type == CpuType::necV20) || (CPU.cpu_type == CpuType::necV30)) {
    // Simply toggle the emulation flag
    CPU.do_prefetch = true;
#if DEBUG_EMU
    DEBUG_SERIAL.println("## cmd_prefetch(): Enabling VX0 prefetch ##");
#endif
    return true;
  }

// Unsupported CPU!
#if DEBUG_EMU
  DEBUG_SERIAL.println("## cmd_prefetch(): Bad CPU type ## ");
#endif
  return false;
}

// The Giga display shield takes a few seconds to initialize, so we don't 
// want to do it in setup. The client can request display initialization instead
// once it has established a connection.  The client should wait approximately 
// 3 seconds after sending the init command.
// Returns one data byte. A value of 0 indicates no display is present.
// A value of 1 indicates the display is present and will be initialized.
template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_init_screen() {
  uint8_t byte0 = 0;
  #if GIGA_DISPLAY_SHIELD
    byte0 = 1;
    screen_init_requested = true;
  #endif
  INBAND_SERIAL.write(byte0);
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_storeall() {
  change_state(ServerState::StoreAll);

  for (int i = 0; i < 300; i++) {
    cycle();
    if (state_ == ServerState::Done) {
      // StoreAll completed
      break;
    }
  }

  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_set_random_seed() {

  uint32_t seed = commandBuffer_[0] | 
                  (static_cast<uint32_t>(commandBuffer_[1]) << 8) |
                  (static_cast<uint32_t>(commandBuffer_[2]) << 16) |
                  (static_cast<uint32_t>(commandBuffer_[3]) << 24);

  randomSeed(static_cast<unsigned long>(seed));  
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_randomize_mem() {

  uint32_t seed = commandBuffer_[0] | 
                  (static_cast<uint32_t>(commandBuffer_[1]) << 8) |
                  (static_cast<uint32_t>(commandBuffer_[2]) << 16) |
                  (static_cast<uint32_t>(commandBuffer_[3]) << 24);

  unsigned long start_time = millis();
  ArduinoX86::Bus->randomize_memory(seed);
  unsigned long end_time = millis();
  unsigned long elapsed = end_time - start_time;
  controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_randomize_mem(): Memory randomized in %lu ms\n\r", elapsed);
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_set_memory() {
    uint8_t read_buffer[MAX_BUFFER_LEN];
    uint32_t address = commandBuffer_[0] | 
                      (static_cast<uint32_t>(commandBuffer_[1]) << 8) |
                      (static_cast<uint32_t>(commandBuffer_[2]) << 16) |
                      (static_cast<uint32_t>(commandBuffer_[3]) << 24);

    uint32_t size = commandBuffer_[4] | 
                    (static_cast<uint32_t>(commandBuffer_[5]) << 8) |
                    (static_cast<uint32_t>(commandBuffer_[6]) << 16) |
                    (static_cast<uint32_t>(commandBuffer_[7]) << 24);

  controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_set_memory(): Setting memory at address: %06lX with size: %lu\n\r", address, size);

  // Read `size` bytes from the serial stream, MAX_BUFFER_LEN bytes at a time.
  size_t total_bytes_read = 0;
  constexpr unsigned long READ_TIMEOUT = 100; // Timeout for reading data in milliseconds
  unsigned long start_time = millis();
  unsigned long timeout_time = start_time + READ_TIMEOUT;

  while (total_bytes_read < size) {
    size_t bytes_available = proto_available();
    if (bytes_available) {
      size_t bytes_to_read = min(bytes_available, MAX_BUFFER_LEN);

      size_t bytes_read = proto_read(read_buffer, bytes_to_read);
      if (bytes_read == 0) {
        controller_.getBoard().debugPrintf(DebugType::ERROR, false, "cmd_set_memory(): Failed to read available bytes\n\r");
        set_error("cmd_set_memory(): Failed to read available bytes");
        return false;
      }
      ArduinoX86::Bus->set_memory(address + total_bytes_read, read_buffer, bytes_read);
      total_bytes_read += bytes_read;
    }
    else {
      // Check for timeout
      if (millis() >= timeout_time) {
        controller_.getBoard().debugPrintf(DebugType::ERROR, false, "cmd_set_memory(): Timeout waiting for memory data\n\r");
        set_error("cmd_set_memory(): Timeout waiting for memory data");
        return false;
      }
      // No data available, wait a bit before checking again
      delay(1);
    }
  }

  controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_set_memory(): Set %lu bytes of memory successfully\n\r", total_bytes_read);
  //ArduinoX86::Bus->debug_memory(address, total_bytes_read);
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_get_cycle_states() {
  ArduinoX86::CycleLogger->dump_states();
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_enable_debug() {
  bool enabled = static_cast<bool>(commandBuffer_[0]);
  if (enabled) {
    flags_ |= CommandServer::FLAG_DEBUG_ENABLED;
    controller_.getBoard().setDebugEnabled(true);
    controller_.getBoard().debugPrintln(DebugType::CMD, "cmd_enable_debug(): Enabling debug mode");
  } else {
    flags_ &= ~CommandServer::FLAG_DEBUG_ENABLED;
    controller_.getBoard().debugPrintln(DebugType::CMD, "cmd_enable_debug(): Disabling debug mode");
    controller_.getBoard().setDebugEnabled(false);
  }
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_set_memory_strategy() {

  IBusBackend::DefaultStrategy strategy = static_cast<IBusBackend::DefaultStrategy>(commandBuffer_[0]);
  uint32_t start_address = commandBuffer_[1] | 
                    (static_cast<uint32_t>(commandBuffer_[2]) << 8) |
                    (static_cast<uint32_t>(commandBuffer_[3]) << 16) |
                    (static_cast<uint32_t>(commandBuffer_[4]) << 24);
  uint32_t end_address = commandBuffer_[5] | 
                    (static_cast<uint32_t>(commandBuffer_[6]) << 8) |
                    (static_cast<uint32_t>(commandBuffer_[7]) << 16) |
                    (static_cast<uint32_t>(commandBuffer_[8]) << 24);
  if (strategy < IBusBackend::DefaultStrategy::Invalid) {
    ArduinoX86::Bus->set_memory_strategy(strategy, start_address, end_address);
    controller_.getBoard().debugPrintf(DebugType::CMD, false, "## cmd_set_memory_strategy(): Set memory strategy to: %d: %06lX %06lX\n\r", strategy, start_address, end_address);
    set_error("No error");
    return true;
  } else {
    controller_.getBoard().debugPrintf(DebugType::ERROR, false, "## cmd_set_memory_strategy(): Invalid memory strategy: %d\n\r", strategy);
    set_error("Invalid memory strategy: %d", strategy);
    return false;
  }
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_get_flags() {
  proto_write(reinterpret_cast<const uint8_t*>(&flags_), sizeof(flags_));
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_read_memory() {
  uint32_t address = commandBuffer_[0] |
                    (static_cast<uint32_t>(commandBuffer_[1]) << 8) |
                    (static_cast<uint32_t>(commandBuffer_[2]) << 16) |
                    (static_cast<uint32_t>(commandBuffer_[3]) << 24);
  uint32_t size = commandBuffer_[4] |
                  (static_cast<uint32_t>(commandBuffer_[5]) << 8) |
                  (static_cast<uint32_t>(commandBuffer_[6]) << 16) |
                  (static_cast<uint32_t>(commandBuffer_[7]) << 24);

  size_t mem_size = ArduinoX86::Bus->mem_size();
  if (address >= mem_size || (address + size) > mem_size) {
    controller_.getBoard().debugPrintf(
      DebugType::ERROR, 
      false, 
      "## cmd_read_memory(): Invalid address range: %08lX - %08lX. Mem size: %08lX\n\r", 
      address, address + size,
      mem_size
    );
    set_error("Invalid address range: %08lX - %08lX", address, address + size);
    //return false;
  }

  // Special-case 1, 2, and 4 bytes.
  switch (size) {
    case 1:
    {
      uint8_t value = ArduinoX86::Bus->mem_read_u8(address, false);
      controller_.getBoard().debugPrintf(DebugType::CMD, false, "## cmd_read_memory(): Sending 1 byte from address: %08lX to client: %02X\n\r", address, value);
      set_error("No error");
      proto_write((uint8_t *)"\x01", 1);
      proto_write(&value, 1);
      break;
    }
    case 2:
    {
      uint16_t value = ArduinoX86::Bus->mem_read_u16(address, false);
      controller_.getBoard().debugPrintf(DebugType::CMD, false, "## cmd_read_memory(): Sending 2 bytes from address: %08lX to client: %04X\n\r", address, value);
      set_error("No error");
      proto_write((uint8_t *)"\x01", 1);
      proto_write((uint8_t *)&value, 2);
      break;
    }
    case 4:
    {
      uint32_t value0 = ArduinoX86::Bus->mem_read_u16(address, false);
      uint32_t value1 = ArduinoX86::Bus->mem_read_u16(address + 2, false);
      uint32_t value = (value1 << 16) | value0;
      controller_.getBoard().debugPrintf(DebugType::CMD, false, "## cmd_read_memory(): Sending 4 bytes from address: %08lX to client: %08lX\n\r", address, value);
      set_error("No error");
      proto_write((uint8_t *)"\x01", 1);
      proto_write((uint8_t *)&value, 4);
      break;
    }
    default:
    {
      uint8_t *ptr = ArduinoX86::Bus->get_ptr(address);

      if (ptr == nullptr) {
        controller_.getBoard().debugPrintf(DebugType::ERROR, false, "## cmd_read_memory(): Invalid address: %08lX\n\r", address);
        set_error("Invalid address: %08lX", address);
        return false;
      }

      controller_.getBoard().debugPrintf(DebugType::CMD, false, "## cmd_read_memory(): Sending %lu bytes from address: %08lX to client...\n\r", size, address);
      set_error("No error");

      // Send an initial success byte, so that the client knows we are sending data.
      // Otherwise it doesn't know if the command failed and will have to time out.
      proto_write((uint8_t *)"\x01", 1);
      proto_write(ptr, size);
      break;
    }
  }

  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_erase_memory() {
  ArduinoX86::Bus->erase_memory();
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_server_status() {
  // Returns the current server status as:
  // 1 byte: Server state (ServerState enum)
  // 8 bytes: Current cycle count (uint64_t)
  // 4 bytes: Current address latch (uint32_t)
  INBAND_SERIAL.write((uint8_t)state_);
  uint64_t cycle_count = CPU.cycle_ct();
  INBAND_SERIAL.write((uint8_t *)&cycle_count, sizeof(cycle_count));
  uint32_t address_latch = CPU.address_latch();
  INBAND_SERIAL.write((uint8_t *)&address_latch, sizeof(address_latch));
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_clear_cycle_log(){
  ArduinoX86::CycleLogger->reset();
  controller_.getBoard().debugPrintln(DebugType::CMD, "## cmd_clear_cycle_log(): Cycle log cleared.");
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_set_program_bounds() {
    uint32_t start = commandBuffer_[0] | 
                      (static_cast<uint32_t>(commandBuffer_[1]) << 8) |
                      (static_cast<uint32_t>(commandBuffer_[2]) << 16) |
                      (static_cast<uint32_t>(commandBuffer_[3]) << 24);

    uint32_t end = commandBuffer_[4] | 
                    (static_cast<uint32_t>(commandBuffer_[5]) << 8) |
                    (static_cast<uint32_t>(commandBuffer_[6]) << 16) |
                    (static_cast<uint32_t>(commandBuffer_[7]) << 24);

  controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_set_program_bounds(): Setting bounds from start: %08lX to end: %08lX\n\r", start, end);

  CPU.set_program_bounds(start, end);
  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_set_jump_hint() {
  bool enabled = static_cast<bool>(commandBuffer_[0]);
  bool odd_address = static_cast<bool>(commandBuffer_[1]);

  if (enabled) {
    CPU.set_jump_hint(true, odd_address);
    controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_set_jump_hint(): Enabling jump hint. Odd address: %d\n\r", odd_address ? 1 : 0);
  } else {
    CPU.set_jump_hint(false, false);
    controller_.getBoard().debugPrintf(DebugType::CMD, false, "cmd_set_jump_hint(): Disabling jump hint.\n\r");
  }

  return true;
}

template<typename BoardType, typename ShieldType>
bool CommandServer<BoardType, ShieldType>::cmd_null() {
  return true;
}

// Error handling methods
template<typename BoardType, typename ShieldType>
void CommandServer<BoardType, ShieldType>::set_error(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(errorBuffer_, MAX_ERROR_LEN, format, args);
  va_end(args);
  
  // Ensure null termination
  errorBuffer_[MAX_ERROR_LEN - 1] = 0;
}

template<typename BoardType, typename ShieldType>
const char* CommandServer<BoardType, ShieldType>::get_last_error() const {
  return errorBuffer_;
}

template class CommandServer<BoardType, ShieldType>;