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

#include <array>
#include <cstdint>
#include <cstdarg>
#include <serial_config.h>
//#include <BoardController.h>
#include <config.h>

enum class ServerState: uint8_t {
  Reset           = 0x00, // The CPU is being reset. Also used as initial state.
  CpuId           = 0x01, // The CPU ID routine is being executed. 
  CpuSetup        = 0x02, // The CPU setup routine is being executed. This is used for 186/386EX. 
  JumpVector      = 0x03, // The Jump Vector routine is being executed. This is used to avoid address wrap during Load.
  Load            = 0x04, // The register load routine is being executed. 
  LoadSmm         = 0x05, // The register load routine is being executed via SMM (386EX)
  LoadDone        = 0x06,
  EmuEnter        = 0x07,
  Prefetch        = 0x08,
  Execute         = 0x09,
  ExecuteFinalize = 0x0A,
  ExecuteDone     = 0x0B,
  EmuExit         = 0x0C,
  Store           = 0x0D,
  StoreDone       = 0x0E, // Store has completed. The CPU may need to be reset at this point if STOREALL was executed.
  StoreDoneSmm    = 0x0F, // Store has completed with SMM enabled. We remain in SMM and can Load registers as we exit.
  Done            = 0x10,
  StoreAll        = 0x11, // STOREALL (or SMM on 386) is in progress.
  Shutdown        = 0x12, // The CPU has shutdown (286/386). The CPU will need to be reset to continue.
  Error           = 0x13,
  TrapStoreDone   = 0x14,
  IceRecovery286  = 0x15  // Restore a paused STOREALL image via the ICE restart vector.
};

template<typename BoardType, typename HatType> class BoardController;

template<typename BoardType, typename HatType>
class CommandServer {
public:

  using CmdFn = bool (CommandServer::*)();

  // Supported features returned by CmdCapabilities, independent of enabled flags.
  // Trap execution includes resumable STOREALL captures between steps.
  static constexpr uint32_t CAP_TRAP_EXECUTION_286 = 0x00000001;
  static constexpr uint32_t CAP_SERVER_TRAP_HANDLER_286 = 0x00000002;

  static constexpr uint32_t FLAG_EMU_8080           = 0x00000001;
  static constexpr uint32_t FLAG_EXECUTE_AUTOMATIC  = 0x00000002;
  static constexpr uint32_t FLAG_MEMORY_BACKEND     = 0x00000004; // 0=SDRAM, 1=Hash Table
  static constexpr uint32_t FLAG_HALT_AFTER_JUMP    = 0x00000008; // Halt after flow control instruction.
  static constexpr uint32_t FLAG_USE_SDRAM_BACKEND  = 0x00000010; // Use SDRAM as memory backend (requires GIGA)
  static constexpr uint32_t FLAG_USE_SMM            = 0x00000020; // Use SMM for register readout on 386/486 CPUs
  static constexpr uint32_t FLAG_DEBUG_ENABLED      = 0x00000040; // Enable debug mode
  static constexpr uint32_t FLAG_LOG_CYCLES         = 0x00000080; // Enable cycle logging
  static constexpr uint32_t FLAG_ALE_INTERRUPT      = 0x00000100; // Enable ALE interrupt to deassert READY at T1
  static constexpr uint32_t FLAG_RESOLVE_BUS_STEP   = 0x00000200; // Resolve the data bus when single-stepping (Execute state)
  static constexpr uint32_t FLAG_ENABLE_TRAP_EXECUTION = 0x00000400; // 286 TF execution, independent of automatic/cycle stepping

  enum class ServerCommand {
    CmdNone            = 0x00,
    CmdVersion         = 0x01,
    CmdResetCpu        = 0x02,
    CmdLoad            = 0x03,
    CmdCycle           = 0x04,
    CmdReadAddressLatch= 0x05,
    CmdReadStatus      = 0x06,
    CmdRead8288Command = 0x07,
    CmdRead8288Control = 0x08, 
    CmdReadDataBus     = 0x09,
    CmdWriteDataBus    = 0x0A,
    CmdFinalize        = 0x0B,
    CmdBeginStore      = 0x0C,
    CmdStore           = 0x0D,
    CmdQueueLen        = 0x0E,
    CmdQueueBytes      = 0x0F,
    CmdWritePin        = 0x10,
    CmdReadPin         = 0x11,
    CmdGetProgramState = 0x12,
    CmdLastError       = 0x13,
    CmdGetCycleState   = 0x14,
    CmdAvailable00     = 0x15,
    CmdPrefetchStore   = 0x16,
    CmdReadAddress     = 0x17,
    CmdCpuType         = 0x18,
    CmdSetFlags        = 0x19,
    CmdPrefetch        = 0x1A,
    CmdInitScreen      = 0x1B,
    CmdStoreAll        = 0x1C,
    CmdSetRandomSeed   = 0x1D,
    CmdRandomizeMem    = 0x1E,
    CmdSetMemory       = 0x1F,
    CmdGetCycleStates  = 0x20,
    CmdEnableDebug     = 0x21,
    CmdSetMemoryStrategy = 0x22,
    CmdGetFlags        = 0x23,
    CmdReadMemory      = 0x24,
    CmdEraseMemory     = 0x25,
    CmdServerStatus    = 0x26,
    CmdClearCycleLog   = 0x27,
    CmdSetProgramBounds = 0x28,
    CmdSetJumpHint     = 0x29,
    CmdStepInstruction = 0x2A,
    CmdGetInstructionState = 0x2B,
    CmdSetTrapEndpoint = 0x2C,
    CmdContinueTrap = 0x2D,
    CmdGetTrapCapture = 0x2E,
    CmdCapabilities = 0x2F,
    CmdSetRealModeTrapHandler = 0x30,
    CmdInvalid
  };

  enum class CommandState: uint8_t {
    WaitingForCommand = 0x01,
    ReadingCommand,
    ExecutingCommand
  };

  void reset();
  inline void run() __attribute__((always_inline));
  void change_state(ServerState new_state);
  ServerState state() const { return state_; }
  const char* get_last_error() const;
  // Execution-state handlers report diagnostics through the same query channel.
  void set_error(const char* format, ...);

  const char* get_command_name(ServerCommand cmd);
  const char* get_state_string(ServerState state);
  char get_state_char(ServerState state);
  
  inline ServerState get_state() const __attribute__((always_inline)) {
    return state_;
  }
  uint32_t get_flags() const {
    return flags_;
  }

  inline bool is_executing() const __attribute__((always_inline)) {
    return executing_;
  }

  bool is_execute_automatic() const {
    return (flags_ & FLAG_EXECUTE_AUTOMATIC) != 0;
  }

  bool halt_after_jump() const {
    return (flags_ & FLAG_HALT_AFTER_JUMP) != 0;
  }

  explicit CommandServer(BoardController<BoardType,HatType>& controller);

private:
  static constexpr std::size_t CMD_COUNT =
    static_cast<std::size_t>(ServerCommand::CmdInvalid);
  std::array<CmdFn, CMD_COUNT> commands_;

  static constexpr uint8_t VERSION_NUM = 3;
  static constexpr uint8_t RESPONSE_FAIL = 0x00;
  static constexpr uint8_t RESPONSE_OK = 0x01;
  static constexpr size_t MAX_COMMAND_BYTES = 255; // Maximum number of bytes for fixed-length parameters.
  static constexpr unsigned long CMD_TIMEOUT_ = CMD_TIMEOUT; // Timeout for command parameter bytes in milliseconds.
  uint8_t commandBuffer_[MAX_COMMAND_BYTES];

  BoardController<BoardType,HatType>& controller_;
  ServerState state_ = ServerState::Reset;
  CommandState commandState_ = CommandState::WaitingForCommand;
  uint8_t commandByte_ = 0;
  ServerCommand cmd_ = ServerCommand::CmdNone;
  bool useSmm_ = false; // Use SM mode for register readout on 386/486 CPUs
  size_t commandBytesExpected_ = 0;
  size_t commandByteN_ = 0;
  unsigned long commandStartTime_ = 0;
  unsigned long stateBeginTime_ = 0;
  uint32_t flags_ = 0; 
  bool ale_interrupt_enabled_ = false;
  bool executing_ = false;
  uint32_t serverTicks_ = 0;

  // Error handling
  static constexpr size_t MAX_ERROR_LEN = 256;
  char errorBuffer_[MAX_ERROR_LEN] = {0};

  bool dispatch_command(ServerCommand cmd);
  uint8_t get_command_input_bytes(ServerCommand cmd);

  // Error handling methods
  void clear_error() {
    set_error("NO ERROR");
  }

  void proto_write(const uint8_t* buf, size_t len) {
    INBAND_SERIAL.write(buf, len);
  }

  void proto_write(uint8_t b) {
    INBAND_SERIAL.write(b);
  }

  void proto_flush() {
    FLUSH;
  }

  inline int proto_available() __attribute__((always_inline)) {
    return INBAND_SERIAL.available();
  }

  int proto_read() {
    return INBAND_SERIAL.read();
  }

  size_t proto_read(uint8_t* buf, size_t len) {
    return INBAND_SERIAL.readBytes(buf, len);
  }

  int proto_peek() {
    return INBAND_SERIAL.peek();
  }

  void send_ok() {
    proto_write(RESPONSE_OK);
    proto_flush();
  }

  void send_fail() {
    proto_write(RESPONSE_FAIL);
    proto_flush();
  }

  void debug_cmd(const char *msg) {
    controller_.getBoard().debugPrintf(DebugType::CMD, false, "## cmd: %s ##\n\r", msg);
  }

  void debug_proto(const char* msg) {
    controller_.getBoard().debugPrintf(DebugType::PROTO, false, "## proto: %s ##\n\r", msg);
  }

  bool cmd_version(void);
  bool cmd_reset_cpu(void);
  bool cmd_load(void);
  bool cmd_cycle(void);
  bool cmd_read_address_latch(void);
  bool cmd_read_status(void);
  bool cmd_read_8288_command(void);
  bool cmd_read_8288_control(void);
  bool cmd_read_data_bus(void);
  bool cmd_write_data_bus(void);
  bool cmd_finalize(void);
  bool cmd_begin_store(void);
  bool cmd_store(void);
  bool cmd_queue_len(void);
  bool cmd_queue_bytes(void);
  bool cmd_write_pin(void);
  bool cmd_read_pin(void);
  bool cmd_get_program_state(void);
  bool cmd_get_last_error(void);
  bool cmd_get_cycle_state(void);
  bool cmd_prefetch_store(void);
  bool cmd_read_address(void);
  bool cmd_cpu_type(void);
  bool cmd_invalid(void);
  bool cmd_set_flags(void);
  bool cmd_prefetch(void);
  bool cmd_init_screen(void);
  bool cmd_storeall(void);
  bool cmd_set_random_seed(void);
  bool cmd_randomize_mem(void);
  bool cmd_set_memory(void);
  bool cmd_get_cycle_states(void);
  bool cmd_enable_debug(void);
  bool cmd_set_memory_strategy(void);
  bool cmd_get_flags(void);
  bool cmd_read_memory(void);
  bool cmd_erase_memory(void);
  bool cmd_server_status(void);
  bool cmd_clear_cycle_log(void);
  bool cmd_set_program_bounds(void);
  bool cmd_set_jump_hint(void);
  bool cmd_step_instruction(void);
  bool cmd_get_instruction_state(void);
  bool cmd_set_trap_endpoint(void);
  bool cmd_continue_trap(void);
  bool cmd_get_trap_capture(void);
  bool cmd_capabilities(void);
  bool cmd_set_real_mode_trap_handler(void);
  bool cmd_run_trap(bool single_step);
  bool cmd_null(void);
};


/// @brief Runs the command server, processing incoming commands and executing them.
/// @tparam BoardType 
/// @tparam ShieldType 
template<typename BoardType, typename ShieldType>
void CommandServer<BoardType,ShieldType>::run()
{

  serverTicks_++;

  // Only process commands every 64 ticks
  if ((serverTicks_ & 0x3F) != 0) {
    return;
  }

  switch (commandState_) {

    case CommandState::WaitingForCommand:
      if (proto_available() > 0) {
        uint8_t cmd_byte = proto_read();

        // DEBUG_SERIAL.print("Received opcode: 0x");
        // DEBUG_SERIAL.println(cmd_byte, HEX);

        if (cmd_byte >= static_cast<uint8_t>(ServerCommand::CmdInvalid)) {
          send_fail();
          break;
        }

        // Valid command, enter ReadingCommand state
        cmd_ = static_cast<ServerCommand>(cmd_byte);
        if (cmd_ != ServerCommand::CmdServerStatus) {
          controller_.getBoard().debugPrintf(
            DebugType::CMD, 
            false, 
            "## CMD: Received command byte: %02X (%s)\n\r", 
            cmd_byte, 
            get_command_name(cmd_)
          );
        }

        size_t command_bytes = get_command_input_bytes(cmd_);

        if (cmd_ == ServerCommand::CmdNone) {
          // We ignore command byte 0 (null command)
          break;
        } else if (command_bytes > 0) {
          // This command requires input bytes before it is executed.
          commandByteN_ = 0;
          commandBytesExpected_ = command_bytes;
          commandStartTime_ = millis();  // Get start time for timeout calculation
          commandState_ = CommandState::ReadingCommand;
        } else {
          // Command requires no input, so execute it immediately
          bool result = dispatch_command(cmd_);
          if (result) {
            debug_proto("Command OK!");
            send_ok();
          } else {
            debug_proto("Command FAIL!");
            send_fail();
          }
        }
      }
      break;

    case CommandState::ReadingCommand:
      // The previously specified command requires parameter bytes, so read them in, or timeout
      if (proto_available() > 0) {
        // TODO: Read more than one byte at a time if available.
        uint8_t param_byte = proto_read();

        if (commandByteN_ < MAX_COMMAND_BYTES) {
          commandBuffer_[commandByteN_++] = param_byte;

          if (commandByteN_ == commandBytesExpected_) {
            // We have received enough parameter bytes to execute the in-progress command.
            bool result = dispatch_command(cmd_);
            if (result) {
              send_ok();
            } else {
              send_fail();
            }

            // Revert to listening for command
            commandByteN_ = 0;
            commandBytesExpected_ = 0;
            commandState_ = CommandState::WaitingForCommand;
          }
        }
      } else {
        // No bytes received yet, so keep track of how long we've been waiting
        uint32_t now = millis();
        uint32_t elapsed = now - commandStartTime_;

        if (elapsed >= CMD_TIMEOUT) {
          // Timed out waiting for parameter bytes. Send failure and revert to listening for command
          commandByteN_ = 0;
          commandBytesExpected_ = 0;
          commandState_ = CommandState::WaitingForCommand;
          debug_proto("Command timeout!");
          send_fail();
        }
      }
      break;

    case CommandState::ExecutingCommand:
      break;
  }
}
