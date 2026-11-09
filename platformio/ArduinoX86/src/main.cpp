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

// This is the main code module for the "CPU Server" sketch that controls
// the CPU and establishes a serial protocol for a CPU client to set up
// and communicate with the CPU after initialization.



#include <Arduino.h>
#include <cstdint>

#include "arduinoX86.h"
#include <globals.h>
#include "opcodes.h"
#include <exceptions.h>
#include "Display.h"

#include <BoardController.h>
#include <bus_emulator/BusEmulator.h>
#include <CommandServer.h>
#include <CycleStateLogger.h>

#ifdef GIGA_DISPLAY_SHIELD
#include "Arduino_GigaDisplay_GFX.h"
#include "GigaDisplay.h"
#endif 

#include <programs.h>

inline void handle_execute_automatic() __attribute__((always_inline));
inline void set_data_bus_width() __attribute__((always_inline));
inline bool is_transfer_done() __attribute__((always_inline));

Cpu CPU;
Intel8288 I8288;

// Global pointer to abstract Display interface
Display* screen = nullptr;

// Timing stuff.

unsigned long frame_ms_accumulator = 0;
unsigned long second_ms_accumulator = 0;
unsigned long last_millis = 0;
unsigned int fps_counter = 0;

// Patch offsets for load program
const size_t LOAD_BX = 0x0B;
const size_t LOAD_CX = 0x0E;
const size_t LOAD_DX = 0x11;
const size_t LOAD_SS = 0x14;
const size_t LOAD_DS = 0x19;
const size_t LOAD_ES = 0x1E;
const size_t LOAD_SP = 0x23;
const size_t LOAD_BP = 0x28;
const size_t LOAD_SI = 0x2D;
const size_t LOAD_DI = 0x32;
const size_t LOAD_AX = 0x37;
const size_t LOAD_IP = 0x3A;
const size_t LOAD_CS = 0x3C;

char LAST_ERR[MAX_ERR_LEN] = { 0 };

const char *CPU_TYPE_STRINGS[] = {
  "Undetected",
  "i8088",
  "i8086",
  "NEC V20",
  "NEC V30",
  "i80188",
  "i80186",
  "i80286",
  "i80386"
};
const char CPU_TYPE_COUNT = sizeof(CPU_TYPE_STRINGS) / sizeof(CPU_TYPE_STRINGS[0]);

const char *FPU_TYPE_STRINGS[] = {
  "None",
  "i8087",
};

// Strings for printing bus transfer cycles.
const char *CYCLE_STRINGS[] = {
  "T1", "T2", "T3", "T4", "Tw", "Ti"
};

const char *SEGMENT_STRINGS[] = {
  "ES", "SS", "CS", "DS"
};

// Storage to write to the stack during NMI
uint8_t NMI_STACK_BUFFER[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Specialize BoardType for the chosen ShieldType
using BoardType = BoardTypeBase<ShieldType>;

// Instantiate board and controller
BoardType Board;
BoardController<BoardType, ShieldType> Controller(Board); // Uses default Shield constructor
// Or if you want to pass Shield constructor parameters:
// BoardController<BoardType, ShieldType> Controller(Board, true); // Pass emulate_bus_controller = true

// Instantiate the command server
namespace ArduinoX86 {  
  CommandServer<BoardType, ShieldType> Server(Controller);
  BusEmulator *Bus = nullptr;
  CycleStateLogger *CycleLogger = nullptr;
};

bool screen_initialized = false;
bool screen_init_requested = false;

//GigaDisplay_GFX display;
//GigaDisplay screen_impl(display);

// Main Sketch setup routine
void setup() {
  // Open the USB CDC serial port for in-band communication.
  // We run the client-server protocol over this port.
  INBAND_SERIAL.begin(BAUD_RATE);

  // Temporarily disabled: 
  //wait for a host to open the USB serial connection.
  // while (!INBAND_SERIAL)
  //   ;

  // Board initialization sets up the debug serial port.
  Board.init();

  // Wait for things to settle down.
  delayMicroseconds(200);

  Board.debugPrintln(DebugType::SETUP, "In setup()...");

  // Initialize the bus emulator
  if (ArduinoX86::Bus == nullptr) {
    ArduinoX86::Bus = new BusEmulator(new HashBackend());
    if (ArduinoX86::Bus == nullptr) {
      Board.debugPrintln(DebugType::SETUP, "Failed to create bus emulator!");
      set_error("Failed to create bus emulator!");
      return;
    }
    else {
      Board.debugPrintln(DebugType::SETUP, "Bus emulator created successfully.");
    }
  }

  // Initialize the cycle state logger
  if (ArduinoX86::CycleLogger == nullptr) {
    ArduinoX86::CycleLogger = new CycleStateLogger();
    if (ArduinoX86::CycleLogger == nullptr) {
      Board.debugPrintln(DebugType::SETUP, "Failed to create cycle state logger!");
      set_error("Failed to create cycle state logger!");
      return;
    }
    else {
      Board.debugPrintln(DebugType::SETUP, "Cycle state logger created successfully.");
    }
  }

#if defined(ARDUINO_GIGA)
  Board.debugPrintln(DebugType::SETUP, "Running on Arduino Giga...");
#endif

  //i8288_status();

#if HAT_8087_V1
  debugPrintlnColor(ansi::bright_cyan, "8087 Shield specified!");
#endif

  // Patch the jumps in programs that jump
  JUMP_VECTOR.patch_vector(LOAD_SEG);
  SETUP_PROGRAM.patch_vector(LOAD_SEG);
  NMI_VECTOR.patch_vector(STORE_SEG);
  
  debugPrintlnColor(ansi::bright_cyan, "Identifying CPU...");
  cpu_id();

  #ifdef ARDUINO_GIGA
    pinMode(86, OUTPUT);
    pinMode(87, OUTPUT);  
    pinMode(88, OUTPUT);
    // Turn LED green
    digitalWrite(88, LOW);
  #endif
  DEBUG_SERIAL.flush();
  clear_error();

  #if defined(ARDUINO_GIGA)
    #if GIGA_DISPLAY_SHIELD
      static GigaDisplay_GFX display;
      static GigaDisplay screen_impl(display);
      screen = &screen_impl;
      Board.debugPrintln(DebugType::SETUP, "Using Giga Display Shield!...");
    #else
      // Optional stub class for headless builds
      class NullDisplay : public Display {};
      static NullDisplay nullDisplay;
      screen = &nullDisplay;
    #endif
  #endif

  //screen->init();
  //beep(100);
  Board.debugPrintln(DebugType::SETUP, "Arduino8088 Server Initialized! Waiting for commands...");
}

bool cpu_id() {

  Board.debugPrintln(DebugType::ID, "cpu_id(): resetting CPU...");
  CpuResetResult reset_result = Controller.resetCpu();
  CPU.reset(reset_result, true);
  if (!reset_result.success) {
    Board.debugPrintln(DebugType::ID, "cpu_id(): Failed to reset CPU!");
    set_error("Failed to reset CPU!");
    return false;
  }
  else {
    Board.debugPrintln(DebugType::ID, "cpu_id(): CPU reset successful.");
  }

#if defined(CPU_186)
  // We can detect 188 vs 186 here. No need to enter CPU id program as we don't support
  // any other variants with this pinout.
  if (CPU.width == CpuBusWidth::Eight) {
    CPU.cpu_type = CpuType::i80188;
  } else {
    CPU.cpu_type = CpuType::i80186;
  }
  Board.debugPrint(DebugType::ID, "cpu_id(): Detected CPU: ");
  Board.debugPrintln(DebugType::ID, CPU_TYPE_STRINGS[static_cast<size_t>(CPU.cpu_type)]);
  ArduinoX86::Bus->set_cpu_type(CPU.cpu_type);
  return true;
#elif defined(CPU_286)
  CPU.cpu_type = CpuType::i80286;
  Board.debugPrint(DebugType::ID, "cpu_id(): Detected CPU: ");
  Board.debugPrintln(DebugType::ID, CPU_TYPE_STRINGS[static_cast<size_t>(CPU.cpu_type)]);
  ArduinoX86::Bus->set_cpu_type(CPU.cpu_type);
  return true;
#elif defined(CPU_386)
  CPU.cpu_type = CpuType::i80386;
  Board.debugPrint(DebugType::ID, "cpu_id(): Detected CPU: ");
  Board.debugPrintln(DebugType::ID, CPU_TYPE_STRINGS[static_cast<size_t>(CPU.cpu_type)]);
  ArduinoX86::Bus->set_cpu_type(CPU.cpu_type);
  return true;
#endif
  

  ArduinoX86::Server.change_state(ServerState::CpuId);
  uint32_t timeout = 0;
  while (ArduinoX86::Server.state() != ServerState::Load) {
    cycle();
    timeout++;
    if (timeout > 200) {
      Board.debugPrintln(DebugType::ID, "cpu_id(): CPU ID timeout!");
      set_error("CPU ID timeout!");
      return false;
    }
  }

  size_t t_idx = static_cast<size_t>(CPU.cpu_type);
  if (t_idx < CPU_TYPE_COUNT) {
    Board.debugPrint(DebugType::ID, "cpu_id(): Detected CPU: ");
    Board.debugPrintln(DebugType::ID, CPU_TYPE_STRINGS[t_idx]);
  } else {
    Board.debugPrintln(DebugType::ID, "Bad CPU type!");
    return false;
  }
  
  ArduinoX86::Bus->set_cpu_type(CPU.cpu_type);
  return true;
}

uint32_t calc_flat_address(uint16_t seg, uint16_t offset) {
  return ((uint32_t)seg << 4) + offset;
}

// Read a byte from the data bus. The half of the bus to read is determined
// by BHE and A0.
uint8_t data_bus_read_byte() {
  CPU.data_bus = Controller.readDataBus(CPU.data_width);
  if (!READ_BHE_PIN) {
    // High byte is active.
    return (uint8_t)(CPU.data_bus >> 8);
  } else {
    // Low byte is active.
    return (uint8_t)CPU.data_bus;
  }
}

void data_bus_set_byte(uint8_t byte) {
  if (!READ_BHE_PIN) {
    // High byte is active.
    CPU.data_bus = ((uint16_t)byte) << 8;
  } else {
    // Low byte is active.
    CPU.data_bus = (uint16_t)byte;
  }
}

void clear_error() {
  strncpy(LAST_ERR, "No error", MAX_ERR_LEN - 1);
}

void set_error(const char *msg) {
  strncpy(LAST_ERR, msg, MAX_ERR_LEN - 1);
  DEBUG_SERIAL.println("");
  debugPrintlnColor(ansi::red, "************ ERROR ************");
  debugPrintlnColor(ansi::red, LAST_ERR);
  debugPrintlnColor(ansi::red, "*******************************");
  error_beep();
}


void patch_vector_pgm(uint8_t *pgm, uint16_t seg, size_t offset) {
  *((uint16_t *)&pgm[offset]) = seg;
}

void patch_load_pgm(InlineProgram *pgm, volatile registers1_t *reg) {
  pgm->write_u16_at(0, reg->flags);
  pgm->write_u16_at(LOAD_BX, reg->bx);
  pgm->write_u16_at(LOAD_CX, reg->cx);
  pgm->write_u16_at(LOAD_DX, reg->dx);
  pgm->write_u16_at(LOAD_SS, reg->ss);
  pgm->write_u16_at(LOAD_DS, reg->ds);
  pgm->write_u16_at(LOAD_ES, reg->es);
  pgm->write_u16_at(LOAD_SP, reg->sp);
  pgm->write_u16_at(LOAD_BP, reg->bp);
  pgm->write_u16_at(LOAD_SI, reg->si);
  pgm->write_u16_at(LOAD_DI, reg->di);
  pgm->write_u16_at(LOAD_AX, reg->ax);
  pgm->write_u16_at(LOAD_IP, reg->ip);
  pgm->write_u16_at(LOAD_CS, reg->cs);
}

void patch_brkem_pgm(InlineProgram *pgm, volatile registers1_t *regs) {
#if DEBUG_EMU
  static char buf[20];
  DEBUG_SERIAL.println("## Patching BRKEM program ##");
  snprintf(buf, 20,
           "CS: %04X IP: %04X",
           regs->cs,
           regs->ip);
  DEBUG_SERIAL.println(buf);
#endif
  pgm->write_u16_at(0, regs->ip);
  pgm->write_u16_at(2, regs->cs);
}

// Fix up the old-style inline regs by swapping fields.
void convert_inline_registers(volatile void *inline_regs) {

  registers2_t *regs2 = (registers2_t *)inline_regs;

  uint16_t ip = regs2->ip;
  uint16_t cs = regs2->cs;
  uint16_t ss = regs2->ss;
  uint16_t sp = regs2->sp;

  registers1_t *regs1 = (registers1_t *)inline_regs;

  regs1->ip = ip;
  regs1->cs = cs;
  regs1->ss = ss;
  regs1->sp = sp;
}

void print_registers(registers1_t *regs) {
  static char buf[130];
  static char flag_buf[17];

  if (!regs) {
    return;
  }

  snprintf(buf, 130,
           "AX: %04x BX: %04x CX: %04x DX: %04x\n"
           "SP: %04x BP: %04x SI: %04x DI: %04x\n"
           "CS: %04x DS: %04x ES: %04x SS: %04x\n"
           "IP: %04x\n"
           "FLAGS: %04x",
           regs->ax, regs->bx, regs->cx, regs->dx,
           regs->sp, regs->bp, regs->si, regs->di,
           regs->cs, regs->ds, regs->es, regs->ss,
           regs->ip,
           regs->flags);

  DEBUG_SERIAL.println(buf);

  // Expand flag info
  uint16_t f = regs->flags;
  char c_chr = CPU_FLAG_CARRY & f ? 'C' : 'c';
  char p_chr = CPU_FLAG_PARITY & f ? 'P' : 'p';
  char a_chr = CPU_FLAG_AUX_CARRY & f ? 'A' : 'a';
  char z_chr = CPU_FLAG_ZERO & f ? 'Z' : 'z';
  char s_chr = CPU_FLAG_SIGN & f ? 'S' : 's';
  char t_chr = CPU_FLAG_TRAP & f ? 'T' : 't';
  char i_chr = CPU_FLAG_INT_ENABLE & f ? 'I' : 'i';
  char d_chr = CPU_FLAG_DIRECTION & f ? 'D' : 'd';
  char o_chr = CPU_FLAG_OVERFLOW & f ? 'O' : 'o';

  snprintf(
    flag_buf, 17,
    "1111%c%c%c%c%c%c0%c0%c1%c",
    o_chr, d_chr, i_chr, t_chr, s_chr, z_chr, a_chr, p_chr, c_chr);

  DEBUG_SERIAL.print("FLAGSINFO: ");
  DEBUG_SERIAL.println(flag_buf);
}

void print_cpu_state() {
  const size_t buf_len = 90;
  static char buf[buf_len];
  const size_t op_len = 9;  //(4 + 4 + 1)
  static char op_buf[op_len];
  static char q_buf[15];
  static char data_buf[6];
  size_t bus_str_width = 0;

  const char *ale_str = Controller.readALEPin() ? "A:" : "  ";

  bool mrdc = (CPU.command_bits & 0x01);
  bool amwc = (CPU.command_bits & 0x02);
  bool mwtc = (CPU.command_bits & 0x04);

  bool iorc = (CPU.command_bits & 0x08);
  bool aiowc = (CPU.command_bits & 0x10);
  bool iowc = (CPU.command_bits & 0x20);

  char rs_chr = !mrdc ? 'R' : '.';
  char aws_chr = !amwc ? 'A' : '.';
  char ws_chr = !mwtc ? 'W' : '.';

  char ior_chr = !iorc ? 'R' : '.';
  char aiow_chr = !aiowc ? 'A' : '.';
  char iow_chr = !iowc ? 'W' : '.';

  char ready_chr = READ_READY_PIN_NORM ? 'R' : '.';
  char reset_chr = READ_RESET_PIN ? 'X' : '.';
  char intr_chr = READ_INTR_PIN ? 'I' : '.';
  char inta_chr = '.';
  char nmi_chr = READ_NMI_PIN ? 'N' : '.';
  char bhe_chr = !READ_BHE_PIN ? 'B' : '.';
  char lock_chr = !READ_LOCK_PIN ? 'L' : '.';
  char smi_chr = !READ_SMI_PIN ? 'S' : '.';
  char test_chr = !READ_TEST_PIN ? 'T' : '.';

  #if defined(FPU_8087)
  
  char rq_chr = !READ_PIN_D03 ? 'R' : '.';
  char fint_chr = READ_PIN_D20 ? 'I' : '.';  
#endif

  char v_chr = ArduinoX86::Server.get_state_char(ArduinoX86::Server.state());
  uint8_t q = (CPU.status0 >> 6) & 0x03;
  char q_char = QUEUE_STATUS_CHARS[q];
  //char s = CPU.status0 & 0x07;
  //char rout_chr = '.';

  // Set the bus string width
  if (CPU.width == CpuBusWidth::Eight) {
    bus_str_width = 2;
  } else {
    bus_str_width = 4;
  }

  // Get segment from S3 & S4 if supported by the current CPU.
  const char *seg_str = "  ";
  if(Controller.hasSegmentStatus()) {
    // Get segment from S3 & S4
    uint8_t seg = ((CPU.status0 & 0x18) >> 3) & 0x03;
    seg_str = SEGMENT_STRINGS[(size_t)seg];
  }

  // Make data bus string and set r/w indicators based on bus size
  data_buf[5] = { ' ' };
  const char *rd_str = "r";
  const char *wr_str = "w";
  if (CPU.data_width == ActiveBusWidth::EightLow) {
    // Write two hex digits, 0-padded
    snprintf(data_buf, 5, "%4.2X", (uint8_t)CPU.data_bus);
  } else if (CPU.data_width == ActiveBusWidth::EightHigh) {
    // Write two hex digits, 0-padded, left-aligned in 4-character field
    snprintf(data_buf, 5, "%-4.2X", (uint8_t)(CPU.data_bus >> 8));
  } else {
    rd_str = "R";
    wr_str = "W";
    // Write four hex digits, 0-padded
    snprintf(data_buf, 5, "%04X", CPU.data_bus);
  }

  // Make string for bus reads and writes
  op_buf[0] = 0;
  if ((!mrdc || !iorc) && (CPU.bus_cycle != T1)) {
    snprintf(op_buf, op_len, "%s-> %s", rd_str, data_buf);
  } else if ((!mwtc || !iowc) && (CPU.bus_cycle != T1)) {
    snprintf(op_buf, op_len, "<-%s %s", wr_str, data_buf);
  } else {
    snprintf(op_buf, op_len, "%*s", (int)(4 + bus_str_width), "");
  }

  const char *q_str = CPU.queue.to_string();

  const char *t_str;
  if ((CPU.bus_cycle == T1) && (CPU.bus_state == PASV)) {
    // Convert T1 to Ti when passive bus
    t_str = "Ti";
  } else {
    t_str = Controller.getTCycleString(CPU.bus_cycle);
  }

// Set reset_out chr
#if CPU_186
  if (READ_PIN_D03) {
    rout_chr = 'r';
  }
#endif

  uint32_t address_bus = CPU.address_bus;
  int address_digits = Controller.getAddressDigits();

  snprintf(
      buf,
      buf_len,
      "%08ld %c %s%0*lX:%0*lX",
      (uint32_t)CPU.cycle_ct(),
      v_chr,
      ale_str,
      address_digits, CPU.address_latch(),
      address_digits, address_bus);

  DEBUG_SERIAL.print(buf);

  // Print the data bus if we don't have a multiplexed bus.
  // If the bus is multiplexed, we effectively print the data bus when printing the 
  // address bus.
  if (!Controller.hasMultiplexedBus()) {
    snprintf(
      buf,
      buf_len,
      ":%04X",
      Controller.readDataBus(ActiveBusWidth::Sixteen, true));

    DEBUG_SERIAL.print(buf);
  }

  

  snprintf(
      buf,
      buf_len,    
      " %2s M:%c%c%c I:%c%c%c P:%c%c%c%c%c%c%c%c%c ",
      seg_str,
      rs_chr, aws_chr, ws_chr,
      ior_chr, aiow_chr, iow_chr,
      reset_chr, ready_chr, lock_chr, intr_chr, inta_chr, nmi_chr, smi_chr, test_chr, bhe_chr);

  DEBUG_SERIAL.print(buf);

  Controller.printPinStates();

#if defined(FPU_8087)
  snprintf(
      buf,
      buf_len,    
      "F:%c%c%c%c ",
      lock_chr, test_chr, rq_chr, fint_chr);
  DEBUG_SERIAL.print(buf);
#endif  

#if defined(CPU_286)
  // Die photography confirms the ICE pins aren't bonded, so sampling them is meaningless.
  // char ice0_chr = READ_ICE_PIN0 ? '1' : '0';
  // char ice1_chr = READ_ICE_PIN1 ? '1' : '0';
  // snprintf(
  //   buf,
  //   buf_len,
  //   "C:%c%c ",
  //   ice0_chr, ice1_chr
  // );
  // DEBUG_SERIAL.print(buf);
#endif

  debugPrintColor(Controller.getBusStatusColor(CPU.bus_state), Controller.getBusStatusString(CPU.bus_state));

  snprintf(
    buf,
    buf_len,
    "[%1X] %s %8s | %c%d [%-*s]",
    CPU.status0 & 0xF,
    t_str,
    op_buf,
    q_char,
    CPU.queue.len(),
    CPU.queue.size() * 2,
    q_str);

  DEBUG_SERIAL.print(buf);

// Print queue status string if we have queue status pins available.
  if (CPU.have_queue_status) {
    if (q == QUEUE_FIRST) {
      // First byte of opcode read from queue. Decode it to opcode
      snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, 0, false));
      DEBUG_SERIAL.print(q_buf);
    } else if (q == QUEUE_SUBSEQUENT) {
      if (!CPU.in_emulation && IS_GRP_OP(CPU.opcode) && CPU.q_fn == 1) {
        // Modrm was just fetched for a group opcode, so display the mnemonic now
        snprintf(q_buf, 15, " <-q %02X %s", CPU.qb, get_opcode_str(CPU.opcode, CPU.qb, true));
      } else {
        snprintf(q_buf, 15, " <-q %02X", CPU.qb);
      }
      DEBUG_SERIAL.print(q_buf);
    }
  }

  if (!Controller.getBoard().haveDeferredBuffer()) {
    DEBUG_SERIAL.println("");
  }
}

// Emulate a code fetch from the specified program.
uint16_t read_program(const uint8_t *program, size_t len, uint16_t *pc, uint32_t address, ActiveBusWidth width) {

  uint16_t data = 0x9090;

  if (*pc >= len) {
    return data;
  }

  if (width == ActiveBusWidth::EightLow) {
    data = program[(*pc)++];
  } else if (width == ActiveBusWidth::EightHigh) {
    // TODO: bounds checks
    //DEBUG_SERIAL.println("## Odd read ##");
    if ((*pc) > 0) {
      // This byte doesn't really matter, but we can simulate fetching more realistically by including it.
      // If this happens to be the start of the program though, it will just have to be 0.
      data = program[(*pc) - 1];
    }
    if (*pc < len) {
      data &= 0x00FF;
      data |= ((uint16_t)program[(*pc)++]) << 8;
    }
  } else {
    // 16-bit read.

    if ((address & 1) == 0) {
      // Even address
      //DEBUG_SERIAL.println("## Even read ##");
      data = program[(*pc)++];
      if (*pc < len) {
        data &= 0x00FF;
        data |= ((uint16_t)program[(*pc)++]) << 8;
      }
    } else {
      // Odd address.
      debugPrintlnColor(ansi::bright_red, "## read_program(): Odd 16-bit read, shouldn't happen! ##");
    }
  }

  return data;
}

void write_buffer(uint8_t *buffer, uint16_t *cursor, uint16_t data, uint32_t address, ActiveBusWidth width) {
  if (width == ActiveBusWidth::EightLow) {
    buffer[(*cursor)++] = (uint8_t)data;
  } else if (width == ActiveBusWidth::EightHigh) {
    buffer[(*cursor)++] = (uint8_t)(data >> 8);
  } else {
    // 16-bit read.
    if ((address & 1) == 0) {
      // Even address
      buffer[(*cursor)++] = (uint8_t)data;
      buffer[(*cursor)++] = (uint8_t)(data >> 8);
    } else {
      // Odd address.
      debugPrintlnColor(ansi::bright_red, "## write_buffer(): Odd 16-bit read, shouldn't happen! ##");
    }
  }
}

// Simulate fetching NOPs based on bus width.
uint16_t read_nops(ActiveBusWidth width) {
  if (width == ActiveBusWidth::EightLow) {
    return 0x90;
  } else {
    return 0x9090;
  }
}


void set_data_bus_width() {
  if (!READ_BHE_PIN) {
    if ((CPU.address_latch() & 1) == 0) {
      // BHE is active, and address is even. Bus width is 16.
      //Controller.getBoard().debugPrintln(DebugType::BUS, "Bus width 16");
      CPU.data_width = ActiveBusWidth::Sixteen;
    } else {
      // BHE is active, and address is odd. Bus width is EightHigh.
      //Controller.getBoard().debugPrintln(DebugType::BUS, "Bus width 8 (Odd)");
      CPU.data_width = ActiveBusWidth::EightHigh;
    }
  } 
  else {
    // If BHE is inactive, then we can't read an even address. So this must be EightLow.
    //Controller.getBoard().debugPrintln(DebugType::BUS, "Bus width 8 (Even)");
    CPU.data_width = ActiveBusWidth::EightLow;
  }
}

void cycle() {
  if (CPU.trap.enabled) {
    if (CPU.trap.state.status == TrapStatus::Paused || CPU.trap.state.status == TrapStatus::Captured
        || CPU.trap.state.status == TrapStatus::Failed) {
      return;
    }
    if (ArduinoX86::Server.state() == ServerState::StoreAll
        || ArduinoX86::Server.state() == ServerState::IceRecovery286) {
      if (!CPU.trap.remaining_cycles) {
        CPU.trap.fail(TrapReason::Timeout);
        ArduinoX86::Server.change_state(ServerState::Error);
        return;
      }
      --CPU.trap.remaining_cycles;
    }
  }

  // Resolve data bus from last cycle.
  // if (!CPU.data_bus_resolved && (!Controller.readMRDCPin() || !Controller.readIORCPin())) {
  //   //Controller.getBoard().debugPrintln(DebugType::BUS, "## Resolving data bus ##");
  //   Controller.writeDataBus(CPU.data_bus, CPU.data_width);
  // }

  // First, tick the CPU and increment cycle count
  Controller.tickCpu();
  CPU.tick();

  CPU.cpuid_counter++;

  // Read the CPU status pins
  CPU.status0 = Controller.readCpuStatusLines();
  CPU.command_bits = Controller.readBusControllerCommandLines();
  // Decode the bus state from the read status. This varies per CPU.
  CPU.bus_state = Controller.decodeBusStatus(CPU.status0);
  // Save last address bus value for skipped bus cycle detection.
  CPU.last_address_bus = CPU.address_bus;
  CPU.address_bus = Controller.readAddressBus(false);

  static CycleState cycle_state;
  bool ale = Controller.readALEPin();
  uint32_t server_flags = ArduinoX86::Server.get_flags();
  cycle_state.cpu_status0 = CPU.status0;
  cycle_state.bus_command_bits = CPU.command_bits;
  cycle_state.bus_control_bits = Controller.readBusControllerControlLines();
  cycle_state.address_bus = CPU.address_bus;
  
  // Extract QS0-QS1 queue status
  uint8_t q = (CPU.status0 >> 6) & 0x03;
  CPU.qb = 0xFF;
  CPU.q_ff = false;

  CPU.wait_state_ct++;
  if (CPU.wait_state_ct >= CPU.wait_states) {
    // If we have waited enough, we can clear the wait state.
    //Controller.writePin(OutputPin::Ready, READY_ASSERT);
    //Controller.getBoard().debugPrintln(DebugType::BUS, "## Wait state cleared ##");
  }

  // Check for CPU shutdown.
  if ((CPU.bus_state == HALT) && (CPU.address_bus == 0x000000) && (!CPU.is_shutdown())) {
    CPU.set_shutdown();
    Controller.getBoard().debugPrintln(DebugType::ERROR, "## CPU shutdown detected ##");
    ArduinoX86::Server.change_state(ServerState::Shutdown);
    set_error("CPU shutdown detected!");
  }

  // The ALE signal is issued to inform the motherboard to latch the address bus.
  // The full address is only valid on T1, when ALE is asserted, so if we need to
  // reference the address of the bus cycle later, we must latch it.
  if (ale) {
    // ALE signals start of bus cycle, so set cycle to t1.
    //Controller.getBoard().debugPrintln(DebugType::TSTATE, "## ALE is high, setting T-cycle to T1 ##");
    
    CPU.bus_cycle = T1;
    // Address lines are only valid when ALE is high, so latch address now.
    CPU.latch_address(CPU.address_bus);
    // Set the data bus width (must happen after latch)
    set_data_bus_width();
    CPU.bus_state_latched = CPU.bus_state;
    CPU.data_bus_resolved = false;

    #if defined(CPU_286)
    // Test for a missed bus cycle (occasionally happens on 286)
    // This is the case if the last bus cycle was Ti, and had the previous address on the bus,
    // and the previous bus address was odd
    if ((CPU.last_bus_cycle == 0) 
      && (CPU.last_address_bus == (CPU.address_bus - 1)) 
      && (CPU.last_address_bus & 1)) {
      // We missed a bus cycle
      Controller.getBoard().debugPrintf(
        DebugType::ERROR, false, 
        "## Missed bus cycle detected. Bus: %06lX, Last Bus: %06lX ##\n\r", 
        CPU.address_bus, CPU.last_address_bus);
      ArduinoX86::Server.change_state(ServerState::Error);
      set_error("Missed bus cycle detected!");
      return;
    }
    #endif      
  }


  // We always enter Tw from T3, as we can't tell if the read/write is done yet on T3.
  // Now that we have cycled, we can know if we need to transition to T4, skipping Tw.
  if (CPU.bus_cycle == TW) {
    if (is_transfer_done()) {
      // Read/write is done, so we can skip Tw and go to T4
      CPU.bus_cycle = T4;
    }
  }

  if (CPU.bus_cycle == T4) {
    if (CPU.have_queue_status) {
      handle_fetch(q);
    }
    CPU.bus_state_latched = PASV;
  }


  // switch (CPU.bus_cycle) {
  //   case TW:
  //     // Transition to T4 if read/write signals are complete
  //     if (is_transfer_done()) {
  //       CPU.bus_cycle = T4;
  //       handle_fetch(q);
  //     }
  //     break;
  //   case T4:
  //     handle_fetch(q);
  //     CPU.bus_state_latched = PASV;
  //     break;
  //   default:
  //     break;
  // }

  cycle_state.cpu_state = static_cast<uint8_t>(CPU.bus_cycle);

  // Handle queue activity
  if (CPU.have_queue_status) {
    if ((q == QUEUE_FIRST) || (q == QUEUE_SUBSEQUENT)) {
      // We fetched a byte from queue last cycle
      if (CPU.queue.len() > 0) {
        CPU.queue.pop(&CPU.qb, &CPU.qt);
        if (q == QUEUE_FIRST) {
          // Set flag for first instruction byte fetched
          CPU.q_ff = true;
          CPU.q_fn = 0;  // First byte of instruction
          CPU.opcode = CPU.qb;
          CPU.mnemonic = get_opcode_str(CPU.opcode, 0, false);
 
          if (!IS_GRP_OP(CPU.opcode)) {
            Controller.getBoard().debugPrintf(DebugType::INSTR, false, "## INST: %s ##\n\r", CPU.mnemonic);
          } else {
            Controller.getBoard().debugPrintln(DebugType::INSTR, "## INST: Decoding GRP... ##");
          }

        } 
        else {
          if (IS_GRP_OP(CPU.opcode) && CPU.q_fn == 1) {
            CPU.mnemonic = get_opcode_str(CPU.opcode, CPU.qb, true);
            Controller.getBoard().debugPrintf(DebugType::INSTR, false, "## INST: %s ##\n\r", CPU.mnemonic);
          }
          // Subsequent byte of instruction fetched
          CPU.q_fn++;
        }
      } 
      else {
        // Queue read while queue empty? Bad condition.
        if (ArduinoX86::Server.state() != ServerState::Reset) {
          // Sometimes we get a spurious queue read signal in Reset.
          // We can safely ignore any queue reads during the Reset state.
          Controller.getBoard().debugPrintln(DebugType::ERROR, "## Error: Invalid Queue Length-- ##");
        }
      }
    } 
    else if (q == QUEUE_FLUSHED) {
      // Queue was flushed last cycle.

      // Warn if queue is flushed during CODE cycle.
      if (CPU.bus_state_latched == CODE) {
        const char *t_cycle_str = Controller.getTCycleString(CPU.bus_cycle);
        Controller.getBoard().debugPrintf(DebugType::ERROR, "## FLUSH during CODE fetch! t-state: %s ##\n\r", t_cycle_str);
      }

      // The queue is flushed once during store program, so we need to adjust s_pc
      // by the length of the queue when it was flushed or else we'll skip bytes
      // of the store program.
      if (CPU.s_pc > 0) {

        if (CPU.s_pc < 4) {
          Controller.getBoard().debugPrintln(DebugType::STORE, "## FLUSHed STORE bytes (early): Reset s_pc ##");
          CPU.s_pc = 0;
        } else if (CPU.s_pc >= CPU.queue.len()) {
          uint16_t pc_adjust = (uint16_t)CPU.queue.len();

          if ((pc_adjust & 1) && (CPU.width == CpuBusWidth::Sixteen)) {
            // If we have an odd queue length and 16-bit fetches, account for one more byte
            //pc_adjust++;
          }
          CPU.s_pc -= pc_adjust;
          Controller.getBoard().debugPrintf(DebugType::STORE, false, "## FLUSHed STORE bytes: Adjusted s_pc by: %d new s_pc: %d ##\n\r", pc_adjust, CPU.s_pc);
        } else {
          Controller.getBoard().debugPrintln(DebugType::STORE, "## FLUSHed STORE bytes: Reset s_pc on flush");
        }
      }

      CPU.queue.flush();
      Controller.getBoard().debugPrintf(DebugType::QUEUE, false, "## Queue Flushed, new PC: %04X ##\n\r", CPU.v_pc);
    }
  }

  uint32_t run_address = 0;

  if (!Controller.readMWTCPin() || !Controller.readIOWCPin()) {
    // CPU is writing to the data bus, latch value
    CPU.data_bus = Controller.readDataBus(CPU.data_width);
  }

  // Handle state machine
  switch (ArduinoX86::Server.state()) {

    // Unlike in run_program, the Execute state in cpu_server is entirely interactive based on
    // commands from the client.
    // This is to support interception of memory reads & writes as instructions execute and to allow
    // the client to query CPU state as it wishes per cpu cycle.
    // When done in the Execute state, a cpu client can end execution by:
    //  - Executing an ExecuteFinalize command.
    //    This is typically done when a CODE fetch occurs past the end of the provided program, although
    //    other end conditions are possible.
    //  - Executing a HALT
    //  - Asserting NMI before the end of the instruction
    case ServerState::Execute:
      if (CPU.trap.enabled) {
        handle_execute_trap();
      }
      else if (server_flags & CommandServer<BoardType, ShieldType>::FLAG_EXECUTE_AUTOMATIC) {
        handle_execute_automatic();
      }
      else {
        handle_execute_state();
      }
      break;

    case ServerState::Reset:
      // We are executing the CPU reset routine.
      // Nothing to do here.
      break;
    case ServerState::CpuId:
      // We are executing the CPU ID routine.
      handle_cpuid_state(q);
      break;

    case ServerState::CpuSetup:
      handle_cpu_setup_state();
      break;

    case ServerState::JumpVector:
      // We are executing the initial jump from the reset vector FFFF:0000.
      // This is to avoid wrapping effective address during load procedure.
      handle_jump_vector_state(q);
      break;

    case ServerState::Load:
      // We are executing the register load routine.
      if (CPU.cpu_type == CpuType::i80286) {
        // 286 just uses LOADALL instead of a load program. 
        handle_loadall_286();
      } 
      else if (CPU.cpu_type == CpuType::i80386) {
        handle_loadall_386();
      }
      else {
        handle_load_state(q);
      }
      break;

    case ServerState::LoadSmm:

      handle_smm_load_386();
      break;

    case ServerState::LoadDone:
      // LoadDone is triggered by the queue flush following the jump in Load.
      // We wait for the next ALE and begin Execute.
      handle_load_done_state();
      break;

    case ServerState::Prefetch:
      // We are executing the prefetch routine.
      // Currently placeholder.
      break;

    case ServerState::EmuEnter:
      // We are executing the BRKEM routine.
      handle_emu_enter_state(q);
      break;

    // Since Execute is an interactive state where the client controls the cpu_server, we need to be able
    // to transition safely from Execute to Store.
    // ExecuteFinalize state feeds the CPU STORE program bytes flagged with DATA_PROGRAM_END and
    // transitions to Store when one of those bytes is fetched as the first byte of an instruction.
    //
    // If NMI is high during ExecuteFinalize, reading the NMI vector completely will reset the store program
    // PC and also transition to ExecuteDone. This is the newer method of ending program execution that
    // supports the 80186 without queue status lines.
    case ServerState::ExecuteFinalize:

      handle_execute_finalize_state();
      break;

    case ServerState::EmuExit:
      if (!Controller.readMRDCPin()) {
        // CPU is reading (MRDC active-low)
        if ((CPU.bus_state_latched == CODE) && (CPU.bus_state == PASV)) {
          // CPU is doing code fetch
          if (CPU.s_pc < sizeof EMU_EXIT_PROGRAM) {
            // Read code byte from EmuExit program
            CPU.data_bus = EMU_EXIT_PROGRAM.read(CPU.address_latch(), CPU.data_width);
            Controller.getBoard().debugPrint(DebugType::EMU, "## EMUEXIT: fetching byte: ");
            Controller.getBoard().debugPrint(DebugType::EMU, CPU.data_bus, 16);
            Controller.getBoard().debugPrint(DebugType::EMU, " new s_pc: ");
            Controller.getBoard().debugPrintln(DebugType::EMU, CPU.s_pc);
            CPU.data_type = QueueDataType::Program;
          } else {
            CPU.data_bus = OPCODE_DOUBLENOP;
            CPU.data_type = QueueDataType::ProgramEnd;
          }
          Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        }

        if ((CPU.bus_state_latched == MEMR) && (CPU.bus_state == PASV)) {
          // CPU is doing memory read
          // This will occur when RETEM pops IP, CS and Flags from the stack.

          if (CPU.width == CpuBusWidth::Eight) {
            // Stack values will be read in two operations
            if (CPU.stack_r_op_ct == 0) {
              // Skip
            } 
            else if (CPU.stack_r_op_ct == 1) {
              // Skip
            } 
            else if (CPU.stack_r_op_ct == 2) {
              Controller.getBoard().debugPrint(DebugType::EMU, "## Reading RETEM CS pop (1/2): ");
              Controller.getBoard().debugPrintln(DebugType::EMU, CPU.load_regs.cs, 16);
              // Write the low byte of CS to the data bus
              data_bus_set_byte((uint8_t)(CPU.load_regs.cs));
            } 
            else if (CPU.stack_r_op_ct == 3) {
              Controller.getBoard().debugPrintln(DebugType::EMU, "## Reading RETEM CS pop (2/2)! ##");
              Controller.getBoard().debugPrintln(DebugType::EMU, CPU.load_regs.cs, 16);
              // Write the high byte of CS to the data bus
              data_bus_set_byte((uint8_t)(CPU.load_regs.cs >> 8));
            } 
            else if (CPU.stack_r_op_ct == 4) {
              Controller.getBoard().debugPrintln(DebugType::EMU, "## Reading RETEM flag pop (1/2)! ##");
              // Write the low byte of flags to the data bus
              data_bus_set_byte((uint8_t)(CPU.pre_emu_flags));
            } 
            else if (CPU.stack_r_op_ct == 5) {
              Controller.getBoard().debugPrintln(DebugType::EMU, "## Reading RETEM flag pop (2/2)! ##");
              // Write the high byte of flags to the data bus
              data_bus_set_byte((uint8_t)(CPU.pre_emu_flags >> 8));
              // Exit emulation mode
              CPU.in_emulation = false;
              ArduinoX86::Server.change_state(ServerState::ExecuteFinalize);
            } else {
              // Not flags, just write 0's so we jump back to CS:IP 0000:0000
              CPU.data_bus = 0;
            }
            CPU.stack_r_op_ct++;
          } else {
            // Sixteen-bit data bus
            if (CPU.stack_r_op_ct == 0) {
              // IP is read in one operation
              Controller.getBoard().debugPrintln(DebugType::EMU, "## Reading RETEM IP pop! ##");
              CPU.data_bus = 0;
            } else if (CPU.stack_r_op_ct == 1) {
              // CS is read in one operation
              Controller.getBoard().debugPrintln(DebugType::EMU, "## Reading RETEM CS pop! ##");
              Controller.getBoard().debugPrintln(DebugType::EMU, CPU.load_regs.cs);
              // We can restore CS from the loaded registers since CS cannot be modified in 8080 emulation mode
              CPU.data_bus = CPU.load_regs.cs;
            } else if (CPU.stack_r_op_ct == 2) {
              // Flags will be read in one operation
              Controller.getBoard().debugPrintln(DebugType::EMU, "## Reading RETEM Flag pop! ##");
              // CPU is writing to the data bus, latch value
              CPU.data_bus = CPU.pre_emu_flags;
              // Exit emulation mode
              CPU.in_emulation = false;
              ArduinoX86::Server.change_state(ServerState::ExecuteFinalize);
            }
            CPU.stack_r_op_ct++;
          }
          Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        }
      }

      if (!Controller.readMWTCPin() && (CPU.bus_state_latched == MEMW) && (CPU.bus_state == PASV)) {
        // CPU is writing. This should only happen during EmuExit when we PUSH PSW
        // to save the 8080 flags.
        if (CPU.width == CpuBusWidth::Eight) {
          // 8-bit data bus
          if (CPU.stack_w_op_ct == 0) {
            // Flags will be in first byte written (second byte will be AL)
            Controller.getBoard().debugPrint(DebugType::EMU, "## Capturing PUSH PSW stack write! ##");
            CPU.emu_flags = (uint8_t)CPU.data_bus;
          }
          CPU.stack_w_op_ct++;
        } else {
          // 16-bit data bus
          if (CPU.stack_w_op_ct == 0) {
            // Flags were pushed in one operation.
            Controller.getBoard().debugPrintln(DebugType::EMU, "## Capturing PUSH PSW stack write! ##");
            CPU.emu_flags = (uint8_t)CPU.data_bus;
          }
          CPU.stack_w_op_ct++;
        }
      }

      break;

    case ServerState::ExecuteDone:
      // We sit in ExecuteDone state until the client requests a Store operation.
      // The client should not cycle the CPU in this state.
      if (!Controller.readMRDCPin() && CPU.bus_state == PASV) {
        // CPU is reading (MRDC active-low)
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);

        if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
          // Since client does not cycle the CPU in this state, we have to fetch from
          // STORE program ourselves
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          //CPU.data_bus = STORE_PROGRAM[CPU.s_pc++];
          CPU.data_type = QueueDataType::ProgramEnd;
          Controller.writeDataBus(CPU.data_bus, CPU.data_width);
          CPU.program->debug_print(Controller.getBoard(), DebugType::STORE, "## STORE", CPU.data_bus);
        } 
        else {
          Controller.getBoard().debugPrintln(DebugType::ERROR, "## Invalid condition: ExecuteDone without loading STORE");
          Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        }
      }
      break;

    case ServerState::Store:
      handle_store_state();
      break;

    case ServerState::StoreAll:
      if (CPU.cpu_type == CpuType::i80286) {
        handle_storeall_286();
      } else if (CPU.cpu_type == CpuType::i80386) {
        handle_smm_store_386();
      } else {
        ArduinoX86::Server.change_state(ServerState::Error);
        set_error("StoreAll not implemented for this CPU type!");
      }
      break;

    case ServerState::IceRecovery286:
      handle_ice_recovery_286();
      break;

    case ServerState::StoreDoneSmm: // FALLTHROUGH
    case ServerState::TrapStoreDone: // FALLTHROUGH
    case ServerState::StoreDone:
      // We are done with the Store program.
      break;
    case ServerState::Done:
      break;
    case ServerState::Shutdown: // FALLTHROUGH
    case ServerState::Error:
      if (CPU.error_cycle_ct < MAX_ERROR_CYCLES) {
        CPU.error_cycle_ct++;
      }
      break;
  }

  if (Controller.getBoard().isDebugEnabled()) {
    // Print instruction state if tracing is enabled
    switch (ArduinoX86::Server.state()) {
      case ServerState::Reset:
        #if TRACE_RESET
          print_cpu_state();
        #endif
        break;
      case ServerState::CpuId:
        #if TRACE_ID
          print_cpu_state();
        #endif
        break;
      case ServerState::CpuSetup:
        #if TRACE_SETUP
          print_cpu_state();  
        #else
          //delayMicroseconds(20);
        #endif
        break;
      case ServerState::JumpVector:
        #if TRACE_VECTOR
          print_cpu_state();
        #endif
        break;
      case ServerState::Load:  // FALLTHROUGH
      case ServerState::LoadSmm: // FALLTHROUGH
      case ServerState::LoadDone:
        #if TRACE_LOAD
          print_cpu_state();
        #endif
        break;
      case ServerState::Prefetch:
        #if TRACE_PREFETCH
          print_cpu_state();
        #endif
        break;
      case ServerState::EmuEnter:
        #if TRACE_EMU_ENTER
          print_cpu_state();
        #endif
        break;
      case ServerState::EmuExit:
        #if TRACE_EMU_EXIT
          print_cpu_state();
        #endif
        break;
      case ServerState::Execute:
        #if TRACE_EXECUTE
          print_cpu_state();
        #endif
        break;
      case ServerState::ExecuteDone:  // FALLTHROUGH
      case ServerState::ExecuteFinalize:
        #if TRACE_FINALIZE
          print_cpu_state();
        #endif
        break;
      case ServerState::Done: // FALLTHROUGH
      case ServerState::StoreDone:  // FALLTHROUGH
      case ServerState::StoreDoneSmm: // FALLTHROUGH
      case ServerState::Store: // FALLTHROUGH
      case ServerState::StoreAll: // FALLTHROUGH
      case ServerState::IceRecovery286:
        #if TRACE_STORE
          print_cpu_state();
        #endif
        break;

      case ServerState::Shutdown: // FALLTHROUGH
      case ServerState::Error:
        print_cpu_state();
        break;
    }
  }

  // Log cycle state.
  if (server_flags & CommandServer<BoardType, ShieldType>::FLAG_LOG_CYCLES) {
    cycle_state.data_bus = CPU.data_bus;
    cycle_state.pins = 0;

    if (ale) {
      cycle_state.pins |= CycleState::ALE;
    }
    if (Controller.readBHEPin()) {
      cycle_state.pins |= CycleState::BHE;
    }
    if (Controller.readLockPin()) {
      cycle_state.pins |= CycleState::LOCK;
    }
    if (Controller.readReadyPin()) {
      cycle_state.pins |= CycleState::READY;
    }    
    ArduinoX86::CycleLogger->log(cycle_state);
  }
  
  // Handle wait states - doing this after logging cycle simulates READY going low sometime during
  // Tc.
  if (ale && CPU.wait_states > 0) {
    // Lower READY line on ALE.
    //Controller.getBoard().debugPrintln(DebugType::BUS, "## Wait state requested ##");
    //Controller.writePin(OutputPin::Ready, READY_DEASSERT);
    CPU.wait_state_ct = 0;
  }

  // Print deferred debug string if it exists
  Controller.getBoard().debugPrintDeferred();

  // Transition to next T-state.
  CPU.last_bus_cycle = CPU.bus_cycle;
  CPU.bus_cycle = Controller.getNextCycle(CPU.bus_cycle, CPU.bus_state, CPU.bus_state_latched);
}

void handle_fetch(uint8_t q) {
  // Did we complete a code fetch? If so, increment queue len
  if (CPU.bus_state_latched == CODE) {
    //Controller.getBoard().debugPrintln(DebugType::QUEUE, "## QUEUE: T4 of code fetch!");

    if (q == QUEUE_FLUSHED) {
      //Controller.getBoard().debugPrintln(DebugType::QUEUE, "## Queue flush during T4.");
      if (CPU.queue.have_room(CPU.data_width)) {
        CPU.queue.push(CPU.data_bus, CPU.data_type, CPU.data_width);
      } else {
        // No room for fetch - this shouldn't happen!
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## Error: Invalid Queue Length++ ##");
      }
    } else {
      if (CPU.queue.have_room(CPU.data_width)) {
        //Controller.getBoard().debugPrint(DebugType::QUEUE, "## QUEUE: T4, Pushing data bus to queue: ");
        //Controller.getBoard().debugPrintln(DebugType::QUEUE, CPU.data_bus, HEX);
        CPU.queue.push(CPU.data_bus, CPU.data_type, CPU.data_width);
      } else {
        // Shouldn't be here
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## Error: Invalid Queue Length++ ##");
      }
    }
  }
}

void handle_cpuid_state(uint8_t q) {

  if (q == QUEUE_FIRST) {
    if (CPU.cpuid_queue_reads == 0) {
      Controller.getBoard().debugPrintln(DebugType::ID, "## CPUID: Starting CPUID counter! ##");
      CPU.cpuid_counter = 0;
    } else if (CPU.cpuid_queue_reads == 1) {
      Controller.getBoard().debugPrint(DebugType::ID, "## CPUID: CPUID counter started at: ");
      Controller.getBoard().debugPrint(DebugType::ID, CPU.cpuid_counter);
      Controller.getBoard().debugPrintln(DebugType::ID, " ##");
      detect_cpu_type(CPU.cpuid_counter);
    }
    CPU.cpuid_queue_reads++;
  }

  // Change state after we have executed a minimum number of instructions
  if (CPU.cpuid_queue_reads > 4) {
#if USE_SETUP_PROGRAM
    ArduinoX86::Server.change_state(ServerState::CpuSetup);
#else
    ArduinoX86::Server.change_state(ServerState::JumpVector);
#endif
  }

  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if ((CPU.bus_state_latched == CODE) && (CPU.bus_cycle == WRITE_CYCLE)) {
      // We are reading a code byte

      // Feed the program if we haven't this bus cycle.
      if (!CPU.data_bus_resolved) {
        // Feed CPU ID instruction to CPU.
        CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
        CPU.data_type = QueueDataType::Program;
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
        CPU.program->debug_print(Controller.getBoard(), DebugType::ID, "## CPUID", CPU.data_bus);
      }
    }
  }

  if (!Controller.readMWTCPin() && READ_TEST_PIN) {
    // FPU is writing to bus
    if (CPU.data_bus == 0x03FF) {
      // Have an 8087!
      Controller.getBoard().debugPrintln(DebugType::ID, "## CPUID: Detected 8087 status word write!");
      // This doesn't do anything atm besides set an 8087
      detect_fpu_type();
    }
  }
}

// Handle the CpuSetup state.
// This state is not for register loading, but to handle CPUs that require some configuration before
// they can be used by the cpu_server. Currently the only CPUs that need setup are the 188 and 186
// which have to have interrupts enabled via the PCB as they are masked off at RESET.
void handle_cpu_setup_state() {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE) {
      // We are reading a code byte

      // Feed the program if we haven't this bus cycle.
      if (!CPU.data_bus_resolved) {
        if (CPU.program->has_remaining()) {
          // Feed SETUP_PROGRAM instruction to CPU.
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
          Controller.writeDataBus(CPU.data_bus, CPU.data_width);
          CPU.program->debug_print(Controller.getBoard(), DebugType::SETUP, "## SETUP_PROGRAM", CPU.data_bus);
        } else {
          // Ran out of program, so return NOP. Doesn't matter what we feed
          // as queue will be reset.
          CPU.data_bus = read_nops(CPU.data_width);
          CPU.data_type = QueueDataType::ProgramEnd;
        }
        CPU.data_bus_resolved = true;
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
      }
    }
  }

  if (Controller.readALEPin()) {
    // Jump is finished on first address latch of LOAD_SEG:0
    uint32_t dest = calc_flat_address(LOAD_SEG, 0);
    if (dest == CPU.address_latch()) {
      Controller.getBoard().debugPrintf(DebugType::SETUP, false, "## ALE at LOAD_SEG. Transitioning to Load state. SEG: %08X\n\r", CPU.address_latch());
      // Transition to Load state.
      ArduinoX86::Server.change_state(ServerState::Load);
    }
  }
}

void handle_jump_vector_state(uint8_t q) {

  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if ((CPU.bus_state_latched == CODE) && (CPU.bus_cycle == WRITE_CYCLE)) {
      // We are reading a code byte.
      // If the data bus hasn't been resolved this m-cycle, feed in the JUMP_VECTOR program
      if (!CPU.data_bus_resolved) {
        if (CPU.program->has_remaining()) {
          // Feed jump instruction to CPU
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
        } else {
          // Ran out of program, so return NOP. Doesn't matter what we feed
          // as queue will be reset.
          CPU.data_bus = read_nops(CPU.data_width);
          CPU.data_type = QueueDataType::ProgramEnd;
        }
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
        CPU.program->debug_print(Controller.getBoard(), DebugType::VECTOR, "## JUMP_VECTOR", CPU.data_bus);
      }
    }
  }

  if (Controller.readALEPin()) {
    // Jump is finished on first address latch of LOAD_SEG:0
    uint32_t dest = calc_flat_address(LOAD_SEG, 0);
    if (dest == (CPU.address_latch() & ~0x000F)) {
      if (dest == (CPU.address_latch())) {
        Controller.getBoard().debugPrintf(DebugType::VECTOR, false, "## ALE at LOAD_SEG: %08X. Transitioning to Load state.", CPU.address_latch());
      } else {
        Controller.getBoard().debugPrintf(DebugType::ERROR, false, "## ALE at LOAD_SEG: %08X. Transitioning to Load state. (masked)", CPU.address_latch());
      }
      ArduinoX86::Server.change_state(ServerState::Load);
    }
  }
}

void handle_loadall_286() {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE) {
      // We are reading a code byte

      // If we haven't resolved the data bus this bus cycle...
      if (!CPU.data_bus_resolved) {
        if (CPU.program->has_remaining()) {
          // Feed load program to CPU
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
        } else {
          // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
          // so we wil see this NOP prefetched.
          CPU.data_bus = OPCODE_DOUBLENOP;
          CPU.data_type = QueueDataType::ProgramEnd;
          //change_state(LoadDone);
        }
        CPU.program->debug_print(Controller.getBoard(), DebugType::LOAD, "## LOADALL_286", CPU.data_bus);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }
    }

    if (CPU.bus_state_latched == MEMR) {
      // We are reading a memory word
      if ((CPU.address_latch() >= LOADALL286_ADDRESS) && (CPU.address_latch() < (LOADALL286_ADDRESS + sizeof CPU.loadall_regs_286))) {
        CPU.loadall_checkpoint++;
        size_t idx = (CPU.address_latch() - LOADALL286_ADDRESS) / 2;
        uint16_t *word_ptr = ((uint16_t *)&CPU.loadall_regs_286);
        CPU.data_bus = word_ptr[idx];
        Controller.getBoard().debugPrint(DebugType::LOAD, "## LOADALL_286: Writing LOADALL word to bus: ", true);
        Controller.getBoard().debugPrintln(DebugType::LOAD, CPU.data_bus, 16, true);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      } else {
        // Unexpected read out of LOADALL range.
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## LOADALL_286: INVALID MEM READ ##");
      }
    }
  }

  // We can't tell when the queue flushed but we can see the initial code fetch at the new CS:IP.
  // We don't need to enter LoadDone in this case, we can jump directly to Execute as all LoadDone does is wait
  // for ALE. (TODO: Should this just be the primary way we leave Load?)
  uint32_t base_address = (static_cast<uint32_t>(CPU.loadall_regs_286.cs_desc.addr_hi)) << 16 | 
                          (static_cast<uint32_t>(CPU.loadall_regs_286.cs_desc.addr_lo));

  uint32_t run_address = base_address + static_cast<uint32_t>(CPU.loadall_regs_286.ip);
  
  if (CPU.loadall_checkpoint > 0 && CPU.bus_state == CODE) {
    if (CPU.address_latch() == run_address) {
      Controller.getBoard().debugPrintln(DebugType::LOAD, "## LOADALL_286: Detected jump to new CS:IP to trigger transition into Execute.");
      ArduinoX86::Server.change_state(ServerState::Execute);
      if (CPU.trap.enabled) {
        handle_execute_trap();
      }
      else {
        handle_execute_state(); // Handle the CODE fetch that triggered the transition
      }
      
    }
    else {
      // CPU is prefetching after LOADALL, but not at the expected address. This is an error.
      
      Controller.getBoard().debugPrintf(
        DebugType::ERROR, false, 
        "## LOADALL_286: Unexpected prefetch address: %06X Expected: %06X\n\r", 
        CPU.address_latch(), run_address);

      set_error("Unexpected prefetch address after LOADALL_286");
      ArduinoX86::Server.change_state(ServerState::Error);
    }
  }
}


void handle_loadall_386() {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE) {
      // We are reading a code byte

      // If we haven't resolved the data bus this bus cycle...
      if (!CPU.data_bus_resolved) {
        if (CPU.program->has_remaining()) {
          // Feed load program to CPU
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
        } else {
          // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
          // so we wil see this NOP prefetched.
          CPU.data_bus = OPCODE_DOUBLENOP;
          CPU.data_type = QueueDataType::ProgramEnd;
          //change_state(LoadDone);
        }
        CPU.program->debug_print(Controller.getBoard(), DebugType::LOAD, "## LOADALL_386", CPU.data_bus);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }
    }

    if (CPU.bus_state_latched == MEMR) {
      // We are reading a memory word

      if (CPU.address_latch() == EXCEPTION_ADDR_GPF) {
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## LOADALL_386: General Protection Fault detected during LOADALL! ##");
        set_error("LOADALL_386 General Protection Fault");
        ArduinoX86::Server.change_state(ServerState::Error);
        return;
      }

      if ((CPU.address_latch() >= LOADALL386_ADDRESS) && (CPU.address_latch() < (LOADALL386_ADDRESS + sizeof CPU.loadall_regs_386))) {

        CPU.loadall_checkpoint++;

        size_t idx = (CPU.address_latch() - LOADALL386_ADDRESS) / 2;
        uint16_t *word_ptr = ((uint16_t *)&CPU.loadall_regs_386);
        CPU.data_bus = word_ptr[idx];
        Controller.getBoard().debugPrint(DebugType::LOAD, "## LOADALL_386: Writing LOADALL word to bus: ", true);
        Controller.getBoard().debugPrintln(DebugType::LOAD, CPU.data_bus, 16, true);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      } else {
        // Unexpected read out of LOADALL range.
        //Controller.getBoard().debugPrintln(DebugType::ERROR, "## LOADALL_386: INVALID MEM READ ##");
      }
    }
  }

  // We can't tell when the queue flushed but we can see the initial code fetch at the new CS:IP.
  // We don't need to enter LoadDone in this case, we can jump directly to Execute as all LoadDone does is wait
  // for ALE. (TODO: Should this just be the primary way we leave Load?)
  uint32_t base_address = CPU.loadall_regs_386.cs_desc.address;
  uint32_t run_address = base_address + CPU.loadall_regs_386.eip;
  
  if (CPU.loadall_checkpoint > 0 && CPU.bus_state_latched == CODE) {
    if (CPU.address_latch() == run_address) {
      Controller.getBoard().debugPrintln(DebugType::LOAD, "## LOADALL_386: Detected jump to new CS:IP to trigger transition into Execute.");
      ArduinoX86::Server.change_state(ServerState::Execute);
      handle_execute_state(); // Handle the CODE fetch that triggered the transition
    }
    else {
      // CPU is prefetching after LOADALL, but not at the expected address. This is an error.
      
      Controller.getBoard().debugPrintf(
        DebugType::ERROR, false, 
        "## LOADALL_386: Unexpected prefetch address: %06X Expected: %06X\n\r", 
        CPU.address_latch(), run_address);

      set_error("Unexpected prefetch address after LOADALL_386");
      ArduinoX86::Server.change_state(ServerState::Error);
    }
  }
}

void handle_smm_load_386() {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE && !CPU.data_bus_resolved) {
      // We are reading a code byte
      if (CPU.address_latch() >= SMM_HANDLER_START_ADDRESS && CPU.address_latch() < SMM_HANDLER_END_ADDRESS) {
        // We are fetching the SMM handler program.
        if (CPU.program->has_remaining()) {
          // Feed load program to CPU
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
        } else {
          CPU.data_bus = OPCODE_DOUBLENOP;
          CPU.data_type = QueueDataType::ProgramEnd;
        }
        CPU.program->debug_print(Controller.getBoard(), DebugType::LOAD, "## LOAD_SMM_386", CPU.data_bus);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }
      else {

      }
    }
    else if (CPU.bus_state_latched == MEMR && !CPU.data_bus_resolved) {

      if (CPU.address_latch() == EXCEPTION_ADDR_GPF) {
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## LOAD_SMM_386: General Protection Fault detected during SMM LOAD! ##");
        set_error("LOAD_SMM_386 General Protection Fault");
        ArduinoX86::Server.change_state(ServerState::Error);
        return;
      }

      if ((CPU.address_latch() >= SMRAM_386EX_DUMP_START) && (CPU.address_latch() < SMRAM_386EX_END_ADDRESS)) {
        // Read from bus emulator with SMIACT set to get word from register struct.
        CPU.data_bus = ArduinoX86::Bus->mem_read_bus(CPU.address_latch(), !Controller.readBHEPin(), false, true);
        Controller.getBoard().debugPrintf(DebugType::LOAD, true, "## LOAD_SMM_386: Writing SMM dump word to bus: %04X\n\r", CPU.data_bus);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }
      else {
        CPU.data_bus = 0x0000;
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }

      if (CPU.loadall_checkpoint == 0 && (CPU.address_latch() == SMM_LOAD_CHECKPOINT)) {
        Controller.getBoard().debugPrintf(DebugType::LOAD, false, "## LOAD_SMM_386: Passed checkpoint. Deasserting SMI. Next fetch should be for program\n\r");
        Controller.writePin(OutputPin::Smi, true); // SMI is active-low
        CPU.loadall_checkpoint = 1;
      }
    }
  }

  // We can't tell when the queue flushed but we can see the initial code fetch at the new CS:IP.
  // We don't need to enter LoadDone in this case, we can jump directly to Execute as all LoadDone does is wait
  // for ALE. (TODO: Should this just be the primary way we leave Load?)
  uint32_t base_address = ArduinoX86::Bus->smm_dump386_regs().cs_desc.address;
  uint32_t run_address = base_address + ArduinoX86::Bus->smm_dump386_regs().eip;
  
  if (CPU.loadall_checkpoint > 0 && CPU.bus_state == CODE) {
    if (CPU.address_latch() == run_address) {
      Controller.getBoard().debugPrintln(DebugType::LOAD, "## LOAD_SMM_386: Detected jump to new CS:IP to trigger transition into Execute.");
      ArduinoX86::Server.change_state(ServerState::Execute);
    }
    else {
      // CPU is prefetching after LOADALL, but not at the expected address. This is an error.
      Controller.getBoard().debugPrintf(
        DebugType::ERROR, false, 
        "## LOAD_SMM_386: Unexpected prefetch address: %06X Expected: %06X\n\r", 
        CPU.address_latch(), run_address);

      set_error("Unexpected prefetch address after LOAD_SMM_386");
      ArduinoX86::Server.change_state(ServerState::Error);
    }
  }  
}

void handle_storeall_286() {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE) {
      // We are reading a code byte

      // If we haven't resolved the data bus this bus cycle...
      if (!CPU.data_bus_resolved) {
        if (CPU.program->has_remaining()) {
          // Feed load program to CPU
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
        } else {
          // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
          // so we wil see this NOP prefetched.
          CPU.data_bus = OPCODE_DOUBLENOP;
          CPU.data_type = QueueDataType::ProgramEnd;
          //change_state(LoadDone);
        }
        CPU.program->debug_print(Controller.getBoard(), DebugType::STORE, "## STOREALL_286", CPU.data_bus);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }
    }
    else {
      if (CPU.address_latch() == 0x000864) {
        // STOREALL terminating read.
        Controller.getBoard().debugPrintln(DebugType::STORE, "## STOREALL_286: Terminating read at 0x000864");
        if (CPU.trap.capturing) {
          CPU.trap.complete_capture(ArduinoX86::Bus->loadall286_regs(), CPU.cycle_ct());
          CPU.data_bus = CPU.trap.capture_image.tss_desc.limit;
          Controller.writeDataBus(CPU.data_bus, CPU.data_width);
          CPU.data_bus_resolved = true;
          ArduinoX86::Server.change_state(CPU.trap.state.status == TrapStatus::Paused
            ? ServerState::TrapStoreDone : ServerState::StoreDone);
        }
        else {
          ArduinoX86::Server.change_state(ServerState::StoreDone);
        }
      }
    }
  }

  if (!Controller.readMWTCPin()) {
    // CPU is writing (MWTC active-low)
    Controller.getBoard().debugPrintf(DebugType::STORE, true, "## STOREALL_286: Sending write to bus emulator: %04X\n\r", CPU.data_bus);
    ArduinoX86::Bus->mem_write_bus(CPU.address_latch(), CPU.data_bus, !Controller.readBHEPin());
  }
}

/// @brief Handle the SMM register store operation for 386 CPUs.
void handle_smm_store_386() {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE) {
      // We are reading a code byte

      // If we haven't resolved the data bus this bus cycle...
      if (!CPU.data_bus_resolved) {
        if (CPU.program->has_remaining()) {
          // Feed load program to CPU
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
        } else {
          // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
          // so we wil see this NOP prefetched.
          CPU.data_bus = OPCODE_DOUBLENOP;
          CPU.data_type = QueueDataType::ProgramEnd;
          //change_state(LoadDone);
        }
        CPU.program->debug_print(Controller.getBoard(), DebugType::STORE, "## STORE_SMM_386", CPU.data_bus);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }
    }
  }

  if (!Controller.readMWTCPin()) {
    // CPU is writing (MWTC active-low)
    Controller.getBoard().debugPrintf(DebugType::STORE, true, "## STORE_SMM_386: Sending write to bus emulator: %04X\n\r", CPU.data_bus);
    ArduinoX86::Bus->mem_write_bus(CPU.address_latch(), CPU.data_bus, !Controller.readBHEPin(), true);

    if (CPU.address_latch() == SMRAM_LAST_WRITE) {
      // SMM terminating write.
      Controller.getBoard().debugPrintf(DebugType::STORE, false, "## STORE_SMM_386:: Terminating read at %08X\n\r", SMRAM_LAST_WRITE);
      ArduinoX86::Server.change_state(ServerState::StoreDoneSmm);
    }
  }
}

void handle_load_state(uint8_t q) {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE) {
      // We are reading a code byte

      // If we haven't resolved the data bus this bus cycle...
      if (!CPU.data_bus_resolved) {
        if (CPU.program->has_remaining()) {
          // Feed load program to CPU
          CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
        } else {
          // Ran out of program, so return NOP. JMP cs:ip will actually fetch once before SUSP,
          // so we wil see this NOP prefetched.

          CPU.data_bus = OPCODE_DOUBLENOP;
          CPU.data_type = QueueDataType::ProgramEnd;
          //change_state(LoadDone);
        }
        Controller.getBoard().debugPrintf(DebugType::LOAD, true, "## LOAD: Writing LOAD program to bus: %04X\n\r", CPU.data_bus);
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
        CPU.data_bus_resolved = true;
      }
    }

    if (CPU.cpu_type != CpuType::i80286) {
      if (CPU.bus_state == MEMR) {
        // We are reading a memory byte
        // This should only occur during Load when flags are popped from 0:0
        if (CPU.address_latch() < 0x00002) {
          // First two bytes of LOAD_PROGRAM were patched with flags
          CPU.data_bus = LOAD_PROGRAM.read_at(0x00000, CPU.address_latch(), CPU.data_width);
          CPU.data_type = QueueDataType::Program;
          Controller.writeDataBus(CPU.data_bus, CPU.data_width);
          CPU.data_bus_resolved = true;
        } else {
          // Unexpected read above address 0x00001
          debugPrintlnColor(ansi::bright_red, "## INVALID MEM READ DURING LOAD ##");
        }
      }
    }

  }

#if HAVE_QUEUE_STATUS
  if (q == QUEUE_FLUSHED) {
    Controller.getBoard().debugPrintln(DebugType::LOAD, "## Detected queue flush to trigger transition into LoadDone");
    // Queue flush after final jump triggers next state.
    ArduinoX86::Server.change_state(ServerState::LoadDone);
  }
#else
  // We can't tell when the queue flushed but we can see the initial code fetch at the new CS:IP.
  // We don't need to enter LoadDone in this case, we can jump directly to Execute as all LoadDone does is wait
  // for ALE. (TODO: Should this just be the primary way we leave Load?)
  uint32_t run_address = calc_flat_address(CPU.load_regs.cs, CPU.load_regs.ip);
  if (CPU.address_latch() == run_address) {
    Controller.getBoard().debugPrint(DebugType::LOAD, "## 186: Detected jump to new CS:IP to trigger transition into Execute.");
    ArduinoX86::Server.change_state(ServerState::Execute);
  }
#endif
}

void handle_load_done_state() {
  if (Controller.readALEPin() && (CPU.bus_state == CODE)) {
    // First bus cycle of the instruction to execute. Transition to Execute or EmuEnter as appropriate.
    if (CPU.do_emulation && !CPU.in_emulation) {
      ArduinoX86::Server.change_state(ServerState::EmuEnter);
    } else {
      ArduinoX86::Server.change_state(ServerState::Execute);
    }
  }
}

void handle_emu_enter_state(uint8_t q) {
  if (!Controller.readMRDCPin()) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state == CODE) {
      // We are reading a code byte
      if (CPU.program->has_remaining()) {
        // Feed load program to CPU
        CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
        CPU.data_type = QueueDataType::Program;
      } else {
        // Ran out of program, so return NOP.
        CPU.data_bus = OPCODE_DOUBLENOP;
        CPU.data_type = QueueDataType::ProgramEnd;
        //change_state(LoadDone);
      }
      Controller.writeDataBus(CPU.data_bus, CPU.data_width);
    }

    if (CPU.bus_state == MEMR) {
      // We are reading from memory
      // This will occur when BRKEM reads the emulation segment vector
      uint32_t vector_base = BRKEM_VECTOR * 4;
      if ((CPU.address_latch() >= vector_base) && (CPU.address_latch() < vector_base + 4)) {
        if (CPU.address_latch() < (vector_base + 2)) {

#if DEBUG_EMU
          // Reading offset, feed IP
          DEBUG_SERIAL.println("## Reading BRKEM offset! ##");
#endif
        } else {
#if DEBUG_EMU
          // Reading segment
          DEBUG_SERIAL.println("## Reading BRKEM segment! ##");
#endif
        }
        CPU.data_bus = EMU_ENTER_PROGRAM.read_at(vector_base, CPU.address_latch(), CPU.data_width);
        CPU.data_type = QueueDataType::Program;
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);
      } else {
        // Unexpected read above address 0x00001
        DEBUG_SERIAL.println("## INVALID MEM READ DURING EMUENTER ##");
      }
    }
  }

  if (!Controller.readMWTCPin()) {
    if (CPU.width == CpuBusWidth::Eight) {
      // Flags will be read in two operations
      if (CPU.stack_w_op_ct == 0) {
#if DEBUG_EMU
        DEBUG_SERIAL.println("## Reading BRKEM flag push (1/2)! ##");
#endif
        CPU.pre_emu_flags = (uint16_t)data_bus_read_byte();
      } else if (CPU.stack_w_op_ct == 1) {
#if DEBUG_EMU
        DEBUG_SERIAL.println("## Reading BRKEM flag push (2/2)! ##");
#endif
        CPU.pre_emu_flags |= ((uint16_t)data_bus_read_byte() << 8);
      }
      CPU.stack_w_op_ct++;
    } else {
      // Flags will be read in one operation
      if (CPU.stack_w_op_ct == 0) {
#if DEBUG_EMU
        DEBUG_SERIAL.println("## Reading BRKEM flag push! ##");
#endif
        // CPU is writing to the data bus, latch value
        CPU.data_bus = Controller.readDataBus(CPU.data_width);
        CPU.pre_emu_flags = CPU.data_bus;
      }
      CPU.stack_w_op_ct++;
    }
  }

  if (q == QUEUE_FLUSHED) {
    // Queue flush after final jump triggers next state.
    CPU.in_emulation = true;
    ArduinoX86::Server.change_state(ServerState::LoadDone);
  }
}

void handle_execute_state() {

  bool cpu_mrdc = !Controller.readMRDCPin();
  bool cpu_iodc = !Controller.readIORCPin();
  bool cpu_mwtc = !Controller.readMWTCPin();
  bool cpu_iowc = !Controller.readIOWCPin();

  if (cpu_mwtc) {
    Controller.getBoard().debugPrintf(
      DebugType::EXECUTE, 
      true, 
      "## EXECUTE: Sending write to bus emulator: [%08X]<-%04X\n\r", 
      CPU.address_latch(), 
      CPU.data_bus
    );
    ArduinoX86::Bus->mem_write_bus(CPU.address_latch(), CPU.data_bus, !Controller.readBHEPin());
  }

  if (CPU.bus_cycle == T1 && (CPU.bus_state == CODE || CPU.bus_state == MEMR)) {
    // Resolve the latched read request on T1. MRDC is asserted later (T2 on the 286)
    if (ArduinoX86::Server.get_flags() & CommandServer<BoardType, ShieldType>::FLAG_RESOLVE_BUS_STEP) {
      CPU.data_bus = ArduinoX86::Bus->mem_read_bus(CPU.address_latch(), !Controller.readBHEPin(), false);
      Controller.getBoard().debugPrintf(
        DebugType::EXECUTE, 
        true, 
        "## EXECUTE: Reading from bus emulator: [%08X]->%04X\n\r", 
        CPU.address_latch(), 
        CPU.data_bus
      );
    }
  }
  else if ((cpu_mrdc || cpu_iodc) && CPU.bus_cycle == WRITE_CYCLE) {
    
    Controller.writeDataBus(CPU.data_bus, CPU.data_width);
    Controller.getBoard().debugPrintf(DebugType::EXECUTE, true, "## EXECUTE: Wrote bus: %04X\n\r", CPU.data_bus);
    //Controller.getBoard().debugPrintln(DebugType::EXECUTE, CPU.data_bus, 16);

    if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
      CPU.program->debug_print(Controller.getBoard(), DebugType::STORE, "## EXECUTE: Prefetching STORE program byte", CPU.data_bus);
    }
  }

  if (CPU.bus_state == HALT) {
    Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: HALT detected - Ending program execution.");
    Controller.writePin(OutputPin::Nmi, true);

    if (CPU.cpu_type != CpuType::i80386) {
      // The 386 keeps a persistent HALT bus state, so we can't return or we'll never escape it.
      return;
    }
  }

  if (READ_NMI_PIN) {
    // Use checkpoint "1" to specify that NMI has been detected. This just prevents the debug message from
    // printing every cycle after NMI.
    if (CPU.nmi_checkpoint == 0) {
      Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: NMI pin high - Execute will end at IVT fetch.");
      CPU.nmi_checkpoint = 1;
    }
  }

  if (Controller.readALEPin() && CPU.bus_state == MEMR) {
    Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: ALE high and MEMR cycle detected.");
    // NMI is active and CPU is starting a memory bus cycle. Let's check if it is the NMI handler.
    if ((READ_NMI_PIN) && (CPU.address_latch() == 0x00008)) {
      Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: NMI high and fetching NMI handler. Entering ExecuteFinalize...");
      CPU.nmi_terminate = true;
      ArduinoX86::Server.change_state(ServerState::ExecuteFinalize);
    }
  }

  if (!READ_SMI_PIN) {
    Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: SMI asserted.");
    // Sometimes the bus is PASV[1] on 386 during ADS. Not sure why.
    Controller.getBoard().debugPrintf(DebugType::EXECUTE, false, "## EXECUTE: SMI asserted, ads is %s, bus state is %d\n\r", Controller.readALEPin() ? "high" : "low", (CPU.status0 & 0xF));
    if (Controller.readALEPin() && ((CPU.bus_state == MEMW) || ((CPU.status0 & 0xF) == 1))) {

      Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: ALE high and MEMW cycle detected.");
      // NMI is active and CPU is starting a memory bus cycle. Let's check if it is the NMI handler.
      if (CPU.address_latch() == SMRAM_FIRST_WRITE) {
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: SMI asserted and writing to SMRAM. Entering Store...");
        CPU.smi_terminate = true;
        ArduinoX86::Server.change_state(ServerState::StoreAll);
      }
    }
  }
}

/// @brief Handle program execution in automatic mode.
void handle_execute_automatic() {
  bool cpu_mrdc = !Controller.readMRDCPin();
  bool cpu_iorc = !Controller.readIORCPin();
  bool cpu_mwtc = !Controller.readMWTCPin();
  bool cpu_iowc = !Controller.readIOWCPin();
  static bool far_call_flag = false;
  static bool exception_address_odd = false;
  
  static uint32_t next_exception_address = 0;
  uint32_t exception_number = 0;

  bool print = Controller.getBoard().isDebugEnabled();

  if (cpu_mwtc) {
    // The CPU is writing to memory. Send it to the bus emulator.
    if (print) {
      // Logging is wrapped in "if (print)" to avoid unnecessary string formatting during normal operation.
      Controller.getBoard().debugPrintf(
        DebugType::EXECUTE,
        true,
        "## EXECUTE: Sending write to bus emulator: [%08X]<-%04X\n\r",
        CPU.address_latch(),
        CPU.data_bus
      );
    }
    ArduinoX86::Bus->mem_write_bus(CPU.address_latch(), CPU.data_bus, !Controller.readBHEPin());

    if (CPU.cpu_type != CpuType::i80386) {
      // Detect farcall / exception via stack frame push.
      // This happens first on the 286, but the 386 performs an initial code fetch before pushing the stack frame,
      // so we can't rely on it.
      far_call_flag = ArduinoX86::Bus->far_call_detected();
    }
    else {
      far_call_flag = true;
    }
  }

  if (cpu_mrdc && CPU.bus_cycle == WRITE_CYCLE) {
    if (CPU.cpu_type != CpuType::i80386) {
      far_call_flag = false;
    }
    // CPU is reading memory. Read from the bus emulator.
    
    // Set the fetch flag if we're in a code fetch cycle.
    bool is_fetch = (CPU.bus_state_latched == CODE);
    CPU.data_bus = ArduinoX86::Bus->mem_read_bus(CPU.address_latch(), !Controller.readBHEPin(), is_fetch);

    if (print) {
      if (is_fetch) {
        Controller.getBoard().debugPrintf(DebugType::EXECUTE, true, "## EXECUTE: Prefetching from bus emulator: %04X\n\r", CPU.data_bus);
      }
      else {
        Controller.getBoard().debugPrintf(DebugType::EXECUTE, true, "## EXECUTE: Reading from bus emulator: %04X\n\r", CPU.data_bus);
      }
    }

    if (CPU.exception_stage == 1) {
      // We are reading the IVT offset.
      if (CPU.data_bus & 1) {
        // Offset is odd. We need to detect this for the 386EX as it only emits even addresses when fetching.
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: Exception address is odd.");
        exception_address_odd = true;
      }
      else {
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: Exception address is even.");
        exception_address_odd = false;
      }
    }

    Controller.writeDataBus(CPU.data_bus, CPU.data_width);

    //Controller.getBoard().debugPrintf(DebugType::EXECUTE, true, "## EXECUTE: Wrote bus: %04X\n\r", CPU.data_bus);

    if ((CPU.bus_state_latched == CODE) && (CPU.prefetching_store)) {
      CPU.program->debug_print(Controller.getBoard(), DebugType::STORE, "## EXECUTE: Prefetching STORE program byte", CPU.data_bus);
    }
  }

  if (cpu_iorc) {
    if (READ_ABUS_23) {
      // NPX read. Read if READYO is high. 
      CPU.data_bus = Controller.readDataBus(CPU.data_width);
    }
    else if (CPU.bus_cycle == WRITE_CYCLE) {
      // The CPU is reading from the I/O bus. Read from the bus emulator.
      CPU.data_bus = ArduinoX86::Bus->io_read_bus(CPU.address_latch(), !Controller.readBHEPin());
      Controller.writeDataBus(CPU.data_bus, CPU.data_width);
    }
  }

  if ((READ_NMI_PIN) && (CPU.nmi_checkpoint == 0)) {
    // Use checkpoint "1" to specify that NMI has been detected. This just prevents the debug message from
    // printing every cycle after NMI.
    Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: NMI pin high - Execute will end at IVT fetch.", true);
    CPU.nmi_checkpoint = 1;
    ArduinoX86::CycleLogger->disable_logging();
  }
  else if ((!READ_SMI_PIN) && (CPU.smi_checkpoint == 0)) {
    Controller.getBoard().debugPrintln(DebugType::ERROR, "## EXECUTE: SMI pin asserted - Execute will end at SMRAM write.", true);
    CPU.smi_checkpoint = 1;
    ArduinoX86::CycleLogger->disable_logging();
  }

  if (CPU.bus_state == HALT) {
    
    if (CPU.address_latch() & 0x00002) {

      Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: HALT detected - Ending program execution.", true);
      
      if (CPU.use_smm()) {
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: HALT with SMM enabled.");
        if (READ_SMI_PIN) {
          Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: HALT - Writing SMI pin low", true);
          //Controller.writePin(OutputPin::Ready, true);
          Controller.writePin(OutputPin::Smi, false);
        }
      }
      else {
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: HALT with SMM disabled.");
        if (!READ_NMI_PIN){
          Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: HALT - Writing NMI pin high.", true);
          Controller.writePin(OutputPin::Nmi, true);
        }
      }
    }
    else {
      Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: SHUTDOWN detected - Ending program execution.", true);
      ArduinoX86::Server.change_state(ServerState::Shutdown);
      return;
    }

  }

  if (Controller.readALEPin()) {
    if (CPU.bus_state == CODE) {
      if (CPU.exception_armed) {
        // Hook code fetch after exception and write halt opcode.
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: Exception armed and CODE fetch detected. Writing HALT opcode.", true);
        // The 386EX only emits even addresses when fetching, so adjust the HALT opcode address by one if the IVT offset was odd.
        if (CPU.cpu_type == CpuType::i80386 && exception_address_odd) {
          ArduinoX86::Bus->mem_write_u8(CPU.address_latch() + 1, OPCODE_HALT);
        }
        else {
          ArduinoX86::Bus->mem_write_u8(CPU.address_latch(), OPCODE_HALT);
        }
      }

      if ((CPU.predicted_fetch > 0) && (CPU.address_latch() != CPU.predicted_fetch)) {
        // We have a code fetch at the predicted address.
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: CODE fetch not at predicted address. Flow control change detected!", true);

        if (ArduinoX86::Server.halt_after_jump()) {
          if (CPU.is_address_in_program(CPU.address_latch())) {
            Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: Suppressing HALT injection within program bounds", false);
          }
          else {
            Controller.getBoard().debugPrintf(DebugType::EXECUTE, false, "## EXECUTE: Injecting HALT opcode at %08X after detecting jump.", CPU.address_latch());
            if (CPU.cpu_type == CpuType::i80386) {
              // The 386 only fetches at even addresses. We rely on a hint from the client to determine which byte to patch.
              // If no hint is provided, we'll be forced to write a word with two HALTs to cover both cases.
              if (CPU.have_jump_hint()) {
                if (CPU.jump_hint_is_odd()) {
                  
                  ArduinoX86::Bus->mem_write_u8(CPU.address_latch() + 1, OPCODE_HALT);
                }
                else {
                  ArduinoX86::Bus->mem_write_u8(CPU.address_latch(), OPCODE_HALT);
                }
              }
              else {
                // No jump hint, so write double HALT to cover both even and odd address jumps.
                ArduinoX86::Bus->mem_write_bus(CPU.address_latch(), OPCODE_DOUBLE_HALT, true);
              }
            }
            else {
              ArduinoX86::Bus->mem_write_u8(CPU.address_latch(), OPCODE_HALT);
            }
          }
        }
        CPU.predicted_fetch = 0;  // Reset predicted fetch
      }

      if (CPU.address_latch() & 1) {
        // Fetch at odd address. Predict next fetch at even address.
        CPU.predicted_fetch = CPU.address_latch() + 1;
      }
      else {
        // Fetch at even address.
        CPU.predicted_fetch = CPU.address_latch() + 2;
      }
    }

    if (CPU.bus_state == MEMR) {
      //Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: ALE high and MEMR cycle detected.", true);

      if (CPU.address_latch() < 0x400) {
        Controller.getBoard().debugPrintf(DebugType::EXECUTE, false, "## EXECUTE: Possible IVT read, far call flag is %d, align is %d\n\r", far_call_flag, (CPU.address_latch() & 0x03));
        if (((CPU.address_latch() & 0x03) == 0) && far_call_flag) {
          Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: Exception/Interrupt candidate. Entering exception stage 1", true);
          CPU.exception_stage = 1;
          next_exception_address = CPU.address_latch() + 2;
        }
        if ((CPU.exception_stage == 1) && (CPU.address_latch() == next_exception_address)) {

          exception_number = (uint32_t)((next_exception_address - 2) / 4);
          Controller.getBoard().debugPrintf(DebugType::EXECUTE, true, "## EXECUTE: Detected Exception/Interrupt #%d!\n\r", exception_number);
          CPU.exception_armed = true;
          CPU.exception_stage = 0;
          next_exception_address = 0;
        }
      }

      // NMI is active and CPU is starting a memory bus cycle. Let's check if it is the NMI handler.
      if ((READ_NMI_PIN) && (CPU.address_latch() == 0x00008)) {
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: NMI high and fetching NMI handler. Entering ExecuteFinalize...", true);
        CPU.nmi_terminate = true;
        ArduinoX86::Server.change_state(ServerState::ExecuteFinalize);
      }
    }

    // Handle entering SMM state.
    if (!READ_SMI_PIN) {
      Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: SMI asserted.");
      if (Controller.readALEPin() && (CPU.address_latch() == SMRAM_FIRST_WRITE)) {
        // Sometimes the bus state is still HALT on ALE, so we don't check for MEMW. Just the address is good enough.
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE: SMI asserted and writing to SMRAM. Entering Store...");
        CPU.smi_terminate = true;
        ArduinoX86::Server.change_state(ServerState::StoreAll);
      }
    }
  }
}

void handle_execute_finalize_state() {
  static uint32_t run_address = 0;

  if (READ_NMI_PIN) {
    if (!CPU.data_bus_resolved && !Controller.readMRDCPin()) {
      // NMI is active and CPU is reading memory.  Let's check if it is the NMI handler.
      if (CPU.address_latch() == 0x00008) {
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE_FINALIZE: CPU is reading NMI IVT entry...");
        CPU.nmi_checkpoint = 1;
      }

      if (CPU.bus_state_latched == CODE) {
        // CPU is reading CODE in ExecuteFinalize with NMI high.
        // This should hopefully be at the address of the NMI vector, so we can enter ExecuteDone
        run_address = calc_flat_address(STORE_SEG, 0);
        if (CPU.address_latch() == run_address) {
          Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE_FINALIZE: Fetch at STORE_SEG.");

          if(CPU.nmi_buf_cursor == 0) {

            if (CPU.cpu_type == CpuType::i80286) {
              Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE_FINALIZE: 286 CPU. Popping stack frame from BusEmulator.");
              CPU.nmi_stack_frame = ArduinoX86::Bus->log_peek_call_frame();

              Controller.getBoard().debugPrintf(DebugType::EXECUTE, false, "## EXECUTE_FINALIZE: Popped NMI stack frame. Flags: %04X CS: %04X IP: %04X\n\r",
                CPU.nmi_stack_frame.flags, CPU.nmi_stack_frame.cs, CPU.nmi_stack_frame.ip);

              // Sanity check, flags cannot be 0 due to reserved bit 1.
              if (CPU.nmi_stack_frame.flags == 0x0000) {
                Controller.getBoard().debugPrintln(DebugType::ERROR, "## EXECUTE_FINALIZE: NMI stack frame flags are 0! Invalid state.");
                ArduinoX86::Server.change_state(ServerState::Error);
                set_error("NMI stack frame flags are 0!");
                return;
              }

              // Write the NMI stack frame to the NMI stack buffer.
              CPU.nmi_buf_cursor = 0;
              write_buffer(NMI_STACK_BUFFER, &CPU.nmi_buf_cursor, CPU.nmi_stack_frame.flags, 0x00000, CPU.data_width);
              write_buffer(NMI_STACK_BUFFER, &CPU.nmi_buf_cursor, CPU.nmi_stack_frame.cs, 0x00002, CPU.data_width);
              write_buffer(NMI_STACK_BUFFER, &CPU.nmi_buf_cursor, CPU.nmi_stack_frame.ip, 0x00004, CPU.data_width);
            }
            else if (CPU.cpu_type == CpuType::i80386) {
              Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE_FINALIZE: 386 CPU. Waiting for deferred stack frame...");
            }
            else {
              Controller.getBoard().debugPrintln(DebugType::ERROR, "## EXECUTE_FINALIZE: NMI buffer is 0, invalid state.");
              ArduinoX86::Server.change_state(ServerState::Error);
              set_error("NMI buffer is 0, invalid state.");
              return;
            }
          }

          if (ArduinoX86::Server.is_execute_automatic()) {
            // ExecuteDone is an interactive state, so we should not transition to it if 
            // automatic execution is enabled. Instead, enter Store.
            #if defined(CPU_286)
              ArduinoX86::Server.change_state(ServerState::StoreAll);
            #else
              ArduinoX86::Server.change_state(ServerState::Store); 
            #endif
          }
          else {
            ArduinoX86::Server.change_state(ServerState::ExecuteDone);
          }
        }
        else if (CPU.address_latch() == 0) {
          // We have a match for the NMI vector
          Controller.getBoard().debugPrintln(DebugType::ERROR, "## EXECUTE_FINALIZE: Fetch at address 0!");
          ArduinoX86::Server.change_state(ServerState::Error);
          set_error("NMI vector fetch at address 0!");
        }
      } 
      else if (CPU.nmi_checkpoint > 0 && NMI_VECTOR.has_remaining()) {
        // Feed the CPU the NMI vector.
        CPU.data_bus = NMI_VECTOR.read(CPU.address_latch(), CPU.data_width);
        Controller.getBoard().debugPrint(DebugType::EXECUTE, "## EXECUTE_FINALIZE: Feeding CPU reset vector data: ");
        Controller.getBoard().debugPrint(DebugType::EXECUTE, CPU.data_bus, 16);
        Controller.getBoard().debugPrint(DebugType::EXECUTE, " new v_pc: ");
        Controller.getBoard().debugPrintln(DebugType::EXECUTE, CPU.v_pc);
        CPU.data_bus_resolved = true;
        Controller.writeDataBus(CPU.data_bus, CPU.data_width);

        if (CPU.nmi_checkpoint == 1 && CPU.address_latch() == 0x0000A) {
          Controller.getBoard().debugPrintln(DebugType::EXECUTE, "## EXECUTE_FINALIZE: Read of NMI IVT with NMI pin high - Resetting STORE PC");
          CPU.nmi_checkpoint = 2;
          CPU.data_bus_resolved = true;
          CPU.s_pc = 0;
        }
        return;
      }
    }

    if (!CPU.data_bus_resolved && !Controller.readMWTCPin() && CPU.nmi_checkpoint > 1) {
      // NMI is active and CPU is writing memory. Probably to stack.
      write_buffer(NMI_STACK_BUFFER, &CPU.nmi_buf_cursor, CPU.data_bus, CPU.address_latch(), CPU.data_width);
      Controller.getBoard().debugPrint(DebugType::EXECUTE, "## EXECUTE_FINALIZE: Stack write: ");
      Controller.getBoard().debugPrint(DebugType::EXECUTE, CPU.data_bus, 16);
      Controller.getBoard().debugPrint(DebugType::EXECUTE, " New buf cursor: ");
      Controller.getBoard().debugPrintln(DebugType::EXECUTE, CPU.nmi_buf_cursor);
      CPU.data_bus_resolved = true;
    }
  }

  if (!Controller.readMRDCPin() && CPU.bus_state == PASV) {
    // CPU is reading (MRDC active-low)
    if (CPU.bus_state_latched == CODE) { 
      // CPU is fetching code

      // Since client does not cycle the CPU in this state, we have to fetch from the
      // STORE or EMU_EXIT program ourselves
      CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
      CPU.data_type = QueueDataType::ProgramEnd;
      Controller.writeDataBus(CPU.data_bus, CPU.data_width);
      Controller.getBoard().debugPrint(DebugType::EXECUTE, "## EXECUTE_FINALIZE: Wrote next PGM word to bus: ");
      Controller.getBoard().debugPrint(DebugType::EXECUTE, CPU.data_bus, 16);
      Controller.getBoard().debugPrint(DebugType::EXECUTE, " new s_pc: ");
      Controller.getBoard().debugPrintln(DebugType::EXECUTE, CPU.s_pc);
    } else {
      Controller.writeDataBus(CPU.data_bus, CPU.data_width);
    }
  }

  if (CPU.q_ff && (CPU.qt == QueueDataType::ProgramEnd)) {
    // We read a flagged NOP, meaning the previous instruction has completed and it is safe to
    // execute the Store routine.
    if (CPU.in_emulation) {
      ArduinoX86::Server.change_state(ServerState::EmuExit);
    } else {
      ArduinoX86::Server.change_state(ServerState::ExecuteDone);
    }
  }
}

/// @brief Handle STORE state, which is used to store the CPU state after execution.
void handle_store_state() {
  if (!Controller.readMRDCPin() && CPU.bus_cycle == WRITE_CYCLE) {
    // CPU is reading

    if (CPU.bus_state_latched == CODE) {
      // CPU is doing code fetch
      if (CPU.program->has_remaining()) {
        // Read code from store program
        CPU.data_bus = CPU.program->read(CPU.address_latch(), CPU.data_width);
        CPU.program->debug_print(Controller.getBoard(), DebugType::STORE, "## STORE", CPU.data_bus);
        CPU.data_type = QueueDataType::Program;
      }
      else {
        CPU.data_bus = OPCODE_DOUBLENOP;
        CPU.data_type = QueueDataType::ProgramEnd;
      }
    }
    else {
      // CPU is reading something else. Stack?
      if (CPU.cpu_type == CpuType::i80386) {
        // Read from bus emulator.
        CPU.data_bus = ArduinoX86::Bus->mem_read_bus(CPU.address_latch(), !Controller.readBHEPin(), false);
        Controller.getBoard().debugPrintf(DebugType::STORE, true, "## STORE: 386: Read from bus emulator: %04X\n\r", CPU.data_bus);
      }
      else {
        CPU.data_bus = read_program(
          NMI_STACK_BUFFER, 
          sizeof NMI_STACK_BUFFER, 
          &CPU.nmi_buf_cursor, 
          CPU.address_latch(), 
          CPU.data_width
        );
        Controller.getBoard().debugPrintf(
          DebugType::STORE, 
          false, 
          "## STORE: Read from stack: %04X new cursor: %04X\n\r", 
          CPU.data_bus, 
          CPU.nmi_buf_cursor
        );
      }

    }
    Controller.writeDataBus(CPU.data_bus, CPU.data_width);
  }

  // CPU is writing to memory address - this should only happen during readback when
  // the flags register is pushed to the stack (The only way to read the full flags)
  if (!Controller.readMWTCPin() && CPU.bus_state != PASV) {
    CPU.data_bus = Controller.readDataBus(CPU.data_width);

    // Store program sets up SS:SP as 0:4, so write should be to the first four memory
    // addresses, for pushing IP and FLAGS.
    if (CPU.address_latch() < 0x00004) {


      Controller.getBoard().debugPrint(DebugType::STORE, "## STORE: Stack push!");

      // Write flags and IP to the register struct
      if (CPU.data_width == ActiveBusWidth::EightLow) {
        Controller.getBoard().debugPrint(DebugType::EMU, "## 8-bit flag read ##");
        *CPU.readback_p = (uint8_t)CPU.data_bus;
        CPU.readback_p++;
      } else if (CPU.data_width == ActiveBusWidth::EightHigh) {
        // We shouldn't have unaligned stack access during STORE. Something has gone wrong.
        debugPrintlnColor(ansi::bright_red, "## Bad Data Bus Width during Store: EightHigh");
      } else {
        // 16-bit data bus
        if ((CPU.address_latch() == 0x00002) && (CPU.do_emulation)) {
          // We ran a program in 8080 emulation. We want to substitute the flags
          // captured in 8080 mode for the native flags now.
          CPU.data_bus = (CPU.data_bus & 0xFF00) | (uint16_t)CPU.emu_flags;
          Controller.getBoard().debugPrintf(DebugType::EMU, false, "## Substituting 8080 flags in stack read: %04X\n\r", CPU.data_bus);
        }

#if DEBUG_EMU
        ptrdiff_t diff = (uint8_t *)&CPU.post_regs.flags - CPU.readback_p;
#endif            
        *((uint16_t *)CPU.readback_p) = CPU.data_bus;
        CPU.readback_p += 2;

#if DEBUG_EMU
        uint16_t *flags_ptr = (uint16_t *)&CPU.post_regs.flags;
        Controller.getBoard().debugPrintf(DebugType::EMU, false, "## New flags are: %04X Readback ptr diff: %td\n\r", *flags_ptr, diff);
#endif
      }
    } else {
      // We shouldn't be writing to any other addresses, something wrong happened
      if (CPU.address_latch() == 0x00004) {
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## STORE: TRAP detected in Store operation! Invalid flags?");
        ArduinoX86::Server.change_state(ServerState::Error);
        set_error("TRAP detected in Store operation! Invalid flags?");
      }

      if (CPU.cpu_type == CpuType::i80386) {
        // Send write to bus emulator - this is likely a stack push.
        Controller.getBoard().debugPrintf(DebugType::STORE, true, "## STORE: 386: Sending write to bus emulator: %04X\n\r", CPU.data_bus);
        ArduinoX86::Bus->mem_write_bus(CPU.address_latch(), CPU.data_bus, !Controller.readBHEPin());
      }
      else {
        Controller.getBoard().debugPrint(DebugType::ERROR, "## STORE: Invalid store memory write: ");
        Controller.getBoard().debugPrintln(DebugType::ERROR, CPU.address_latch(), HEX);
        set_error("Invalid store memory write");
      }

      // TODO: handle error gracefully
    }
    Controller.getBoard().debugPrint(DebugType::STORE, "## STORE: memory write: ");
    Controller.getBoard().debugPrintln(DebugType::STORE, CPU.data_bus, HEX);
  }

  // CPU is writing to IO address - this indicates we are saving a register value.
  // We structured the register struct in the right order, so we can overwrite it
  // directly.
  if (!Controller.readIOWCPin()) {
    if (CPU.address_latch() == 0xFD) {
      // Write to 0xFD indicates end of store procedure.

      // Adjust IP by offset of CALL instruction.
      Controller.getBoard().debugPrintf(DebugType::STORE, true, "## STORE: Unadjusted IP: %04X\n\r", CPU.post_regs.ip);
      //CPU.post_regs.ip -= 0x24;
      //CPU.post_regs.ip -= (0x24 + 6); // added 6 NOPs to start of STORE program

      ArduinoX86::Server.change_state(ServerState::StoreDone);
    } 
    else {
      CPU.data_bus = Controller.readDataBus(CPU.data_width);

      if (CPU.cpu_type == CpuType::i80386) {
        Controller.getBoard().debugPrintf(DebugType::STORE, true, "## STORE: 386: Sending write to bus emulator: %04X\n\r", CPU.data_bus);
        ArduinoX86::Bus->io_write_bus(CPU.address_latch(), CPU.data_bus, !Controller.readBHEPin());
      }
      else {
        if (CPU.data_width == ActiveBusWidth::EightLow) {
          *CPU.readback_p = (uint8_t)CPU.data_bus;
          CPU.readback_p++;
        } else if (CPU.data_width == ActiveBusWidth::EightHigh) {
          Controller.getBoard().debugPrintln(DebugType::ERROR, "## STORE: Bad Data Bus Width during Store: EightHigh");
        } else {
          *(uint16_t *)CPU.readback_p = CPU.data_bus;
          CPU.readback_p += 2;
        }
      }
      Controller.getBoard().debugPrintf(DebugType::STORE, true, "## STORE: IO write: %04X\n\r", CPU.data_bus);
    }
  }
}

// Reverse the order of the stack buffer.
void reverse_stack_buf() {
  uint16_t temp;
  temp = ((uint16_t *)NMI_STACK_BUFFER)[2];
  ((uint16_t *)NMI_STACK_BUFFER)[2] = ((uint16_t *)NMI_STACK_BUFFER)[0];
  ((uint16_t *)NMI_STACK_BUFFER)[0] = temp;
}

// Returns true if the current cycle is the cycle we should write or read
// the data bus. Before calling this function it is assumed we have ensured
// that we are in a bus cycle and one of the 8288 signals is active.
bool is_transfer_cycle() {
  return (READ_READY_PIN && (CPU.bus_cycle == T3 || CPU.bus_cycle == TW));
}

// Return true if the current m-cycle has finished
bool is_transfer_done() {
  switch (CPU.bus_state_latched) {
    case IOR:
      // IORC is active-low, so we are returning true if it is off
      return Controller.readIORCPin();
    case IOW:
      // IOWC is active-low, so we are returning true if it is off
      return Controller.readIOWCPin();
    case CODE:
      // FALLTHRU
    case MEMR:
      // MRDC is active-low, so we are returning true if it is off
      return Controller.readMRDCPin();
    case MEMW:
      // MWTC is active-low, so we are returning true if it is off
      return Controller.readMWTCPin();
    default:
      // Rely on external READY pin
      return READ_READY_PIN;
      break;
  }
}

void print_addr(unsigned long addr) {
  static char addr_buf[6];
  snprintf(addr_buf, 6, "%05lX", addr);
  DEBUG_SERIAL.println(addr_buf);
}

void reset_screen() {
  #if GIGA_DISPLAY_SHIELD
    // By freak occurence, the display shield and the CPU share a reset pin.
    if (screen) {
      screen->init();
      Board.debugPrintln(DebugType::SETUP, "Resetting Giga display...");
      screen->updateCell(0, 5, screen->makeColor(255, 255, 255), "Reset...");
    }
  #endif
}

// Detect the CPU type based on the number of CPU cycles spent executing the
// CPU ID program.
void detect_cpu_type(uint32_t cpuid_cycles) {
  if (CPU.width == CpuBusWidth::Eight) {
    if (cpuid_cycles > 5) {
      Controller.getBoard().debugPrintln(DebugType::ID, "detect_cpu_type(): Detected NEC V30H");
      CPU.cpu_type = CpuType::necV20;
    } else {
      Controller.getBoard().debugPrintln(DebugType::ID, "detect_cpu_type(): Detected i8088");
      CPU.cpu_type = CpuType::i8088;
    }
  } else {
    if (cpuid_cycles > 5) {
      Controller.getBoard().debugPrintln(DebugType::ID, "detect_cpu_type(): Detected NEC V30");
      CPU.cpu_type = CpuType::necV30;
    } else {
      Controller.getBoard().debugPrintln(DebugType::ID, "detect_cpu_type(): Detected i8086");
      CPU.cpu_type = CpuType::i8086;
    }
  }
}

void detect_fpu_type() {
  CPU.fpu_type = FpuType::i8087;
}

void do_frame_update() {
  if (!screen_initialized) {
    // Screen not initialized yet, so skip frame update
    return;
  }
  unsigned long current_ms = millis();
  unsigned long delta_ms = current_ms - last_millis;
  
  frame_ms_accumulator += delta_ms;
  second_ms_accumulator += delta_ms;
  if (frame_ms_accumulator >= SCREEN_UPDATE_MS) {
    if (frame_ms_accumulator > SCREEN_UPDATE_MS) {
      frame_ms_accumulator -= SCREEN_UPDATE_MS;
    } else {
      frame_ms_accumulator = 0;
    }
    // Write the address latch.
    char buf[6];
    snprintf(buf, sizeof(buf), "%05lX", CPU.address_latch());
    screen->updateCell(0, 1, screen->makeColor(128, 128, 255), buf);
    screen->updateCell(1, 1, screen->makeColor(128, 128, 255), ArduinoX86::Server.get_state_string(ArduinoX86::Server.state()));
    fps_counter++;


    // Get video memory buffer.
    //uint8_t *vga_memory = ArduinoX86::Bus->get_ptr(0xA0000);
    //screen->vga()->convert(vga_memory);
  }

  if (second_ms_accumulator >= 1000) {
    if (second_ms_accumulator > 1000) {
      second_ms_accumulator -= 1000;
    } else {
      second_ms_accumulator = 0;
    }
    // Update the FPS counter
    int row = screen->rows() - 1;
    screen->updateCell(row, 1, screen->makeColor(255, 255, 255), (String(fps_counter) + "fps").c_str());
    fps_counter = 0;
  }

  last_millis = current_ms;
}

/// Read the `len` number of bytes from the serial port into the specified buffer.
bool readParameterBytes(uint8_t *buf, size_t buf_len, size_t len)  {

  Controller.getBoard().debugPrintln(DebugType::PROTO, "## readParameterBytes(): Reading " + String(len) + " parameter bytes...");
  size_t bytes_read = 0;
  unsigned long start_time = millis();
  uint8_t byte = 0;

  while (bytes_read < len) {
    if (INBAND_SERIAL.available() > 0) {
      byte = (uint8_t)INBAND_SERIAL.read();
      buf[bytes_read++] = byte;
      Controller.getBoard().debugPrintln(DebugType::PROTO, "## readParameterBytes(): Read byte " + String(bytes_read) + ": 0x" + String(byte, HEX));
    } else {
      // Check for timeout
      if (millis() - start_time > CMD_TIMEOUT) {
        Controller.getBoard().debugPrintln(DebugType::ERROR, "## readParameterBytes(): Timeout!");
        return false; // Timeout
      }
    }
  }
  Controller.getBoard().debugPrintln(DebugType::PROTO, "## readParameterBytes(): Successfully read " + String(bytes_read) + " bytes.");
  return true; // Successfully read all bytes
}

// Main sketch loop
void loop() {

  // if (screen_init_requested && !screen_initialized) {
  //   Board.debugPrintln(DebugType::SETUP, "Initializing screen...");
  //   screen->init();
  //   Board.debugPrintln(DebugType::SETUP, "Screen initialized!");
  //   last_millis = millis();

  //   // Draw cpu status.
  //   size_t idx = static_cast<size_t>(CPU.cpu_type);
  //   screen->updateCell(0, 0, screen->makeColor(255, 255, 255), CPU_TYPE_STRINGS[idx]);

  //   screen_initialized = true;
  // }
  //do_frame_update();
  
  ArduinoX86::Server.run();

  bool do_cycle = false;
  if (ArduinoX86::Server.is_executing() && (ArduinoX86::Server.is_execute_automatic())) {
    CPU.execute_cycle_ct++;
    do_cycle = true;

    // if (CPU.execute_cycle_ct < EXECUTE_TIMEOUT) {
    //   cycle();
    // }
    // else if (CPU.execute_cycle_ct == EXECUTE_TIMEOUT) {
    //   Controller.getBoard().debugPrintln(DebugType::ERROR, "## Execute cycle timeout reached! Ending execution.");
    //   ArduinoX86::Server.change_state(ServerState::Error);
    // }
  }
  else if ((ArduinoX86::Server.get_state() == ServerState::Error) && (CPU.error_cycle_ct < MAX_ERROR_CYCLES)) {
    // If we are in error state, we still want to cycle the CPU to allow it to
    // process the error.
    CPU.error_cycle_ct++;
    do_cycle = true;
  }

  if (do_cycle) {
    cycle();
  }
}
