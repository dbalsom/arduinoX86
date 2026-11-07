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

// Forward declaration of cycle() function
void cycle();

#include <arduinoX86.h>
#include <serial_config.h>
#include <gpio_pins.h>
#include <BusTypes.h>
#include <shields/ShieldBase.h>
#include <shields/Pins.h>
#include <DebugFilter.h>

#include <i82288Emulator.h>

#define CPU_286

#define USE_SMI 0
#define USE_SETUP_PROGRAM 0
#define SETUP_PROGRAM SETUP_PROGRAM_86
#define SETUP_PROGRAM_PATCH_OFFSET 0

#define STORE_IO_BASE 0x0000

#define WRITE_BIT(data, mask, set_macro, clear_macro) \
    do { if ((data) & (mask)) { set_macro; } else { clear_macro; } } while (0)


#define ADDRESS_SPACE_MASK 0x3FFFFF // 4MB address space for 80286
#define WRITE_CYCLE T2
#define STORE_TIMEOUT 300
#define LOAD_TIMEOUT 1000

#define PRE_RESET_CYCLE_COUNT 5 // How many cycles to wait before asserting RESET. This gives time for pins to settle.
// How many cycles to hold RESET asserted. Intel says 18 cycles for the 80286.
#define RESET_HOLD_CYCLE_COUNT 20
// How many cycles it takes to reset the CPU after RESET is deasserted. Intel says 38 cycles for the 80286.
// If we didn't see an ALE after this many cycles, give up
#define RESET_CYCLE_TIMEOUT 40
// Arduino pin levels; the 82284 inverts RESET when it is in the signal path.
#ifdef USE_82284
  #define RESET_ASSERT 0
  #define RESET_DEASSERT 1
#else
  #define RESET_ASSERT 1
  #define RESET_DEASSERT 0
#endif

// ------------------------- CPU Control pins ---------------------------------
#define CLK_PIN 4
//#define RESET_PIN 5
#define RESET_PIN 77 // A1

// -------------------------- CPU Input pins ----------------------------------
// We use the analog pins for CPU inputs as they are not 5v tolerant.
#define BHE_PIN 13

#define NMI_PIN 78 // A2
#define INTR_PIN 79 // A3
#define BUSY_PIN 80 // A4
#define ERROR_PIN 81 // A5
#define READ_ERROR_PIN READ_PIN_D81
#define READ_BHE_PIN READ_PIN_D13

#define READY_ASSERT 0
#define READY_DEASSERT 1
#define READY_PIN 76 // A0
#define READ_READY_PIN (READ_PIN_D76)
#define READ_READY_PIN_NORM (!READ_READY_PIN)

#define READ_RESET_PIN (digitalRead(RESET_PIN) == RESET_ASSERT)
#define READ_NMI_PIN READ_PIN_D78
#define READ_SMI_PIN 1
#define READ_INTR_PIN READ_PIN_D79

#define READ_TEST_PIN READ_PIN_D80
#define READ_LOCK_PIN READ_PIN_D07

#define S0_PIN 11
#define S1_PIN 12
#define READ_S0_PIN READ_PIN_D11
#define READ_S1_PIN READ_PIN_D12
#define READ_M_IO_PIN READ_PIN_D10
#define READ_S2_PIN READ_M_IO_PIN
#define READ_C_I_PIN READ_PIN_D09

// The ICE pins aren't bonded on this CPU, as confirmed by die photography.
// #define ICE_PIN0 20
// #define ICE_PIN1 21
// #define READ_ICE_PIN0 READ_PIN_D20
// #define READ_ICE_PIN1 READ_PIN_D21
// ------------------------- 82288 status pins --------------------------------

#define ALE_PIN 8
#define ALE_TRIGGER RISING
#define READ_ALE_PIN READ_PIN_D08

#define READ_MRDC_PIN READ_PIN_D07
#define READ_AMWC_PIN (1)
#define READ_MWTC_PIN READ_PIN_D06
#define READ_IORC_PIN READ_PIN_D82
#define READ_AIOWC_PIN (1) 
#define READ_IOWC_PIN READ_PIN_D83

// -------------------------- CPU Bus pins ------------------------------------
#define SET_DBUS_00 do { SET_PIN_D22; } while (0)
#define CLEAR_DBUS_00 do { CLEAR_PIN_D22; } while (0)

#define SET_DBUS_01 do { SET_PIN_D23; } while (0)
#define CLEAR_DBUS_01 do { CLEAR_PIN_D23; } while (0)

#define SET_DBUS_02 do { SET_PIN_D24; } while (0)
#define CLEAR_DBUS_02 do { CLEAR_PIN_D24; } while (0)

#define SET_DBUS_03 do { SET_PIN_D25; } while (0)
#define CLEAR_DBUS_03 do { CLEAR_PIN_D25; } while (0)

#define SET_DBUS_04 do { SET_PIN_D26; } while (0)
#define CLEAR_DBUS_04 do { CLEAR_PIN_D26; } while (0)

#define SET_DBUS_05 do { SET_PIN_D27; } while (0)
#define CLEAR_DBUS_05 do { CLEAR_PIN_D27; } while (0)

#define SET_DBUS_06 do { SET_PIN_D28; } while (0)
#define CLEAR_DBUS_06 do { CLEAR_PIN_D28; } while (0)

#define SET_DBUS_07 do { SET_PIN_D29; } while (0)
#define CLEAR_DBUS_07 do { CLEAR_PIN_D29; } while (0)

#define SET_DBUS_08 do { SET_PIN_D30; } while (0)
#define CLEAR_DBUS_08 do { CLEAR_PIN_D30; } while (0)

#define SET_DBUS_09 do { SET_PIN_D31; } while (0)
#define CLEAR_DBUS_09 do { CLEAR_PIN_D31; } while (0)

#define SET_DBUS_10 do { SET_PIN_D32; } while (0)
#define CLEAR_DBUS_10 do { CLEAR_PIN_D32; } while (0)

#define SET_DBUS_11 do { SET_PIN_D33; } while (0)
#define CLEAR_DBUS_11 do { CLEAR_PIN_D33; } while (0)

#define SET_DBUS_12 do { SET_PIN_D34; } while (0)
#define CLEAR_DBUS_12 do { CLEAR_PIN_D34; } while (0)

#define SET_DBUS_13 do { SET_PIN_D35; } while (0)
#define CLEAR_DBUS_13 do { CLEAR_PIN_D35; } while (0)

#define SET_DBUS_14 do { SET_PIN_D36; } while (0)
#define CLEAR_DBUS_14 do { CLEAR_PIN_D36; } while (0)

#define SET_DBUS_15 do { SET_PIN_D37; } while (0)
#define CLEAR_DBUS_15 do { CLEAR_PIN_D37; } while (0)

#define READ_DBUS_00 READ_PIN_D22
#define READ_DBUS_01 READ_PIN_D23
#define READ_DBUS_02 READ_PIN_D24
#define READ_DBUS_03 READ_PIN_D25
#define READ_DBUS_04 READ_PIN_D26
#define READ_DBUS_05 READ_PIN_D27
#define READ_DBUS_06 READ_PIN_D28
#define READ_DBUS_07 READ_PIN_D29
#define READ_DBUS_08 READ_PIN_D30
#define READ_DBUS_09 READ_PIN_D31
#define READ_DBUS_10 READ_PIN_D32
#define READ_DBUS_11 READ_PIN_D33
#define READ_DBUS_12 READ_PIN_D34
#define READ_DBUS_13 READ_PIN_D35
#define READ_DBUS_14 READ_PIN_D36
#define READ_DBUS_15 READ_PIN_D37

#define READ_ABUS_00 READ_PIN_D38
#define READ_ABUS_01 READ_PIN_D39
#define READ_ABUS_02 READ_PIN_D40
#define READ_ABUS_03 READ_PIN_D41
#define READ_ABUS_04 READ_PIN_D42
#define READ_ABUS_05 READ_PIN_D43
#define READ_ABUS_06 READ_PIN_D44
#define READ_ABUS_07 READ_PIN_D45
#define READ_ABUS_08 READ_PIN_D46
#define READ_ABUS_09 READ_PIN_D47
#define READ_ABUS_10 READ_PIN_D48
#define READ_ABUS_11 READ_PIN_D49
#define READ_ABUS_12 READ_PIN_D50
#define READ_ABUS_13 READ_PIN_D51
#define READ_ABUS_14 READ_PIN_D52
#define READ_ABUS_15 READ_PIN_D53
#define READ_ABUS_16 READ_PIN_D17 // We skip 18 & 19 as they are used for Serial Debugging UART (Serial2)
#define READ_ABUS_17 READ_PIN_D16 
#define READ_ABUS_18 READ_PIN_D15 
#define READ_ABUS_19 READ_PIN_D14
#define READ_ABUS_20 READ_PIN_D00
#define READ_ABUS_21 READ_PIN_D01
#define READ_ABUS_22 READ_PIN_D02
#define READ_ABUS_23 READ_PIN_D03

#define LOOP_COUNT 20

#define SPIN_LOOP(count)                          \
  do {                                            \
    for (volatile unsigned int _spin_i = 0;       \
         _spin_i < (count);                      \
         ++_spin_i) {                             \
      /* prevent the compiler from discarding the loop */ \
      __asm__ __volatile__ ("" ::: "memory");    \
    }                                             \
  } while (0)

class Shield80286 : public ShieldBase<Shield80286> {

private:
  // Address pins, used for slow address reading via digitalRead()
  static constexpr std::array<int,22> ADDRESS_PINS = {{
    38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, // (16) Bottom half of double-row header
    21, 20, 17, 16, 15, 14
  }};
  static constexpr int ADDRESS_LINES = 22;
  static constexpr int ADDRESS_DIGITS = 6; // 22 bits = 6 digits in hex

  static constexpr std::array<int,7> OUTPUT_PINS = {
    4,  // CLK
    RESET_PIN,
    76, // READY (A0)
    78, // NMI (A2)
    79, // INTR (A3)
    BUSY_PIN,
    ERROR_PIN,
  };

  // All input pins, used to set pin direction on setup
  static constexpr std::array<int,36> INPUT_PINS = {{
    13, 12, 11, 10, 9, 8, 7, 6, // (6) Various signal pins
    38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, // (16) Address pins - Bottom half of double-row header
    21, 20, 17, 16, 15, 14, 3, 2, 1, 0, // (6) Address pins - Top row of GPIO pins
    82, 83, // (2) IORC and IOWC pins
  }};

  
  BusController i82288_; // 82288 bus controller emulator
  bool emulate_bus_controller_ = false;
  uint8_t currentCpuStatus_ = 0x03; // Current CPU status read from the status lines. Initialized to 0x03 (S0 and S1 high).
  uint8_t lastCpuStatus_ = 0x03; // Last CPU status read from the status lines. Initialized to 0x03 (S0 and S1 high).
  uint8_t latchedStatus_ = 0x03; // Status latched on ALE signal. 
  bool emulatedALE_ = false; // Emulated ALE signal. We ignore ALE from the bus controller as it is asynchronous. 
  bool stickyALE_ = false; // Sticky ALE signal. This is set when we see an ALE signal and remains true until reset.
  bool stickyIORC_ = true; // Sticky IORC signal. This is set when we see an IORC signal and remains true until reset.
  bool stickyIOWC_ = true; // Sticky IOWC signal. This is set when we see an IOWC signal and remains true until reset.

protected:
  static constexpr unsigned ClockDivisor = 2;
  static constexpr unsigned ClockHighDelay = 1; // Delay for clock high in microseconds
  static constexpr unsigned ClockLowDelay = 1; // Delay for clock low in microseconds
  size_t addressBusWidth = 24; // Default address bus width is 24 bits
  
public:

  Shield80286(bool emulate_bus_controller = false) : emulate_bus_controller_(emulate_bus_controller)
    {}

  static void initPins() {
    // BUSY polarity is tested empirically; HIGH is the normal level.
    digitalWrite(BUSY_PIN, HIGH);
    digitalWrite(ERROR_PIN, HIGH);

    // Set initial pin modes for all output and input pins
    for (auto pin : OUTPUT_PINS) {
      pinMode(pin, OUTPUT);
    }

    for (auto pin : INPUT_PINS) {
      pinMode(pin, INPUT_PULLDOWN);
    }

    writeResetPinImpl(RESET_DEASSERT);

    // The ICE pins aren't bonded; leave their GPIOs at the default input pulldown.
    // pinMode(ICE_PIN0, INPUT); // ICE pin 0
    // pinMode(ICE_PIN1, INPUT); // ICE pin 1

    // Pull up S0 and S1 pins.
    pinMode(S0_PIN, INPUT_PULLUP);
    pinMode(S1_PIN, INPUT_PULLUP);
    pinMode(7, INPUT_PULLUP); // MRDC
    pinMode(8, INPUT_PULLUP); // MWTC
    pinMode(82, INPUT_PULLUP); // IORC
    pinMode(83, INPUT_PULLUP); // IOWC

    digitalWrite(7, HIGH);

    digitalWrite(INTR_PIN, LOW);  // Must set these to a known value or risk spurious interrupts!
    digitalWrite(NMI_PIN, LOW);   // Must set these to a known value or risk spurious interrupts!
  }

  static int getAddressBusWidth() {
    return ADDRESS_LINES;
  }

  static int getAddressDigits() {
    return ADDRESS_DIGITS;
  }

  /// @brief Return true if the current shield has a multiplexed bus.
  static bool hasMultiplexedBusImpl() {
    return false;
  }

  static BusStatus decodeBusStatus(uint8_t status_byte) {
    switch (status_byte & 0x0F) {
      case 0b0000: return BusStatus::INTA;
      case 0b0001: return BusStatus::PASV; // Reserved
      case 0b0010: return BusStatus::PASV; // Reserved
      case 0b0011: return BusStatus::PASV; // None
      case 0b0100: return BusStatus::HALT;
      case 0b0101: return BusStatus::MEMR;
      case 0b0110: return BusStatus::MEMW;
      case 0b0111: return BusStatus::PASV; // None
      case 0b1000: return BusStatus::PASV; // Reserved
      case 0b1001: return BusStatus::IOR;
      case 0b1010: return BusStatus::IOW;
      case 0b1011: return BusStatus::PASV; // None
      case 0b1100: return BusStatus::PASV; // Reserved
      case 0b1101: return BusStatus::CODE;
      case 0b1110: return BusStatus::PASV; // Reserved
      case 0b1111: return BusStatus::PASV; // None
      default: return BusStatus::PASV;
    }
  }

  static bool isTransferDoneImpl(BusStatus latched_status) {
    return true;
    // switch (latched_status) {
    //   case IOR:
    //     // IORC is active-low, so we are returning true if it is off
    //     return READ_IORC_PIN;
    //   case IOW:
    //     // IOWC is active-low, so we are returning true if it is off
    //     return READ_IOWC_PIN;
    //   case CODE:
    //     // FALLTHRU
    //   case MEMR:
    //     // MRDC is active-low, so we are returning true if it is off
    //     return READ_MRDC_PIN;
    //   case MEMW:
    //     // MWTC is active-low, so we are returning true if it is off
    //     return READ_MWTC_PIN;
    //   default:
    //     // Rely on external READY pin
    //     return READ_READY_PIN;
    //     break;
    // }
  }

  static TCycle getNextCycleImpl(TCycle current_cycle, BusStatus current_status, BusStatus latched_status) {
    // Return the next cycle for the CPU
     switch (current_cycle) {
      case TI:
        return TI;
        break;
      
      case T1:
        // Begin a bus cycle only if signalled, otherwise wait in T1
        if (current_status != PASV) {
          return T2;
        }
        break;

      case T2:
        if (Shield80286::isTransferDoneImpl(latched_status)) {
          return TI;
        } else {
  #if DEBUG_TSTATE
          debugPrintlnColor(ansi::yellow, "Setting T-cycle to Tw");
  #endif
          return TW;
        }
        break;

      case TW:
        // Transition to TI if read/write signals are complete
        if (Shield80286::isTransferDoneImpl(latched_status)) {
          return TI;
        }
        break;

      default:
        break;
    }

    return TI; // Default to TI if no other condition matches
  }

  static const char * getTCycleString(TCycle cycle) {
    switch (cycle) {
      case T1: return "Ts";
      case T2: return "Tc";
      case TW: return "Tw";
      case TI: return "Ti";
      default: return "T?"; // Unknown cycle
    }
  }

  static bool hasSegmentStatus() {
    return false; // 80286 does not have segment status lines.
  }

  /// @brief Tick the CPU one clock cycle.
  /// For 286, this is two external clock cycles per CPU clock cycle.
  void tickCpuImpl() {
    // stickyIORC_ = READ_IORC_PIN;
    // stickyIOWC_ = READ_IOWC_PIN;
    WRITE_PIN_D04(1);
    if (ClockLowDelay > 0) {
      delayMicroseconds(ClockLowDelay);
      //SPIN_LOOP(LOOP_COUNT);
    }
    WRITE_PIN_D04(0);
    if (ClockHighDelay > 0) {
      delayMicroseconds(ClockHighDelay);
      //SPIN_LOOP(LOOP_COUNT);
    }
    // // Check 82288 outputs here.
    // if(!READ_IORC_PIN) {
    //   stickyIORC_ = false;
    // }
    // if(!READ_IOWC_PIN) {
    //   stickyIOWC_ = false;
    // }

    WRITE_PIN_D04(1);
    if (ClockLowDelay > 0) {
      delayMicroseconds(ClockLowDelay);
      //SPIN_LOOP(LOOP_COUNT);
    }
    WRITE_PIN_D04(0);
    if (ClockHighDelay > 0) {
      delayMicroseconds(ClockHighDelay);
      //SPIN_LOOP(LOOP_COUNT);
    }

    // if(!READ_IORC_PIN) {
    //   stickyIORC_ = false;
    // }
    // if(!READ_IOWC_PIN) {
    //   stickyIOWC_ = false;
    // }

    currentCpuStatus_ = (READ_S0_PIN << 0) | (READ_S1_PIN << 1) | (READ_M_IO_PIN << 2) | (READ_C_I_PIN << 3);
    i82288_.tick(currentCpuStatus_, true);
  }

  uint16_t readDataBus(ActiveBusWidth width, bool peek = false) {
    if(!peek) {
      setBusDirection(BusDirection::Input, width);
    }
    
    uint16_t data = 0;

    if (peek || (width == ActiveBusWidth::EightLow) || (width == ActiveBusWidth::Sixteen)) {
      // Read data from bus pins
      if (READ_DBUS_00) data |= 0x0001;
      if (READ_DBUS_01) data |= 0x0002;
      if (READ_DBUS_02) data |= 0x0004;
      if (READ_DBUS_03) data |= 0x0008;
      if (READ_DBUS_04) data |= 0x0010;
      if (READ_DBUS_05) data |= 0x0020;
      if (READ_DBUS_06) data |= 0x0040;
      if (READ_DBUS_07) data |= 0x0080;
    }
    if (peek || (width == ActiveBusWidth::EightHigh) || (width == ActiveBusWidth::Sixteen)) {
      if (READ_DBUS_08) data |= 0x0100;
      if (READ_DBUS_09) data |= 0x0200;
      if (READ_DBUS_10) data |= 0x0400;
      if (READ_DBUS_11) data |= 0x0800;
      if (READ_DBUS_12) data |= 0x1000;
      if (READ_DBUS_13) data |= 0x2000;
      if (READ_DBUS_14) data |= 0x4000;
      if (READ_DBUS_15) data |= 0x8000;
    }
    return data;
  }

  void writeDataBus(uint16_t data, ActiveBusWidth width) {
    
    #if defined(__SAM3X8E__) // If Arduino DUE

      setBusDirection(BusDirection::Output, width);

      if ((width == EightLow) || (width == Sixteen)) {
        // Write low-order byte to data bus pins
        (data & 0x01) ? PIOB->PIO_SODR = BIT26 : PIOB->PIO_CODR = BIT26;      // Pin 22
        (data & 0x02) ? PIOA->PIO_SODR = BIT14 : PIOA->PIO_CODR = BIT14;      // Pin 23
        (data & 0x04) ? PIOA->PIO_SODR = BIT15 : PIOA->PIO_CODR = BIT15;      // Pin 24
        (data & 0x08) ? PIOD->PIO_SODR = BIT00 : PIOD->PIO_CODR = BIT00;      // Pin 25
        (data & 0x10) ? PIOD->PIO_SODR = BIT01 : PIOD->PIO_CODR = BIT01;      // Pin 26
        (data & 0x20) ? PIOD->PIO_SODR = BIT02 : PIOD->PIO_CODR = BIT02;      // Pin 27
        (data & 0x40) ? PIOD->PIO_SODR = BIT03 : PIOD->PIO_CODR = BIT03;      // Pin 28
        (data & 0x80) ? PIOD->PIO_SODR = BIT06 : PIOD->PIO_CODR = BIT06;      // Pin 29
      }

      if ((width == EightHigh) || (width == Sixteen)) {
        (data & 0x0100) ? PIOD->PIO_SODR = BIT09 : PIOD->PIO_CODR = BIT09;    // AD8 Pin 30 (PD9)
        (data & 0x0200) ? PIOA->PIO_SODR = BIT07 : PIOA->PIO_CODR = BIT07;    // AD9 Pin 31 (PA7)
        (data & 0x0400) ? PIOD->PIO_SODR = BIT10 : PIOD->PIO_CODR = BIT10;    // AD10 Pin 32 (PD10)
        (data & 0x0800) ? PIOC->PIO_SODR = BIT01 : PIOC->PIO_CODR = BIT01;    // AD11 Pin 33 (PC1)
        (data & 0x1000) ? PIOC->PIO_SODR = BIT02 : PIOC->PIO_CODR = BIT02;    // AD12 Pin 34 (PC2)
        (data & 0x2000) ? PIOC->PIO_SODR = BIT03 : PIOC->PIO_CODR = BIT03;    // AD13 Pin 35 (PC3)
        (data & 0x4000) ? PIOC->PIO_SODR = BIT04 : PIOC->PIO_CODR = BIT04;    // AD14 Pin 36 (PC4)
        (data & 0x8000) ? PIOC->PIO_SODR = BIT05 : PIOC->PIO_CODR = BIT05;    // AD15 Pin 37 (PC5)
      }
    #elif defined(ARDUINO_GIGA)
      setBusDirection(BusDirection::Output, width);

      if ((width == ActiveBusWidth::EightLow) || (width == ActiveBusWidth::Sixteen)) {
        WRITE_BIT(data, 0x01, SET_DBUS_00, CLEAR_DBUS_00);
        WRITE_BIT(data, 0x02, SET_DBUS_01, CLEAR_DBUS_01);
        WRITE_BIT(data, 0x04, SET_DBUS_02, CLEAR_DBUS_02);
        WRITE_BIT(data, 0x08, SET_DBUS_03, CLEAR_DBUS_03);
        WRITE_BIT(data, 0x10, SET_DBUS_04, CLEAR_DBUS_04);
        WRITE_BIT(data, 0x20, SET_DBUS_05, CLEAR_DBUS_05);
        WRITE_BIT(data, 0x40, SET_DBUS_06, CLEAR_DBUS_06);
        WRITE_BIT(data, 0x80, SET_DBUS_07, CLEAR_DBUS_07);
      }

      if ((width == ActiveBusWidth::EightHigh) || (width == ActiveBusWidth::Sixteen)) {
        WRITE_BIT(data, 0x0100, SET_DBUS_08, CLEAR_DBUS_08);
        WRITE_BIT(data, 0x0200, SET_DBUS_09, CLEAR_DBUS_09);
        WRITE_BIT(data, 0x0400, SET_DBUS_10, CLEAR_DBUS_10);
        WRITE_BIT(data, 0x0800, SET_DBUS_11, CLEAR_DBUS_11);
        WRITE_BIT(data, 0x1000, SET_DBUS_12, CLEAR_DBUS_12);
        WRITE_BIT(data, 0x2000, SET_DBUS_13, CLEAR_DBUS_13);
        WRITE_BIT(data, 0x4000, SET_DBUS_14, CLEAR_DBUS_14);
        WRITE_BIT(data, 0x8000, SET_DBUS_15, CLEAR_DBUS_15);        
      }
    #endif
  }

  /// Read the address bus.
  uint32_t readAddressBusImpl(bool peek = true) {
    // If we're not peeking, set the bus direction to input
    if (!peek) {
      setBusDirection(BusDirection::Input, ActiveBusWidth::Sixteen);
    }

    uint32_t address = 0;
    // Read the address bus pins
    if (READ_ABUS_00) address |= 0x00000001;
    if (READ_ABUS_01) address |= 0x00000002;
    if (READ_ABUS_02) address |= 0x00000004;
    if (READ_ABUS_03) address |= 0x00000008;
    if (READ_ABUS_04) address |= 0x00000010;
    if (READ_ABUS_05) address |= 0x00000020;
    if (READ_ABUS_06) address |= 0x00000040;
    if (READ_ABUS_07) address |= 0x00000080;
    if (READ_ABUS_08) address |= 0x00000100;
    if (READ_ABUS_09) address |= 0x00000200;
    if (READ_ABUS_10) address |= 0x00000400;
    if (READ_ABUS_11) address |= 0x00000800;
    if (READ_ABUS_12) address |= 0x00001000;
    if (READ_ABUS_13) address |= 0x00002000;
    if (READ_ABUS_14) address |= 0x00004000;
    if (READ_ABUS_15) address |= 0x00008000;
    if (READ_ABUS_16) address |= 0x00010000;
    if (READ_ABUS_17) address |= 0x00020000;
    if (READ_ABUS_18) address |= 0x00040000;
    if (READ_ABUS_19) address |= 0x00080000;
    if (READ_ABUS_20) address |= 0x00100000;
    if (READ_ABUS_21) address |= 0x00200000;
    if (READ_ABUS_22) address |= 0x00400000;
    if (READ_ABUS_23) address |= 0x00800000;
    return address;
  }

  static bool readPinImpl(OutputPin pin) {
    switch (pin) {
      case OutputPin::Ready:
        return READ_READY_PIN_NORM;
      case OutputPin::Test:
        return READ_TEST_PIN;
      case OutputPin::Error:
        return READ_ERROR_PIN;
      case OutputPin::Intr:
        return READ_INTR_PIN;
      case OutputPin::Nmi:
        return READ_NMI_PIN;
      default:
        return false;
    }
  }

  static void writePinImpl(OutputPin pin, bool value) {
    switch (pin) {
      case OutputPin::Ready:
        // The client uses logical READY; the CPU has an active-low READY# pin.
        WRITE_PIN_A0(value ? READY_ASSERT : READY_DEASSERT);
        break;
      case OutputPin::Test:
        WRITE_PIN_A4(value);
        break;
      case OutputPin::Error:
        WRITE_PIN_A5(value);
        break;
      case OutputPin::Intr:
        WRITE_PIN_A3(value);
        break;
      case OutputPin::Nmi:
        WRITE_PIN_A2(value);
        break;
      default:
        // Handle other pins if necessary
        break;
    }
  }

  template<typename Board>
  CpuResetResult resetCpuImpl(Board& board) {

    CpuResetResult result = {};
    bool s0 = false;
    bool s1 = false;
    //uint32_t address = 0;
    result.success = false;
    result.queueStatus = false;
    result.busWidth = BusWidth::Sixteen; // 286 is always 16-bit bus width
    
    digitalWrite(INTR_PIN, LOW); // INTR must be low or CPU will immediately interrupt.
    digitalWrite(NMI_PIN, LOW); // NMI must be low or CPU will immediately interrupt.
    // Release any manually inserted wait state before reset and LOADALL.
    digitalWrite(READY_PIN, READY_ASSERT);

    bool ale = false;
    //bool bhe_went_off = false;

    // Clock the CPU a few times before asserting RESET.
    for (int i = 0; i < PRE_RESET_CYCLE_COUNT; i++) {
      tickCpu();
    }
    
    // Assert RESET pin for hold count.
    for (int i = 0; i < RESET_HOLD_CYCLE_COUNT; i++) {
      // Write pin in loop to try to synchronize with CLK as best as we can.
      writeResetPinImpl(RESET_ASSERT);
      cycle();
    }

    // // Address lines should be high at this point. We don't know exactly how many 
    // // address pins might be connected, so we'll just read 20 of them which is the required minimum.
    // address = readAddressBus();
    // if (address & 0xFFFFF != 0xFFFFF) {
    //   //set_error("CPU failed to reset: Address lines not high!");   
    //   return result;
    // }

    if (!READ_ABUS_00 || !READ_ABUS_01 || !READ_ABUS_02 || !READ_ABUS_03 ||
        !READ_ABUS_04 || !READ_ABUS_05 || !READ_ABUS_06 || !READ_ABUS_07 ||
        !READ_ABUS_08 || !READ_ABUS_09 || !READ_ABUS_10 || !READ_ABUS_11 ||
        !READ_ABUS_12 || !READ_ABUS_13 || !READ_ABUS_14 || !READ_ABUS_15 ||
        !READ_ABUS_16 || !READ_ABUS_17 || !READ_ABUS_18 || !READ_ABUS_19) {
      //set_error("CPU failed to reset: Address lines not high!");   
      return result;
    }


    // S0 & S1 should be high at this point.
    s0 = READ_S0_PIN;
    s1 = READ_S1_PIN;
    
    if (!s0 || !s1) {
      //set_error("CPU failed to reset: S0 and S1 not high!");   
      return result;
    }

    // Clock CPU while waiting for ALE
    int ale_cycles = 0;

    // Wait for ALE (S0 or S1 to go low) to indicate the CPU has reset.
    for ( int i = 0; i < RESET_CYCLE_TIMEOUT; i++ ) {
      // Deassert RESET pin to allow CPU to start.
      writeResetPinImpl(RESET_DEASSERT);
      cycle();

      s0 = READ_S0_PIN;
      s1 = READ_S1_PIN;

      if (!s0 || !s1) {
        // S0 or S1 is low, indicating the start of a bus cycle.
        // This should be the first code fetch after reset.
        ale = true;
      }

      // if (!READ_BHE_PIN) {
      //     bhe_went_off = true;
      // }

      ale_cycles++;      

      if (ale) {
        // ALE is active! CPU has successfully reset
        //CPU.doing_reset = false;      
  
        if (!result.success) {
          board.debugPrintln(DebugType::RESET, "###########################################");      
          board.debugPrintln(DebugType::RESET, "## Reset CPU!                            ##");
          board.debugPrintln(DebugType::RESET, "###########################################");
          result.success = true;
          break;
        }
      }
    }

    if (!result.success) {
      // ALE did not turn on within the specified cycle timeout, so we failed to reset the cpu.
      #if DEBUG_RESET
        board.debugPrintln(DebugType::ERROR, "## Failed to reset CPU! ##");
      #endif
      //set_error("CPU failed to reset: No ALE!");   
    }
    
    return result;
  }

  static bool readBHEPinImpl() {
    // Read the BHE pin (Bus High Enable)
    return READ_BHE_PIN;
  }
  
  bool readALEPinImpl() {

    if(emulatedALE_ != stickyALE_) {
      //DEBUG_SERIAL.println(" >>>>> 82288 mismatch: emulated ALE is " + String(emulatedALE_) + ", sticky ALE is " + String(stickyALE_));
    }
    return emulatedALE_;
  }

  static bool readLockPinImpl() {
    return READ_LOCK_PIN;
  }

  static bool readReadyPinImpl() {
    // Read the READY pin
    return READ_READY_PIN;
  }

  bool readMRDCPinImpl() {
    //return !(!emulatedALE_ && ((latchedStatus_ & 0x07) == 0x05));
    return i82288_.mrdc();
  }
  bool readAMWCPinImpl() {
    return READ_AMWC_PIN;
  }
  bool readMWTCPinImpl() {
    //return !(!emulatedALE_ && ((latchedStatus_ & 0x07) == 0x06));
    return i82288_.mwtc();
  }
  bool readIORCPinImpl() {
    //return !(!emulatedALE_ && ((latchedStatus_ & 0x0F) == 0x09));
    return i82288_.iorc();
  }
  bool readAIOWCPinImpl() {
    return READ_AIOWC_PIN;
  }
  bool readIOWCPinImpl() {
    //return !(!emulatedALE_ && ((latchedStatus_ & 0x0F) == 0x0A)); 
    return i82288_.iowc();
  }
  bool readINTAPinImpl() {
    return false;
  }
  
  static void writeResetPinImpl(bool value) {
    // Write to the RESET pin
    digitalWrite(RESET_PIN, value ? HIGH : LOW);
  }

  uint8_t readCpuStatusLinesImpl() {
    // Read the CPU status lines

    currentCpuStatus_ = 0x00;
    //pinMode(S0_PIN, INPUT_PULLUP);
    //pinMode(S1_PIN, INPUT_PULLUP);
    if (READ_S0_PIN) { currentCpuStatus_ |= 0x01; }; // S0
    if (READ_S1_PIN) { currentCpuStatus_ |= 0x02; }; // S1
    if (READ_M_IO_PIN) { currentCpuStatus_ |= 0x04; }; // M/!IO
    if (READ_C_I_PIN) { currentCpuStatus_ |= 0x08; }; // COD/!INTA

    // Check for ALE.
    if (((lastCpuStatus_ & 0x03) == 0x03) && ((currentCpuStatus_ & 0x03) < 0x03)) {
      // The last status was passive, and the current status is active (S0 or S1 is low).
      // This means the start of a bus cycle and Ts state. 
      emulatedALE_ = true;
    }
    else {
      emulatedALE_ = false;
    }
    lastCpuStatus_ = currentCpuStatus_;
    return currentCpuStatus_;
  }

  uint8_t readBusControllerCommandLinesImpl() {
    // Read the bus controller command lines
    uint8_t command = 0;
    command |= readMRDCPinImpl() ? 0x01 : 0;     // MRDC - Pin 51
    command |= readAMWCPinImpl() ? 0x02 : 0;     // AMWC - Pin 52
    command |= readMWTCPinImpl() ? 0x04 : 0;     // MWTC - Pin 53
    command |= readIORCPinImpl() ? 0x08 : 0;     // IORC - Pin 46
    command |= readAIOWCPinImpl() ? 0x10 : 0;    // AIOWC- Pin 48
    command |= readIOWCPinImpl() ? 0x20 : 0;     // IOWC - Pin 47
    command |= readINTAPinImpl() ? 0x40 : 0;     // INTA - Pin 45
    // Although not an 8288 command status, we have an extra bit, so we can stick BHE in here.
    // This saves us from needing to add an extra byte - that adds up!
    command |= READ_BHE_PIN ? 0x80 : 0;
    return command;
  }

  uint8_t readBusControllerControlLinesImpl() {
    // Read the bus controller control lines
    uint8_t control = 0;
    control |= emulatedALE_ ? 0x01 : 0;     // ALE      - Pin 50
    //control |= READ_PIN_D49 ? 0x02 : 0;     // DTR      - Pin 49
    //control |= READ_PIN_D43 ? 0x04 : 0;     // MCE/PDEN - Pin 43
    //control |= READ_PIN_D44 ? 0x08 : 0;     // DEN      - Pin 44
    return control;
  }

  template<typename Board>
  void printPinStatesImpl(Board&) {
    // The common trace already prints the 286's control pins.
  }
};
