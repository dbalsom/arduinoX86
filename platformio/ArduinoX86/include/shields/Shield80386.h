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

#define CPU_386

#define USE_SMI 1
#define USE_SETUP_PROGRAM 1
#define SETUP_PROGRAM SETUP_PROGRAM_386EX
#define SETUP_PROGRAM_PATCH_OFFSET 0

#define STORE_IO_BASE 0x0080

#define WRITE_BIT(data, mask, set_macro, clear_macro) \
    do { if ((data) & (mask)) { set_macro; } else { clear_macro; } } while (0)


#define ADDRESS_SPACE_MASK 0x3FFFFF // 4MB address space for 80286
#define WRITE_CYCLE T2
#define STORE_TIMEOUT 3000
#define LOAD_TIMEOUT 25000

// ------------------------- CPU Control pins ---------------------------------
#define CLK_PIN 4
#define RESET_PIN 5

// -------------------------- CPU Input pins ----------------------------------
// We use the analog pins for CPU inputs as they are not 5v tolerant.
#define ALE_PIN 9
#define ALE_TRIGGER FALLING
#define BHE_PIN 13
#define HOLD_PIN 76 // A0
#define BS8_PIN 79 // A3
#define RW_PIN 82 // A6
#define INTR_PIN 0

#define READ_BHE_PIN READ_PIN_D13
#define READ_RESET_PIN READ_PIN_D05

#define READ_INTR_PIN READ_PIN_A1
#define WRITE_PIN_INTR(x) WRITE_PIN_A1(x)

#define READY_ASSERT 0
#define READY_DEASSERT 1
#define READ_READY_PIN_NORM (!READ_READY_PIN)

#if defined(SHIELD_386EX_V2)
  #define READY_PIN 83
  #define READ_READY_PIN READ_PIN_A7
  #define WRITE_READY_PIN(x) WRITE_PIN_A7(x)

  #define NMI_PIN 93
  #define READ_NMI_PIN READ_PIN_D93
  #define WRITE_NMI_PIN(x) WRITE_PIN_D93(x)
#else
  #define READY_PIN 84
  #define READ_READY_PIN READ_PIN_D84
  #define WRITE_READY_PIN(x) WRITE_PIN_D84(x)

  #define NMI_PIN 85
  #define READ_NMI_PIN READ_PIN_D85
  #define WRITE_NMI_PIN(x) WRITE_PIN_D85(x)
#endif

#define READ_SMI_PIN READ_PIN_D02
#define WRITE_SMI_PIN(x) WRITE_PIN_D02(x)

#define READ_RW_PIN READ_PIN_D82
#define READ_DC_PIN READ_PIN_D08

#define READ_BUSY_PIN READ_PIN_D06
#define READ_TEST_PIN READ_PIN_D06
#define WRITE_TEST_PIN(x) WRITE_PIN_D06(x) 

#define WRITE_BS8_PIN(x) WRITE_PIN_A3(x)

#define READ_LOCK_PIN READ_PIN_D07 
#define READ_ERROR_PIN READ_PIN_D03
#define READYO_PIN 94
#define READ_READYO_PIN READ_PIN_D94
#define READ_PEREQ_PIN READ_PIN_D11
#define READ_S0_PIN READ_MRDC_PIN
#define READ_S1_PIN READ_MWTC_PIN
#define READ_M_IO_PIN READ_PIN_D10

// ------------------------- CPU bus status pins -------------------------------
#define READ_ALE_PIN (!READ_PIN_D09)

#define READ_MRDC_PIN (!(!READ_PIN_A5 && READ_M_IO_PIN))
#define READ_AMWC_PIN (1)
#define READ_MWTC_PIN (!(!READ_PIN_A4 && READ_M_IO_PIN))
#define READ_IORC_PIN (!(!READ_PIN_A5 && !READ_M_IO_PIN))
#define READ_AIOWC_PIN (1)
#define READ_IOWC_PIN (!(!READ_PIN_A4 && !READ_M_IO_PIN))

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
#define READ_ABUS_16 READ_PIN_D21
#define READ_ABUS_17 READ_PIN_D20 
#define READ_ABUS_18 READ_PIN_D17 // We skip 18 & 19 as they are used for Serial Debugging UART (Serial2)
#define READ_ABUS_19 READ_PIN_D16
#define READ_ABUS_20 READ_PIN_D15
#define READ_ABUS_21 READ_PIN_D14
#define READ_ABUS_22 READ_PIN_D00
#define READ_ABUS_23 READ_PIN_D01

#define ABUS_00 ((GPIOJ->IDR & BIT07) >> 7)  // 7 - 7 = 0
#define ABUS_01 ((GPIOI->IDR & BIT14) >> 13) // 14 - 13 = 1
#define ABUS_02 ((GPIOE->IDR & BIT06) >> 4)  // 6 - 4 = 2
#define ABUS_03 ((GPIOK->IDR & BIT07) >> 4)  // 7 - 4 = 3 
#define ABUS_04 ((GPIOI->IDR & BIT15) >> 11) // 15 - 11 = 4
#define ABUS_05 ((GPIOI->IDR & BIT10) >> 5)  // 10 - 5 = 5
#define ABUS_06 ((GPIOG->IDR & BIT10) >> 4)  // 10 - 4 = 6
#define ABUS_07 ((GPIOI->IDR & BIT13) >> 6)  // 13 - 6 = 7
#define ABUS_08 ((GPIOH->IDR & BIT15) >> 7)  // 15 - 7 = 8
#define ABUS_09 ((GPIOB->IDR & BIT02) << 7)  // 2 + 7 = 9
#define ABUS_10 ((GPIOK->IDR & BIT00) << 10) // 0 + 10 = 10
#define ABUS_11 ((GPIOE->IDR & BIT04) << 7)  // 4 + 7 = 11
#define ABUS_12 ((GPIOI->IDR & BIT11) << 1)  // 11 + 1 = 12
#define ABUS_13 ((GPIOE->IDR & BIT05) << 8)  // 5 + 8 = 13
#define ABUS_14 ((GPIOK->IDR & BIT02) << 12) // 2 + 12 = 14
#define ABUS_15 ((GPIOG->IDR & BIT07) << 8)  // 7 + 8 = 15
#define ABUS_16 ((GPIOH->IDR & BIT04) << 12) // 4 + 12 = 16
#define ABUS_17 ((GPIOB->IDR & BIT11) << 6) // 11 + 6 = 17 
#define ABUS_18 ((GPIOI->IDR & BIT09) << 9) // 9 + 9 = 18
#define ABUS_19 ((GPIOH->IDR & BIT13) << 6) // 13 + 6 = 19
#define ABUS_20 ((GPIOC->IDR & BIT07) << 13) // 7 + 13 = 20
#define ABUS_21 ((GPIOG->IDR & BIT14) << 7) // 14 + 7 = 21
#define ABUS_22 ((GPIOB->IDR & BIT07) << 15) // 7 + 15 = 22
#define ABUS_23 ((GPIOA->IDR & BIT09) << 14) // 9 + 14 = 23

#define PRE_RESET_CYCLE_COUNT 5 // How many cycles to wait before asserting RESET. This gives time for pins to settle.

// How many cycles to hold the RESET signal high. Intel says "greater than 4" although 4 seems to work.
#define RESET_HOLD_CYCLE_COUNT 90
// How many cycles it takes to reset the CPU after RESET signal goes low. First ALE should occur after this many cycles.
#define RESET_CYCLE_COUNT 500
// If we didn't see an ALE after this many cycles, give up
#define RESET_CYCLE_TIMEOUT 1000
// What logic level RESET is when asserted
#define RESET_ASSERT 1
// What logic level RESET is when deasserted
#define RESET_DEASSERT 0

#define LOOP_COUNT 20

#define SPIN_LOOP(count)                          \
  do {                                            \
    for (volatile unsigned int _spin_i = 0;       \
         _spin_i < (count);                       \
         ++_spin_i) {                             \
      __asm__ __volatile__ ("" ::: "memory");     \
    }                                             \
  } while (0)

class Shield80386 : public ShieldBase<Shield80386> {

private:
  // Address pins, used for slow address reading via digitalRead()
  static constexpr std::array<int,22> ADDRESS_PINS = {{
    38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, // (16) Bottom half of double-row header
    21, 20, 17, 16, 15, 14
  }};
  static constexpr int ADDRESS_LINES = 22;
  static constexpr int ADDRESS_DIGITS = 6; // 22 bits = 6 digits in hex

  static constexpr std::array<int,10> OUTPUT_PINS = {
    2,  // SMI
    4,  // CLK
    5,  // RESET
    76, // HOLD (A0)
    77, // READY (A1)
    79, // BS8 (A3)
    83, // unused (A7)
    84, // READY (V1)
    85, // NMI (V1)
    93 // NMI (V2)
  };

  // All input pins, used to set pin direction on setup
  static constexpr std::array<int,38> INPUT_PINS = {{
    13, 12, 11, 10, 9, 8, 7, 6, 3, // (8) Various signal pins
    78, 80, 81, 82, 94, // (5) SMIACT(A2), R, W, RW, READYO
    38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, // (16) Address pins - Bottom half of double-row header
    21, 20, 17, 16, 15, 14, 1, 0, // (8) Address pins - Top row of GPIO pins
  }};
  
  bool _emulate_bus_controller = false;

protected:
  static constexpr unsigned ClockDivisor = 2;
  static constexpr unsigned ClockHighDelay = 2;
  static constexpr unsigned ClockLowDelay = 2;
  size_t addressBusWidth = 22; // Default address bus width is 22 bits
  
public:

  Shield80386(bool emulate_bus_controller = false) : _emulate_bus_controller(emulate_bus_controller) 
    {}

  static void initPins() {
    // Set initial pin modes for all output and input pins
    for (auto pin : OUTPUT_PINS) {
      pinMode(pin, OUTPUT);
    }

    for (auto pin : INPUT_PINS) {
      pinMode(pin, INPUT);
    }

    digitalWrite(INTR_PIN, LOW);  // Must set these to a known value or risk spurious interrupts!
    digitalWrite(NMI_PIN, LOW);   // Must set these to a known value or risk spurious interrupts!
    digitalWrite(HOLD_PIN, LOW);  // Hold pin is active high, make sure it is low by default.
    digitalWrite(BS8_PIN, HIGH); // BS8 pin is active low, so we set it high by default.
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
    switch (status_byte & 0x07) {
        case 0: return BusStatus::INTA;
        case 1: return BusStatus::PASV;
        case 2: return BusStatus::IOR;
        case 3: return BusStatus::IOW;
        case 4: return BusStatus::CODE;
        case 5: return BusStatus::HALT;
        case 6: return BusStatus::MEMR;
        case 7: return BusStatus::MEMW;
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
        if (isTransferDoneImpl(latched_status)) {
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
        if (isTransferDoneImpl(latched_status)) {
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
      case T1: return "T1";
      case T2: return "T2";
      case TW: return "Tw";
      case TI: return "Ti";
      default: return "T?"; // Unknown cycle
    }
  }

  static bool hasSegmentStatus() {
    return false; // 386 does not have segment status lines.
  }

  /// @brief Tick the CPU one clock cycle.
  /// For 386, this is two external clock cycles per CPU clock cycle.
  static void tickCpuImpl() {
    WRITE_PIN_D04(1);
    if (ClockHighDelay > 0) {
      delayMicroseconds(ClockHighDelay);
    }
    WRITE_PIN_D04(0);
    if (ClockLowDelay > 0) {
      delayMicroseconds(ClockLowDelay);
    }
    WRITE_PIN_D04(1);
    if (ClockHighDelay > 0) {
      delayMicroseconds(ClockHighDelay);
    }
    WRITE_PIN_D04(0);
    // if (ClockLowDelay > 0) {
    //   delayMicroseconds(ClockLowDelay);
    // }
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

    
  inline uint32_t readAddressBusImpl(bool peek = true) __attribute__((always_inline));

  static bool readPinImpl(OutputPin pin) {
    switch (pin) {
      case OutputPin::Hold:
        return digitalRead(HOLD_PIN) != LOW;
      case OutputPin::Ready:
        return READ_READY_PIN_NORM;
        break;
      case OutputPin::Test:
        // !BUSY is tied to Vcc, so we don't control it.
        //WRITE_PIN_D05(value);
        break;
      case OutputPin::Intr:
        return READ_PIN_A3;
        break;
      case OutputPin::Nmi:
        return READ_NMI_PIN;
        break;
      case OutputPin::Smi:
        return READ_SMI_PIN;
        break;
      default:
        // Handle other pins if necessary
        break;
    }

    return false;
  }

  static void writePinImpl(OutputPin pin, bool value) {
    switch (pin) {
      case OutputPin::Hold:
        digitalWrite(HOLD_PIN, value ? HIGH : LOW);
        break;
      case OutputPin::Ready:
        // The client uses logical READY; the CPU has an active-low READY# pin.
        WRITE_READY_PIN(value ? READY_ASSERT : READY_DEASSERT);
        break;
      case OutputPin::Test:
        // !BUSY is tied to Vcc, so we don't control it.
        //WRITE_PIN_D05(value);
        break;
      case OutputPin::Intr:
        WRITE_PIN_A3(value);
        break;
      case OutputPin::Nmi:
        WRITE_NMI_PIN(value);
        break;
      case OutputPin::Smi:
        WRITE_SMI_PIN(value);
        break;
      default:
        // Handle other pins if necessary
        break;
    }
  }

  template<typename Board>
  CpuResetResult resetCpuImpl(Board& board) {

    CpuResetResult result = {};
    result.success = false;
    result.queueStatus = false;
    result.busWidth = BusWidth::Sixteen; // We're using a 386EX, so 16-bit bus width
    setBusDirection(BusDirection::Input, ActiveBusWidth::Sixteen);
    digitalWrite(INTR_PIN, LOW); // INTR must be low or CPU will immediately interrupt.
    digitalWrite(NMI_PIN, LOW); // NMI must be low or CPU will immediately interrupt.
    digitalWrite(HOLD_PIN, LOW); // HOLD must be low or CPU will not reset.
    WRITE_READY_PIN(READY_ASSERT); // Default READY for CPU setup
    WRITE_SMI_PIN(1); // SMI inactive
    
    bool ale_went_off = false;
    //bool bhe_went_off = false;

    // Clock the CPU a few times before asserting RESET.
    for (int i = 0; i < PRE_RESET_CYCLE_COUNT; i++) {
      //tickCpu();
      cycle();
    }

    WRITE_PIN_D05(1); // Assert RESET high for hold count.

    for (int i = 0; i < RESET_HOLD_CYCLE_COUNT; i++) {
      if (READ_ALE_PIN == false) {
        ale_went_off = true;
      }
      //tickCpu();
      cycle();
    }

    // // CPU didn't reset for some reason.
    // if (ale_went_off == false) {
    //   //set_error("CPU failed to reset: ALE not off!");   
    //   return result;
    // }


    WRITE_TEST_PIN(1); // Set TEST pin high to allow CPU to reset without entering test mode.
    delayMicroseconds(10); // Make sure that TEST pin is settled high before de-asserting RESET.

    // Deassert RESET pin to allow CPU to start.
    WRITE_PIN_D05(0);
    
    

    // Clock CPU while waiting for ALE
    int ale_cycles = 0;

    // Reset takes 7 cycles, bit we can try for longer
    for ( int i = 0; i < RESET_CYCLE_TIMEOUT; i++ ) {
      cycle();

      if (!READ_ALE_PIN) {
        if (!ale_went_off) {
          ale_went_off = true;
        }
      }

      // if (!READ_BHE_PIN) {
      //     bhe_went_off = true;
      // }

      ale_cycles++;      

      if (ale_went_off && READ_ALE_PIN) {
        // ALE is active! CPU has successfully reset
        //CPU.doing_reset = false;      
  
        board.debugPrintln(DebugType::RESET, "###########################################");      
        board.debugPrintln(DebugType::RESET, "## Reset CPU!                            ##");
        board.debugPrintln(DebugType::RESET, "###########################################");
        result.success = true;
        break;
      }
    }

    if (!result.success) {
      // ALE did not turn on within the specified cycle timeout, so we failed to reset the cpu.
      #if DEBUG_RESET
        board.debugPrintln(DebugType::ERROR, "## Failed to reset CPU! ##");
      #endif
      //set_error("CPU failed to reset: No ALE!");   
    }
    
    //WRITE_PIN_SMI(0); // Enter SMM immediately (testing purposes only)
    return result;
  }

  static bool readBHEPinImpl() {
    // Read the BHE pin (Bus High Enable)
    return READ_BHE_PIN;
  }
  static bool readALEPinImpl() {
    return READ_ALE_PIN;
  }
  static bool readLockPinImpl() {
    return READ_LOCK_PIN;
  }
  static bool readReadyPinImpl() {
    return READ_READY_PIN;
  }
  static bool readMRDCPinImpl() {
    return READ_MRDC_PIN;
  }
  static bool readAMWCPinImpl() {
    return READ_AMWC_PIN;
  }
  static bool readMWTCPinImpl() {
    return READ_MWTC_PIN;
  }
  static bool readIORCPinImpl() {
    return READ_IORC_PIN;
  }
  static bool readIOWCPinImpl() {
    return READ_IOWC_PIN;
  }
  static bool readAIOWCPinImpl() {
    return READ_AIOWC_PIN;
  }
  static bool readINTAPinImpl() {
    return false;
  }
  static void writeResetPinImpl(bool value) {
    // Write to the RESET pin
    WRITE_PIN_D05(value);
  }

  static uint8_t readCpuStatusLinesImpl() {
    // Read the CPU status lines
    uint8_t status = 0;
    if (READ_RW_PIN) { status |= 0x01; };
    if (READ_DC_PIN) { status |= 0x02; }; 
    if (READ_M_IO_PIN) { status |= 0x04; };

    return status;
  }

  static uint8_t readBusControllerCommandLinesImpl() {
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

  static uint8_t readBusControllerControlLinesImpl() {
    // Read the bus controller control lines
    uint8_t control = 0;
    control |= READ_ALE_PIN ? 0x01 : 0;     // ALE      - Pin 50
    //control |= READ_PIN_D49 ? 0x02 : 0;     // DTR      - Pin 49
    //control |= READ_PIN_D43 ? 0x04 : 0;     // MCE/PDEN - Pin 43
    //control |= READ_PIN_D44 ? 0x08 : 0;     // DEN      - Pin 44
    return control;
  }

  template<typename Board>
  void printPinStatesImpl(Board& board) {
    char error_chr = 'e';
    if (!READ_ERROR_PIN) { 
      error_chr = 'E';
    }

    char ready_chr = 'r';
    if (!READ_READY_PIN) {
      ready_chr = 'R';
    }

    char readyo_chr = 'o';
    if (!READ_READYO_PIN) {
      readyo_chr = 'O';
    }

    char pereq_chr = 'q';
    if (READ_PEREQ_PIN) {
      pereq_chr = 'Q';
    }

    char busy_chr = 'b';
    if (!READ_BUSY_PIN) {
      busy_chr = 'B';
    }
    
    board.debugPrintf(DebugType::EMIT, false, "P2:%c%c%c%c%c ", ready_chr, readyo_chr, error_chr, pereq_chr, busy_chr);
  }
};

uint32_t Shield80386::readAddressBusImpl(bool peek) {
  // uint32_t address = 0;
  // // Read the address bus pins
  // if (READ_ABUS_00) address |= 0x00000001;
  // if (READ_ABUS_01) address |= 0x00000002;
  // if (READ_ABUS_02) address |= 0x00000004;
  // if (READ_ABUS_03) address |= 0x00000008;
  // if (READ_ABUS_04) address |= 0x00000010;
  // if (READ_ABUS_05) address |= 0x00000020;
  // if (READ_ABUS_06) address |= 0x00000040;
  // if (READ_ABUS_07) address |= 0x00000080;
  // if (READ_ABUS_08) address |= 0x00000100;
  // if (READ_ABUS_09) address |= 0x00000200;
  // if (READ_ABUS_10) address |= 0x00000400;
  // if (READ_ABUS_11) address |= 0x00000800;
  // if (READ_ABUS_12) address |= 0x00001000;
  // if (READ_ABUS_13) address |= 0x00002000;
  // if (READ_ABUS_14) address |= 0x00004000;
  // if (READ_ABUS_15) address |= 0x00008000;
  // if (READ_ABUS_16) address |= 0x00010000;
  // if (READ_ABUS_17) address |= 0x00020000;
  // if (READ_ABUS_18) address |= 0x00040000;
  // if (READ_ABUS_19) address |= 0x00080000;
  // if (READ_ABUS_20) address |= 0x00100000;
  // if (READ_ABUS_21) address |= 0x00200000;
  // if (READ_ABUS_22) address |= 0x00400000;
  // if (READ_ABUS_23) address |= 0x00800000;
  return ABUS_00 | 
          ABUS_01 | 
          ABUS_02 |
          ABUS_03 |
          ABUS_04 |
          ABUS_05 |
          ABUS_06 |
          ABUS_07 |
          ABUS_08 |
          ABUS_09 |
          ABUS_10 |
          ABUS_11 |
          ABUS_12 |
          ABUS_13 |
          ABUS_14 |
          ABUS_15 |
          ABUS_16 |
          ABUS_17 |
          ABUS_18 |
          ABUS_19 |
          ABUS_20 |
          ABUS_21 |
          ABUS_22 |
          ABUS_23;
}
