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

#include <CpuTypes.h>
#include <BusTypes.h>
#include <shields/Pins.h>
#include <interrupts.h>

template<typename Board, typename Shield>
class BoardController {
  Board& board;
  Shield shield_;  // Add Shield instance to maintain state
  bool ale_interrupt_enabled_ = false;

public:
  explicit BoardController(Board& b) : board(b), shield_() {}

  // Constructor that allows passing Shield constructor parameters
  template<typename... HatArgs>
  explicit BoardController(Board& b, HatArgs&&... hatArgs) : board(b), shield_(std::forward<HatArgs>(hatArgs)...) {}

  CpuResetResult resetCpu() {
    return shield_.resetCpu(board);
  }

  Board& getBoard() {
    return board;
  }

  void setAleInterrupt(bool enabled) {
    if (enabled != ale_interrupt_enabled_) {
      ale_interrupt_enabled_ = enabled;
      if (enabled) {
        board.debugPrintln(DebugType::EMIT, "BoardController: Enabling ALE interrupt");
        attachInterrupt(digitalPinToInterrupt(ALE_PIN), ale_interrupt_handler, ALE_TRIGGER);
        #ifdef READYO_PIN
        attachInterrupt(digitalPinToInterrupt(READYO_PIN), readyo_interrupt_handler, FALLING);
        #endif
        attachInterrupt(digitalPinToInterrupt(CLK_PIN), cycle_edge_interrupt_handler, RISING);
      } else {
        detachInterrupt(digitalPinToInterrupt(ALE_PIN));
        #ifdef READYO_PIN
        detachInterrupt(digitalPinToInterrupt(READYO_PIN));
        #endif
        detachInterrupt(digitalPinToInterrupt(CLK_PIN));
      }
    }
    if (enabled) {
      
    }
    else {
      board.debugPrintln(DebugType::EMIT, "BoardController: ALE interrupt disabled");
    }
  }

  static int getAddressBusWidth() {
    return Shield::getAddressBusWidth();
  }

  static int getAddressDigits() {
    return Shield::getAddressDigits();
  }

  static bool hasSegmentStatus() {
    return Shield::hasSegmentStatus();
  }

  static BusStatus decodeBusStatus(uint8_t status_byte) {
    return Shield::decodeBusStatus(status_byte);
  }

  static TCycle getNextCycle(TCycle current_cycle, BusStatus current_status, BusStatus latched_status) {
    return Shield::getNextCycle(current_cycle, current_status, latched_status);
  }

  inline uint16_t readDataBus(ActiveBusWidth width, bool peek = false) __attribute__((always_inline));
  inline void writeDataBus(uint16_t data, ActiveBusWidth width) __attribute__((always_inline));
  inline void tickCpu() __attribute__((always_inline));
  inline uint32_t readAddressBus(bool peek) __attribute__((always_inline));

  static void writePin(OutputPin pin, bool value) {
    Shield::writePin(pin, value);
  }

  static bool readPin(OutputPin pin) {
    return Shield::readPin(pin);
  }

  uint8_t readCpuStatusLines() {
    return shield_.readCpuStatusLines();
  }

  uint8_t readBusControllerCommandLines() {
    return shield_.readBusControllerCommandLines();
  }

  uint8_t readBusControllerControlLines() {
    return shield_.readBusControllerControlLines();
  }

  bool cpuIsReading(BusTransferType &read_type) const {
    return Shield::cpuIsReading(read_type);
  }

  bool cpuIsWriting(BusTransferType &write_type) const {
    return Shield::cpuIsWriting(write_type);
  }

  static const char* getBusStatusString(BusStatus status) {
    return Shield::getBusStatusString(status);
  }

  static const char* getBusStatusColor(BusStatus status) {
    return Shield::getBusStatusColor(status);
  }

  static const char* getTCycleString(TCycle cycle) {
    return Shield::getTCycleString(cycle);
  }

  static bool hasMultiplexedBus() {
    return Shield::hasMultiplexedBus();
  }

  bool readLockPin() {
    return shield_.readLockPin();
  }

  bool readReadyPin() {
    return shield_.readReadyPin();
  }
  
  inline bool readBHEPin() __attribute__((always_inline)); 
  inline bool readALEPin() __attribute__((always_inline));
  inline bool readMRDCPin() __attribute__((always_inline));
  inline bool readAMWCPin() __attribute__((always_inline));
  inline bool readMWTCPin() __attribute__((always_inline));
  inline bool readIORCPin() __attribute__((always_inline));
  inline bool readIOWCPin() __attribute__((always_inline));
  inline bool readAIOWCPin() __attribute__((always_inline));

  void printPinStates() {
    shield_.printPinStates(board);
  }
};

template<class BoardType, class ShieldType>
uint32_t BoardController<BoardType, ShieldType>::readAddressBus(bool peek) {
  return shield_.readAddressBus(peek);
}

template<class BoardType, class ShieldType>
void BoardController<BoardType, ShieldType>::tickCpu() {
    shield_.tickCpu();
}

template<class BoardType, class ShieldType>
uint16_t BoardController<BoardType, ShieldType>::readDataBus(ActiveBusWidth width, bool peek) {
  return shield_.readDataBus(width, peek);
}

template<class BoardType, class ShieldType>
void BoardController<BoardType, ShieldType>::writeDataBus(uint16_t data, ActiveBusWidth width) {
  shield_.writeDataBus(data, width);
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readBHEPin() {
  return shield_.readBHEPin();
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readALEPin() {
  return shield_.readALEPin();
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readMRDCPin() {
  return shield_.readMRDCPin();
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readAMWCPin() {
  return shield_.readAMWCPin();
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readMWTCPin() {
  return shield_.readMWTCPin();
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readIORCPin() {
  return shield_.readIORCPin();
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readIOWCPin() {
  return shield_.readIOWCPin();
}

template<class BoardType, class ShieldType>
bool BoardController<BoardType, ShieldType>::readAIOWCPin() {
  return shield_.readAIOWCPin();
}

