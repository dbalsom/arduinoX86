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


#include <arduinox86.h>
#include <config.h>
#include <globals.h>
#include <Shield.h>

/// @brief Handle assertion of ALE/ADS by deasserting READY pin.
void ale_interrupt_handler() {
  DEBUG_SERIAL.println(">ADS");

  // The 386EX coprocessor handshake requires its dedicated READYO signal.
  #ifdef READYO_PIN
  if (READ_ABUS_23 &&!READ_M_IO_PIN) {
    DEBUG_SERIAL.println(">NPX");
    WRITE_READY_PIN(READY_DEASSERT);
  }
  #endif
}

void readyo_interrupt_handler() {
  #ifdef READYO_PIN
  WRITE_READY_PIN(READY_ASSERT);
  #endif
}

void cycle_edge_interrupt_handler() {
  DEBUG_SERIAL.println("@");
}
