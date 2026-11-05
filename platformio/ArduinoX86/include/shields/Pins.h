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

#include <Arduino.h>

/// @brief Enumeration of input pins used by the CPU.
/// These pins can be referenced as parameters to a Shield's writePin() or readPin() methods,
/// but there is no guarantee that all pins are implemented on all Hats.
enum class OutputPin : uint8_t {
  Ready,
  Test,
  Intr,
  Nmi,
  Invalid, // Invalid pin variant is used as only previous pins in the list can be sent as 
           // parameters to cmd_write_pin().
  Reset,
  Clock,
  Hold,
  Busy,
  Smi,
  Error, // Optional shield-specific ERROR input; 286 wire protocol index4.
};

