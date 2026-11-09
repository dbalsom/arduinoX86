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

#include <Board.h>
#include <Shield.h>
#include <bus_emulator/BusEmulator.h>
#include <CycleStateLogger.h>

template<typename BoardType, typename ShieldType> class BoardController;
template<typename BoardType, typename ShieldType> class CommandServer;

extern Cpu CPU;
extern Intel8288 I8288;
extern BoardController<BoardType,ShieldType> Controller;

namespace ArduinoX86 {
  extern CommandServer<BoardType,ShieldType> Server;
  extern BusEmulator *Bus;
  extern CycleStateLogger *CycleLogger;
}

extern bool screen_init_requested;

// cpu_server.cpp
extern const char RESPONSE_CHRS[];
extern const char VERSION_DAT[];
extern const size_t VERSION_DAT_LEN;
extern const char MACHINE_STATE_CHARS[];
extern const char * const MACHINE_STATE_STRINGS[];
extern const char * const CMD_STRINGS[];
