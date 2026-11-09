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
#include <cstdint>
#include <cstddef>


enum class IBusBackendType : uint8_t {
  Null,
  HashTable,
  Sdram,
  Invalid,
};

// Abstract interface for bus backing implementations
class IBusBackend {
public:

  enum class DefaultStrategy: uint8_t {
    Random,
    Zero,
    Ones,
    Invalid,
  };

  virtual IBusBackendType type() const = 0;
  virtual size_t   size() const = 0;
  virtual uint8_t  read_u8(uint32_t address) = 0;
  virtual uint16_t read_u16(uint32_t address) = 0;
  virtual uint16_t read_bus(uint32_t address, bool bhe) = 0;
  virtual uint8_t *get_ptr(uint32_t address) = 0;
  // Copy a range even when the backend cannot expose a contiguous pointer.
  virtual void read_bytes(uint32_t address, uint8_t* buffer, size_t length) {
    if (length && (address & 1)) {
      *buffer++ = read_u8(address++);
      --length;
    }
    while (length >= 2) {
      const uint16_t word = read_u16(address);
      buffer[0] = uint8_t(word);
      buffer[1] = uint8_t(word >> 8);
      buffer += 2;
      address += 2;
      length -= 2;
    }
    if (length) {
      *buffer = read_u8(address);
    }
  }
  virtual void     write_u8(uint32_t address, uint8_t  value) = 0;
  virtual void     write_u16(uint32_t address, uint16_t value) = 0;
  virtual void     write_bus(uint32_t address, uint16_t value, bool bhe) = 0;
  virtual uint8_t  io_read_u8(uint16_t port) = 0;
  virtual uint16_t io_read_u16(uint16_t port) = 0;
  virtual uint16_t io_read_bus(uint16_t port, bool bhe) = 0;
  virtual void     io_write_u8(uint16_t port, uint8_t  value) = 0;
  virtual void     io_write_u16(uint16_t port, uint16_t value) = 0;
  virtual void     io_write_bus(uint16_t port, uint16_t value, bool bhe) = 0;
  virtual void     set_memory(uint32_t address, const uint8_t* buffer, size_t length) = 0;
  virtual void     erase_memory() = 0;
  virtual void     set_strategy(DefaultStrategy strategy, uint32_t start, uint32_t end) = 0;
  virtual void     randomize_memory(uint32_t seed) = 0;
  virtual void     debug_mem(uint32_t address, size_t length) = 0;

  virtual ~IBusBackend() {}
};
