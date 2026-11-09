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

#include <cstdio>
#include <limits>

#include <bus_emulator/IBusBackend.h>
#include <StaticHashTable.h>
#include <serial_config.h>

class HashBackend : public IBusBackend {
public:
  explicit HashBackend(size_t mem_capacity = 65536)
    : mem_table_(mem_capacity), base_seed_(0), strategy_start_(0x1024), strategy_end_(0xFFFFFF) {}

  ~HashBackend() {
    DEBUG_SERIAL.println("## HASH_BACKEND: Memory freed");
  }

  IBusBackendType type() const override {
    return IBusBackendType::HashTable;
  }

  size_t size() const override {
    return 0;
  }

  // ---------------- Memory Interface ----------------
  uint8_t read_u8(uint32_t address) override {
    const uint32_t addr16 = address >> 1;
    uint16_t word = 0;
    if (!mem_table_.find(addr16, word)) {
      // If the address is not found, generate a default value based on the strategy.
      word = gen_default_u16(addr16);
    }
    return (address & 1) ? (word >> 8) : (word & 0xFF);
  }

  uint16_t read_u16(uint32_t address) override {
    // Even reads
    if ((address & 1) == 0) {
      const uint32_t addr16 = address >> 1;
      uint16_t word = 0;
      if (!mem_table_.find(addr16, word)) {
        // If the address is not found, generate a default value based on the strategy.
        word = gen_default_u16(addr16);
      }
      return word;
    }
    else {
      // Odd reads: combine two adjacent words
      const uint32_t addr16_low = address >> 1;
      const uint32_t addr16_high = addr16_low + 1;
      uint16_t word_low = 0;
      uint16_t word_high = 0;
      if (!mem_table_.find(addr16_low, word_low)) {
        word_low = gen_default_u16(addr16_low);
      }
      if (!mem_table_.find(addr16_high, word_high)) {
        word_high = gen_default_u16(addr16_high);
      }
      return (word_high << 8) | ((word_low >> 8) & 0x00FF);
    }
  }

  uint16_t read_bus(uint32_t address, bool bhe) override {
    const uint32_t addr16 = address >> 1;
    uint16_t word = 0;
    if (!mem_table_.find(addr16, word)) {
      // If the address is not found, generate a default value based on the strategy.
      word = gen_default_u16(addr16);
    }
    return word;
  }

  uint8_t *get_ptr(uint32_t addr) override { 
    // Can't get a pointer with this backend.
    return NULL; 
  }

  void write_u8(uint32_t address, uint8_t value) override {
    const uint32_t addr16 = address >> 1;
    uint16_t word = 0;
    if (!mem_table_.find(addr16, word)) {
      // If the address is not found, generate a default value based on the strategy.
      word = gen_default_u16(addr16);
    }
    if (address & 1) {
      word = (word & 0x00FF) | (static_cast<uint16_t>(value) << 8); // write high byte
    } else {
      word = (word & 0xFF00) | static_cast<uint16_t>(value);        // write low byte
    }
    mem_table_.insert(addr16, word);
  }

  void write_u16(uint32_t address, uint16_t value) override {
    const uint32_t addr16 = address >> 1;
    mem_table_.insert(addr16, value);
  }

  void write_bus(uint32_t address, uint16_t value, bool bhe) override {
    const uint32_t addr16 = address >> 1;
    const bool a0 = address & 1;
    uint16_t word = 0;
    if (!mem_table_.find(addr16, word)) {
      // If the address is not found, generate a default value based on the strategy.      
      word = gen_default_u16(addr16);
    }

    if (a0 && bhe) {
      // Write high byte only
      word = (word & 0x00FF) | (value & 0xFF00);
    } else if (!a0 && bhe) {
      // Write full word
      word = value;
    } else if (!a0 && !bhe) {
      // Write low byte only
      word = (word & 0xFF00) | (value & 0x00FF);
    } else {
      // a0 == 1 && bhe == 0: refresh cycle
      return;
    }

    mem_table_.insert(addr16, word);
  }

  void set_memory(uint32_t address, const uint8_t* buffer, size_t length) override {

    uint8_t *buf_ptr = (uint8_t *)buffer;
    if ((address & 0x01) == 1) {
      // Unaligned start address
      write_u8(address, buf_ptr[0]);
      address += 1;
      buf_ptr += 1;
      length -= 1;
    }

    // Write remaining buffer in 16-bit chunks
    for (size_t i = 0; i + 1 < length; i += 2) {
      uint16_t word = buf_ptr[i] | (static_cast<uint16_t>(buf_ptr[i + 1]) << 8);
      write_u16(address, word);
      address += 2;
    }

    // Write trailing byte if length is odd
    if (length & 0x01) {
      write_u8(address, buf_ptr[length - 1]);
    }
  }

  void erase_memory() override {
    mem_table_.clear();
    DEBUG_SERIAL.println("## HASH_BACKEND: Memory erased");
  }

  void randomize_memory(uint32_t seed) override {
    base_seed_ = seed;
    mem_table_.clear();
  }

  void debug_mem(uint32_t address, size_t length) override {
    char buf[64];
    
    for (size_t i = 0; i < length; ++i) {
      uint8_t value = read_u8(address + i);
      snprintf(buf, 64, "0x%08lX: 0x%02X\n\r", address + i, value);
      DEBUG_SERIAL.print(buf);
    }
  }

  // ---------------- I/O Interface (Stubbed) ----------------
  uint8_t io_read_u8(uint16_t) override { return 0xFF; }
  uint16_t io_read_u16(uint16_t) override { return 0xFFFF; }
  uint16_t io_read_bus(uint16_t, bool) override { return 0xFFFF; }
  void io_write_u8(uint16_t, uint8_t) override {}
  void io_write_u16(uint16_t, uint16_t) override {}
  void io_write_bus(uint16_t, uint16_t, bool) override {}

  void set_strategy(DefaultStrategy strategy, uint32_t start, uint32_t end) override {
    strategy_ = strategy;
    strategy_start_ = start;
    strategy_end_ = end;
  }

private:
  StaticHashTable<uint32_t, uint16_t> mem_table_;

  uint32_t base_seed_;
  uint32_t strategy_start_; // Address below which to ignore strategy.
  uint32_t strategy_end_;   // Address above which to ignore strategy.
  DefaultStrategy strategy_ = DefaultStrategy::Random; // Default strategy for generating values.

  uint16_t gen_default_u16(uint32_t address) {
    // Generate a default 16-bit value based on the strategy.
    if (address < strategy_start_ || address > strategy_end_) {
      //DEBUG_SERIAL.println("Using Random strategy for address: " + String(address, HEX));
      return gen_random_u16(address);
    }
    switch (strategy_) {
      case DefaultStrategy::Zero:
        //DEBUG_SERIAL.println("Using Zero strategy for address: " + String(address, HEX));
        return 0x0000;
      case DefaultStrategy::Ones:
        //DEBUG_SERIAL.println("Using Ones strategy for address: " + String(address, HEX));
        return 0xFFFF;
      case DefaultStrategy::Random: // FALLTHROUGH
      default:
        //DEBUG_SERIAL.println("Using Random strategy for address: " + String(address, HEX));
        return gen_random_u16(address);
    }
  }

  // uint16_t gen_random_u16(uint32_t address) {
  //   // combine seed and input into one 64-bit word
  //   uint64_t z = (uint64_t(base_seed_) << 32) | address;

  //   // avalanche (SplitMix64)
  //   z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  //   z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  //   z = z ^ (z >> 31);

  //   // return    16 bits
  //   return uint16_t(z & 0xFFFFU);
  // }  


  static inline uint32_t murmur3_fmix32(uint32_t h) {
    h ^= h >> 16;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return h;
  }

  /// 32→16-bit hash with seed
  static inline uint16_t hash16_murmur3(uint32_t x, uint32_t seed) {
      // combine seed and input
      uint32_t h = x ^ seed;
      // avalanche
      h = murmur3_fmix32(h);
      // take the upper 16 bits
      return uint16_t(h >> 16);
  }
  
  // Return a random 
  inline uint16_t gen_random_u16(uint32_t address) {
    return hash16_murmur3(address, base_seed_); // Example seed
  }
};
