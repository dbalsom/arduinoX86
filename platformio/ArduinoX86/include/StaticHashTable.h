#pragma once
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <cstddef>

#ifdef ARDUINO_GIGA
  #include <SDRAM.h>
  #define HT_ALLOC(sz) SDRAM.malloc(sz)
  #define HT_FREE(ptr) SDRAM.free(ptr)
#else
  #include <cstdlib>
  #define HT_ALLOC(sz) malloc(sz)
  #define HT_FREE(ptr) free(ptr)
#endif

template<typename Key = uint32_t>
struct DefaultHash {
  size_t operator()(Key key, uint8_t shift) const {
    return (static_cast<uint32_t>(key) * 2654435769u) >> shift;
  }
};

template<typename Key = uint32_t, typename Value = uint16_t, typename HashFn = DefaultHash<Key>>
class StaticHashTable {
public:
  struct Entry {
    Key key;
    Value value;
    bool in_use;
  };

  explicit StaticHashTable(size_t capacity)
      : capacity_(capacity), count_(0)
  {
    if ((capacity_ & (capacity_ - 1)) != 0) {
      assert(!"Hash table capacity must be a power of two");
      entry_pool_ = nullptr;
      return;
    }

    shift_ = 32 - __builtin_ctz(capacity_);
    entry_pool_ = static_cast<Entry *>(HT_ALLOC(sizeof(Entry) * capacity_));
    if (entry_pool_) {
      memset(entry_pool_, 0, sizeof(Entry) * capacity_);
    }
  }

  ~StaticHashTable() {
    if (entry_pool_) {
      HT_FREE(entry_pool_);
    }
  }

  bool insert(Key key, Value value) {
    if (!entry_pool_) return false;

    size_t index = hasher_(key, shift_);
    for (size_t i = 0; i < capacity_; ++i) {
      Entry &entry = entry_pool_[index];
      if (!entry.in_use || entry.key == key) {
        entry.key = key;
        entry.value = value;
        if (!entry.in_use) {
          entry.in_use = true;
          ++count_;
        }
        return true;
      }
      index = (index + 1) & (capacity_ - 1);
    }
    return false;
  }

  bool find(Key key, Value &value_out) const {
    if (!entry_pool_) return false;

    size_t index = hasher_(key, shift_);
    for (size_t i = 0; i < capacity_; ++i) {
      const Entry &entry = entry_pool_[index];
      if (!entry.in_use) return false;
      if (entry.key == key) {
        value_out = entry.value;
        return true;
      }
      index = (index + 1) & (capacity_ - 1);
    }
    return false;
  }

  void clear() {
    if (entry_pool_) {
      memset(entry_pool_, 0, sizeof(Entry) * capacity_);
      count_ = 0;
    }
  }

  size_t size() const { return count_; }
  size_t capacity() const { return capacity_; }

private:
  Entry *entry_pool_ = nullptr;
  size_t capacity_;
  size_t count_;
  uint8_t shift_;
  HashFn hasher_;
};
