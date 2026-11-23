#pragma once

// Bounded little-endian cursors over one payload, shared by the Driver
// Station codec (driver_station.cc), the hardware Command and State codecs
// (talOS/hardware/messages.cc), and the hardware configuration wire codec
// (talOS/hardware/config_wire.h). Each codec derives its own Writer and
// Reader from these and adds the field encodings that differ between them
// (the Driver Station's strings and finite-only floats, the hardware's
// doubles, tenths, signed 32-bit ids, and length-prefixed names). It lives
// in talOS/driver_station because talOS/hardware already depends on this
// package and this package depends on nothing. Header only, so the RoboRIO
// program's GradleRIO build compiles it too.

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace talos::driver_station {

// Writes the low `size` bytes of each integer, least significant first. A
// byte that would pass the end of `data` is not written and clears `ok`;
// the bytes before it stay written.
struct LittleEndianWriter {
  std::span<uint8_t> data;
  std::size_t pos{};
  bool ok{true};

  void Int(uint64_t value, int size) noexcept {
    for (int i = 0; i < size; ++i) {
      if (pos >= data.size()) {
        ok = false;
        return;
      }
      data[pos++] = static_cast<uint8_t>(value);
      value >>= 8;
    }
  }
};

// Reads what LittleEndianWriter writes. A read past the end of `data`
// returns 0 and clears `ok`, as does a float that is not finite.
struct LittleEndianReader {
  std::span<const uint8_t> data;
  std::size_t pos{};
  bool ok{true};

  uint64_t Int(int size) noexcept {
    uint64_t value{};
    for (int i = 0; i < size; ++i) {
      if (pos >= data.size()) {
        ok = false;
        return 0;
      }
      value |= uint64_t{data[pos++]} << (i * 8);
    }
    return value;
  }

  float Float() noexcept {
    const float x = std::bit_cast<float>(static_cast<uint32_t>(Int(4)));
    ok &= std::isfinite(x);
    return x;
  }

  // Every read was in bounds (and finite) and the payload is used up.
  bool Done() const noexcept { return ok && pos == data.size(); }
};

}  // namespace talos::driver_station
