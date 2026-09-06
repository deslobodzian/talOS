#include "studio/schema/wire.h"

#include <array>
#include <cstdlib>
int main() {
  auto check = [](bool b) {
    if (!b) std::abort();
  };
  std::array<std::byte, 32> bytes{};
  check(studio::payload(bytes).empty());
  bytes[0] = std::byte{8};
  auto p = studio::payload(bytes);
  check(p.size() == 12 &&
        p.data() == reinterpret_cast<const char*>(bytes.data()));
  bytes[0] = std::byte{29};
  check(studio::payload(bytes).empty());
  bytes[0] = std::byte{0xff};
  bytes[1] = std::byte{0xff};
  check(studio::payload(bytes).empty());
  check(studio::payload(std::span(bytes).first(3)).empty());
  std::array<std::byte, studio::slot_bytes> maximum{};
  maximum[0] = std::byte{0xdf};
  maximum[1] = std::byte{0xff};  // 65503 + prefix
  check(studio::payload(maximum).size() == 65507);
  maximum[0] = std::byte{0xe0};
  check(studio::payload(maximum).empty());
}
