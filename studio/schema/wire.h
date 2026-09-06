#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
namespace studio {
inline constexpr std::size_t slot_bytes = 65536;
inline constexpr std::size_t slot_count = 256;
// RTMS slots contain a little-endian byte count, then an ordinary FlatBuffer.
// Keep the FlatBuffers size prefix on WS/UDP; omit unused slot padding.
inline std::string_view payload(std::span<const std::byte> slot) {
  if (slot.size() < 4) return {};
  std::uint32_t n = 0;
  for (unsigned i = 0; i < 4; ++i)
    n |= std::to_integer<std::uint32_t>(slot[i]) << (8 * i);
  if (n < 8 || n > 65507 - 4 || n > slot.size() - 4) return {};
  return {reinterpret_cast<const char*>(slot.data()), n + 4};
}
}  // namespace studio
