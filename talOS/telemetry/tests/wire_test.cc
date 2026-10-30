#include "talOS/telemetry/wire.h"

#include <array>
#include <cstdlib>
#include <cstring>

namespace {

void Check(bool ok) {
  if (!ok) std::abort();
}

// Writes a fake size-prefixed TLM2 header: n = FlatBuffer bytes after the
// 4-byte prefix, identifier at bytes [8, 12).
void Header(std::span<std::byte> slot, std::uint32_t n,
            const char* identifier = "TLM2") {
  for (unsigned i = 0; i < 4; ++i) {
    slot[i] = static_cast<std::byte>((n >> (8 * i)) & 0xff);
  }
  std::memcpy(slot.data() + 8, identifier, 4);
}

}  // namespace

int main() {
  using talos::telemetry_wire::payload;

  // The contract constants: value frames fit an MTU and the R704 budget;
  // slots hold even the largest dictionary frame.
  Check(talos::telemetry_wire::kMaxValueFrameBytes == 1200);
  Check(talos::telemetry_wire::slot_bytes == 16384);
  Check(talos::telemetry_wire::slot_count == 64);
  Check(talos::telemetry_wire::kMaxValueFrameBytes <=
        talos::telemetry_wire::slot_bytes);

  std::array<std::byte, 64> bytes{};

  // An all-zero slot has size 0: not a frame.
  Check(payload(bytes).empty());
  // Fewer bytes than the prefix itself.
  Check(payload(std::span(bytes).first(3)).empty());

  // Minimum frame: 8 FlatBuffer bytes (root offset + identifier).
  Header(bytes, 8);
  auto p = payload(bytes);
  Check(p.size() == 12 &&
        p.data() == reinterpret_cast<const char*>(bytes.data()));

  // Below the minimum.
  Header(bytes, 7);
  Check(payload(bytes).empty());

  // The v1 identifier is not v2.
  Header(bytes, 8, "TLMS");
  Check(payload(bytes).empty());
  Header(bytes, 8, "XXXX");
  Check(payload(bytes).empty());

  // A declared size beyond the slot is rejected.
  Header(bytes, 61);  // 61 > 64 - 4
  Check(payload(bytes).empty());

  // A full-size slot at the geometry limit is accepted; one byte more is not.
  std::array<std::byte, talos::telemetry_wire::slot_bytes> maximum{};
  Header(maximum, talos::telemetry_wire::slot_bytes - 4);
  Check(payload(maximum).size() == talos::telemetry_wire::slot_bytes);
  Header(maximum, talos::telemetry_wire::slot_bytes - 3);
  Check(payload(maximum).empty());
}
