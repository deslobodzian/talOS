// The flatc C++ bytes of fixed schema_test values, one "<label> <hex>" line
// each, for //talOS/python:messages_test to compare the Python codec against.
// messages_test.py holds the same values; change both together.

#include <cstddef>
#include <cstdio>

#include "schema_test/shapes_generated.h"

namespace {

template <typename T>
void Print(const char* label, const T& value) {
  std::printf("%s ", label);
  const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
  for (std::size_t i = 0; i < sizeof(T); ++i) std::printf("%02x", bytes[i]);
  std::printf("\n");
}

}  // namespace

int main() {
  using schema_test::Arrays;
  using schema_test::Inner;
  using schema_test::Level;
  using schema_test::Mixed;
  using schema_test::Mode;

  Print("inner", Inner(Mode::HIGH, -1.5f));
  Print("mixed_zero", Mixed());
  Print("mixed_top", Mixed(true, 1234.5678, Level::TOP, 4000000000u,
                           Inner(Mode::LOW, 0.25f), -0.75f));
  Print("mixed_bottom",
        Mixed(false, -0.5, Level::BOTTOM, 7u, Inner(Mode::OFF, -8.0f), 1.0f));

  Arrays arrays;
  const double samples[] = {1.5, -2.25, 1e9};
  for (int i = 0; i < 3; ++i) arrays.mutable_samples_m()->Mutate(i, samples[i]);
  arrays.mutable_modes()->Mutate(0, Mode::HIGH);
  arrays.mutable_modes()->Mutate(1, Mode::LOW);
  *arrays.mutable_inners()->GetMutablePointer(0) = Inner(Mode::LOW, 3.5f);
  *arrays.mutable_inners()->GetMutablePointer(1) = Inner(Mode::HIGH, -0.125f);
  const unsigned char channel[] = {1, 2, 3, 254, 255};
  for (int i = 0; i < 5; ++i)
    arrays.mutable_channel_id()->Mutate(i, channel[i]);
  Print("arrays", arrays);
  Print("arrays_zero", Arrays());
  return 0;
}
