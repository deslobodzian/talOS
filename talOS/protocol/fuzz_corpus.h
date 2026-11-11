#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace talos::protocol::testing {

// Exact-sized inputs make even one-byte overreads visible to ASan. Each seed
// visits every truncation, then deterministic bit flips, insertions and erases.
// Harnesses also export LLVMFuzzerTestOneInput; define TALOS_LIBFUZZER and link
// with -fsanitize=fuzzer to replace this bounded runner with libFuzzer.
template <typename Decode>
void RunCorpus(const std::vector<std::vector<uint8_t>>& seeds, Decode decode) {
  std::mt19937 random(0x74616c6f);
  for (const auto& seed : seeds) {
    for (std::size_t size = 0; size <= seed.size(); ++size) {
      const std::vector<uint8_t> input(
          seed.begin(), seed.begin() + static_cast<std::ptrdiff_t>(size));
      decode(input.data(), input.size());
    }
    // Visit every possible length/count-field byte, including multi-byte
    // all-ones values, independently of the random mutation sequence.
    for (std::size_t position = 0; position < seed.size(); ++position) {
      for (const uint8_t value : {uint8_t{0}, uint8_t{1}, uint8_t{0xff}}) {
        auto input = seed;
        for (std::size_t width = 1;
             width <= 4 && position + width <= input.size(); ++width) {
          input[position + width - 1] = value;
          const std::vector<uint8_t> exact(input.begin(), input.end());
          decode(exact.data(), exact.size());
        }
      }
    }
    for (int iteration = 0; iteration < 2048; ++iteration) {
      auto input = seed;
      const auto edits = 1 + random() % 8;
      for (unsigned edit = 0; edit < edits; ++edit) {
        const auto position = random() % (input.size() + 1);
        switch (random() % 4) {
          case 0:
            if (position < input.size()) {
              input[position] ^= uint8_t{1} << (random() % 8);
            }
            break;
          case 1:
            input.insert(input.begin() + static_cast<std::ptrdiff_t>(position),
                         static_cast<uint8_t>(random()));
            break;
          case 2:
            if (position < input.size())
              input.erase(input.begin() +
                          static_cast<std::ptrdiff_t>(position));
            break;
          case 3:
            if (position < input.size())
              input[position] = static_cast<uint8_t>(random());
            break;
          default:
            break;
        }
      }
      // Copy to an exact allocation: vector's spare capacity must not mask OOB.
      const std::vector<uint8_t> exact(input.begin(), input.end());
      decode(exact.data(), exact.size());
    }
  }
}

}  // namespace talos::protocol::testing
