#pragma once

#include <algorithm>
#include <array>
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

// Deterministic seeded mutation campaign: runs truncations and byte-width
// sweeps via RunCorpus, then `random_iterations` diverse multi-edit mutations
// (bit/byte flips, boundary integers, block insert/erase, and cross-seed
// splicing) plus fresh random byte buffers up to `max_random_bytes`.
template <typename Decode>
void RunMutatedCampaign(const std::vector<std::vector<uint8_t>>& seeds,
                        Decode decode, std::size_t random_iterations,
                        std::size_t max_random_bytes,
                        uint32_t rng_seed = 0x74616c6f) {
  RunCorpus(seeds, decode);
  if (seeds.empty()) return;

  constexpr std::array<uint8_t, 6> kBoundaryBytes{0x00, 0x01, 0x7f,
                                                  0x80, 0xfe, 0xff};
  constexpr std::array<uint32_t, 8> kBoundaryWords{
      0x00000000u, 0x00000001u, 0x7fffffffu, 0x80000000u,
      0xffffffffu, 0x7f800000u, 0xff800000u, 0x7fc00000u};

  std::mt19937 random(rng_seed);
  for (std::size_t iteration = 0; iteration < random_iterations; ++iteration) {
    std::vector<uint8_t> input;
    if ((iteration % 5) == 0) {
      // Fresh random buffer across lengths 0..max_random_bytes.
      const std::size_t len =
          max_random_bytes == 0 ? 0 : (random() % (max_random_bytes + 1));
      input.resize(len);
      for (std::size_t i = 0; i < len; ++i) {
        input[i] = static_cast<uint8_t>(random());
      }
    } else {
      input = seeds[random() % seeds.size()];
      const auto edits = 1 + (random() % 8);
      for (unsigned edit = 0; edit < edits; ++edit) {
        const std::size_t pos =
            input.empty() ? 0 : (random() % (input.size() + 1));
        switch (random() % 8) {
          case 0:  // Bit flip
            if (!input.empty()) {
              input[random() % input.size()] ^= uint8_t{1} << (random() % 8);
            }
            break;
          case 1:  // Boundary byte overwrite
            if (!input.empty()) {
              input[random() % input.size()] =
                  kBoundaryBytes[random() % kBoundaryBytes.size()];
            }
            break;
          case 2:  // Random byte overwrite
            if (!input.empty()) {
              input[random() % input.size()] = static_cast<uint8_t>(random());
            }
            break;
          case 3: {  // Block insert (1..16 bytes)
            const std::size_t count = 1 + (random() % 16);
            if (input.size() + count <= max_random_bytes) {
              const uint8_t fill = static_cast<uint8_t>(random());
              input.insert(input.begin() + static_cast<std::ptrdiff_t>(pos),
                           count, fill);
            }
            break;
          }
          case 4: {  // Block erase (1..16 bytes)
            if (!input.empty()) {
              const std::size_t start = random() % input.size();
              const std::size_t count = std::min<std::size_t>(
                  1 + (random() % 16), input.size() - start);
              input.erase(
                  input.begin() + static_cast<std::ptrdiff_t>(start),
                  input.begin() + static_cast<std::ptrdiff_t>(start + count));
            }
            break;
          }
          case 5: {  // Little-endian boundary 32-bit overwrite
            if (input.size() >= 4) {
              const std::size_t start = random() % (input.size() - 3);
              const uint32_t word =
                  kBoundaryWords[random() % kBoundaryWords.size()];
              for (std::size_t b = 0; b < 4; ++b) {
                input[start + b] = static_cast<uint8_t>(word >> (8 * b));
              }
            }
            break;
          }
          case 6: {  // Byte arithmetic +/- delta
            if (!input.empty()) {
              const std::size_t idx = random() % input.size();
              const uint8_t delta = static_cast<uint8_t>(1 + (random() % 16));
              if (random() & 1) {
                input[idx] = static_cast<uint8_t>(input[idx] + delta);
              } else {
                input[idx] = static_cast<uint8_t>(input[idx] - delta);
              }
            }
            break;
          }
          case 7: {  // Splice / crossover with another seed
            const auto& other = seeds[random() % seeds.size()];
            if (!input.empty() && !other.empty()) {
              const std::size_t cut_a = random() % input.size();
              const std::size_t cut_b = random() % other.size();
              const std::size_t take = std::min<std::size_t>(
                  other.size() - cut_b,
                  max_random_bytes > cut_a ? max_random_bytes - cut_a : 0);
              input.resize(cut_a);
              input.insert(
                  input.end(),
                  other.begin() + static_cast<std::ptrdiff_t>(cut_b),
                  other.begin() + static_cast<std::ptrdiff_t>(cut_b + take));
            }
            break;
          }
          default:
            break;
        }
      }
    }
    const std::vector<uint8_t> exact(input.begin(), input.end());
    decode(exact.empty() && (iteration & 1) ? nullptr : exact.data(),
           exact.size());
  }
}

}  // namespace talos::protocol::testing
