#pragma once

// SHA-256 (FIPS 180-4) of a file, so a session records which binary each
// member ran: the same lowercase hex digest `sha256sum` prints and the bundle
// builder writes into bundle_manifest.toml (hashlib.sha256).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <string_view>

namespace talos::launcher {

class Sha256 {
 public:
  void Update(const unsigned char* data, std::size_t size) {
    total_bytes_ += size;
    while (size > 0) {
      const std::size_t take = std::min(size, block_.size() - used_);
      for (std::size_t i = 0; i < take; ++i) block_[used_ + i] = data[i];
      used_ += take;
      data += take;
      size -= take;
      if (used_ == block_.size()) {
        Compress();
        used_ = 0;
      }
    }
  }

  void Update(std::string_view bytes) {
    Update(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size());
  }

  // The digest as 64 lowercase hex digits. Ends this hash.
  std::string HexDigest() {
    const uint64_t bits = total_bytes_ * 8;
    const unsigned char pad = 0x80;
    Update(&pad, 1);
    const unsigned char zero = 0;
    while (used_ != 56) Update(&zero, 1);
    std::array<unsigned char, 8> length{};
    for (std::size_t i = 0; i < 8; ++i)
      length[i] = static_cast<unsigned char>(bits >> (56 - 8 * i));
    Update(length.data(), length.size());
    static constexpr std::string_view kHex = "0123456789abcdef";
    std::string out;
    out.reserve(64);
    for (const uint32_t word : state_) {
      for (int shift = 28; shift >= 0; shift -= 4)
        out += kHex[(word >> shift) & 0xF];
    }
    return out;
  }

 private:
  static constexpr uint32_t Rotr(uint32_t x, int n) {
    return (x >> n) | (x << (32 - n));
  }

  void Compress() {
    static constexpr std::array<uint32_t, 64> kK{
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
        0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
        0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
        0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
        0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
        0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
        0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    std::array<uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16; ++i) {
      w[i] = (uint32_t{block_[4 * i]} << 24) |
             (uint32_t{block_[4 * i + 1]} << 16) |
             (uint32_t{block_[4 * i + 2]} << 8) | uint32_t{block_[4 * i + 3]};
    }
    for (std::size_t i = 16; i < 64; ++i) {
      const uint32_t s0 =
          Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 =
          Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
    uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
    for (std::size_t i = 0; i < 64; ++i) {
      const uint32_t t1 = h + (Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25)) +
                          ((e & f) ^ (~e & g)) + kK[i] + w[i];
      const uint32_t t2 = (Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22)) +
                          ((a & b) ^ (a & c) ^ (b & c));
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<uint32_t, 8> state_{0x6a09e667, 0xbb67ae85, 0x3c6ef372,
                                 0xa54ff53a, 0x510e527f, 0x9b05688c,
                                 0x1f83d9ab, 0x5be0cd19};
  std::array<unsigned char, 64> block_{};
  std::size_t used_{0};
  uint64_t total_bytes_{0};
};

// The SHA-256 of the file at `path`, streamed; empty when it cannot be read
// whole.
inline std::string FileSha256(const std::string& path) {
  std::ifstream in{path, std::ios::binary};
  if (!in) return {};
  Sha256 hash;
  std::array<char, 1 << 16> buffer{};
  while (in) {
    in.read(buffer.data(), buffer.size());
    hash.Update(reinterpret_cast<const unsigned char*>(buffer.data()),
                static_cast<std::size_t>(in.gcount()));
  }
  if (!in.eof()) return {};
  return hash.HexDigest();
}

}  // namespace talos::launcher
