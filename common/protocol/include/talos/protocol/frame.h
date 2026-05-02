#pragma once

#include <cstddef>
#include <cstdint>

#include "talos/protocol/types.h"

namespace talos::protocol {

constexpr std::size_t kFrameHeaderSize = 40;
constexpr std::size_t kMaxPayloadSize = 1200;
constexpr std::size_t kMaxFrameSize = kFrameHeaderSize + kMaxPayloadSize;

enum class DecodeStatus : uint8_t {
  kOk = 0,
  kTooSmall,
  kBadMagic,
  kUnsupportedVersion,
  kBadHeaderSize,
  kPayloadTooLarge,
  kTruncated,
};

struct FrameHeader {
  uint64_t magic = kFrameMagic;
  uint16_t version = kProtocolVersion;
  uint16_t header_size = kFrameHeaderSize;
  FrameType type = FrameType::kHeartbeat;
  uint16_t flags = 0;
  uint64_t sequence = 0;
  uint64_t monotonic_time_us = 0;
  uint32_t payload_size = 0;
  uint32_t reserved = 0;
};

struct DecodedFrame {
  FrameHeader header;
  const uint8_t* payload = nullptr;
  std::size_t payload_size = 0;
};

std::size_t EncodeFrame(FrameType type, uint16_t flags, uint64_t sequence,
                        uint64_t monotonic_time_us, const void* payload,
                        std::size_t payload_size, uint8_t* out,
                        std::size_t out_capacity);

std::size_t EncodeFrameInPlace(FrameType type, uint16_t flags,
                               uint64_t sequence, uint64_t monotonic_time_us,
                               std::size_t payload_size, uint8_t* frame,
                               std::size_t frame_capacity);

DecodeStatus DecodeFrame(const uint8_t* data, std::size_t size,
                         DecodedFrame* out);

const char* DecodeStatusName(DecodeStatus status);

}  // namespace talos::protocol
