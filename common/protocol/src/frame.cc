#include "talos/protocol/frame.h"

#include <cstring>

namespace talos::protocol {
namespace {

void WriteU16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}

void WriteU32(uint8_t* out, uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    out[i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

void WriteU64(uint8_t* out, uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    out[i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

uint16_t ReadU16(const uint8_t* in) {
  return static_cast<uint16_t>(in[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(in[1]) << 8);
}

uint32_t ReadU32(const uint8_t* in) {
  uint32_t value = 0;
  for (int i = 0; i < 4; ++i) {
    value |= static_cast<uint32_t>(in[i]) << (8 * i);
  }
  return value;
}

uint64_t ReadU64(const uint8_t* in) {
  uint64_t value = 0;
  for (int i = 0; i < 8; ++i) {
    value |= static_cast<uint64_t>(in[i]) << (8 * i);
  }
  return value;
}

}  // namespace

std::size_t EncodeFrame(FrameType type, uint16_t flags, uint64_t sequence,
                        uint64_t monotonic_time_us, const void* payload,
                        std::size_t payload_size, uint8_t* out,
                        std::size_t out_capacity) {
  if (payload_size > kMaxPayloadSize ||
      out_capacity < kFrameHeaderSize + payload_size) {
    return 0;
  }
  if (payload_size > 0 && payload == nullptr) {
    return 0;
  }

  WriteU64(out + 0, kFrameMagic);
  WriteU16(out + 8, kProtocolVersion);
  WriteU16(out + 10, static_cast<uint16_t>(kFrameHeaderSize));
  WriteU16(out + 12, static_cast<uint16_t>(type));
  WriteU16(out + 14, flags);
  WriteU64(out + 16, sequence);
  WriteU64(out + 24, monotonic_time_us);
  WriteU32(out + 32, static_cast<uint32_t>(payload_size));
  WriteU32(out + 36, 0);

  if (payload_size > 0) {
    std::memcpy(out + kFrameHeaderSize, payload, payload_size);
  }

  return EncodeFrameInPlace(type, flags, sequence, monotonic_time_us,
                            payload_size, out, out_capacity);
}

std::size_t EncodeFrameInPlace(FrameType type, uint16_t flags,
                               uint64_t sequence, uint64_t monotonic_time_us,
                               std::size_t payload_size, uint8_t* frame,
                               std::size_t frame_capacity) {
  if (payload_size > kMaxPayloadSize ||
      frame_capacity < kFrameHeaderSize + payload_size || frame == nullptr) {
    return 0;
  }

  WriteU64(frame + 0, kFrameMagic);
  WriteU16(frame + 8, kProtocolVersion);
  WriteU16(frame + 10, static_cast<uint16_t>(kFrameHeaderSize));
  WriteU16(frame + 12, static_cast<uint16_t>(type));
  WriteU16(frame + 14, flags);
  WriteU64(frame + 16, sequence);
  WriteU64(frame + 24, monotonic_time_us);
  WriteU32(frame + 32, static_cast<uint32_t>(payload_size));
  WriteU32(frame + 36, 0);

  return kFrameHeaderSize + payload_size;
}

DecodeStatus DecodeFrame(const uint8_t* data, std::size_t size,
                         DecodedFrame* out) {
  if (data == nullptr || out == nullptr) {
    return DecodeStatus::kTooSmall;
  }
  if (size < kFrameHeaderSize) {
    return DecodeStatus::kTooSmall;
  }

  FrameHeader header;
  header.magic = ReadU64(data + 0);
  header.version = ReadU16(data + 8);
  header.header_size = ReadU16(data + 10);
  header.type = static_cast<FrameType>(ReadU16(data + 12));
  header.flags = ReadU16(data + 14);
  header.sequence = ReadU64(data + 16);
  header.monotonic_time_us = ReadU64(data + 24);
  header.payload_size = ReadU32(data + 32);
  header.reserved = ReadU32(data + 36);

  if (header.magic != kFrameMagic) {
    return DecodeStatus::kBadMagic;
  }
  if (header.version != kProtocolVersion) {
    return DecodeStatus::kUnsupportedVersion;
  }
  if (header.header_size != kFrameHeaderSize) {
    return DecodeStatus::kBadHeaderSize;
  }
  if (header.payload_size > kMaxPayloadSize) {
    return DecodeStatus::kPayloadTooLarge;
  }
  const std::size_t frame_size = kFrameHeaderSize + header.payload_size;
  if (size < frame_size) {
    return DecodeStatus::kTruncated;
  }
  out->header = header;
  out->payload = data + kFrameHeaderSize;
  out->payload_size = header.payload_size;
  return DecodeStatus::kOk;
}

const char* DecodeStatusName(DecodeStatus status) {
  switch (status) {
    case DecodeStatus::kOk:
      return "Ok";
    case DecodeStatus::kTooSmall:
      return "TooSmall";
    case DecodeStatus::kBadMagic:
      return "BadMagic";
    case DecodeStatus::kUnsupportedVersion:
      return "UnsupportedVersion";
    case DecodeStatus::kBadHeaderSize:
      return "BadHeaderSize";
    case DecodeStatus::kPayloadTooLarge:
      return "PayloadTooLarge";
    case DecodeStatus::kTruncated:
      return "Truncated";
  }
  return "Unknown";
}

}  // namespace talos::protocol
