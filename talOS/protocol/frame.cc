#include "frame.h"

#include <concepts>
#include <cstring>

namespace talos::protocol {
namespace {

template <std::unsigned_integral T>
void WriteUInt(uint8_t* out, T value) {
  std::memcpy(out, &value, sizeof(value));
}

/* memcpy and bit shift have same # of instructions when optimized
 "ReadUInt<uint16_t>(unsigned char const*)":
        movzx   eax, WORD PTR [rdi]
        ret
 "ReadUInt<uint16_t>memcpy(unsigned char const*)":
        movzx   eax, WORD PTR [rdi]
        ret
 */
template <std::unsigned_integral T>
T ReadUInt(const uint8_t* in) {
  T out;
  std::memcpy(&out, in, sizeof(T));
  return out;
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

  WriteUInt<uint64_t>(out + 0, kFrameMagic);
  WriteUInt<uint16_t>(out + 8, kProtocolVersion);
  WriteUInt<uint16_t>(out + 10, static_cast<uint16_t>(kFrameHeaderSize));
  WriteUInt<uint16_t>(out + 12, static_cast<uint16_t>(type));
  WriteUInt<uint16_t>(out + 14, flags);
  WriteUInt<uint64_t>(out + 16, sequence);
  WriteUInt<uint64_t>(out + 24, monotonic_time_us);
  WriteUInt<uint32_t>(out + 32, static_cast<uint32_t>(payload_size));
  WriteUInt<uint32_t>(out + 36, 0);

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

  WriteUInt<uint64_t>(frame + 0, kFrameMagic);
  WriteUInt<uint16_t>(frame + 8, kProtocolVersion);
  WriteUInt<uint16_t>(frame + 10, static_cast<uint16_t>(kFrameHeaderSize));
  WriteUInt<uint16_t>(frame + 12, static_cast<uint16_t>(type));
  WriteUInt<uint16_t>(frame + 14, flags);
  WriteUInt<uint64_t>(frame + 16, sequence);
  WriteUInt<uint64_t>(frame + 24, monotonic_time_us);
  WriteUInt<uint32_t>(frame + 32, static_cast<uint32_t>(payload_size));
  WriteUInt<uint32_t>(frame + 36, 0);

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
  header.magic = ReadUInt<uint64_t>(data + 0);
  header.version = ReadUInt<uint16_t>(data + 8);
  header.header_size = ReadUInt<uint16_t>(data + 10);
  header.type = static_cast<FrameType>(ReadUInt<uint16_t>(data + 12));
  header.flags = ReadUInt<uint16_t>(data + 14);
  header.sequence = ReadUInt<uint64_t>(data + 16);
  header.monotonic_time_us = ReadUInt<uint64_t>(data + 24);
  header.payload_size = ReadUInt<uint32_t>(data + 32);
  header.reserved = ReadUInt<uint32_t>(data + 36);

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
