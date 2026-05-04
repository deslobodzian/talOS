#pragma once

#include <cstdint>
#include <concepts>

namespace talos::protocol {

constexpr uint16_t kProtocolVersion = 1;
constexpr uint64_t kFrameMagic = 0x4F4952534F4C4154ull;  // "TALOSRIO" LE

constexpr uint16_t kTcpConfigPort = 5801;
constexpr uint16_t kUdpRuntimePort = 5802;

enum class FrameType : uint16_t {
  kHello = 1,
  kManifestRequest = 2,
  kManifest = 3,
  kManifestAck = 4,
  kRioState = 10,
  kRioCommand = 11,
  kHeartbeat = 12,
  kFault = 13,
  kConfigError = 14,
};

enum FrameFlags : uint16_t {
  kFlagNone = 0,
  kFlagAckRequested = 1u << 0,
  kFlagAck = 1u << 1,
  kFlagFragment = 1u << 2,
};

enum class SignalDirection : uint8_t {
  kUnknown = 0,
  kRioToMac = 1,
  kMacToRio = 2,
};

enum class SignalType : uint8_t {
  kUnknown = 0,
  kState = 1,
  kCommand = 2,
  kFault = 3,
};

enum class ValueType : uint8_t {
  kUnknown = 0,
  kBool = 1,
  kInt64 = 2,
  kUint64 = 3,
  kFloat64 = 4,
};

struct RioHello {
  uint16_t protocol_version;
  uint16_t reserved;
  uint32_t capabilities;
  uint32_t max_payload_size;
  uint32_t control_period_us;
  uint64_t rio_boot_id;
  char systemcore_version[32];
};

struct ManifestHeader {
  uint64_t manifest_id;
  uint32_t signal_count;
  uint32_t command_count;
  uint32_t state_count;
  uint32_t fault_count;
};

struct SignalDescriptor {
  uint16_t id;
  SignalDirection direction;
  SignalType signal_type;
  ValueType value_type;
  uint8_t flags;
  uint16_t reserved;
  uint32_t freshness_timeout_us;
  double min_value;
  double max_value;
  char name[48];
};

struct SignalValue {
  uint16_t id;
  ValueType value_type;
  uint8_t reserved;
  union {
    bool bool_value;
    int64_t int64_value;
    uint64_t uint64_value;
    double float64_value;
  };
};

struct RuntimeValuesHeader {
  uint64_t manifest_id;
  uint32_t value_count;
  uint32_t reserved;
};

struct FaultPayload {
  uint16_t fault_id;
  uint8_t severity;
  uint8_t reserved;
  uint32_t detail_code;
  char message[96];
};

static_assert(sizeof(RioHello) == 56);
static_assert(sizeof(ManifestHeader) == 24);
static_assert(sizeof(SignalDescriptor) == 80);
static_assert(sizeof(SignalValue) == 16);
static_assert(sizeof(RuntimeValuesHeader) == 16);
static_assert(sizeof(FaultPayload) == 104);

}  // namespace talos::protocol
