#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "../driver_station/little_endian.h"
#include "../protocol/frame.h"
#include "config.h"

namespace talos::hardware {

inline constexpr double kCeilingMaxSupplyCurrentA = 60.0;
inline constexpr double kCeilingMaxStatorCurrentA = 100.0;
inline constexpr double kCeilingMaxVoltageV = 12.0;
inline constexpr double kCeilingMaxVelocityRps = 100.0;

struct CeilingCheckResult {
  bool ok{true};
  std::string reason;
};

inline bool CheckCeilingsInto(const Config& config,
                              std::span<char> reason) noexcept {
  const auto fail = [reason](const ConfigString& name, const char* what,
                             double value, double ceiling) noexcept {
    if (!reason.empty()) {
      std::snprintf(
          reason.data(), reason.size(), "motor '%.*s' %s %f > ceiling %f",
          static_cast<int>(name.size()), name.data(), what, value, ceiling);
    }
    return false;
  };
  for (const auto& m : config.motors) {
    if (m.supply_limit_a > kCeilingMaxSupplyCurrentA) {
      return fail(m.name, "supply limit", m.supply_limit_a,
                  kCeilingMaxSupplyCurrentA);
    }
    if (m.stator_limit_a > kCeilingMaxStatorCurrentA) {
      return fail(m.name, "stator limit", m.stator_limit_a,
                  kCeilingMaxStatorCurrentA);
    }
    if (m.max_voltage > kCeilingMaxVoltageV) {
      return fail(m.name, "voltage", m.max_voltage, kCeilingMaxVoltageV);
    }
    if (m.max_velocity_rps > kCeilingMaxVelocityRps) {
      return fail(m.name, "velocity", m.max_velocity_rps,
                  kCeilingMaxVelocityRps);
    }
  }
  if (!reason.empty()) reason[0] = '\0';
  return true;
}

inline CeilingCheckResult CheckCeilings(const Config& config) {
  for (const auto& m : config.motors) {
    if (m.supply_limit_a > kCeilingMaxSupplyCurrentA) {
      return {false, "motor '" + m.name + "' supply limit " +
                         std::to_string(m.supply_limit_a) + " > ceiling " +
                         std::to_string(kCeilingMaxSupplyCurrentA)};
    }
    if (m.stator_limit_a > kCeilingMaxStatorCurrentA) {
      return {false, "motor '" + m.name + "' stator limit " +
                         std::to_string(m.stator_limit_a) + " > ceiling " +
                         std::to_string(kCeilingMaxStatorCurrentA)};
    }
    if (m.max_voltage > kCeilingMaxVoltageV) {
      return {false, "motor '" + m.name + "' voltage " +
                         std::to_string(m.max_voltage) + " > ceiling " +
                         std::to_string(kCeilingMaxVoltageV)};
    }
    if (m.max_velocity_rps > kCeilingMaxVelocityRps) {
      return {false, "motor '" + m.name + "' velocity " +
                         std::to_string(m.max_velocity_rps) + " > ceiling " +
                         std::to_string(kCeilingMaxVelocityRps)};
    }
  }
  return {true, ""};
}

inline uint32_t ComputeCrc32(std::span<const uint8_t> bytes) {
  static const auto table = [] {
    std::array<uint32_t, 256> t{};
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) {
        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      }
      t[i] = c;
    }
    return t;
  }();

  uint32_t crc = 0xFFFFFFFFu;
  for (uint8_t b : bytes) {
    crc = table[(crc ^ b) & 0xFFu] ^ (crc >> 8);
  }
  return ~crc;
}

namespace detail {
struct ByteWriter {
  std::vector<uint8_t>& buf;
  void Int(uint64_t value, int bytes) {
    for (int i = 0; i < bytes; ++i) {
      buf.push_back(static_cast<uint8_t>(value & 0xFF));
      value >>= 8;
    }
  }
  void Real(double x) { Int(std::bit_cast<uint64_t>(x), 8); }
  void Text(std::string_view s) {
    Int(s.size(), 2);
    for (char c : s) buf.push_back(static_cast<uint8_t>(c));
  }
};

struct SpanByteWriter : driver_station::LittleEndianWriter {
  void Real(double x) noexcept { Int(std::bit_cast<uint64_t>(x), 8); }
  void Text(std::string_view s) noexcept {
    Int(s.size(), 2);
    if (!ok || s.size() > data.size() - pos) {
      ok = false;
      return;
    }
    if (!s.empty()) std::memcpy(data.data() + pos, s.data(), s.size());
    pos += s.size();
  }
};

struct ByteReader : driver_station::LittleEndianReader {
  // Signed 32-bit fields (CAN ids, DIO/PWM channels) travel as their
  // two's-complement bits; the conversion back is modular (C++20).
  int32_t Int32() noexcept {
    return static_cast<int32_t>(static_cast<uint32_t>(Int(4)));
  }
  double Real() noexcept {
    const double x = std::bit_cast<double>(Int(8));
    ok &= std::isfinite(x);
    return x;
  }
  ConfigString Text() noexcept {
    const auto len = Int(2);
    if (!ok || len > kMaxConfigNameLength || len > data.size() - pos) {
      ok = false;
      return {};
    }
    ConfigString s(std::string_view(
        reinterpret_cast<const char*>(data.data() + pos), len));
    pos += len;
    return s;
  }
};

template <typename Writer>
void WriteConfig(Writer& w, const Config& c) {
  w.Int(c.period_us, 4);
  w.Int(c.command_timeout_us, 4);
  w.Real(c.status_hz);
  w.Int(c.commissioned ? 1 : 0, 1);

  w.Int(c.motors.size(), 2);
  for (const auto& m : c.motors) {
    w.Int(m.id, 2);
    w.Text(m.name);
    w.Int(m.can_id, 4);
    w.Text(m.bus);
    w.Int(m.inverted ? 1 : 0, 1);
    w.Int(m.brake ? 1 : 0, 1);
    w.Int(m.master_id, 2);
    w.Int(m.oppose_master ? 1 : 0, 1);
    w.Real(m.supply_limit_a);
    w.Real(m.stator_limit_a);
    w.Real(m.max_voltage);
    w.Int(static_cast<uint8_t>(m.feedback), 1);
    w.Int(m.feedback_sensor_id, 2);
    w.Real(m.rotor_to_sensor_ratio);
    w.Real(m.sensor_to_mechanism_ratio);
    w.Int(m.continuous_wrap ? 1 : 0, 1);
    w.Int(m.soft_limits ? 1 : 0, 1);
    w.Real(m.reverse_limit_rot);
    w.Real(m.forward_limit_rot);
    w.Int(static_cast<uint8_t>(m.forward_limit_switch), 1);
    w.Int(static_cast<uint8_t>(m.reverse_limit_switch), 1);
    w.Real(m.max_velocity_rps);
    w.Real(m.cruise_velocity_rps);
    w.Real(m.acceleration_rps2);
    w.Real(m.jerk_rps3);
    for (const auto& g : m.slots) {
      for (double val : {g.p, g.i, g.d, g.s, g.v, g.a, g.g}) w.Real(val);
      w.Int(static_cast<uint8_t>(g.gravity_type), 1);
    }
    w.Int(m.foc ? 1 : 0, 1);
    w.Real(m.max_torque_current_a);
  }

  w.Int(c.sensors.size(), 2);
  for (const auto& s : c.sensors) {
    w.Int(s.id, 2);
    w.Text(s.name);
    w.Int(static_cast<uint8_t>(s.kind), 1);
    w.Int(s.can_id, 4);
    w.Text(s.bus);
    w.Int(s.inverted ? 1 : 0, 1);
    w.Real(s.offset_rot);
  }

  w.Int(c.digital_inputs.size(), 2);
  for (const auto& di : c.digital_inputs) {
    w.Int(di.id, 2);
    w.Text(di.name);
    w.Int(di.dio, 4);
  }

  w.Int(c.digital_outputs.size(), 2);
  for (const auto& do_dev : c.digital_outputs) {
    w.Int(do_dev.id, 2);
    w.Text(do_dev.name);
    w.Int(do_dev.dio, 4);
  }

  w.Int(c.analog_inputs.size(), 2);
  for (const auto& ai : c.analog_inputs) {
    w.Int(ai.id, 2);
    w.Text(ai.name);
    w.Int(ai.channel, 4);
  }

  w.Int(c.encoders.size(), 2);
  for (const auto& enc : c.encoders) {
    w.Int(enc.id, 2);
    w.Text(enc.name);
    w.Int(static_cast<uint8_t>(enc.kind), 1);
    w.Int(enc.dio_a, 4);
    w.Int(enc.dio_b, 4);
    w.Real(enc.counts_per_rev);
    w.Real(enc.offset_rot);
  }

  w.Int(c.pwm_outputs.size(), 2);
  for (const auto& pwm : c.pwm_outputs) {
    w.Int(pwm.id, 2);
    w.Text(pwm.name);
    w.Int(pwm.channel, 4);
  }
}
}  // namespace detail

inline std::size_t SerializeConfigInto(const Config& c,
                                       std::span<uint8_t> out) noexcept {
  detail::SpanByteWriter w{{out}};
  detail::WriteConfig(w, c);
  return w.ok ? w.pos : 0;
}

inline std::vector<uint8_t> SerializeConfig(const Config& c) {
  std::vector<uint8_t> buf;
  detail::ByteWriter w{buf};
  detail::WriteConfig(w, c);
  return buf;
}

inline bool DeserializeConfig(std::span<const uint8_t> bytes,
                              Config& out) noexcept {
  Config c{};
  detail::ByteReader r{{bytes}};
  c.period_us = r.Int(4);
  c.command_timeout_us = r.Int(4);
  c.status_hz = r.Real();
  c.commissioned = (r.Int(1) == 1);

  const auto motor_count = r.Int(2);
  if (motor_count > kMaxMotors) return false;
  c.motors.resize(motor_count);
  for (std::size_t i = 0; i < motor_count; ++i) {
    auto& m = c.motors[i];
    m.id = r.Int(2);
    m.name = r.Text();
    m.can_id = r.Int32();
    m.bus = r.Text();
    m.inverted = (r.Int(1) == 1);
    m.brake = (r.Int(1) == 1);
    m.master_id = r.Int(2);
    const auto oppose_master = r.Int(1);
    if (oppose_master > 1) return false;
    m.oppose_master = oppose_master != 0;
    m.supply_limit_a = r.Real();
    m.stator_limit_a = r.Real();
    m.max_voltage = r.Real();
    const auto feedback = r.Int(1);
    if (feedback != static_cast<uint8_t>(Feedback::kRotor) &&
        feedback != static_cast<uint8_t>(Feedback::kRemoteCANcoder)) {
      return false;
    }
    m.feedback = static_cast<Feedback>(feedback);
    m.feedback_sensor_id = r.Int(2);
    m.rotor_to_sensor_ratio = r.Real();
    m.sensor_to_mechanism_ratio = r.Real();
    m.continuous_wrap = (r.Int(1) == 1);
    m.soft_limits = (r.Int(1) == 1);
    m.reverse_limit_rot = r.Real();
    m.forward_limit_rot = r.Real();
    const auto forward_switch = r.Int(1);
    const auto reverse_switch = r.Int(1);
    if (forward_switch > static_cast<uint8_t>(LimitSwitch::kDisabled) ||
        reverse_switch > static_cast<uint8_t>(LimitSwitch::kDisabled))
      return false;
    m.forward_limit_switch = static_cast<LimitSwitch>(forward_switch);
    m.reverse_limit_switch = static_cast<LimitSwitch>(reverse_switch);
    m.max_velocity_rps = r.Real();
    m.cruise_velocity_rps = r.Real();
    m.acceleration_rps2 = r.Real();
    m.jerk_rps3 = r.Real();
    for (auto& g : m.slots) {
      g.p = r.Real();
      g.i = r.Real();
      g.d = r.Real();
      g.s = r.Real();
      g.v = r.Real();
      g.a = r.Real();
      g.g = r.Real();
      const auto gravity_type = r.Int(1);
      if (gravity_type > static_cast<uint8_t>(GravityType::kArmCosine))
        return false;
      g.gravity_type = static_cast<GravityType>(gravity_type);
    }
    const auto foc = r.Int(1);
    if (foc > 1) return false;
    m.foc = foc != 0;
    m.max_torque_current_a = r.Real();
  }

  const auto sensor_count = r.Int(2);
  if (sensor_count > kMaxSensors) return false;
  c.sensors.resize(sensor_count);
  for (std::size_t i = 0; i < sensor_count; ++i) {
    auto& s = c.sensors[i];
    s.id = r.Int(2);
    s.name = r.Text();
    const auto kind = r.Int(1);
    if (kind != static_cast<uint8_t>(SensorKind::kCANcoder) &&
        kind != static_cast<uint8_t>(SensorKind::kPigeon2)) {
      return false;
    }
    s.kind = static_cast<SensorKind>(kind);
    s.can_id = r.Int32();
    s.bus = r.Text();
    s.inverted = (r.Int(1) == 1);
    s.offset_rot = r.Real();
  }

  const auto di_count = r.Int(2);
  if (di_count > kMaxDigitalInputs) return false;
  c.digital_inputs.resize(di_count);
  for (std::size_t i = 0; i < di_count; ++i) {
    c.digital_inputs[i].id = r.Int(2);
    c.digital_inputs[i].name = r.Text();
    c.digital_inputs[i].dio = r.Int32();
  }

  const auto do_count = r.Int(2);
  if (do_count > kMaxDigitalOutputs) return false;
  c.digital_outputs.resize(do_count);
  for (std::size_t i = 0; i < do_count; ++i) {
    c.digital_outputs[i].id = r.Int(2);
    c.digital_outputs[i].name = r.Text();
    c.digital_outputs[i].dio = r.Int32();
  }

  const auto ai_count = r.Int(2);
  if (ai_count > kMaxAnalogInputs) return false;
  c.analog_inputs.resize(ai_count);
  for (std::size_t i = 0; i < ai_count; ++i) {
    c.analog_inputs[i].id = r.Int(2);
    c.analog_inputs[i].name = r.Text();
    c.analog_inputs[i].channel = r.Int32();
  }

  const auto enc_count = r.Int(2);
  if (enc_count > kMaxEncoders) return false;
  c.encoders.resize(enc_count);
  for (std::size_t i = 0; i < enc_count; ++i) {
    c.encoders[i].id = r.Int(2);
    c.encoders[i].name = r.Text();
    const auto kind = r.Int(1);
    if (kind != static_cast<uint8_t>(EncoderKind::kQuadrature) &&
        kind != static_cast<uint8_t>(EncoderKind::kDutyCycle)) {
      return false;
    }
    c.encoders[i].kind = static_cast<EncoderKind>(kind);
    c.encoders[i].dio_a = r.Int32();
    c.encoders[i].dio_b = r.Int32();
    c.encoders[i].counts_per_rev = r.Real();
    c.encoders[i].offset_rot = r.Real();
  }

  const auto pwm_count = r.Int(2);
  if (pwm_count > kMaxPwmOutputs) return false;
  c.pwm_outputs.resize(pwm_count);
  for (std::size_t i = 0; i < pwm_count; ++i) {
    c.pwm_outputs[i].id = r.Int(2);
    c.pwm_outputs[i].name = r.Text();
    c.pwm_outputs[i].channel = r.Int32();
  }

  if (!r.Done()) return false;
  out = c;
  return true;
}

#pragma pack(push, 1)
struct ConfigChunkHeader {
  uint64_t config_id{};
  uint32_t total_crc32{};
  uint32_t total_bytes{};
  uint16_t chunk_index{};
  uint16_t chunk_count{};
  uint32_t chunk_bytes{};
};

enum class ConfigAckStatus : uint8_t {
  kOk = 0,
  kInvalidCrc = 1,
  kMalformed = 2,
  kCeilingExceeded = 3,
  kBackendFailed = 4,
};

struct ConfigAckPayload {
  uint64_t config_id{};
  ConfigAckStatus status{ConfigAckStatus::kOk};
  char reason[64]{};
};
#pragma pack(pop)

static_assert(sizeof(ConfigChunkHeader) == 24, "Update docs/DATA_FLOW.md");
static_assert(sizeof(ConfigAckPayload) == 73, "Update docs/DATA_FLOW.md");

// The packed structs fix the sizes; their bytes travel field by field,
// little-endian, like every other talOS wire payload, never as the host's
// object representation.
namespace detail {
inline void PutLittleEndian(uint8_t* out, uint64_t value, int bytes) {
  for (int i = 0; i < bytes; ++i) {
    out[i] = static_cast<uint8_t>(value & 0xFF);
    value >>= 8;
  }
}
inline uint64_t GetLittleEndian(const uint8_t* in, int bytes) {
  uint64_t value{};
  for (int i = 0; i < bytes; ++i) value |= uint64_t{in[i]} << (i * 8);
  return value;
}
}  // namespace detail

inline void EncodeConfigChunkHeader(const ConfigChunkHeader& header,
                                    uint8_t* out) {
  detail::PutLittleEndian(out + 0, header.config_id, 8);
  detail::PutLittleEndian(out + 8, header.total_crc32, 4);
  detail::PutLittleEndian(out + 12, header.total_bytes, 4);
  detail::PutLittleEndian(out + 16, header.chunk_index, 2);
  detail::PutLittleEndian(out + 18, header.chunk_count, 2);
  detail::PutLittleEndian(out + 20, header.chunk_bytes, 4);
}

inline ConfigChunkHeader DecodeConfigChunkHeader(const uint8_t* in) {
  ConfigChunkHeader header;
  header.config_id = detail::GetLittleEndian(in + 0, 8);
  header.total_crc32 =
      static_cast<uint32_t>(detail::GetLittleEndian(in + 8, 4));
  header.total_bytes =
      static_cast<uint32_t>(detail::GetLittleEndian(in + 12, 4));
  header.chunk_index =
      static_cast<uint16_t>(detail::GetLittleEndian(in + 16, 2));
  header.chunk_count =
      static_cast<uint16_t>(detail::GetLittleEndian(in + 18, 2));
  header.chunk_bytes =
      static_cast<uint32_t>(detail::GetLittleEndian(in + 20, 4));
  return header;
}

// Validate() limits device names and CAN bus names to 63 bytes. Text fields
// also carry a two-byte length; the remaining sizes mirror SerializeConfig.
inline constexpr std::size_t kMaxConfigTextBytes = 2 + 63;
inline constexpr std::size_t kMaxSerializedConfigSize =
    17 + 7 * 2 + kMaxMotors * (286 + 2 * kMaxConfigTextBytes) +
    kMaxSensors * (16 + 2 * kMaxConfigTextBytes) +
    (kMaxDigitalInputs + kMaxDigitalOutputs + kMaxAnalogInputs +
     kMaxPwmOutputs) *
        (6 + kMaxConfigTextBytes) +
    kMaxEncoders * (27 + kMaxConfigTextBytes);
static_assert(kMaxSerializedConfigSize == 10227,
              "Serialized config maximum changed; update docs/DATA_FLOW.md");
inline constexpr std::size_t kMaxConfigChunkData =
    protocol::kMaxPayloadSize - sizeof(ConfigChunkHeader);
// A custom payload budget may leave only one data byte per chunk.
inline constexpr std::size_t kMaxConfigChunkCount = kMaxSerializedConfigSize;
static_assert(kMaxConfigChunkCount <= std::numeric_limits<uint16_t>::max());

inline std::vector<std::vector<uint8_t>> CreateConfigChunks(
    const Config& config, std::size_t max_payload = protocol::kMaxPayloadSize) {
  const auto serialized = SerializeConfig(config);
  const uint64_t config_id = ConfigurationId(config);
  const uint32_t total_crc32 = ComputeCrc32(serialized);
  const uint32_t total_bytes = static_cast<uint32_t>(serialized.size());

  const std::size_t max_chunk_data = max_payload - sizeof(ConfigChunkHeader);
  const uint16_t chunk_count = static_cast<uint16_t>(
      (total_bytes + max_chunk_data - 1) / max_chunk_data);

  std::vector<std::vector<uint8_t>> chunks;
  chunks.reserve(chunk_count);

  for (uint16_t idx = 0; idx < chunk_count; ++idx) {
    const std::size_t offset = idx * max_chunk_data;
    const std::size_t chunk_size =
        std::min(max_chunk_data, serialized.size() - offset);

    std::vector<uint8_t> chunk(sizeof(ConfigChunkHeader) + chunk_size);
    ConfigChunkHeader hdr;
    hdr.config_id = config_id;
    hdr.total_crc32 = total_crc32;
    hdr.total_bytes = total_bytes;
    hdr.chunk_index = idx;
    hdr.chunk_count = chunk_count;
    hdr.chunk_bytes = static_cast<uint32_t>(chunk_size);

    EncodeConfigChunkHeader(hdr, chunk.data());
    std::memcpy(chunk.data() + sizeof(hdr), serialized.data() + offset,
                chunk_size);
    chunks.push_back(std::move(chunk));
  }
  return chunks;
}

class ConfigAssembler {
 public:
  enum class Status {
    kIncomplete,
    kComplete,
    kInvalidChunk,
    kCrcMismatch,
    kMalformedConfig,
    kCeilingExceeded,
  };

  Status AddChunk(std::span<const uint8_t> payload,
                  std::span<char> error_reason) noexcept {
    const auto set_reason = [error_reason](const char* msg) noexcept {
      if (!error_reason.empty()) {
        std::snprintf(error_reason.data(), error_reason.size(), "%s", msg);
      }
    };
    if (!error_reason.empty()) error_reason[0] = '\0';
    if (payload.size() < sizeof(ConfigChunkHeader)) {
      last_config_id_ = 0;
      set_reason("chunk payload smaller than header");
      return Status::kInvalidChunk;
    }

    const ConfigChunkHeader hdr = DecodeConfigChunkHeader(payload.data());
    last_config_id_ = hdr.config_id;

    if (hdr.chunk_bytes == 0 || hdr.chunk_bytes > kMaxConfigChunkData ||
        payload.size() != sizeof(ConfigChunkHeader) + hdr.chunk_bytes) {
      set_reason("chunk byte count mismatch");
      return Status::kInvalidChunk;
    }

    if (hdr.total_bytes == 0 || hdr.total_bytes > kMaxSerializedConfigSize ||
        hdr.chunk_count == 0 || hdr.chunk_count > kMaxConfigChunkCount ||
        hdr.chunk_count > hdr.total_bytes ||
        hdr.chunk_index >= hdr.chunk_count) {
      set_reason("invalid chunk index or count");
      return Status::kInvalidChunk;
    }

    // Every non-final chunk has the same stride. Infer it from a final
    // chunk too, so out-of-order delivery gets the same bounds checks.
    const bool final_chunk = hdr.chunk_index == hdr.chunk_count - 1;
    std::size_t stride = hdr.chunk_bytes;
    if (final_chunk && hdr.chunk_count > 1) {
      if (hdr.chunk_bytes >= hdr.total_bytes ||
          (hdr.total_bytes - hdr.chunk_bytes) % (hdr.chunk_count - 1) != 0) {
        set_reason("invalid final chunk size");
        return Status::kInvalidChunk;
      }
      stride = (hdr.total_bytes - hdr.chunk_bytes) / (hdr.chunk_count - 1);
    }
    const std::size_t offset = std::size_t{hdr.chunk_index} * stride;
    if (stride == 0 || stride > kMaxConfigChunkData ||
        hdr.chunk_count != (hdr.total_bytes + stride - 1) / stride ||
        offset >= hdr.total_bytes ||
        hdr.chunk_bytes !=
            std::min(stride, std::size_t{hdr.total_bytes} - offset)) {
      set_reason("chunk data exceeds transfer bounds");
      return Status::kInvalidChunk;
    }

    // New session / config ID
    if (chunk_count_ == 0 || active_config_id_ != hdr.config_id) {
      active_config_id_ = hdr.config_id;
      total_crc32_ = hdr.total_crc32;
      total_bytes_ = hdr.total_bytes;
      chunk_count_ = hdr.chunk_count;
      chunk_stride_ = stride;
      received_chunks_.reset();
      std::memset(assembled_buffer_.data(), 0, total_bytes_);
      received_count_ = 0;
    } else {
      if (hdr.total_bytes != total_bytes_ || hdr.total_crc32 != total_crc32_ ||
          hdr.chunk_count != chunk_count_ || stride != chunk_stride_) {
        set_reason("inconsistent chunk metadata");
        ResetAssembly();
        return Status::kInvalidChunk;
      }
    }

    if (!received_chunks_.test(hdr.chunk_index)) {
      std::memcpy(assembled_buffer_.data() + offset,
                  payload.data() + sizeof(ConfigChunkHeader), hdr.chunk_bytes);
      received_chunks_.set(hdr.chunk_index);
      ++received_count_;
    }

    if (received_count_ < chunk_count_) {
      return Status::kIncomplete;
    }

    // All chunks arrived! Verify CRC32.
    const std::span<const uint8_t> assembled{assembled_buffer_.data(),
                                             total_bytes_};
    const uint32_t actual_crc = ComputeCrc32(assembled);
    if (actual_crc != total_crc32_) {
      if (!error_reason.empty()) {
        std::snprintf(error_reason.data(), error_reason.size(),
                      "assembled CRC mismatch (expected %u, got %u)",
                      total_crc32_, actual_crc);
      }
      ResetAssembly();
      return Status::kCrcMismatch;
    }

    Config parsed;
    if (!DeserializeConfig(assembled, parsed)) {
      set_reason("failed to deserialize config bytes");
      ResetAssembly();
      return Status::kMalformedConfig;
    }

    const uint64_t actual_id = TryConfigurationId(parsed);
    if (actual_id == 0 || actual_id != active_config_id_) {
      set_reason(actual_id == 0 ? "config failed validation"
                                : "config id mismatch");
      ResetAssembly();
      return Status::kMalformedConfig;
    }

    // Check ceilings!
    if (!CheckCeilingsInto(parsed, error_reason)) {
      ResetAssembly();
      return Status::kCeilingExceeded;
    }

    config_ = parsed;
    ResetAssembly();
    return Status::kComplete;
  }

  Status AddChunk(std::span<const uint8_t> payload,
                  std::string& error_reason) noexcept try {
    std::array<char, 160> buf{};
    const Status status = AddChunk(payload, std::span<char>{buf});
    if (buf[0] != '\0') error_reason = buf.data();
    return status;
  } catch (...) {
    // Allocation failures in error reporting must not escape the gateway's
    // safety loop.
    Reset();
    error_reason.clear();
    return Status::kInvalidChunk;
  }

  const Config& config() const { return config_; }
  // The header config_id of the most recently processed chunk (0 before any
  // chunk, after Reset(), or after a chunk shorter than ConfigChunkHeader).
  // Preserved across completion and refusal resets so Endpoint::Tick stamps
  // the rejected transfer's config_id onto refusal ACKs (REQ-CONTRACT-04).
  uint64_t config_id() const { return last_config_id_; }

  void Reset() noexcept {
    ResetAssembly();
    last_config_id_ = 0;
  }

 private:
  void ResetAssembly() noexcept {
    active_config_id_ = 0;
    total_crc32_ = 0;
    total_bytes_ = 0;
    chunk_count_ = 0;
    received_count_ = 0;
    chunk_stride_ = 0;
    received_chunks_.reset();
  }

  uint64_t active_config_id_{0};
  uint64_t last_config_id_{0};
  uint32_t total_crc32_{0};
  uint32_t total_bytes_{0};
  uint16_t chunk_count_{0};
  uint16_t received_count_{0};
  std::size_t chunk_stride_{0};
  std::bitset<kMaxConfigChunkCount> received_chunks_{};
  std::array<uint8_t, kMaxSerializedConfigSize> assembled_buffer_{};
  Config config_{};
};

// The RoboRIO's saved configuration file (ConfigStore in the RoboRIO
// program), little-endian:
//
//   offset 0      8 B  magic "TALOSCFG" (ASCII, in this order)
//   offset 8      2 B  u16 protocol::kProtocolVersion of the program that
//                      wrote it
//   offset 10     n B  SerializeConfig bytes
//   offset 10+n   4 B  CRC32 (ComputeCrc32) of bytes [0, 10+n)
//
// The tag is the protocol version because every config codec change bumps it
// (wire_golden_test pins the config bytes to it), so there is no second
// number to remember. A file of any other version is refused by the tag
// however its body reads, and so is a file written before the tag (the
// config bytes and their CRC32, through protocol 11): it begins with
// period_us, which validation has always held at or below 20000, so its
// bytes 2 and 3 are zero where the magic's are not. A refused file is not
// migrated: the RoboRIO boots unconfigured and waits for the bridge's push.
inline constexpr std::array<uint8_t, 8> kPersistedConfigMagic{
    'T', 'A', 'L', 'O', 'S', 'C', 'F', 'G'};
inline constexpr std::size_t kPersistedConfigHeaderBytes =
    kPersistedConfigMagic.size() + 2;
inline constexpr std::size_t kMaxPersistedConfigSize =
    kPersistedConfigHeaderBytes + kMaxSerializedConfigSize + 4;

inline std::size_t PersistedConfigInto(const Config& config,
                                       std::span<uint8_t> out) noexcept {
  if (out.size() < kPersistedConfigHeaderBytes + 4) return 0;
  std::memcpy(out.data(), kPersistedConfigMagic.data(),
              kPersistedConfigMagic.size());
  detail::PutLittleEndian(out.data() + kPersistedConfigMagic.size(),
                          protocol::kProtocolVersion, 2);
  const std::size_t body_budget = out.size() - kPersistedConfigHeaderBytes - 4;
  const std::size_t body_size = SerializeConfigInto(
      config, out.subspan(kPersistedConfigHeaderBytes, body_budget));
  if (body_size == 0) return 0;
  const std::size_t covered = kPersistedConfigHeaderBytes + body_size;
  detail::PutLittleEndian(out.data() + covered,
                          ComputeCrc32(out.first(covered)), 4);
  return covered + 4;
}

inline std::vector<uint8_t> PersistedConfig(const Config& config) {
  const auto body = SerializeConfig(config);
  std::vector<uint8_t> bytes;
  bytes.reserve(kPersistedConfigHeaderBytes + body.size() + 4);
  bytes.assign(kPersistedConfigMagic.begin(), kPersistedConfigMagic.end());
  detail::ByteWriter writer{bytes};
  writer.Int(protocol::kProtocolVersion, 2);
  bytes.insert(bytes.end(), body.begin(), body.end());
  writer.Int(ComputeCrc32(bytes), 4);
  return bytes;
}

// True only for a file PersistedConfig of this protocol version wrote, whose
// CRC holds and whose configuration decodes, keeps the ceilings and
// validates; `config` is left unchanged otherwise.
inline bool LoadPersistedConfig(std::span<const uint8_t> bytes,
                                Config& config) {
  if (bytes.size() < kPersistedConfigHeaderBytes + 4) return false;
  if (!std::equal(kPersistedConfigMagic.begin(), kPersistedConfigMagic.end(),
                  bytes.begin()))
    return false;
  detail::ByteReader version{{bytes.subspan(kPersistedConfigMagic.size(), 2)}};
  if (version.Int(2) != protocol::kProtocolVersion) return false;
  const auto covered = bytes.first(bytes.size() - 4);
  detail::ByteReader trailer{{bytes.last(4)}};
  if (ComputeCrc32(covered) != trailer.Int(4)) return false;
  Config parsed;
  if (!DeserializeConfig(covered.subspan(kPersistedConfigHeaderBytes),
                         parsed) ||
      !CheckCeilingsInto(parsed, {}) || !IsValid(parsed)) {
    return false;
  }
  config = std::move(parsed);
  return true;
}

inline std::size_t EncodeConfigAck(const ConfigAckPayload& ack,
                                   std::span<uint8_t> out) {
  if (out.size() < sizeof(ConfigAckPayload)) return 0;
  detail::PutLittleEndian(out.data(), ack.config_id, 8);
  out[8] = static_cast<uint8_t>(ack.status);
  std::memcpy(out.data() + 9, ack.reason, sizeof(ack.reason));
  return sizeof(ConfigAckPayload);
}

inline bool DecodeConfigAck(std::span<const uint8_t> in,
                            ConfigAckPayload& out) {
  if (in.size() != sizeof(ConfigAckPayload)) return false;
  out.config_id = detail::GetLittleEndian(in.data(), 8);
  out.status = static_cast<ConfigAckStatus>(in[8]);
  std::memcpy(out.reason, in.data() + 9, sizeof(out.reason));
  return true;
}

}  // namespace talos::hardware
