#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

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

struct ByteReader {
  std::span<const uint8_t> data;
  std::size_t pos{};
  bool ok{true};
  uint64_t Int(int bytes) {
    uint64_t val{};
    for (int i = 0; i < bytes; ++i) {
      if (pos >= data.size()) {
        ok = false;
        return 0;
      }
      val |= uint64_t{data[pos++]} << (i * 8);
    }
    return val;
  }
  double Real() {
    const double x = std::bit_cast<double>(Int(8));
    ok &= std::isfinite(x);
    return x;
  }
  std::string Text() {
    const auto len = Int(2);
    if (!ok || pos + len > data.size()) {
      ok = false;
      return {};
    }
    std::string s(reinterpret_cast<const char*>(data.data() + pos), len);
    pos += len;
    return s;
  }
  bool Done() const { return ok && pos == data.size(); }
};
}  // namespace detail

inline std::vector<uint8_t> SerializeConfig(const Config& c) {
  std::vector<uint8_t> buf;
  detail::ByteWriter w{buf};
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
    w.Real(m.max_velocity_rps);
    w.Real(m.cruise_velocity_rps);
    w.Real(m.acceleration_rps2);
    w.Real(m.jerk_rps3);
    for (const auto& g : m.slots) {
      for (double val : {g.p, g.i, g.d, g.s, g.v, g.a, g.g}) w.Real(val);
    }
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

  return buf;
}

inline bool DeserializeConfig(std::span<const uint8_t> bytes, Config& out) {
  Config c{};
  detail::ByteReader r{bytes};
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
    m.can_id = r.Int(4);
    m.bus = r.Text();
    m.inverted = (r.Int(1) == 1);
    m.brake = (r.Int(1) == 1);
    m.supply_limit_a = r.Real();
    m.stator_limit_a = r.Real();
    m.max_voltage = r.Real();
    m.feedback = static_cast<Feedback>(r.Int(1));
    m.feedback_sensor_id = r.Int(2);
    m.rotor_to_sensor_ratio = r.Real();
    m.sensor_to_mechanism_ratio = r.Real();
    m.continuous_wrap = (r.Int(1) == 1);
    m.soft_limits = (r.Int(1) == 1);
    m.reverse_limit_rot = r.Real();
    m.forward_limit_rot = r.Real();
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
    }
  }

  const auto sensor_count = r.Int(2);
  if (sensor_count > kMaxSensors) return false;
  c.sensors.resize(sensor_count);
  for (std::size_t i = 0; i < sensor_count; ++i) {
    auto& s = c.sensors[i];
    s.id = r.Int(2);
    s.name = r.Text();
    s.kind = static_cast<SensorKind>(r.Int(1));
    s.can_id = r.Int(4);
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
    c.digital_inputs[i].dio = r.Int(4);
  }

  const auto do_count = r.Int(2);
  if (do_count > kMaxDigitalOutputs) return false;
  c.digital_outputs.resize(do_count);
  for (std::size_t i = 0; i < do_count; ++i) {
    c.digital_outputs[i].id = r.Int(2);
    c.digital_outputs[i].name = r.Text();
    c.digital_outputs[i].dio = r.Int(4);
  }

  const auto ai_count = r.Int(2);
  if (ai_count > kMaxAnalogInputs) return false;
  c.analog_inputs.resize(ai_count);
  for (std::size_t i = 0; i < ai_count; ++i) {
    c.analog_inputs[i].id = r.Int(2);
    c.analog_inputs[i].name = r.Text();
    c.analog_inputs[i].channel = r.Int(4);
  }

  const auto enc_count = r.Int(2);
  if (enc_count > kMaxEncoders) return false;
  c.encoders.resize(enc_count);
  for (std::size_t i = 0; i < enc_count; ++i) {
    c.encoders[i].id = r.Int(2);
    c.encoders[i].name = r.Text();
    c.encoders[i].kind = static_cast<EncoderKind>(r.Int(1));
    c.encoders[i].dio_a = r.Int(4);
    c.encoders[i].dio_b = r.Int(4);
    c.encoders[i].counts_per_rev = r.Real();
    c.encoders[i].offset_rot = r.Real();
  }

  const auto pwm_count = r.Int(2);
  if (pwm_count > kMaxPwmOutputs) return false;
  c.pwm_outputs.resize(pwm_count);
  for (std::size_t i = 0; i < pwm_count; ++i) {
    c.pwm_outputs[i].id = r.Int(2);
    c.pwm_outputs[i].name = r.Text();
    c.pwm_outputs[i].channel = r.Int(4);
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

static_assert(sizeof(ConfigChunkHeader) == 24);
static_assert(sizeof(ConfigAckPayload) == 73);

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

    std::memcpy(chunk.data(), &hdr, sizeof(hdr));
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

  Status AddChunk(std::span<const uint8_t> payload, std::string& error_reason) {
    if (payload.size() < sizeof(ConfigChunkHeader)) {
      error_reason = "chunk payload smaller than header";
      return Status::kInvalidChunk;
    }

    ConfigChunkHeader hdr;
    std::memcpy(&hdr, payload.data(), sizeof(hdr));

    if (payload.size() != sizeof(ConfigChunkHeader) + hdr.chunk_bytes) {
      error_reason = "chunk byte count mismatch";
      return Status::kInvalidChunk;
    }

    if (hdr.chunk_count == 0 || hdr.chunk_index >= hdr.chunk_count) {
      error_reason = "invalid chunk index or count";
      return Status::kInvalidChunk;
    }

    // New session / config ID
    if (active_config_id_ != hdr.config_id) {
      active_config_id_ = hdr.config_id;
      total_crc32_ = hdr.total_crc32;
      total_bytes_ = hdr.total_bytes;
      chunk_count_ = hdr.chunk_count;
      received_chunks_.assign(chunk_count_, false);
      assembled_buffer_.assign(total_bytes_, 0);
      received_count_ = 0;
    } else {
      if (hdr.total_bytes != total_bytes_ ||
          hdr.total_crc32 != total_crc32_ ||
          hdr.chunk_count != chunk_count_) {
        error_reason = "inconsistent chunk metadata";
        Reset();
        return Status::kInvalidChunk;
      }
    }

    std::size_t offset = 0;
    if (hdr.chunk_index < chunk_count_ - 1) {
      offset = hdr.chunk_index * hdr.chunk_bytes;
    } else {
      offset = total_bytes_ - hdr.chunk_bytes;
    }

    if (offset + hdr.chunk_bytes > assembled_buffer_.size()) {
      error_reason = "chunk data exceeds buffer bounds";
      Reset();
      return Status::kInvalidChunk;
    }

    if (!received_chunks_[hdr.chunk_index]) {
      std::memcpy(assembled_buffer_.data() + offset,
                  payload.data() + sizeof(ConfigChunkHeader), hdr.chunk_bytes);
      received_chunks_[hdr.chunk_index] = true;
      ++received_count_;
    }

    if (received_count_ < chunk_count_) {
      return Status::kIncomplete;
    }

    // All chunks arrived! Verify CRC32.
    const uint32_t actual_crc = ComputeCrc32(assembled_buffer_);
    if (actual_crc != total_crc32_) {
      error_reason = "assembled CRC mismatch (expected " +
                     std::to_string(total_crc32_) + ", got " +
                     std::to_string(actual_crc) + ")";
      Reset();
      return Status::kCrcMismatch;
    }

    Config parsed;
    if (!DeserializeConfig(assembled_buffer_, parsed)) {
      error_reason = "failed to deserialize config bytes";
      Reset();
      return Status::kMalformedConfig;
    }

    // Check ceilings!
    const auto ceiling_res = CheckCeilings(parsed);
    if (!ceiling_res.ok) {
      error_reason = ceiling_res.reason;
      Reset();
      return Status::kCeilingExceeded;
    }

    config_ = parsed;
    return Status::kComplete;
  }

  const Config& config() const { return config_; }
  uint64_t config_id() const { return active_config_id_; }

  void Reset() {
    active_config_id_ = 0;
    total_crc32_ = 0;
    total_bytes_ = 0;
    chunk_count_ = 0;
    received_count_ = 0;
    received_chunks_.clear();
    assembled_buffer_.clear();
  }

 private:
  uint64_t active_config_id_{0};
  uint32_t total_crc32_{0};
  uint32_t total_bytes_{0};
  uint16_t chunk_count_{0};
  uint16_t received_count_{0};
  std::vector<bool> received_chunks_{};
  std::vector<uint8_t> assembled_buffer_{};
  Config config_{};
};

inline std::size_t EncodeConfigAck(const ConfigAckPayload& ack,
                                   std::span<uint8_t> out) {
  if (out.size() < sizeof(ConfigAckPayload)) return 0;
  std::memcpy(out.data(), &ack, sizeof(ack));
  return sizeof(ack);
}

inline bool DecodeConfigAck(std::span<const uint8_t> in,
                            ConfigAckPayload& out) {
  if (in.size() < sizeof(ConfigAckPayload)) return false;
  std::memcpy(&out, in.data(), sizeof(out));
  return true;
}

}  // namespace talos::hardware
