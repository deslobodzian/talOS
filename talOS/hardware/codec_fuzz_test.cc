#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <vector>

#include "talOS/hardware/clock_offset.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/protocol/fuzz_corpus.h"
#include "talOS/protocol/types.h"

namespace {

using namespace talos::hardware;

void CheckCommandRoundTrip(std::span<const uint8_t> input) {
  Command command;
  if (!Decode(input, command)) return;
  std::vector<uint8_t> encoded(kMaxCommandSize);
  const auto written = Encode(command, encoded);
  ASSERT_GT(written, 0u);
  if (written > 0) {
    std::vector<uint8_t> too_small(written - 1);
    EXPECT_EQ(Encode(command, too_small), 0u);
  }
  Command again;
  ASSERT_TRUE(Decode({encoded.data(), written}, again));
  encoded.resize(written);
  std::vector<uint8_t> roundtrip(kMaxCommandSize);
  const auto rt_written = Encode(again, roundtrip);
  ASSERT_EQ(rt_written, written);
  roundtrip.resize(rt_written);
  EXPECT_EQ(roundtrip, encoded);
}

void CheckStateRoundTrip(std::span<const uint8_t> input) {
  State state;
  if (!Decode(input, state)) return;
  std::vector<uint8_t> encoded(kMaxStateSize);
  const auto written = Encode(state, encoded);
  ASSERT_GT(written, 0u);
  if (written > 0) {
    std::vector<uint8_t> too_small(written - 1);
    EXPECT_EQ(Encode(state, too_small), 0u);
  }
  State again;
  ASSERT_TRUE(Decode({encoded.data(), written}, again));
  encoded.resize(written);
  std::vector<uint8_t> roundtrip(kMaxStateSize);
  const auto rt_written = Encode(again, roundtrip);
  ASSERT_EQ(rt_written, written);
  roundtrip.resize(rt_written);
  EXPECT_EQ(roundtrip, encoded);
}

void CheckConfigRoundTrip(std::span<const uint8_t> input) {
  Config config;
  if (!DeserializeConfig(input, config)) return;
  const auto encoded = SerializeConfig(config);
  Config again;
  ASSERT_TRUE(DeserializeConfig(encoded, again));
  EXPECT_EQ(SerializeConfig(again), encoded);
  if (encoded.size() <= kMaxSerializedConfigSize) {
    std::vector<uint8_t> into(encoded.size());
    EXPECT_EQ(SerializeConfigInto(config, into), encoded.size());
    EXPECT_EQ(into, encoded);
    if (!encoded.empty()) {
      std::vector<uint8_t> too_small(encoded.size() - 1);
      EXPECT_EQ(SerializeConfigInto(config, too_small), 0u);
    }
  }
  (void)IsValid(config);
  (void)TryConfigurationId(config);
  std::array<char, 160> reason{};
  (void)CheckCeilingsInto(config, reason);
}

void CheckPersistedConfigRoundTrip(std::span<const uint8_t> input) {
  Config config;
  if (!LoadPersistedConfig(input, config)) return;
  const auto encoded = PersistedConfig(config);
  Config again;
  ASSERT_TRUE(LoadPersistedConfig(encoded, again));
  EXPECT_EQ(PersistedConfig(again), encoded);
  std::vector<uint8_t> into(encoded.size());
  EXPECT_EQ(PersistedConfigInto(config, into), encoded.size());
  EXPECT_EQ(into, encoded);
}

void CheckConfigAckRoundTrip(std::span<const uint8_t> input) {
  ConfigAckPayload ack;
  if (!DecodeConfigAck(input, ack)) return;
  std::vector<uint8_t> encoded(sizeof(ConfigAckPayload));
  ASSERT_EQ(EncodeConfigAck(ack, encoded), encoded.size());
  EXPECT_EQ(encoded, std::vector<uint8_t>(input.begin(), input.end()));
  ConfigAckPayload again;
  ASSERT_TRUE(DecodeConfigAck(encoded, again));
  std::vector<uint8_t> roundtrip(encoded.size());
  ASSERT_EQ(EncodeConfigAck(again, roundtrip), roundtrip.size());
  EXPECT_EQ(roundtrip, encoded);
}

Config MakePopulatedConfig() {
  Config config;
  config.motors.push_back({1, "drive", 1, "rio"});
  config.motors.back().foc = true;
  config.motors.back().max_torque_current_a = 20.0;
  config.sensors.push_back({2, "encoder", SensorKind::kCANcoder, 2, "rio"});
  config.digital_inputs.push_back({3, "limit", 0});
  config.digital_outputs.push_back({4, "solenoid", 1});
  config.analog_inputs.push_back({5, "pressure", 0});
  config.encoders.push_back({6, "wheel", EncoderKind::kQuadrature, 2, 3});
  config.pwm_outputs.push_back({7, "servo", 0});
  return config;
}

std::vector<uint8_t> MakeSeedCommand(bool full) {
  Command command;
  if (full) {
    command.count = kMaxMotors;
    command.digital_output_count = kMaxDigitalOutputs;
    command.pwm_output_count = kMaxPwmOutputs;
    command.pwm_outputs[1] = {1, PwmMode::kPulse, 0.0};
    command.motors[1] = {2, Mode::kTorqueCurrent, 0, 2.5, 0.0};
  }
  std::vector<uint8_t> encoded(kMaxCommandSize);
  encoded.resize(Encode(command, encoded));
  return encoded;
}

std::vector<uint8_t> MakeSeedState(bool full) {
  State state;
  if (full) {
    state.motor_count = kMaxMotors;
    state.sensor_count = kMaxSensors;
    state.digital_input_count = kMaxDigitalInputs;
    state.digital_output_count = kMaxDigitalOutputs;
    state.analog_input_count = kMaxAnalogInputs;
    state.encoder_count = kMaxEncoders;
    state.pwm_output_count = kMaxPwmOutputs;
    state.health.can_bus_count = kMaxCanBuses;
    state.health.can_bus_index = kMaxCanBuses - 1;
    state.health.can_bus = {CanBusReadState::kStale, 0.5F, 1, 2, 3, 4, 5};
  }
  std::vector<uint8_t> encoded(kMaxStateSize);
  encoded.resize(Encode(state, encoded));
  return encoded;
}

void MutateBytes(std::mt19937_64& rng, std::vector<uint8_t>& buf,
                 std::size_t max_len) {
  static constexpr std::array<uint64_t, 6> kHostileFloats = {
      0x7ff8000000000000ULL,  // quiet NaN
      0x7ff0000000000001ULL,  // signaling NaN
      0x7ff0000000000000ULL,  // +Inf
      0xfff0000000000000ULL,  // -Inf
      0x8000000000000000ULL,  // -0.0
      0x7fefffffffffffffULL,  // DBL_MAX
  };
  const auto ops = 1 + (rng() % 5);
  for (std::size_t op = 0; op < ops; ++op) {
    switch (rng() % 6) {
      case 0:  // Bit flip
        if (!buf.empty()) {
          buf[rng() % buf.size()] ^= uint8_t{1} << (rng() % 8);
        }
        break;
      case 1:  // Byte overwrite with boundary value
        if (!buf.empty()) {
          static constexpr std::array<uint8_t, 5> kEdgeBytes = {
              0x00, 0x01, 0x02, 0x7f, 0xff};
          buf[rng() % buf.size()] = (rng() & 1)
                                        ? kEdgeBytes[rng() % kEdgeBytes.size()]
                                        : static_cast<uint8_t>(rng());
        }
        break;
      case 2:  // Truncate or shrink
        if (!buf.empty()) {
          buf.resize(rng() % buf.size());
        }
        break;
      case 3:  // Append/insert bytes up to max_len
        if (buf.size() < max_len) {
          const std::size_t pos = rng() % (buf.size() + 1);
          buf.insert(buf.begin() + static_cast<std::ptrdiff_t>(pos),
                     static_cast<uint8_t>(rng()));
        }
        break;
      case 4:  // Erase byte
        if (!buf.empty()) {
          const std::size_t pos = rng() % buf.size();
          buf.erase(buf.begin() + static_cast<std::ptrdiff_t>(pos));
        }
        break;
      case 5:  // Inject 8-byte IEEE-754 hostile double pattern
        if (buf.size() >= 8) {
          const std::size_t pos = rng() % (buf.size() - 7);
          const uint64_t pat = kHostileFloats[rng() % kHostileFloats.size()];
          for (std::size_t i = 0; i < 8; ++i) {
            buf[pos + i] = static_cast<uint8_t>((pat >> (i * 8)) & 0xff);
          }
        }
        break;
      default:
        break;
    }
  }
}

std::vector<uint8_t> WrapPersistedWithValidCrc(std::span<const uint8_t> body) {
  std::vector<uint8_t> out;
  out.reserve(kPersistedConfigHeaderBytes + body.size() + 4);
  out.assign(kPersistedConfigMagic.begin(), kPersistedConfigMagic.end());
  out.push_back(static_cast<uint8_t>(talos::protocol::kProtocolVersion & 0xff));
  out.push_back(static_cast<uint8_t>(talos::protocol::kProtocolVersion >> 8));
  out.insert(out.end(), body.begin(), body.end());
  const uint32_t crc = ComputeCrc32(out);
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<uint8_t>((crc >> (i * 8)) & 0xff));
  }
  return out;
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  const std::span<const uint8_t> input(data, size);
  CheckCommandRoundTrip(input);
  CheckStateRoundTrip(input);
  CheckConfigRoundTrip(input);
  CheckPersistedConfigRoundTrip(input);
  CheckConfigAckRoundTrip(input);
  if (size >= sizeof(ConfigChunkHeader)) {
    const auto hdr = DecodeConfigChunkHeader(data);
    std::array<uint8_t, sizeof(ConfigChunkHeader)> hdr_bytes{};
    EncodeConfigChunkHeader(hdr, hdr_bytes.data());
  }
  ConfigAssembler assembler;
  std::string reason;
  const auto check_chunk = [&](std::span<const uint8_t> chunk) {
    if (assembler.AddChunk(chunk, reason) ==
        ConfigAssembler::Status::kComplete) {
      const auto encoded = SerializeConfig(assembler.config());
      Config again;
      EXPECT_TRUE(DeserializeConfig(encoded, again));
      EXPECT_EQ(SerializeConfig(again), encoded);
    }
  };
  check_chunk(input);
  check_chunk(input);
  if (size > 1) check_chunk(input.subspan(1));
  return 0;
}

#ifndef TALOS_LIBFUZZER
TEST(HardwareCodecFuzzTest, BoundedMutations) {
  std::vector<std::vector<uint8_t>> seeds;
  for (int full = 0; full < 2; ++full) {
    auto state_seed = MakeSeedState(full != 0);
    ASSERT_FALSE(state_seed.empty());
    State decoded_state;
    ASSERT_TRUE(Decode(state_seed, decoded_state));
    seeds.push_back(std::move(state_seed));

    auto cmd_seed = MakeSeedCommand(full != 0);
    ASSERT_FALSE(cmd_seed.empty());
    Command decoded_command;
    ASSERT_TRUE(Decode(cmd_seed, decoded_command));
    seeds.push_back(std::move(cmd_seed));
  }
  const Config config = MakePopulatedConfig();
  seeds.push_back(SerializeConfig(config));
  seeds.push_back(PersistedConfig(config));
  ConfigAckPayload ack;
  std::vector<uint8_t> encoded_ack(sizeof(ack));
  ASSERT_EQ(EncodeConfigAck(ack, encoded_ack), encoded_ack.size());
  seeds.push_back(encoded_ack);

  ConfigAssembler assembler;
  std::string reason;
  for (const auto& chunk : CreateConfigChunks(config, 180)) {
    seeds.push_back(chunk);
    const auto status = assembler.AddChunk(chunk, reason);
    ASSERT_TRUE(status == ConfigAssembler::Status::kIncomplete ||
                status == ConfigAssembler::Status::kComplete);
  }
  EXPECT_EQ(SerializeConfig(assembler.config()), SerializeConfig(config));

  ConfigAssembler stateful;
  const auto chunks = CreateConfigChunks(config, 128);
  talos::protocol::testing::RunCorpus(
      chunks, [&](const uint8_t* data, std::size_t size) {
        (void)stateful.AddChunk({data, size}, reason);
      });
  stateful.Reset();
  for (auto it = chunks.rbegin(); it != chunks.rend(); ++it) {
    const auto status = stateful.AddChunk(*it, reason);
    EXPECT_EQ(status, it + 1 == chunks.rend()
                          ? ConfigAssembler::Status::kComplete
                          : ConfigAssembler::Status::kIncomplete);
  }
  EXPECT_EQ(SerializeConfig(stateful.config()), SerializeConfig(config));
  talos::protocol::testing::RunCorpus(seeds, LLVMFuzzerTestOneInput);
}

TEST(HardwareCodecFuzzTest, SeededCodecMutationsAndRoundTrips) {
  std::mt19937_64 rng(0x4857434f444543ULL);

  // 1. Command codec: 4000 seeded mutations (bitflips, truncations, overlong,
  //    hostile NaN/Inf/-0.0, max/overflow counts, invalid enums).
  {
    const std::array<std::vector<uint8_t>, 2> cmd_seeds = {
        MakeSeedCommand(false), MakeSeedCommand(true)};
    for (int i = 0; i < 4000; ++i) {
      auto buf = cmd_seeds[i % cmd_seeds.size()];
      MutateBytes(rng, buf, kMaxCommandSize + 64);
      const std::vector<uint8_t> exact(buf.begin(), buf.end());
      CheckCommandRoundTrip(exact);
    }
  }

  // 2. State codec: 4000 seeded mutations + non-finite struct sanitization.
  {
    const std::array<std::vector<uint8_t>, 2> state_seeds = {
        MakeSeedState(false), MakeSeedState(true)};
    for (int i = 0; i < 4000; ++i) {
      auto buf = state_seeds[i % state_seeds.size()];
      MutateBytes(rng, buf, kMaxStateSize + 64);
      const std::vector<uint8_t> exact(buf.begin(), buf.end());
      CheckStateRoundTrip(exact);
    }
    // Adversarial State struct values (NaN, +/-Inf, DBL_MAX, -0.0).
    for (double val : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::max(),
                       -std::numeric_limits<double>::max(), -0.0}) {
      State s;
      s.motor_count = kMaxMotors;
      s.sensor_count = kMaxSensors;
      s.analog_input_count = kMaxAnalogInputs;
      s.encoder_count = kMaxEncoders;
      s.pwm_output_count = kMaxPwmOutputs;
      for (auto& m : s.motors) {
        m.valid = true;
        m.position_rot = val;
        m.velocity_rps = val;
        m.voltage = val;
        m.stator_current_a = val;
        m.temperature_c = val;
        m.supply_current_a = val;
      }
      s.health.valid = true;
      s.health.battery_v = val;
      s.health.can_bus.read_state = CanBusReadState::kOk;
      s.health.can_bus.utilization_frac = static_cast<float>(val);
      std::array<uint8_t, kMaxStateSize> out{};
      const auto n = Encode(s, out);
      ASSERT_GT(n, 0u);
      State decoded;
      ASSERT_TRUE(Decode(std::span{out}.first(n), decoded));
    }
  }

  // 3. Config & PersistedConfig codecs: 4000 mutations including CRC-valid
  //    persisted payloads so mutations reach DeserializeConfig/Validate.
  {
    const Config cfg = MakePopulatedConfig();
    const auto ser_seed = SerializeConfig(cfg);
    const auto pers_seed = PersistedConfig(cfg);
    for (int i = 0; i < 4000; ++i) {
      auto body = ser_seed;
      MutateBytes(rng, body, kMaxSerializedConfigSize + 64);
      const std::vector<uint8_t> exact_body(body.begin(), body.end());
      CheckConfigRoundTrip(exact_body);

      if (i & 1) {
        const auto wrapped = WrapPersistedWithValidCrc(exact_body);
        CheckPersistedConfigRoundTrip(wrapped);
      } else {
        auto pers = pers_seed;
        MutateBytes(rng, pers, kMaxPersistedConfigSize + 64);
        const std::vector<uint8_t> exact_pers(pers.begin(), pers.end());
        CheckPersistedConfigRoundTrip(exact_pers);
      }
    }
  }

  // 4. ConfigChunk / ConfigAssembler multi-chunk state machine: 4000 mutated
  //    and reordered chunk steps, plus CRC-valid 1-chunk malformed configs.
  {
    const Config cfg = MakePopulatedConfig();
    const auto base_chunks = CreateConfigChunks(cfg, 140);
    ConfigAssembler assembler;
    std::string reason;
    std::array<char, 64> short_reason{};
    for (int i = 0; i < 4000; ++i) {
      if ((i % 250) == 0) assembler.Reset();
      auto chunk = base_chunks[rng() % base_chunks.size()];
      if ((rng() % 3) != 0) {
        MutateBytes(rng, chunk, talos::protocol::kMaxPayloadSize + 32);
      }
      const std::vector<uint8_t> exact(chunk.begin(), chunk.end());
      const auto st = assembler.AddChunk(exact, reason);
      if (st == ConfigAssembler::Status::kComplete) {
        const auto encoded = SerializeConfig(assembler.config());
        Config again;
        ASSERT_TRUE(DeserializeConfig(encoded, again));
        EXPECT_EQ(SerializeConfig(again), encoded);
      }
      (void)assembler.AddChunk(exact, short_reason);
    }
    // Single-chunk valid-CRC wrapper with mutated config body.
    const auto ser_seed = SerializeConfig(cfg);
    for (int i = 0; i < 1000; ++i) {
      auto body = ser_seed;
      MutateBytes(rng, body, kMaxConfigChunkData);
      if (body.empty() || body.size() > kMaxConfigChunkData) continue;
      std::vector<uint8_t> chunk(sizeof(ConfigChunkHeader) + body.size());
      ConfigChunkHeader hdr;
      hdr.config_id = 0x123456789abcdef0ULL;
      hdr.total_crc32 = ComputeCrc32(body);
      hdr.total_bytes = static_cast<uint32_t>(body.size());
      hdr.chunk_index = 0;
      hdr.chunk_count = 1;
      hdr.chunk_bytes = static_cast<uint32_t>(body.size());
      EncodeConfigChunkHeader(hdr, chunk.data());
      std::memcpy(chunk.data() + sizeof(ConfigChunkHeader), body.data(),
                  body.size());
      assembler.Reset();
      (void)assembler.AddChunk(chunk, reason);
    }
  }

  // 5. ConfigAck & Packet::bytes: 4000 mutations.
  {
    ConfigAckPayload ack_seed;
    ack_seed.config_id = 0xdeadbeefcafebabeULL;
    ack_seed.status = ConfigAckStatus::kBackendFailed;
    std::vector<uint8_t> ack_bytes(sizeof(ConfigAckPayload));
    ASSERT_EQ(EncodeConfigAck(ack_seed, ack_bytes), ack_bytes.size());
    for (int i = 0; i < 4000; ++i) {
      auto buf = ack_bytes;
      MutateBytes(rng, buf, sizeof(ConfigAckPayload) + 64);
      const std::vector<uint8_t> exact(buf.begin(), buf.end());
      CheckConfigAckRoundTrip(exact);
    }
    Packet pkt;
    for (uint32_t sz : {uint32_t{0}, uint32_t{1}, uint32_t{1432},
                        uint32_t{1433}, std::numeric_limits<uint32_t>::max()}) {
      pkt.size = sz;
      EXPECT_LE(pkt.bytes().size(), pkt.data.size());
    }
  }

  // 6. ClockOffsetEstimator: 4000 seeded extreme timestamp observations.
  {
    ClockOffsetEstimator est;
    static constexpr std::array<uint64_t, 6> kSpecialTimes = {
        0ULL,
        1ULL,
        static_cast<uint64_t>(std::numeric_limits<int64_t>::max()),
        static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) + 1ULL,
        std::numeric_limits<uint64_t>::max() - 1ULL,
        std::numeric_limits<uint64_t>::max(),
    };
    for (int i = 0; i < 4000; ++i) {
      const uint64_t boot = (rng() % 10) + 1;
      const uint64_t t1 =
          (rng() & 1) ? kSpecialTimes[rng() % kSpecialTimes.size()] : rng();
      const uint64_t t2 =
          (rng() & 1) ? kSpecialTimes[rng() % kSpecialTimes.size()] : rng();
      const uint64_t t4 =
          (rng() & 1) ? kSpecialTimes[rng() % kSpecialTimes.size()] : rng();
      if (i & 1) {
        est.ObserveExchange(boot, t1, t2, t4);
      } else {
        (void)est.Observe(boot, t1, t2);
      }
    }
  }
}
#endif
