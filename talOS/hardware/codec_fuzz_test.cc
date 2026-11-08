#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/protocol/fuzz_corpus.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  using namespace talos::hardware;
  const std::span<const uint8_t> input(data, size);
  Command command;
  State state;
  Config config;
  ConfigAckPayload ack;
  if (Decode(input, command)) {
    std::vector<uint8_t> encoded(kMaxCommandSize);
    const auto written = Encode(command, encoded);
    Command again;
    if (!written || !Decode({encoded.data(), written}, again)) std::abort();
    encoded.resize(written);
    std::vector<uint8_t> roundtrip(kMaxCommandSize);
    roundtrip.resize(Encode(again, roundtrip));
    EXPECT_EQ(roundtrip, encoded);
  }
  if (Decode(input, state)) {
    std::vector<uint8_t> encoded(kMaxStateSize);
    const auto written = Encode(state, encoded);
    State again;
    if (!written || !Decode({encoded.data(), written}, again)) std::abort();
    encoded.resize(written);
    std::vector<uint8_t> roundtrip(kMaxStateSize);
    roundtrip.resize(Encode(again, roundtrip));
    EXPECT_EQ(roundtrip, encoded);
  }
  if (DeserializeConfig(input, config)) {
    const auto encoded = SerializeConfig(config);
    Config again;
    if (!DeserializeConfig(encoded, again)) std::abort();
    EXPECT_EQ(SerializeConfig(again), encoded);
  }
  if (LoadPersistedConfig(input, config)) {
    const auto encoded = PersistedConfig(config);
    Config again;
    EXPECT_TRUE(LoadPersistedConfig(encoded, again));
    EXPECT_EQ(PersistedConfig(again), encoded);
  }
  if (DecodeConfigAck(input, ack)) {
    std::vector<uint8_t> encoded(sizeof(ConfigAckPayload));
    EXPECT_EQ(EncodeConfigAck(ack, encoded), encoded.size());
    ConfigAckPayload again;
    EXPECT_TRUE(DecodeConfigAck(encoded, again));
    std::vector<uint8_t> roundtrip(encoded.size());
    EXPECT_EQ(EncodeConfigAck(again, roundtrip), roundtrip.size());
    EXPECT_EQ(roundtrip, encoded);
  }
  ConfigAssembler assembler;
  std::string reason;
  // Exercise metadata changes, duplicate chunks and reset after rejection.
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
  using namespace talos::hardware;
  std::vector<std::vector<uint8_t>> seeds;
  for (int full = 0; full < 2; ++full) {
    Command command;
    State state;
    if (full) {
      command.count = kMaxMotors;
      command.digital_output_count = kMaxDigitalOutputs;
      command.pwm_output_count = kMaxPwmOutputs;
      state.motor_count = kMaxMotors;
      state.sensor_count = kMaxSensors;
      state.digital_input_count = kMaxDigitalInputs;
      state.digital_output_count = kMaxDigitalOutputs;
      state.analog_input_count = kMaxAnalogInputs;
      state.encoder_count = kMaxEncoders;
      state.pwm_output_count = kMaxPwmOutputs;
    }
    std::vector<uint8_t> encoded(kMaxStateSize);
    encoded.resize(Encode(state, encoded));
    ASSERT_FALSE(encoded.empty());
    State decoded_state;
    ASSERT_TRUE(Decode(encoded, decoded_state));
    seeds.push_back(encoded);
    encoded.resize(kMaxCommandSize);
    encoded.resize(Encode(command, encoded));
    ASSERT_FALSE(encoded.empty());
    Command decoded_command;
    ASSERT_TRUE(Decode(encoded, decoded_command));
    seeds.push_back(encoded);
  }
  Config config;
  config.motors.push_back({1, "drive", 1, "rio"});
  config.sensors.push_back({2, "encoder", SensorKind::kCANcoder, 2, "rio"});
  config.digital_inputs.push_back({3, "limit", 0});
  config.digital_outputs.push_back({4, "solenoid", 1});
  config.analog_inputs.push_back({5, "pressure", 0});
  config.encoders.push_back({6, "wheel", EncoderKind::kQuadrature, 2, 3});
  config.pwm_outputs.push_back({7, "servo", 0});
  seeds.push_back(SerializeConfig(config));
  seeds.push_back(PersistedConfig(config));
  ConfigAckPayload ack;
  std::vector<uint8_t> encoded_ack(sizeof(ack));
  ASSERT_EQ(EncodeConfigAck(ack, encoded_ack), encoded_ack.size());
  seeds.push_back(encoded_ack);
  ConfigAssembler assembler;
  std::string reason;
  for (const auto& chunk : CreateConfigChunks(config, 48)) {
    seeds.push_back(chunk);
    const auto status = assembler.AddChunk(chunk, reason);
    if (status != ConfigAssembler::Status::kIncomplete &&
        status != ConfigAssembler::Status::kComplete)
      std::abort();
  }
  EXPECT_EQ(SerializeConfig(assembler.config()), SerializeConfig(config));
  // Carry mutated chunks across calls, then prove rejection leaves the
  // assembler able to receive a complete known-good transfer.
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
#endif
