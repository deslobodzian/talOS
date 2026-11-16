// The State and Command encodings, pinned to the protocol version they are
// spoken under (docs/DATA_FLOW.md, "UDP frame and payload sizes"). Recorded
// sessions keep these bytes and are decoded by later builds' codecs only
// under the version their manifest records (REQ-OPS-51), so an encoding
// change that keeps every length, made without bumping kProtocolVersion,
// would decode every earlier session wrongly with no warning. This test
// fails on any change to the bytes of a fully populated packet: bump
// kProtocolVersion with the change and regenerate the constants it prints.
// Every value is exactly representable (dyadic fractions, tenths that the
// signed-tenths fields hold exactly), so the bytes are the same on every
// host.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>

#include "talOS/hardware/messages.h"
#include "talOS/protocol/types.h"

namespace talos::hardware {
namespace {

// The encodings this file pins and the protocol version they belong to.
constexpr std::uint16_t kGoldenVersion = 9;
constexpr std::size_t kStateBytes = 1419;
constexpr std::uint64_t kStateHash = 0x270781b104f788b0;
constexpr std::size_t kCommandBytes = 414;
constexpr std::uint64_t kCommandHash = 0xe43b8a06779c5ed5;

std::uint64_t Fnv1a64(std::span<const std::uint8_t> bytes) {
  std::uint64_t hash = 0xcbf29ce484222325ull;
  for (const std::uint8_t byte : bytes) {
    hash ^= byte;
    hash *= 0x100000001b3ull;
  }
  return hash;
}

std::string Hex(std::uint64_t value) {
  char text[32];
  std::snprintf(text, sizeof text, "0x%016llx",
                static_cast<unsigned long long>(value));
  return text;
}

// Every device slot filled, every field a value of its own.
State FullState() {
  State s;
  s.config_id = 0x0102030405060708ull;
  s.boot_id = 0x1112131415161718ull;
  s.epoch = 0x2122232425262728ull;
  s.sample_time_us = 0x3132333435363738ull;
  s.last_command_sequence = 0x4142434445464748ull;
  s.clock_probe_time_us = 0x5152535455565758ull;
  s.sequence = 0x6162636465666768ull;
  s.build = {0x7172737475767778ull, GatewayProgram::kRoboRio, true};
  s.flags = kConfigured | kEnabled | kCommandActive;
  s.motor_count = kMaxMotors;
  s.sensor_count = kMaxSensors;
  s.digital_input_count = kMaxDigitalInputs;
  s.digital_output_count = kMaxDigitalOutputs;
  s.analog_input_count = kMaxAnalogInputs;
  s.encoder_count = kMaxEncoders;
  s.pwm_output_count = kMaxPwmOutputs;
  for (std::size_t i = 0; i < kMaxMotors; ++i) {
    const double k = static_cast<double>(i);
    auto& m = s.motors[i];
    m.id = static_cast<std::uint16_t>(0x100 + i);
    m.valid = i % 2 == 0;
    m.position_rot = 1.25 + k;
    m.velocity_rps = -2.5 - k;
    m.voltage = 0.75 + k;
    m.stator_current_a = 10.5 + k;
    m.position_age_us = static_cast<std::uint32_t>(1000 + i);
    m.velocity_age_us = static_cast<std::uint32_t>(2000 + i);
    m.fault_flags = static_cast<std::uint32_t>(0x00010001u << (i % 16));
    m.temperature_c = 30.5 + k;
    m.supply_current_a = -1.5 - k;
  }
  for (std::size_t i = 0; i < kMaxSensors; ++i) {
    const double k = static_cast<double>(i);
    auto& m = s.sensors[i];
    m.id = static_cast<std::uint16_t>(0x200 + i);
    m.valid = i % 2 == 1;
    m.position_rot = 0.125 + k;
    m.velocity_rps = -0.25 - k;
    m.position_age_us = static_cast<std::uint32_t>(3000 + i);
    m.velocity_age_us = static_cast<std::uint32_t>(4000 + i);
  }
  for (std::size_t i = 0; i < kMaxDigitalInputs; ++i) {
    s.digital_inputs[i] = {static_cast<std::uint16_t>(0x300 + i), true,
                           i % 2 == 0};
  }
  for (std::size_t i = 0; i < kMaxDigitalOutputs; ++i) {
    s.digital_outputs[i] = {static_cast<std::uint16_t>(0x400 + i), i % 2 == 0,
                            true};
  }
  for (std::size_t i = 0; i < kMaxAnalogInputs; ++i) {
    s.analog_inputs[i] = {static_cast<std::uint16_t>(0x500 + i), true,
                          2.5 + static_cast<double>(i),
                          static_cast<std::uint32_t>(5000 + i)};
  }
  for (std::size_t i = 0; i < kMaxEncoders; ++i) {
    s.encoders[i] = {static_cast<std::uint16_t>(0x600 + i), true,
                     3.5 + static_cast<double>(i),
                     -4.5 - static_cast<double>(i),
                     static_cast<std::uint32_t>(6000 + i)};
  }
  for (std::size_t i = 0; i < kMaxPwmOutputs; ++i) {
    s.pwm_outputs[i] = {static_cast<std::uint16_t>(0x700 + i), true,
                        -0.5 + static_cast<double>(i) / 4};
  }
  s.health.valid = true;
  s.health.battery_v = 12.25;
  s.health.brownout = true;
  s.health.can_bus_count = 3;
  s.health.can_bus_index = 2;
  s.health.can_bus = {CanBusReadState::kStale,
                      0.375f,
                      0x11121314,
                      0x21222324,
                      200,
                      100,
                      0x31323334};
  s.counters = {0x41424344, 0x51525354, 0x61626364, 0x71727374};
  return s;
}

Command FullCommand() {
  Command c;
  c.config_id = 0x8182838485868788ull;
  c.boot_id = 0x9192939495969798ull;
  c.epoch = 0xA1A2A3A4A5A6A7A8ull;
  c.observed_time_us = 0xB1B2B3B4B5B6B7B8ull;
  c.count = kMaxMotors;
  c.digital_output_count = kMaxDigitalOutputs;
  c.pwm_output_count = kMaxPwmOutputs;
  for (std::size_t i = 0; i < kMaxMotors; ++i) {
    c.motors[i] = {static_cast<std::uint16_t>(0x800 + i),
                   static_cast<Mode>(i % 6), static_cast<std::uint8_t>(i % 2),
                   0.5 + static_cast<double>(i),
                   -0.25 - static_cast<double>(i)};
  }
  for (std::size_t i = 0; i < kMaxDigitalOutputs; ++i) {
    c.digital_outputs[i] = {static_cast<std::uint16_t>(0x900 + i), i % 2 == 1};
  }
  // Pulses and outputs that send none, alternately.
  for (std::size_t i = 0; i < kMaxPwmOutputs; ++i) {
    const auto id = static_cast<std::uint16_t>(0xA00 + i);
    c.pwm_outputs[i] =
        i % 2 == 0
            ? PwmRequest{id, PwmMode::kPulse, 0.75 - static_cast<double>(i) / 2}
            : PwmRequest{.id = id};
  }
  return c;
}

// Guards: the State and Command bytes of protocol kGoldenVersion. A failure
// here is a wire change: bump kProtocolVersion (docs/DATA_FLOW.md) and
// regenerate the constants above from the values this prints.
// REQ-OPS-51
TEST(WireGolden, StateAndCommandBytesBelongToTheirProtocolVersion) {
  ASSERT_EQ(protocol::kProtocolVersion, kGoldenVersion)
      << "kProtocolVersion changed: regenerate this file's constants for "
         "protocol "
      << protocol::kProtocolVersion;
  std::array<std::uint8_t, kMaxStateSize> state{};
  const std::size_t state_size = Encode(FullState(), state);
  const std::uint64_t state_hash = Fnv1a64(std::span{state}.first(state_size));
  EXPECT_TRUE(state_size == kStateBytes && state_hash == kStateHash)
      << "the State encoding changed: bump kProtocolVersion and regenerate "
         "this constant (now "
      << state_size << " bytes, hash " << Hex(state_hash) << ")";
  std::array<std::uint8_t, kMaxCommandSize> command{};
  const std::size_t command_size = Encode(FullCommand(), command);
  const std::uint64_t command_hash =
      Fnv1a64(std::span{command}.first(command_size));
  EXPECT_TRUE(command_size == kCommandBytes && command_hash == kCommandHash)
      << "the Command encoding changed: bump kProtocolVersion and regenerate "
         "this constant (now "
      << command_size << " bytes, hash " << Hex(command_hash) << ")";
}

}  // namespace
}  // namespace talos::hardware
