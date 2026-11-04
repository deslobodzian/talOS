#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace talos::driver_station {

enum class Alliance : uint8_t {
  kUnknown = 0,
  kRed = 1,
  kBlue = 2,
};

enum class MatchType : uint8_t {
  kNone = 0,
  kPractice = 1,
  kQualification = 2,
  kElimination = 3,
};

enum DsFlags : uint32_t {
  kDsAttached = 1u << 0,
  kFmsAttached = 1u << 1,
  kEnabled = 1u << 2,
  kAutonomous = 1u << 3,
  kTeleop = 1u << 4,
  kTest = 1u << 5,
  kEStop = 1u << 6,
};

// The flags that make up the Driver Station mode. A change in any of them
// starts a new mode generation (DriverStationData::mode_since_us); match
// data, FMS attachment and joysticks do not.
inline constexpr uint32_t kModeFlags =
    kDsAttached | kEnabled | kAutonomous | kTeleop | kTest | kEStop;
constexpr uint32_t ModeKey(uint32_t flags) { return flags & kModeFlags; }

constexpr std::size_t kMaxJoysticks = 6;
constexpr std::size_t kMaxAxes = 12;
constexpr std::size_t kMaxPovs = 4;
constexpr std::size_t kMaxNameLength = 32;
constexpr std::size_t kMaxGameMessageLength = 32;
constexpr std::size_t kMaxEventNameLength = 32;

// The Driver Station itself only produces new data at 50 Hz, so sampling any
// faster only duplicates packets. Must stay in sync with the
// [subsystems.driver_station] period_us in the robot configuration.
inline constexpr uint64_t kSamplePeriodUs = 20000;

struct JoystickData {
  bool connected{false};
  uint8_t axis_count{0};
  uint8_t button_count{0};
  uint8_t pov_count{0};
  uint32_t buttons{0};
  std::array<float, kMaxAxes> axes{};
  std::array<int16_t, kMaxPovs> povs{};
  char name[kMaxNameLength]{};

  bool operator==(const JoystickData&) const = default;
};

struct FmsData {
  uint32_t flags{0};
  Alliance alliance{Alliance::kUnknown};
  uint8_t station{0};
  MatchType match_type{MatchType::kNone};
  uint16_t match_number{0};
  uint8_t replay_number{0};
  float match_time_s{0.0f};
  char game_specific_message[kMaxGameMessageLength]{};
  char event_name[kMaxEventNameLength]{};

  bool operator==(const FmsData&) const = default;
};

struct DriverStationData {
  uint64_t sample_time_us{0};
  // The mode generation: the sample_time_us of the first sample in the
  // current mode (ModeKey of the flags), never 0. Stamped by the sampler's
  // ModeTracker; a packet that never went through one carries 0, which no
  // target accepts.
  uint64_t mode_since_us{0};
  FmsData fms{};
  std::array<JoystickData, kMaxJoysticks> joysticks{};

  bool operator==(const DriverStationData&) const = default;
};

// What every Driver Station sampler (the RoboRIO program, SimHardware,
// send_joystick) runs on each sample before encoding it, so a target issued
// under an earlier enable or mode can be told from one issued under this one
// (talOS/devices/target.h):
//
//   DriverStationData ds = Sample(now_us);
//   tracker.Stamp(ds);
//   Encode(ds, buffer);
//
// Stamp sets ds.mode_since_us to the sample_time_us of the first sample whose
// ModeKey differs from the previous sample's. Each new generation is at least
// 1 and later than the one before, even if sample times repeat or step back.
class ModeTracker {
 public:
  void Stamp(DriverStationData& ds);

 private:
  bool have_{false};
  uint32_t key_{0};
  uint64_t since_us_{0};
};

std::size_t Encode(const DriverStationData& ds, std::span<uint8_t> out);
bool Decode(std::span<const uint8_t> data, DriverStationData& out);

}  // namespace talos::driver_station
