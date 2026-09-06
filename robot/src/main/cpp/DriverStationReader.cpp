#include "DriverStationReader.h"

#include <frc/DriverStation.h>

#include <algorithm>
#include <string_view>

namespace talos::driver_station {

// The wire enum is cast straight from WPILib's below, so a reordering on
// either side has to fail the build rather than mislabel a match.
static_assert(static_cast<int>(MatchType::kNone) ==
              static_cast<int>(frc::DriverStation::MatchType::kNone));
static_assert(static_cast<int>(MatchType::kPractice) ==
              static_cast<int>(frc::DriverStation::MatchType::kPractice));
static_assert(static_cast<int>(MatchType::kQualification) ==
              static_cast<int>(frc::DriverStation::MatchType::kQualification));
static_assert(static_cast<int>(MatchType::kElimination) ==
              static_cast<int>(frc::DriverStation::MatchType::kElimination));

namespace {

template <std::size_t N>
void CopyString(std::string_view src, char (&dst)[N]) {
  static_assert(N > 0);
  const std::size_t count = std::min(src.size(), N - 1);
  std::copy_n(src.data(), count, dst);
  dst[count] = '\0';
}

}  // namespace

DriverStationData SampleDriverStation(uint64_t now_us) {
  DriverStationData ds{};
  ds.sample_time_us = now_us;

  // Set flags
  if (frc::DriverStation::IsDSAttached()) ds.fms.flags |= kDsAttached;
  if (frc::DriverStation::IsFMSAttached()) ds.fms.flags |= kFmsAttached;
  if (frc::DriverStation::IsEnabled()) ds.fms.flags |= kEnabled;
  if (frc::DriverStation::IsAutonomous()) ds.fms.flags |= kAutonomous;
  if (frc::DriverStation::IsTeleop()) ds.fms.flags |= kTeleop;
  if (frc::DriverStation::IsTest()) ds.fms.flags |= kTest;
  if (frc::DriverStation::IsEStopped()) ds.fms.flags |= kEStop;

  // Alliance & Location
  const auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value()) {
    ds.fms.alliance = (*alliance == frc::DriverStation::Alliance::kRed)
                          ? Alliance::kRed
                          : Alliance::kBlue;
  }
  const auto location = frc::DriverStation::GetLocation();
  if (location.has_value()) {
    ds.fms.station = static_cast<uint8_t>(*location);
  }

  // Match info
  ds.fms.match_type = static_cast<MatchType>(frc::DriverStation::GetMatchType());
  ds.fms.match_number = static_cast<uint16_t>(frc::DriverStation::GetMatchNumber());
  ds.fms.replay_number = static_cast<uint8_t>(frc::DriverStation::GetReplayNumber());
  ds.fms.match_time_s = static_cast<float>(frc::DriverStation::GetMatchTime().value());
  CopyString(frc::DriverStation::GetGameSpecificMessage(), ds.fms.game_specific_message);
  CopyString(frc::DriverStation::GetEventName(), ds.fms.event_name);

  // Joysticks
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    auto& j = ds.joysticks[i];
    j.connected = frc::DriverStation::IsJoystickConnected(static_cast<int>(i));
    if (!j.connected) continue;

    j.axis_count = static_cast<uint8_t>(std::min<int>(
        frc::DriverStation::GetStickAxisCount(static_cast<int>(i)),
        static_cast<int>(kMaxAxes)));
    for (std::size_t a = 0; a < j.axis_count; ++a) {
      j.axes[a] = static_cast<float>(
          frc::DriverStation::GetStickAxis(static_cast<int>(i), static_cast<int>(a)));
    }

    j.button_count = static_cast<uint8_t>(std::min<int>(
        frc::DriverStation::GetStickButtonCount(static_cast<int>(i)), 32));
    j.buttons = static_cast<uint32_t>(
        frc::DriverStation::GetStickButtons(static_cast<int>(i)));

    j.pov_count = static_cast<uint8_t>(std::min<int>(
        frc::DriverStation::GetStickPOVCount(static_cast<int>(i)),
        static_cast<int>(kMaxPovs)));
    for (std::size_t p = 0; p < j.pov_count; ++p) {
      j.povs[p] = static_cast<int16_t>(
          frc::DriverStation::GetStickPOV(static_cast<int>(i), static_cast<int>(p)));
    }

    CopyString(frc::DriverStation::GetJoystickName(static_cast<int>(i)), j.name);
  }

  return ds;
}

}  // namespace talos::driver_station
