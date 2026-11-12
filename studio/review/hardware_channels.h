#pragma once

// The hardware packet topics (/hw/state, /hw/command, /hw/request/<node>)
// as channels: each packet is decoded by the one decoder log_dump prints
// with (launcher::HardwareRecordJson), and that JSON object is flattened
// into one sample per scalar, so a reviewed value is log_dump's by
// construction.
//
//   name   `<topic without its leading slash>/<member path>`: nested objects
//          as path segments, a device array element by its logical `id`
//          (a joystick by its `port`), a number array element by its index
//          ("hw/state/motors/1/velocity_rps", "hw/state/health/battery_v")
//   kind   bool for a JSON bool, double for every number (a JSON number does
//          not say whether its field was an integer, and one that prints
//          as "6" may be a double)
//   unit   from the member name's suffix (talOS/schema/unit_suffix.h)
//
// Strings (device names, the 64-bit ids, which are decimal strings, motor
// modes, the gateway program) are not channels. A null is a non-finite
// number, and reads as NaN for a channel already known to be a number.
// /hw/state/driver_station is not flattened here: its channels are the ones
// live telemetry names (telemetry::DriverStationChannels).

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/launcher/hardware_record.h"
#include "talOS/telemetry/frames.h"

namespace studio::review {

struct HardwareSample {
  std::string name;
  talos::telemetry::ChannelKind kind{Talos::Telemetry::ChannelKind_Double};
  std::uint64_t raw{0};  // telemetry raw sample (RawFromDouble for a double)
  bool null{false};      // a JSON null: kind is unknown, raw is NaN
};

// Every scalar of `payload`'s decoded packet, appended to `out` (cleared
// first). Nothing for a payload that does not decode (its JSON has an
// "error" member).
void FlattenHardware(std::string_view topic,
                     talos::launcher::HardwareRecord kind,
                     std::span<const std::byte> payload,
                     std::vector<HardwareSample>& out);

}  // namespace studio::review
