// Sim/bench tool only: publishes a chassis target on one drivetrain lane,
// /drivetrain/target/<source>, bypassing the operator interface. The
// drivetrain follows it only while the robot is enabled and the Driver
// Station mode's [robot.targets] policy selects that source (teleop by
// default). Like a TargetLane, each envelope carries the mode generation of
// the newest Driver Station packet on /hw/state/driver_station (0 while there
// is none or it names no mode), so it never drives a later enable or mode
// than the one it saw. Never use it on a robot at a match.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "drivetrain/drive_message_generated.h"
#include "talOS/devices/access.h"
#include "talOS/devices/target.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/names.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"

namespace {

constexpr const char* kUsage =
    "usage: send_target [--source NAME] VX_MPS VY_MPS OMEGA_RADPS DURATION_S\n"
    "  Sim/bench tool only: writes /drivetrain/target/<NAME> (default teleop)\n"
    "  directly, bypassing the operator interface. The drivetrain drives only\n"
    "  while enabled in a Driver Station mode whose policy selects NAME.";

// The generation a producer that drained this packet stamps.
std::uint64_t Generation(const talos::hardware::Packet& packet) {
  talos::driver_station::DriverStationData ds{};
  if (!talos::driver_station::Decode(packet.bytes(), ds)) return 0;
  return talos::detail::DsModeOf(ds.fms.flags) != talos::detail::DsMode::kNone
             ? ds.mode_since_us
             : 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string source = "teleop";
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--source" && i + 1 < argc) {
        source = argv[++i];
      } else {
        positional.push_back(arg);
      }
    }
    if (positional.size() != 4 || source.empty()) {
      throw std::invalid_argument(kUsage);
    }
    const double vx = std::stod(positional[0]), vy = std::stod(positional[1]),
                 omega = std::stod(positional[2]);
    const int seconds = std::stoi(positional[3]);
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) ||
        seconds <= 0) {
      throw std::invalid_argument("invalid target");
    }
    using Envelope = talos::TargetEnvelope<drivetrain::ChassisTarget>;
    ipc::Publisher<Envelope> publisher{
        talos::introspect::naming::TargetLaneTopic("drivetrain", source)};
    ipc::Subscriber<talos::hardware::Packet> driver_station{
        talos::hardware::kDriverStationTopic,
        RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
    if (!driver_station.registered()) {
      throw std::runtime_error("reader slots exhausted");
    }
    std::uint64_t generation = 0;
    const auto send = [&](bool active) {
      if (const auto packet = driver_station.read_next()) {
        generation = Generation(packet->message);
      }
      Envelope envelope{};
      envelope.set_mode_since_us(generation);
      envelope.active = active ? 1 : 0;
      if (active) envelope.message = drivetrain::ChassisTarget{vx, vy, omega};
      publisher.write(envelope);
    };
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds{seconds};
    while (std::chrono::steady_clock::now() < end) {
      send(true);
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    send(false);  // release: active = 0
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
