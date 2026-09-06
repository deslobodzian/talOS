// Publishes synthetic Driver Station packets onto /hw/state/driver_station at
// the real 50 Hz sample rate, so the teleop path -- joystick axes to
// ChassisTarget to swerve motor requests -- can be driven and watched without a
// Driver Station or a controller processor attached. The payload is the same
// wire encoding the Rio sends, so nodes downstream cannot tell the difference.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "2026-robot/main_processor/driver_station/packet.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/os/poller.h"
#include "talOS/ipc/publisher.h"
#include "talOS/process/process.h"

namespace {

float AxisValue(const std::string& text) {
  const double value = std::stod(text);
  if (!std::isfinite(value) || value < -1.0 || value > 1.0)
    throw std::invalid_argument("axis values must be within [-1, 1]");
  return static_cast<float>(value);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    float axis0 = 0, axis1 = 0, axis2 = 0;
    uint32_t buttons = 0;
    int duration_s = 5;
    bool disabled = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--x" && i + 1 < argc)
        axis1 = -AxisValue(argv[++i]);  // Forward is negative axis 1.
      else if (arg == "--y" && i + 1 < argc)
        axis0 = -AxisValue(argv[++i]);  // Left is negative axis 0.
      else if (arg == "--rot" && i + 1 < argc)
        axis2 = -AxisValue(argv[++i]);
      else if (arg == "--buttons" && i + 1 < argc)
        buttons = static_cast<uint32_t>(std::stoul(argv[++i]));
      else if (arg == "--duration-s" && i + 1 < argc)
        duration_s = std::stoi(argv[++i]);
      else if (arg == "--disabled")
        disabled = true;
      else
        throw std::invalid_argument(
            "usage: send_joystick [--x -1..1] [--y -1..1] [--rot -1..1] "
            "[--buttons MASK] [--duration-s N] [--disabled]");
    }
    if (duration_s <= 0)
      throw std::invalid_argument("duration must be positive");

    talos::process::InstallStopHandlers();
    ipc::Publisher<talos::driver_station::Packet> publisher{
        talos::driver_station::kHwDsTopic};

    talos::driver_station::DriverStationData ds{};
    ds.fms.flags = talos::driver_station::kDsAttached;
    if (!disabled)
      ds.fms.flags |=
          talos::driver_station::kEnabled | talos::driver_station::kTeleop;
    ds.fms.alliance = talos::driver_station::Alliance::kBlue;
    auto& stick = ds.joysticks[0];
    stick.connected = true;
    stick.axis_count = 4;
    stick.button_count = 12;
    stick.buttons = buttons;
    stick.axes[0] = axis0;
    stick.axes[1] = axis1;
    stick.axes[2] = axis2;

    const auto period =
        std::chrono::microseconds{talos::driver_station::kSamplePeriodUs};
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds{duration_s};
    auto next = std::chrono::steady_clock::now();
    uint64_t sent = 0, dropped = 0;
    while (!talos::process::stop_requested.load() &&
           std::chrono::steady_clock::now() < end) {
      ds.sample_time_us =
          static_cast<uint64_t>(talos::event::Poller::now().nanos() / 1000);
      talos::driver_station::Packet packet{};
      packet.size =
          static_cast<uint32_t>(talos::driver_station::Encode(ds, packet.data));
      if (packet.size) {
        publisher.write(packet);
        ++sent;
      } else {
        ++dropped;
      }
      next += period;
      std::this_thread::sleep_until(next);
    }

    // A disabled packet on the way out, so the teleop node takes its falling
    // edge instead of leaving the drivetrain to time the last target out.
    ds.fms.flags = talos::driver_station::kDsAttached;
    ds.sample_time_us =
        static_cast<uint64_t>(talos::event::Poller::now().nanos() / 1000);
    talos::driver_station::Packet packet{};
    packet.size =
        static_cast<uint32_t>(talos::driver_station::Encode(ds, packet.data));
    if (packet.size) publisher.write(packet);

    std::printf("send_joystick: packets=%llu encode_failures=%llu\n",
                static_cast<unsigned long long>(sent),
                static_cast<unsigned long long>(dropped));
    return dropped ? 1 : 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
