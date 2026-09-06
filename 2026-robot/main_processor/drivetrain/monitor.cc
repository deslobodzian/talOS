// A read-only view of the drivetrain link, for watching a simulation or a real
// robot work.
//
// It subscribes to the same three local topics the node and bridge use and
// prints what is on them, so it answers the questions you actually have while
// something is running: is the gateway talking, is the robot enabled, is the
// node commanding anything, and are the wheels responding. It publishes
// nothing and holds no hardware, so it is safe to start and stop at any time.

#include <chrono>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>

#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/drivetrain/geometry.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "talOS/hardware/messages.h"
#include "talOS/ipc/subscriber.h"
#include "talOS/process/process.h"

namespace {

using Clock = std::chrono::steady_clock;

// Ages are measured against our own clock, from the moment we saw the message.
// The timestamps inside the messages come from other processes' clocks, and a
// monitor should not quietly assume those share an epoch with ours.
//
// The rate comes from the publisher's sequence numbers, not from how often we
// polled. These topics are read in LATEST mode, so the monitor deliberately
// skips messages; counting the ones it happened to see would report its own
// refresh rate back as the link's rate, which is the one number here that has
// to be trustworthy.
struct Freshness {
  std::optional<Clock::time_point> last;
  std::uint64_t sequence{0};
  bool seen{false};
  std::uint64_t sequence_at_mark{0};
  Clock::time_point mark{Clock::now()};
  double rate_hz{0};

  void observe(std::uint64_t published) {
    last = Clock::now();
    if (!seen) {
      sequence_at_mark = published;
      seen = true;
    }
    sequence = published;
  }

  void update_rate(Clock::time_point now) {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - mark)
            .count();
    if (elapsed < 0.5) {
      return;
    }
    rate_hz = static_cast<double>(sequence - sequence_at_mark) / elapsed;
    sequence_at_mark = sequence;
    mark = now;
  }

  std::string age(Clock::time_point now) const {
    if (!last) {
      return "never";
    }
    const auto ms = std::chrono::duration_cast<std::chrono::duration<double>>(
                        now - *last)
                        .count() *
                    1e3;
    char text[32];
    std::snprintf(text, sizeof(text), "%.0f ms", ms);
    return text;
  }
};

const char* on_off(bool value) { return value ? "yes" : " no"; }

// The flags are the whole point of the display: a robot that will not move is
// almost always a robot that is not enabled.
void PrintGateway(const talos::hardware::State& state, bool have_state,
                  const Freshness& freshness, Clock::time_point now) {
  std::printf("gateway   ");
  if (!have_state) {
    std::printf("no state received. Is the gateway running?\n\n");
    return;
  }

  const auto flag = [&](std::uint32_t bit) {
    return on_off((state.flags & bit) != 0);
  };

  std::printf("configured %s   enabled %s   commanding %s   fault %s\n",
              flag(talos::hardware::kConfigured),
              flag(talos::hardware::kEnabled),
              flag(talos::hardware::kCommandActive),
              flag(talos::hardware::kHardwareFault));
  std::printf("          epoch %llu   age %s   %.0f Hz   seq %llu\n\n",
              static_cast<unsigned long long>(state.epoch),
              freshness.age(now).c_str(), freshness.rate_hz,
              static_cast<unsigned long long>(freshness.sequence));
}

void PrintTarget(const talos::drive::ChassisTarget& target, bool have_target,
                 const Freshness& freshness, Clock::time_point now) {
  std::printf("target    ");
  if (!have_target) {
    std::printf("none. Try: bazel run //2026-robot/main_processor/drivetrain:send_target -- "
                "1 0 0 5\n\n");
    return;
  }
  std::printf(
      "vx %6.2f m/s   vy %6.2f m/s   omega %6.2f rad/s   enabled %s   age %s\n"
      "\n",
      target.vx_mps(), target.vy_mps(), target.omega_radps(),
      on_off(target.enabled()), freshness.age(now).c_str());
}

const talos::hardware::MotorSample* FindSample(
    const talos::hardware::State& state, std::uint16_t id) {
  for (std::size_t i = 0; i < state.motor_count && i < state.motors.size();
       ++i) {
    if (state.motors[i].id == id) {
      return &state.motors[i];
    }
  }
  return nullptr;
}

const talos::hardware::MotorRequest* FindRequest(
    const talos::hardware::Command& command, std::uint16_t id) {
  for (std::size_t i = 0; i < command.count && i < command.motors.size(); ++i) {
    if (command.motors[i].id == id) {
      return &command.motors[i];
    }
  }
  return nullptr;
}

void PrintModules(const talos::drive::SwerveGeometry& geometry,
                  const talos::hardware::State& state, bool have_state,
                  const talos::hardware::Command& command, bool have_command,
                  const Freshness& command_freshness, Clock::time_point now) {
  std::printf("command   age %s   %.0f Hz   seq %llu\n\n",
              have_command ? command_freshness.age(now).c_str() : "never",
              command_freshness.rate_hz,
              static_cast<unsigned long long>(command_freshness.sequence));

  std::printf("%-13s %11s %11s %13s %13s %11s\n", "module", "steer cmd",
              "steer now", "drive cmd", "drive now", "travelled");
  std::printf("%-13s %11s %11s %13s %13s %11s\n", "", "rot", "rot", "rot/s",
              "rot/s", "rot");

  for (const auto& module : geometry.modules) {
    std::printf("%-13s", module.name.data());

    const auto* steer_request =
        have_command ? FindRequest(command, module.steer_id) : nullptr;
    const auto* drive_request =
        have_command ? FindRequest(command, module.drive_id) : nullptr;
    const auto* steer_sample =
        have_state ? FindSample(state, module.steer_id) : nullptr;
    const auto* drive_sample =
        have_state ? FindSample(state, module.drive_id) : nullptr;

    const auto number = [](const double* value, int width, int precision) {
      char text[32];
      if (value == nullptr) {
        std::snprintf(text, sizeof(text), "%*s", width, "-");
      } else {
        std::snprintf(text, sizeof(text), "%*.*f", width, precision, *value);
      }
      std::printf("%s", text);
    };

    // A neutral request is not a zero request: showing 0.000 for it would
    // suggest the node is actively commanding a stop when it is not.
    const bool steering =
        steer_request != nullptr &&
        steer_request->mode != talos::hardware::Mode::kNeutral;
    const bool driving = drive_request != nullptr &&
                         drive_request->mode != talos::hardware::Mode::kNeutral;

    number(steering ? &steer_request->demand : nullptr, 12, 4);
    number(steer_sample ? &steer_sample->position_rot : nullptr, 12, 4);
    number(driving ? &drive_request->demand : nullptr, 14, 4);
    number(drive_sample ? &drive_sample->velocity_rps : nullptr, 14, 4);
    number(drive_sample ? &drive_sample->position_rot : nullptr, 12, 2);
    std::printf("\n");
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    int duration_s = 0;
    double rate_hz = 10;
    std::string config_path =
        "2026-robot/main_processor/configuration/robot.toml";
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--duration-s" && i + 1 < argc) {
        duration_s = std::stoi(argv[++i]);
      } else if (arg == "--rate-hz" && i + 1 < argc) {
        rate_hz = std::stod(argv[++i]);
      } else if (arg == "--config" && i + 1 < argc) {
        config_path = argv[++i];
      } else {
        throw std::invalid_argument(
            "usage: monitor [--duration-s N] [--rate-hz N] [--config PATH]");
      }
    }
    if (duration_s < 0) {
      throw std::invalid_argument("duration must be nonnegative");
    }
    if (rate_hz <= 0 || rate_hz > 100) {
      throw std::invalid_argument("rate must be between 0 and 100 Hz");
    }

    // Module names and ids come from the same configuration the drivetrain
    // node runs on, so the table cannot drift from what is being commanded.
    const auto geometry = talos::drive::BuildSwerveGeometry(
        talos::config::ParseRobotConfig(config_path));

    talos::process::InstallStopHandlers();

    const RTMSOptions latest{OverflowPolicy::OVERWRITE_OLDEST,
                             ReadMode::LATEST};
    ipc::Subscriber<talos::drive::Packet> states{talos::drive::kStateTopic,
                                                 latest};
    ipc::Subscriber<talos::drive::Packet> commands{talos::drive::kCommandTopic,
                                                   latest};
    ipc::Subscriber<talos::drive::ChassisTarget> targets{
        talos::drive::kTargetTopic, latest};

    if (!states.registered() || !commands.registered() ||
        !targets.registered()) {
      throw std::runtime_error("reader slots exhausted");
    }

    talos::hardware::State state{};
    talos::hardware::Command command{};
    talos::drive::ChassisTarget target{};
    bool have_state = false, have_command = false, have_target = false;
    Freshness state_seen, command_seen, target_seen;

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>{1.0 / rate_hz});
    const auto end = Clock::now() + std::chrono::seconds{duration_s};
    auto next = Clock::now();

    while (!talos::process::stop_requested.load() &&
           (!duration_s || Clock::now() < end)) {
      if (const auto packet = states.read_next()) {
        if (talos::hardware::Decode(packet->message.bytes(), state)) {
          have_state = true;
          state_seen.observe(packet->sequence);
        }
      }
      if (const auto packet = commands.read_next()) {
        if (talos::hardware::Decode(packet->message.bytes(), command)) {
          have_command = true;
          command_seen.observe(packet->sequence);
        }
      }
      if (const auto latest_target = targets.read_next()) {
        target = latest_target->message;
        have_target = true;
        target_seen.observe(latest_target->sequence);
      }

      const auto now = Clock::now();
      state_seen.update_rate(now);
      command_seen.update_rate(now);
      target_seen.update_rate(now);

      std::printf("\033[H\033[J");  // Home, then clear: redraw without flicker.
      std::printf("talOS drivetrain monitor        Ctrl-C to stop\n\n");
      PrintGateway(state, have_state, state_seen, now);
      PrintTarget(target, have_target, target_seen, now);
      PrintModules(geometry, state, have_state, command, have_command,
                   command_seen, now);
      std::fflush(stdout);

      next += period;
      std::this_thread::sleep_until(next);
    }

    std::printf("\nmonitor: state seq=%llu command seq=%llu target seq=%llu\n",
                static_cast<unsigned long long>(state_seen.sequence),
                static_cast<unsigned long long>(command_seen.sequence),
                static_cast<unsigned long long>(target_seen.sequence));
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
