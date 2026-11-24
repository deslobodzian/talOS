#include "talOS/simulation/mujoco/gateway_extension.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "sim/sim_descriptor.h"
#include "sim/topics.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/sim_plant.h"
#include "talOS/simulation/mujoco/fields/rebuilt_2026/rules.h"
#include "talOS/simulation/mujoco/scripted_shooter.h"

namespace talos::simulation::mujoco {

namespace {

using hardware::sim::PlantMotor;
using rules_cc::cc::runfiles::Runfiles;

// Each field model's runfiles path, from its package's `data` ($(rlocationpath)
// in BUILD), so a robot repository that names talOS @talos finds it too, and
// its game's rules.
constexpr std::array<FieldEntry, 1> kFields = {{
    {"rebuilt_2026", TALOS_MUJOCO_FIELD_REBUILT_2026, &MakeRebuilt2026Rules},
}};

const FieldEntry* FindField(std::string_view name) {
  for (const FieldEntry& entry : kFields) {
    if (entry.name == name) return &entry;
  }
  return nullptr;
}

// The game message a packet carries is the game's, whole.
static_assert(std::tuple_size_v<decltype(FmsReport::game_specific_message)> ==
              driver_station::kMaxGameMessageLength);

std::string OwnsChassis(std::string_view key) {
  return std::string(key) +
         ": the robot model's root body has a freejoint, so the model owns "
         "the chassis and its wheels; remove it from a MuJoCo gateway config";
}

constexpr int64_t kReportPeriodNs = 1'000'000'000;

}  // namespace

std::span<const FieldEntry> GatewayFields() { return kFields; }

FieldResolver RunfilesFieldResolver(std::string argv0) {
  return [argv0 = std::move(argv0)](std::string_view field) -> std::string {
    const FieldEntry* known = FindField(field);
    if (known == nullptr) {
      throw std::invalid_argument(
          "[simulation.mujoco] field = '" + std::string(field) +
          "': the MuJoCo gateway has no such field model");
    }
    const std::string rlocation(known->rlocation);
    // The environment's runfiles first (bazel test, talos_sim), then the tree
    // beside the binary: a launcher started by `bazel run` may export its own
    // runfiles, which do not hold this program's data.
    std::array<std::unique_ptr<Runfiles>, 2> candidates = {
        std::unique_ptr<Runfiles>(
            Runfiles::Create(argv0, BAZEL_CURRENT_REPOSITORY)),
        std::unique_ptr<Runfiles>(
            Runfiles::Create(argv0, "", "", BAZEL_CURRENT_REPOSITORY)),
    };
    for (const auto& runfiles : candidates) {
      if (runfiles == nullptr) continue;
      const std::string path = runfiles->Rlocation(rlocation);
      std::error_code ec;
      if (!path.empty() && std::filesystem::is_regular_file(path, ec)) {
        return path;
      }
    }
    throw std::invalid_argument(
        "[simulation.mujoco] field = '" + std::string(field) +
        "': " + rlocation + " is not in the runfiles of " + argv0 +
        " (run it with bazel run or bazel test, or from bazel-bin after "
        "building it)");
  };
}

MujocoGatewayExtension::MujocoGatewayExtension(Options options)
    : options_(std::move(options)) {}

MujocoGatewayExtension::~MujocoGatewayExtension() = default;

void MujocoGatewayExtension::Configure(
    const config::RobotConfig& config,
    const hardware::sim::PhysicsOptions& options,
    hardware::sim::PhysicsSimBackend& backend) {
  if (!config.simulation.mujoco) {
    throw std::invalid_argument(
        "[simulation.mujoco] robot_model: the config sets no robot model; "
        "the MuJoCo gateway simulates one (sim_gateway runs a config without "
        "[simulation.mujoco])");
  }
  const config::SimulationMujocoConfig& model = *config.simulation.mujoco;

  // The motors as the backend will bind them (config.hardware carries the
  // same local motor types the gateway hands SetLocalMotorTypes): one source
  // for the gear and winding constants the model's damping G^2*b_m is
  // compiled from, and that Bind then compares exactly. Their names view
  // into `config`, which outlives MakePlant; the plant keeps no name.
  const std::vector<PlantMotor> motors =
      hardware::sim::PhysicsSimBackend::PlantMotors(config.hardware);

  const std::string field_xml =
      model.field.empty() ? std::string() : options_.field(model.field);
  plant_ = MakePlant(PlantModel{.field_xml = field_xml,
                                .robot_xml = model.robot_model,
                                .start_x_m = options.start_x_m,
                                .start_y_m = options.start_y_m,
                                .start_yaw_rad = options.start_yaw_rad},
                     motors);

  if (plant_->chassis()) {
    const toml::node_view<const toml::node> simulation =
        config.toml_data["simulation"];
    if (simulation["wheel_radius_m"]) {
      throw std::invalid_argument(OwnsChassis("[simulation] wheel_radius_m"));
    }
    if (!config.simulation.swerve_modules.empty()) {
      throw std::invalid_argument(OwnsChassis("[[simulation.swerve_modules]]"));
    }
    if (config.simulation.chassis) {
      throw std::invalid_argument(OwnsChassis("[simulation.chassis]"));
    }
    if (options.wheel_radius_error_frac != 0.0) {
      throw std::invalid_argument(OwnsChassis("--wheel-radius-error"));
    }
    if (options.wheel_slip_gain != 0.0) {
      throw std::invalid_argument(OwnsChassis("--wheel-slip-gain"));
    }
  }

  std::vector<hardware::sim::PhysicsSimBackend::WorldInput> inputs;
  for (const hardware::DigitalInputConfig& input :
       config.hardware.digital_inputs) {
    if (input.sim_sensor.sensor.empty()) continue;
    inputs.push_back({.input = input.name.str(),
                      .sensor = input.sim_sensor.sensor,
                      .closed_reads = input.sim_sensor.closed_reads});
  }
  // World rules, before the plant's first step: the robot's scripted
  // shooters, then the field's game, so a FUEL a shooter launches in a step
  // is judged at its exit in that step. A bad contract throws
  // std::invalid_argument naming the element: a config refusal.
  std::unique_ptr<WorldRules> shooters = MakeScriptedShooters(*plant_->model());
  if (shooters != nullptr) {
    shooters_ = dynamic_cast<ScriptedShooters*>(shooters.get());
    plant_->AddRules(*shooters);
    rules_.push_back(std::move(shooters));
  }
  if (const FieldEntry* field = FindField(model.field);
      field != nullptr && field->make_rules != nullptr) {
    game_ = field->make_rules(GameOptions{.seed = model.seed});
    plant_->AddRules(*game_);
  }
  if (!options_.tests_only_skip_match_writes) {
    publisher_.emplace(::sim::topics::kMatchState);
  }

  backend.SetPlant(plant_.get());
  backend.SetWorldInputs(std::move(inputs));
  // The gateway's Reconfigure binds the plant next; a refusal there exits 1
  // naming the key and the motor or input.
  backend_ = &backend;
}

void MujocoGatewayExtension::ReportClock(std::string_view why) {
  const auto& time = backend_->plant_time();
  char line[256];
  std::snprintf(line, sizeof(line),
                "mujoco_gateway: %.*s: plant time %.3f s stepped, %.3f s "
                "dropped in %llu ticks\n",
                static_cast<int>(why.size()), why.data(),
                static_cast<double>(time.stepped_us) * 1e-6,
                static_cast<double>(time.dropped_us) * 1e-6,
                static_cast<unsigned long long>(time.dropped_ticks));
  *options_.log << line << std::flush;
}

event::Manifest MujocoGatewayExtension::Topics() const {
  return {{
      .kind = event::SourceKind::SENDER,
      .name = ::sim::topics::kMatchState,
      .message_bytes = sizeof(::sim::MatchState),
      .alignment = alignof(::sim::MatchState),
      .schema = schema::Of<::sim::MatchState>(),
  }};
}

void MujocoGatewayExtension::Counts(std::span<TopicCount> out) const {
  out[0] = match_count_;
}

void MujocoGatewayExtension::BeforeTick(const TickInput& tick) {
  if (plant_ != nullptr) plant_->SetMatchInput(MatchInputOf(tick));
}

void MujocoGatewayExtension::Stamp(FmsStamp& fms) {
  if (game_ == nullptr) return;
  const FmsReport report = game_->Fms();
  fms.match_time_s = report.match_time_s;
  static_assert(sizeof(report.game_specific_message) ==
                sizeof(fms.game_specific_message));
  fms.game_specific_message = report.game_specific_message;
}

MatchInput MatchInputOf(const TickInput& tick) {
  MatchInput input;
  if (tick.driver_station != nullptr) {
    namespace ds = driver_station;
    const uint32_t flags = tick.driver_station->fms.flags;
    // The gateway's enable (gateway_main.cc GatewayEnabled), and the mode a
    // Driver Station shows (its bits only while one is attached).
    input.enabled = (flags & ds::kEnabled) != 0 && (flags & ds::kEStop) == 0;
    const uint32_t shown = ds::ShownFlags(flags);
    if ((shown & ds::kTest) != 0) {
      input.mode = MatchInput::Mode::kTest;
    } else if ((shown & ds::kAutonomous) != 0) {
      input.mode = MatchInput::Mode::kAutonomous;
    } else if ((shown & ds::kTeleop) != 0) {
      input.mode = MatchInput::Mode::kTeleop;
    }
    input.red_alliance =
        tick.driver_station->fms.alliance == ds::Alliance::kRed;
  }
  input.scheduled = tick.schedule_us.has_value();
  input.schedule_us = tick.schedule_us.value_or(0);
  return input;
}

void MujocoGatewayExtension::AfterTick(uint64_t now_us) {
  if (plant_ == nullptr || backend_ == nullptr) return;
  if (!table_printed_) {  // the first tick, after the gateway bound the plant
    *options_.log << plant_->binding_table() << std::flush;
    table_printed_ = true;
  }
  if (const std::string_view failure = plant_->failure(); !failure.empty()) {
    ReportClock("the world failed");
    throw std::runtime_error("mujoco_gateway: " + std::string(failure));
  }
  PublishMatch(now_us);
  const auto& time = backend_->plant_time();
  if (time.dropped_us > reported_dropped_us_) {
    const int64_t now_ns = options_.wall_clock_ns();
    if (!dilation_logged_ || now_ns - dilation_logged_ns_ >= kReportPeriodNs) {
      ReportClock("time dilation, the host ran slower than real time");
      reported_dropped_us_ = time.dropped_us;
      dilation_logged_ns_ = now_ns;
      dilation_logged_ = true;
    }
  }
  const uint64_t excursions = plant_->overspeed_excursions();
  if (excursions > reported_excursions_) {
    const int64_t now_ns = options_.wall_clock_ns();
    if (!overspeed_logged_ ||
        now_ns - overspeed_logged_ns_ >= kReportPeriodNs) {
      *options_.log << "mujoco_gateway: " << excursions
                    << " overspeed excursions so far (a bound rotor above "
                    << hardware::Gateway::kMaxRotorRps
                    << " rps; check the model's inertia and gear)\n"
                    << std::flush;
      reported_excursions_ = excursions;
      overspeed_logged_ns_ = now_ns;
      overspeed_logged_ = true;
    }
  }
}

void MujocoGatewayExtension::PublishMatch(uint64_t now_us) {
  match_ = ::sim::MatchState{};
  if (game_ != nullptr) game_->Report(match_);
  // The tick's node-clock time, the clock /sim/state is stamped on.
  match_.mutate_sample_time_ns(now_us * 1000);
  match_.mutate_shots_count(
      shooters_ != nullptr ? static_cast<uint32_t>(shooters_->shots()) : 0);
  if (!publisher_) return;
  const auto status = publisher_->write(match_);
  if (status == WriteResult::SUCCESS) {
    ++match_count_.samples;
    match_count_.sequence = status.sequence;
    match_count_.last_ns = static_cast<int64_t>(now_us) * 1000;
  } else {
    ++match_count_.dropped;
  }
}

void MujocoGatewayExtension::Finish() {
  if (plant_ == nullptr || backend_ == nullptr) return;
  const auto& time = backend_->plant_time();
  char line[256];
  std::snprintf(line, sizeof(line),
                "mujoco_gateway: stepped_s=%.3f dropped_s=%.3f "
                "dropped_ticks=%llu overspeed_excursions=%llu period=%s "
                "blue_points=%u red_points=%u shots=%u\n",
                static_cast<double>(time.stepped_us) * 1e-6,
                static_cast<double>(time.dropped_us) * 1e-6,
                static_cast<unsigned long long>(time.dropped_ticks),
                static_cast<unsigned long long>(plant_->overspeed_excursions()),
                ::sim::EnumNameMatchPeriod(match_.period()),
                match_.blue_points_count(), match_.red_points_count(),
                match_.shots_count());
  *options_.summary << line << std::flush;
}

}  // namespace talos::simulation::mujoco
