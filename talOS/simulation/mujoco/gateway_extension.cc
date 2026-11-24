#include "talOS/simulation/mujoco/gateway_extension.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/hardware/config_error.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/sim_plant.h"

namespace talos::simulation::mujoco {

namespace {

using hardware::sim::PlantMotor;
using rules_cc::cc::runfiles::Runfiles;

// Each field model's runfiles path, from its package's `data` ($(rlocationpath)
// in BUILD), so a robot repository that names talOS @talos finds it too.
struct Field {
  std::string_view name;
  std::string_view rlocation;
};
constexpr std::array<Field, 1> kFields = {{
    {"rebuilt_2026", TALOS_MUJOCO_FIELD_REBUILT_2026},
}};
constexpr std::array<std::string_view, kFields.size()> kFieldNames = [] {
  std::array<std::string_view, kFields.size()> names{};
  for (std::size_t i = 0; i < kFields.size(); ++i) names[i] = kFields[i].name;
  return names;
}();

// Records the motors PhysicsSimBackend::Configure hands a plant (their gear
// and winding constants, as the backend computes them) and owns none, so the
// model compiles with exactly what the backend's Bind will compare against.
class MotorProbe final : public hardware::sim::ExternalPlant {
 public:
  uint32_t step_us() const override { return 5000; }
  bool Bind(std::span<const PlantMotor> motors,
            std::span<const hardware::sim::PlantInput> /*inputs*/,
            std::span<bool> /*owned*/,
            hardware::ConfigError& /*error*/) override {
    names_.assign(motors.size(), std::string());
    motors_.assign(motors.begin(), motors.end());
    for (std::size_t i = 0; i < motors.size(); ++i) {
      names_[i] = std::string(motors[i].name);
      motors_[i].name = names_[i];
    }
    bound_ = true;
    return true;
  }
  bool Step(int /*steps*/,
            std::span<const hardware::sim::PlantDrive> /*drives*/) override {
    return false;
  }
  hardware::sim::PlantRotor rotor(std::size_t /*motor*/) const override {
    return {};
  }
  std::optional<hardware::sim::PlantChassis> chassis() const override {
    return std::nullopt;
  }
  bool input_closed(std::size_t /*input*/) const override { return false; }
  std::string_view failure() const override { return {}; }

  bool bound() const { return bound_; }
  std::span<const PlantMotor> motors() const { return motors_; }

 private:
  std::vector<std::string> names_;  // motors_' names point here
  std::vector<PlantMotor> motors_;
  bool bound_{false};
};

std::string OwnsChassis(std::string_view key) {
  return std::string(key) +
         ": the robot model's root body has a freejoint, so the model owns "
         "the chassis and its wheels; remove it from a mujoco_gateway config";
}

// Why the backend's last Configure refused.
std::string RefusalText(const hardware::sim::PhysicsSimBackend& backend) {
  const std::optional<hardware::ConfigError> error =
      backend.last_config_error();
  return error ? hardware::FormatConfigError(*error)
               : std::string("the physics backend gave no reason");
}

constexpr int64_t kReportPeriodNs = 1'000'000'000;

}  // namespace

std::span<const std::string_view> GatewayFields() { return kFieldNames; }

FieldResolver RunfilesFieldResolver(std::string argv0) {
  return [argv0 = std::move(argv0)](std::string_view field) -> std::string {
    const Field* known = nullptr;
    for (const Field& entry : kFields) {
      if (entry.name == field) known = &entry;
    }
    if (known == nullptr) {
      throw std::invalid_argument("[simulation.mujoco] field = '" +
                                  std::string(field) +
                                  "': mujoco_gateway has no such field model");
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
        "mujoco_gateway simulates one (sim_gateway runs a config without "
        "[simulation.mujoco])");
  }
  const config::SimulationMujocoConfig& model = *config.simulation.mujoco;

  // The motors as the backend will bind them: one source for the gear and
  // winding constants the model's damping G^2*b_m is compiled from, and that
  // Bind then compares exactly. The backend has no public way to compute
  // them, so a probe plant records them from a Configure.
  MotorProbe probe;
  backend.SetPlant(&probe);
  backend.Configure(config.hardware);
  backend.SetPlant(nullptr);
  // Unreachable while the backend refuses nothing before Bind with no
  // world inputs set; kept so a future early refusal is not read as a model
  // that names no motor.
  if (!probe.bound()) {
    throw std::invalid_argument(
        "mujoco_gateway: the physics backend refused the config before "
        "binding its motors: " +
        RefusalText(backend));
  }

  const std::string field_xml =
      model.field.empty() ? std::string() : options_.field(model.field);
  plant_ = MakePlant(PlantModel{.field_xml = field_xml,
                                .robot_xml = model.robot_model,
                                .start_x_m = options.start_x_m,
                                .start_y_m = options.start_y_m,
                                .start_yaw_rad = options.start_yaw_rad},
                     probe.motors());

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

void MujocoGatewayExtension::AfterTick(uint64_t /*now_us*/) {
  if (plant_ == nullptr || backend_ == nullptr) return;
  if (!table_printed_) {  // the first tick, after the gateway bound the plant
    *options_.log << plant_->binding_table() << std::flush;
    table_printed_ = true;
  }
  if (const std::string_view failure = plant_->failure(); !failure.empty()) {
    ReportClock("the world failed");
    throw std::runtime_error("mujoco_gateway: " + std::string(failure));
  }
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

void MujocoGatewayExtension::Finish() {
  if (plant_ == nullptr || backend_ == nullptr) return;
  const auto& time = backend_->plant_time();
  char line[256];
  std::snprintf(
      line, sizeof(line),
      "mujoco_gateway: stepped_s=%.3f dropped_s=%.3f "
      "dropped_ticks=%llu overspeed_excursions=%llu\n",
      static_cast<double>(time.stepped_us) * 1e-6,
      static_cast<double>(time.dropped_us) * 1e-6,
      static_cast<unsigned long long>(time.dropped_ticks),
      static_cast<unsigned long long>(plant_->overspeed_excursions()));
  *options_.summary << line << std::flush;
}

}  // namespace talos::simulation::mujoco
