#include "talOS/simulation/mujoco/gateway_extension.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <limits>
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
#include "talOS/hardware/sim_rings.h"
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

// sim.GamePieces's three coordinate arrays and its in-play flags hold
// kMaxGamePieces entries after its 16 B header.
static_assert(sizeof(::sim::GamePieces) == 16 + kMaxGamePieces * 13);

// A body's name for a refusal: its own, else its id.
std::string BodyName(const mjModel& model, int body) {
  const char* name = mj_id2name(&model, mjOBJ_BODY, body);
  return name != nullptr && name[0] != '\0' ? std::string(name)
                                            : "body " + std::to_string(body);
}

struct ScopedHook {
  explicit ScopedHook(bool& active) : active_(active) { active_ = true; }
  ~ScopedHook() { active_ = false; }
  ScopedHook(const ScopedHook&) = delete;
  ScopedHook& operator=(const ScopedHook&) = delete;
  bool& active_;
};

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
  if (in_hook_) {
    throw std::logic_error(
        "MujocoGatewayExtension::Configure called reentrantly");
  }
  if (plant_ != nullptr) {
    throw std::logic_error(
        "MujocoGatewayExtension::Configure: already configured");
  }
  ScopedHook guard(in_hook_);
  if (!config.simulation.mujoco) {
    throw std::invalid_argument(
        "[simulation.mujoco] robot_model: the config sets no robot model; "
        "the MuJoCo gateway simulates one (sim_gateway runs a config without "
        "[simulation.mujoco])");
  }
  const config::SimulationMujocoConfig& model = *config.simulation.mujoco;
  if (!model.field.empty() && !options_.field) {
    throw std::invalid_argument("[simulation.mujoco] field = '" + model.field +
                                "': no field resolver was configured");
  }

  // The motors as the backend will bind them (config.hardware carries the
  // same local motor types the gateway hands SetLocalMotorTypes): one source
  // for the gear and winding constants the model's damping G^2*b_m is
  // compiled from, and that Bind then compares exactly. Their names view
  // into `config`, which outlives MakePlant; the plant keeps no name.
  const std::vector<PlantMotor> motors =
      hardware::sim::PhysicsSimBackend::PlantMotors(config.hardware);

  const std::string field_xml =
      model.field.empty() ? std::string() : options_.field(model.field);

  // Build all owned state in locals first (new_plant declared after new_rules
  // and new_game so exception unwind destroys new_plant before the rules it
  // references), and commit only once every check and publisher succeeds.
  std::vector<std::unique_ptr<WorldRules>> new_rules;
  std::unique_ptr<GameRules> new_game;
  const ScriptedShooters* new_shooters = nullptr;
  std::unique_ptr<MujocoPlant> new_plant =
      MakePlant(PlantModel{.field_xml = field_xml,
                           .robot_xml = model.robot_model,
                           .start_x_m = options.start_x_m,
                           .start_y_m = options.start_y_m,
                           .start_yaw_rad = options.start_yaw_rad,
                           .sleeping = model.sleeping},
                motors);

  if (new_plant->chassis()) {
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

  // The game pieces: every free body but the robot's root (the robot's
  // other bodies hang below it and have no free joint), in body order, at
  // most kMaxGamePieces, all of one bounding radius.
  std::vector<int> new_piece_bodies;
  const mjModel& compiled = *new_plant->model();
  for (int body = 1; body < compiled.nbody; ++body) {
    if (body == new_plant->robot_root_body() ||
        compiled.body_jntnum[body] < 1) {
      continue;
    }
    const int jnt = compiled.body_jntadr[body];
    if (jnt < 0 || jnt >= compiled.njnt ||
        compiled.jnt_type[jnt] != mjJNT_FREE) {
      continue;
    }
    new_piece_bodies.push_back(body);
  }
  if (new_piece_bodies.size() > kMaxGamePieces) {
    throw std::invalid_argument(
        "[simulation.mujoco] field = '" + model.field + "': the model has " +
        std::to_string(new_piece_bodies.size()) +
        " game pieces (free bodies besides the robot), more than the " +
        std::to_string(kMaxGamePieces) + " /sim/telemetry/pieces carries");
  }
  const auto radius_of = [&compiled](int body) {
    if (compiled.body_geomnum[body] <= 0) return 0.0F;
    const int geom = compiled.body_geomadr[body];
    if (geom < 0 || geom >= compiled.ngeom) return 0.0F;
    return static_cast<float>(compiled.geom_rbound[geom]);
  };
  const float first_radius =
      new_piece_bodies.empty() ? 0.0F : radius_of(new_piece_bodies.front());
  for (const int body : new_piece_bodies) {
    const float radius = radius_of(body);
    if (!std::isfinite(radius) || radius < 0.0F) {
      throw std::invalid_argument(
          "[simulation.mujoco] field = '" + model.field + "': game piece " +
          BodyName(compiled, body) + " has a non-finite or negative radius");
    }
    if (radius != first_radius) {
      throw std::invalid_argument(
          "[simulation.mujoco] field = '" + model.field + "': game pieces " +
          BodyName(compiled, new_piece_bodies.front()) + " and " +
          BodyName(compiled, body) +
          " differ in radius; /sim/telemetry/pieces carries one");
    }
  }
  ::sim::GamePieces new_pieces{};
  new_pieces.mutate_pieces_count(
      static_cast<uint32_t>(new_piece_bodies.size()));
  new_pieces.mutate_radius_m(first_radius);

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
  std::unique_ptr<WorldRules> shooters =
      MakeScriptedShooters(*new_plant->model());
  if (shooters != nullptr) {
    auto* typed = dynamic_cast<ScriptedShooters*>(shooters.get());
    new_plant->AddRules(*shooters);
    new_rules.push_back(std::move(shooters));
    new_shooters = typed;
  }
  if (const FieldEntry* field = FindField(model.field);
      field != nullptr && field->make_rules != nullptr) {
    new_game = field->make_rules(GameOptions{.seed = model.seed});
    new_plant->AddRules(*new_game);
  }
  const uint32_t new_pieces_every = PiecesEveryTicks(config.hardware.period_us);
  std::optional<ipc::Publisher<::sim::MatchState>> new_publisher;
  std::optional<ipc::Publisher<::sim::GamePieces>> new_pieces_publisher;
  if (!options_.tests_only_skip_writes) {
    new_publisher.emplace(::sim::topics::kMatchState);
    new_pieces_publisher.emplace(
        ::sim::topics::kPieces,
        RTMSOptions{.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                    .read_mode = ReadMode::SEQUENCE},
        hardware::sim::kPiecesSlots);
  }

  // Commit: nothing below throws.
  backend.SetPlant(new_plant.get());
  backend.SetWorldInputs(std::move(inputs));
  rules_ = std::move(new_rules);
  game_ = std::move(new_game);
  shooters_ = new_shooters;
  plant_ = std::move(new_plant);
  piece_bodies_ = std::move(new_piece_bodies);
  pieces_ = new_pieces;
  pieces_every_ = new_pieces_every;
  publisher_ = std::move(new_publisher);
  pieces_publisher_ = std::move(new_pieces_publisher);
  // The gateway's Reconfigure binds the plant next; a refusal there exits 1
  // naming the key and the motor or input.
  backend_ = &backend;
}

void MujocoGatewayExtension::ReportClock(std::string_view why) {
  if (options_.log == nullptr || backend_ == nullptr) return;
  const auto& time = backend_->plant_time();
  char line[256];
  const char* ptr = why.empty() ? "" : why.data();
  const int len = static_cast<int>(std::min(why.size(), sizeof(line)));
  std::snprintf(line, sizeof(line),
                "mujoco_gateway: %.*s: plant time %.3f s stepped, %.3f s "
                "dropped in %llu ticks\n",
                len, ptr, static_cast<double>(time.stepped_us) * 1e-6,
                static_cast<double>(time.dropped_us) * 1e-6,
                static_cast<unsigned long long>(time.dropped_ticks));
  *options_.log << line << std::flush;
}

event::Manifest MujocoGatewayExtension::Topics() const {
  return {
      {
          .kind = event::SourceKind::SENDER,
          .name = ::sim::topics::kMatchState,
          .message_bytes = sizeof(::sim::MatchState),
          .alignment = alignof(::sim::MatchState),
          .schema = schema::Of<::sim::MatchState>(),
      },
      {
          .kind = event::SourceKind::SENDER,
          .name = ::sim::topics::kPieces,
          .message_bytes = sizeof(::sim::GamePieces),
          .alignment = alignof(::sim::GamePieces),
          .schema = schema::Of<::sim::GamePieces>(),
      },
  };
}

uint32_t MujocoGatewayExtension::TopicSlots(std::size_t index) const {
  return index == 1 ? static_cast<uint32_t>(hardware::sim::kPiecesSlots) : 0;
}

void MujocoGatewayExtension::Counts(std::span<TopicCount> out) const {
  if (!out.empty()) out[0] = match_count_;
  if (out.size() > 1) out[1] = pieces_count_;
}

void MujocoGatewayExtension::BeforeTick(const TickInput& tick) {
  if (in_hook_) {
    throw std::logic_error(
        "MujocoGatewayExtension::BeforeTick called reentrantly");
  }
  ScopedHook guard(in_hook_);
  if (plant_ != nullptr) plant_->SetMatchInput(MatchInputOf(tick));
}

void MujocoGatewayExtension::Stamp(FmsStamp& fms) {
  if (game_ == nullptr) return;
  const FmsReport report = game_->Fms();
  fms.match_time_s =
      std::isfinite(report.match_time_s) ? report.match_time_s : 0.0F;
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
  if (in_hook_) {
    throw std::logic_error(
        "MujocoGatewayExtension::AfterTick called reentrantly");
  }
  if (plant_ == nullptr || backend_ == nullptr) return;
  ScopedHook guard(in_hook_);
  if (!table_printed_) {  // the first tick, after the gateway bound the plant
    if (options_.log != nullptr) {
      *options_.log << plant_->binding_table() << std::flush;
    }
    table_printed_ = true;
  }
  if (const std::string_view failure = plant_->failure(); !failure.empty()) {
    ReportClock("the world failed");
    throw std::runtime_error("mujoco_gateway: " + std::string(failure));
  }
  PublishMatch(now_us);
  if (ticks_ % pieces_every_ == 0) PublishPieces(now_us);
  ++ticks_;
  const auto& time = backend_->plant_time();
  if (time.dropped_us > reported_dropped_us_) {
    const int64_t now_ns =
        options_.wall_clock_ns ? options_.wall_clock_ns() : int64_t{0};
    if (!dilation_logged_ ||
        (now_ns >= dilation_logged_ns_ &&
         now_ns - dilation_logged_ns_ >= kReportPeriodNs)) {
      ReportClock("time dilation, the host ran slower than real time");
      reported_dropped_us_ = time.dropped_us;
      dilation_logged_ns_ = now_ns;
      dilation_logged_ = true;
    }
  }
  const uint64_t excursions = plant_->overspeed_excursions();
  if (excursions > reported_excursions_) {
    const int64_t now_ns =
        options_.wall_clock_ns ? options_.wall_clock_ns() : int64_t{0};
    if (!overspeed_logged_ ||
        (now_ns >= overspeed_logged_ns_ &&
         now_ns - overspeed_logged_ns_ >= kReportPeriodNs)) {
      if (options_.log != nullptr) {
        *options_.log << "mujoco_gateway: " << excursions
                      << " overspeed excursions so far (a bound rotor above "
                      << hardware::Gateway::kMaxRotorRps
                      << " rps; check the model's inertia and gear)\n"
                      << std::flush;
      }
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
  const uint64_t sample_time_ns = now_us * 1000;
  match_.mutate_sample_time_ns(sample_time_ns);
  const uint64_t shots = shooters_ != nullptr ? shooters_->shots() : 0;
  match_.mutate_shots_count(static_cast<uint32_t>(
      std::min<uint64_t>(shots, std::numeric_limits<uint32_t>::max())));
  if (!publisher_) return;
  const auto status = publisher_->write(match_);
  if (status == WriteResult::SUCCESS) {
    ++match_count_.samples;
    match_count_.sequence = status.sequence;
    match_count_.last_ns = static_cast<int64_t>(sample_time_ns);
  } else {
    ++match_count_.dropped;
  }
}

void MujocoGatewayExtension::PublishPieces(uint64_t now_us) {
  const mjData& data = *plant_->data();
  const uint64_t sample_time_ns = now_us * 1000;
  pieces_.mutate_sample_time_ns(sample_time_ns);
  auto& x = *pieces_.mutable_x_m();
  auto& y = *pieces_.mutable_y_m();
  auto& z = *pieces_.mutable_z_m();
  auto& in_play = *pieces_.mutable_in_play();
  for (std::size_t i = 0; i < piece_bodies_.size(); ++i) {
    const int body = piece_bodies_[i];
    const mjtNum* at = data.xpos + static_cast<std::ptrdiff_t>(body) * 3;
    const auto index = static_cast<flatbuffers::uoffset_t>(i);
    x.Mutate(index, static_cast<float>(at[0]));
    y.Mutate(index, static_cast<float>(at[1]));
    z.Mutate(index, static_cast<float>(at[2]));
    in_play.Mutate(index, game_ == nullptr || game_->InPlay(body));
  }
  if (!pieces_publisher_) return;
  const auto status = pieces_publisher_->write(pieces_);
  if (status == WriteResult::SUCCESS) {
    ++pieces_count_.samples;
    pieces_count_.sequence = status.sequence;
    pieces_count_.last_ns = static_cast<int64_t>(sample_time_ns);
  } else {
    ++pieces_count_.dropped;
  }
}

void MujocoGatewayExtension::Finish() {
  if (plant_ == nullptr || backend_ == nullptr || options_.summary == nullptr) {
    return;
  }
  const auto& time = backend_->plant_time();
  const char* period = ::sim::EnumNameMatchPeriod(match_.period());
  char line[256];
  std::snprintf(line, sizeof(line),
                "mujoco_gateway: stepped_s=%.3f dropped_s=%.3f "
                "dropped_ticks=%llu overspeed_excursions=%llu period=%s "
                "blue_points=%u red_points=%u shots=%u\n",
                static_cast<double>(time.stepped_us) * 1e-6,
                static_cast<double>(time.dropped_us) * 1e-6,
                static_cast<unsigned long long>(time.dropped_ticks),
                static_cast<unsigned long long>(plant_->overspeed_excursions()),
                period != nullptr ? period : "Unknown",
                match_.blue_points_count(), match_.red_points_count(),
                match_.shots_count());
  *options_.summary << line << std::flush;
}

}  // namespace talos::simulation::mujoco
