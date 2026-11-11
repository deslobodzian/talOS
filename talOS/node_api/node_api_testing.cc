// The test-harness section of node_api.h: talos::testing::Sim and
// SimHardware. Linked only into libtalos_node_testing.so.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>

#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/messages.h"
#include "talOS/node/sim_access.h"
#include "talOS/node/testing.h"
#include "talOS/node_api/internal.h"
#include "talOS/node_api/node_api.h"

namespace {

using namespace talos::node_api;
using talos::detail::SimAccess;
using talos::testing::Sim;
using talos::testing::SimHardware;

Sim& AsSim(TalosSim* sim) {
  if (sim == nullptr) throw std::invalid_argument("null TalosSim");
  return *reinterpret_cast<Sim*>(sim);
}
const Sim& AsSim(const TalosSim* sim) {
  if (sim == nullptr) throw std::invalid_argument("null TalosSim");
  return *reinterpret_cast<const Sim*>(sim);
}
TalosSim* AsHandle(Sim* sim) { return reinterpret_cast<TalosSim*>(sim); }

SimHardware& AsHardware(TalosSimHardware* hw) {
  if (hw == nullptr) throw std::invalid_argument("null TalosSimHardware");
  return *reinterpret_cast<SimHardware*>(hw);
}
const SimHardware& AsHardware(const TalosSimHardware* hw) {
  if (hw == nullptr) throw std::invalid_argument("null TalosSimHardware");
  return *reinterpret_cast<const SimHardware*>(hw);
}

}  // namespace

extern "C" {

// --- Sim ---------------------------------------------------------------------

TalosSim* talos_sim_create(const char* config_path) {
  return Guard<TalosSim*>(nullptr, [&] {
    return AsHandle(new Sim{NonNull(config_path, "config_path")});
  });
}

TalosSim* talos_sim_from_toml(const char* robot_toml) {
  return Guard<TalosSim*>(nullptr, [&] {
    return AsHandle(new Sim{Sim::FromToml(NonNull(robot_toml, "robot_toml"))});
  });
}

void talos_sim_destroy(TalosSim* sim) {
  Guard<int>(0, [&] {
    delete reinterpret_cast<Sim*>(sim);
    return 0;
  });
}

int32_t talos_sim_set_session_dir(TalosSim* sim, const char* path) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    Sim& s = AsSim(sim);
    if (path == nullptr || path[0] == '\0') {
      s.set_session_dir(std::nullopt);
    } else {
      s.set_session_dir(std::filesystem::path{path});
    }
    return TALOS_OK;
  });
}

TalosNode* talos_sim_add(TalosSim* sim, const char* node_name,
                         TalosFactory factory, void* user) {
  return Guard<TalosNode*>(nullptr, [&] {
    Sim& s = AsSim(sim);
    const char* name = NonNull(node_name, "node_name");
    SimAccess::add_instance(s, name, MakeFactory(factory, user));
    return talos::node_api::AsHandle(s.node(name));
  });
}

TalosNode* talos_sim_node(TalosSim* sim, const char* node_name) {
  return Guard<TalosNode*>(nullptr, [&] {
    return talos::node_api::AsHandle(
        AsSim(sim).node(NonNull(node_name, "node_name")));
  });
}

int32_t talos_sim_step(TalosSim* sim, int64_t duration_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsSim(sim).step(std::chrono::nanoseconds{duration_ns});
    return TALOS_OK;
  });
}

int32_t talos_sim_pause(TalosSim* sim, const char* node_name,
                        int64_t duration_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsSim(sim).pause(NonNull(node_name, "node_name"),
                     std::chrono::nanoseconds{duration_ns});
    return TALOS_OK;
  });
}

int32_t talos_sim_resume(TalosSim* sim, const char* node_name) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsSim(sim).resume(NonNull(node_name, "node_name"));
    return TALOS_OK;
  });
}

int64_t talos_sim_now_ns(const TalosSim* sim) {
  return Guard<int64_t>(0, [&] { return AsSim(sim).now().nanos(); });
}

int32_t talos_sim_publish_raw(TalosSim* sim, const char* topic,
                              const void* data, uint32_t size) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (data == nullptr) throw std::invalid_argument("null message");
    SimAccess::publish_raw(AsSim(sim), NonNull(topic, "topic"), data, size);
    return TALOS_OK;
  });
}

int32_t talos_sim_latest_raw(const TalosSim* sim, const char* topic, void* out,
                             uint32_t size) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    const Sim& s = AsSim(sim);
    const char* t = NonNull(topic, "topic");
    if (out == nullptr) throw std::invalid_argument("null output buffer");
    const std::uint64_t count = SimAccess::message_count(s, t, size);
    if (count == 0) return 0;
    SimAccess::read_raw(s, t, size, count - 1, out);
    return 1;
  });
}

int32_t talos_sim_latest_bytes(const TalosSim* sim, const char* topic,
                               void* out, uint32_t capacity,
                               uint32_t* out_size) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out_size == nullptr) throw std::invalid_argument("null out_size");
    *out_size = 0;
    const auto bytes = AsSim(sim).latest_bytes(NonNull(topic, "topic"));
    if (!bytes) return 0;
    *out_size = static_cast<uint32_t>(bytes->size());
    if (out != nullptr) {
      if (capacity < bytes->size()) {
        throw std::invalid_argument("output buffer too small");
      }
      if (!bytes->empty()) std::memcpy(out, bytes->data(), bytes->size());
    }
    return 1;
  });
}

uint64_t talos_sim_driver_station_generation(const TalosSim* sim) {
  return Guard<uint64_t>(
      0, [&] { return talos::testing::DriverStationGeneration(AsSim(sim)); });
}

int32_t talos_sim_publish_target(TalosSim* sim, const char* subsystem,
                                 const char* source, const void* message,
                                 uint32_t size, uint32_t alignment,
                                 const uint64_t* mode_since_us) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    talos::testing::Sim& s = AsSim(sim);
    talos::testing::PublishTargetRaw(
        s, NonNull(subsystem, "subsystem"), NonNull(source, "source"), message,
        size, alignment,
        mode_since_us != nullptr ? *mode_since_us
                                 : talos::testing::DriverStationGeneration(s));
    return TALOS_OK;
  });
}

int32_t talos_sim_latest_target(const TalosSim* sim, const char* subsystem,
                                const char* source, void* out, uint32_t size,
                                uint32_t alignment, int32_t* active) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out == nullptr || active == nullptr) {
      throw std::invalid_argument("null output");
    }
    talos::detail::TargetEnvelopeHeader header;
    if (!talos::testing::LatestTargetRaw(
            AsSim(sim), NonNull(subsystem, "subsystem"),
            NonNull(source, "source"), out, size, alignment, header)) {
      return 0;
    }
    *active = header.active ? 1 : 0;
    return 1;
  });
}

// --- SimHardware -------------------------------------------------------------

TalosSimHardware* talos_sim_hardware_create(TalosSim* sim) {
  return Guard<TalosSimHardware*>(nullptr, [&] {
    return reinterpret_cast<TalosSimHardware*>(new SimHardware{AsSim(sim)});
  });
}

void talos_sim_hardware_destroy(TalosSimHardware* hw) {
  Guard<int>(0, [&] {
    delete reinterpret_cast<SimHardware*>(hw);
    return 0;
  });
}

int32_t talos_sim_hardware_enable(TalosSimHardware* hw, int32_t on) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).enable(on != 0);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_ideal_motors(TalosSimHardware* hw, int32_t on) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).ideal_motors(on != 0);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_motor(
    TalosSimHardware* hw, const char* node, const char* device, double position,
    double velocity, uint32_t position_age_us, uint32_t velocity_age_us,
    double temperature_c, double supply_current_a, double stator_current_a) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_motor(NonNull(node, "node"), NonNull(device, "device"),
                             position, velocity, position_age_us,
                             velocity_age_us, temperature_c, supply_current_a,
                             stator_current_a);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_limit_switches(TalosSimHardware* hw,
                                              const char* node,
                                              const char* device,
                                              int32_t forward,
                                              int32_t reverse) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_limit_switches(NonNull(node, "node"),
                                      NonNull(device, "device"), forward != 0,
                                      reverse != 0);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_digital_input(TalosSimHardware* hw,
                                             const char* node,
                                             const char* device,
                                             int32_t value) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_digital_input(NonNull(node, "node"),
                                     NonNull(device, "device"), value != 0);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_analog_input(TalosSimHardware* hw,
                                            const char* node,
                                            const char* device, double volts) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_analog_input(NonNull(node, "node"),
                                    NonNull(device, "device"), volts);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_encoder(TalosSimHardware* hw, const char* node,
                                       const char* device, double position,
                                       double velocity) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_encoder(NonNull(node, "node"), NonNull(device, "device"),
                               position, velocity);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_cancoder(TalosSimHardware* hw, const char* node,
                                        const char* device, double position,
                                        double velocity,
                                        uint32_t position_age_us,
                                        uint32_t velocity_age_us) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_cancoder(NonNull(node, "node"),
                                NonNull(device, "device"), position, velocity,
                                position_age_us, velocity_age_us);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_gyro(TalosSimHardware* hw, const char* node,
                                    const char* device, double yaw,
                                    double yaw_rate, uint32_t yaw_age_us,
                                    uint32_t yaw_rate_age_us) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_gyro(NonNull(node, "node"), NonNull(device, "device"),
                            yaw, yaw_rate, yaw_age_us, yaw_rate_age_us);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_gateway_clock(TalosSimHardware* hw,
                                             int64_t gateway_offset_ns,
                                             int64_t latency_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_gateway_clock(
        std::chrono::nanoseconds{gateway_offset_ns},
        std::chrono::nanoseconds{latency_ns});
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_connected(TalosSimHardware* hw, const char* node,
                                         const char* device,
                                         int32_t connected) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).set_connected(NonNull(node, "node"),
                                 NonNull(device, "device"), connected != 0);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_reset_device(TalosSimHardware* hw, const char* node,
                                        const char* device) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).reset_device(NonNull(node, "node"),
                                NonNull(device, "device"));
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_lose_states(TalosSimHardware* hw, int32_t count) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).lose_states(count);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_reboot(TalosSimHardware* hw) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsHardware(hw).reboot();
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_set_driver_station(
    TalosSimHardware* hw, const TalosDriverStationPacket* packet) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (packet == nullptr) {
      AsHardware(hw).stop_driver_station();
      return TALOS_OK;
    }
    namespace ds = talos::driver_station;
    ds::DriverStationData data{};
    data.fms.flags = packet->flags;
    data.fms.alliance = static_cast<ds::Alliance>(packet->alliance);
    data.fms.station = static_cast<std::uint8_t>(packet->station);
    data.fms.match_time_s = packet->match_time_s;
    for (std::size_t port = 0; port < ds::kMaxJoysticks; ++port) {
      const TalosJoystick& in = packet->joysticks[port];
      ds::JoystickData& out = data.joysticks[port];
      out.connected = in.connected != 0;
      out.axis_count = static_cast<std::uint8_t>(
          std::clamp<int32_t>(in.axis_count, 0, ds::kMaxAxes));
      out.button_count = static_cast<std::uint8_t>(
          std::clamp<int32_t>(in.button_count, 0, 32));
      out.pov_count = static_cast<std::uint8_t>(
          std::clamp<int32_t>(in.pov_count, 0, ds::kMaxPovs));
      out.buttons = in.buttons;
      std::copy(std::begin(in.axes), std::end(in.axes), out.axes.begin());
      std::transform(
          std::begin(in.povs), std::end(in.povs), out.povs.begin(),
          [](int32_t pov) { return static_cast<std::int16_t>(pov); });
      std::memcpy(out.name, in.name, sizeof out.name);
    }
    AsHardware(hw).set_driver_station(data);
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_motor_request(const TalosSimHardware* hw,
                                         const char* node, const char* device,
                                         TalosMotorRequest* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out == nullptr) throw std::invalid_argument("null TalosMotorRequest");
    const talos::hardware::MotorRequest request = AsHardware(hw).motor_request(
        NonNull(node, "node"), NonNull(device, "device"));
    *out = TalosMotorRequest{
        .mode = static_cast<int32_t>(request.mode),
        .slot = request.slot,
        .demand = request.demand,
        .ff_volts = request.feedforward_v,
    };
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_digital_output(const TalosSimHardware* hw,
                                          const char* node,
                                          const char* device) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    return AsHardware(hw).digital_output(NonNull(node, "node"),
                                         NonNull(device, "device"))
               ? 1
               : 0;
  });
}

int32_t talos_sim_hardware_pwm(const TalosSimHardware* hw, const char* node,
                               const char* device, double* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out == nullptr) throw std::invalid_argument("null output");
    *out = AsHardware(hw).pwm(NonNull(node, "node"), NonNull(device, "device"));
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_applied_motor(const TalosSimHardware* hw,
                                         const char* node, const char* device,
                                         TalosMotorRequest* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out == nullptr) throw std::invalid_argument("null TalosMotorRequest");
    const talos::hardware::MotorRequest request = AsHardware(hw).applied_motor(
        NonNull(node, "node"), NonNull(device, "device"));
    *out = TalosMotorRequest{
        .mode = static_cast<int32_t>(request.mode),
        .slot = request.slot,
        .demand = request.demand,
        .ff_volts = request.feedforward_v,
    };
    return TALOS_OK;
  });
}

int32_t talos_sim_hardware_applied_digital_output(const TalosSimHardware* hw,
                                                  const char* node,
                                                  const char* device) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    return AsHardware(hw).applied_digital_output(NonNull(node, "node"),
                                                 NonNull(device, "device"))
               ? 1
               : 0;
  });
}

int32_t talos_sim_hardware_applied_pwm(const TalosSimHardware* hw,
                                       const char* node, const char* device,
                                       double* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out == nullptr) throw std::invalid_argument("null output");
    *out = AsHardware(hw).applied_pwm(NonNull(node, "node"),
                                      NonNull(device, "device"));
    return TALOS_OK;
  });
}

}  // extern "C"
