#include "studio/review/hardware_channels.h"

#include <limits>
#include <optional>
#include <string>
#include <string_view>

#include "nlohmann/json.hpp"
#include "talOS/hardware/messages.h"

namespace studio::review {
namespace {

using nlohmann::json;

// The key an array element of objects is named by: a device's logical id,
// a joystick's port; empty when it has neither (it is then named by index).
std::string ElementKey(const json& element, std::size_t index) {
  for (const char* key : {"id", "port"}) {
    const auto it = element.find(key);
    if (it != element.end() && it->is_number_integer()) {
      return std::to_string(it->get<std::int64_t>());
    }
  }
  return std::to_string(index);
}

// Whether `prefix` names an element of a `pwm_outputs` array:
// ".../pwm_outputs/<id>".
bool IsPwmOutput(std::string_view prefix) {
  const auto slash = prefix.rfind('/');
  return slash != std::string_view::npos &&
         prefix.substr(0, slash).ends_with("/pwm_outputs");
}

// The wire value (hardware::PwmMode) of a PWM request's mode as log_dump
// names it (talOS/launcher/hardware_record.cc); none for another name.
std::optional<talos::hardware::PwmMode> PwmModeNamed(std::string_view name) {
  if (name == "off") return talos::hardware::PwmMode::kOff;
  if (name == "pulse") return talos::hardware::PwmMode::kPulse;
  return std::nullopt;
}

void Flatten(const json& value, const std::string& name,
             std::vector<HardwareSample>& out);

void FlattenObject(const json& object, const std::string& prefix,
                   std::vector<HardwareSample>& out) {
  for (const auto& [key, member] : object.items()) {
    // The element's own key is its path segment; its name is a string.
    if (key == "id" || key == "port") continue;
    // A PWM request's mode is a channel of its wire value, so off (no
    // pulse) and a centre pulse, both an output of 0, read apart.
    if (key == "mode" && member.is_string() && IsPwmOutput(prefix)) {
      if (const auto mode = PwmModeNamed(member.get<std::string>())) {
        out.push_back({prefix + "/mode", Talos::Telemetry::ChannelKind_Double,
                       talos::telemetry::RawFromDouble(
                           static_cast<double>(static_cast<int>(*mode))),
                       false});
      }
      continue;
    }
    Flatten(member, prefix + "/" + key, out);
  }
}

void Flatten(const json& value, const std::string& name,
             std::vector<HardwareSample>& out) {
  switch (value.type()) {
    case json::value_t::boolean:
      out.push_back({name, Talos::Telemetry::ChannelKind_Bool,
                     value.get<bool>() ? 1u : 0u, false});
      return;
    case json::value_t::number_integer:
    case json::value_t::number_unsigned:
    case json::value_t::number_float:
      out.push_back({name, Talos::Telemetry::ChannelKind_Double,
                     talos::telemetry::RawFromDouble(value.get<double>()),
                     false});
      return;
    case json::value_t::null:
      out.push_back({name, Talos::Telemetry::ChannelKind_Double,
                     talos::telemetry::RawFromDouble(
                         std::numeric_limits<double>::quiet_NaN()),
                     true});
      return;
    case json::value_t::object:
      FlattenObject(value, name, out);
      return;
    case json::value_t::array:
      for (std::size_t i = 0; i < value.size(); ++i) {
        const json& element = value[i];
        Flatten(element,
                name + "/" +
                    (element.is_object() ? ElementKey(element, i)
                                         : std::to_string(i)),
                out);
      }
      return;
    default:  // strings, binary, discarded
      return;
  }
}

}  // namespace

void FlattenHardware(std::string_view topic,
                     talos::launcher::HardwareRecord kind,
                     std::span<const std::byte> payload,
                     std::vector<HardwareSample>& out) {
  out.clear();
  const json decoded = json::parse(
      talos::launcher::HardwareRecordJson(kind, payload, nullptr), nullptr,
      /*allow_exceptions=*/false);
  if (!decoded.is_object() || decoded.contains("error")) return;
  std::string prefix{topic};
  if (prefix.starts_with('/')) prefix.erase(0, 1);
  for (const auto& [key, member] : decoded.items()) {
    if (key == "type") continue;
    Flatten(member, prefix + "/" + key, out);
  }
}

}  // namespace studio::review
