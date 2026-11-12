#include "studio/review/hardware_channels.h"

#include <limits>
#include <string>

#include "nlohmann/json.hpp"

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

void Flatten(const json& value, const std::string& name,
             std::vector<HardwareSample>& out);

void FlattenObject(const json& object, const std::string& prefix,
                   std::vector<HardwareSample>& out) {
  for (const auto& [key, member] : object.items()) {
    // The element's own key is its path segment; its name is a string.
    if (key == "id" || key == "port") continue;
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
