#include "talOS/telemetry/config_validation.h"

#include <fnmatch.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/introspection/names.h"
#include "talOS/telemetry/channels.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"
#include "toml.hpp"

namespace talos::telemetry {
namespace {

constexpr std::string_view kReadmeHint = " (see talOS/telemetry/README.md)";

[[noreturn]] void RefuseLabel(const std::string& key, std::string_view field,
                              const std::string& why) {
  throw std::invalid_argument("telemetry: " + key + "." + std::string{field} +
                              " " + why + std::string{kReadmeHint});
}

}  // namespace

std::size_t AppendValidatedChannel(
    std::vector<ChannelDef>& dictionary,
    const std::vector<std::string>& exclude_globs, std::string name,
    ChannelKind kind, std::string unit, std::vector<std::string> labels,
    AlertLevel alert) {
  if (MatchesAnyGlob(name, exclude_globs)) {
    return std::numeric_limits<std::size_t>::max();
  }
  for (const ChannelDef& existing : dictionary) {
    if (existing.name == name) {
      throw std::invalid_argument("telemetry: duplicate channel '" + name +
                                  "'");
    }
  }
  if (name.starts_with("derived/")) {
    throw std::invalid_argument("telemetry: channel '" + name +
                                "' is in Studio's reserved derived/ namespace" +
                                std::string{kReadmeHint});
  }
  const auto check_length = [&name](std::string_view what,
                                    std::string_view text) {
    if (text.size() > telemetry_wire::kMaxDictionaryStringBytes) {
      throw std::invalid_argument(
          "telemetry: channel '" + name + "' " + std::string{what} + " is " +
          std::to_string(text.size()) + " bytes, over the " +
          std::to_string(telemetry_wire::kMaxDictionaryStringBytes) +
          "-byte dictionary string limit" + std::string{kReadmeHint});
    }
  };
  check_length("name", name);
  check_length("unit", unit);
  for (const std::string& label : labels) check_length("label", label);
  dictionary.push_back(ChannelDef{std::move(name), std::move(unit),
                                  std::move(labels), kind, alert});
  return dictionary.size() - 1;
}

void CheckDictionaryChannelCap(
    const std::vector<ChannelDef>& dictionary,
    const std::map<std::string, std::size_t>& uncounted) {
  std::size_t total = dictionary.size();
  for (const auto& [topic, count] : uncounted) total += count;
  if (total <= telemetry_wire::kMaxChannels) return;
  std::map<std::string, std::size_t> per_topic = uncounted;
  for (const ChannelDef& channel : dictionary) {
    const std::size_t owner = channel.name.find('/');
    const std::size_t role = owner == std::string::npos
                                 ? std::string::npos
                                 : channel.name.find('/', owner + 1);
    ++per_topic[channel.name.substr(0, role)];
  }
  std::vector<std::pair<std::size_t, std::string>> largest;
  largest.reserve(per_topic.size());
  for (const auto& [topic, count] : per_topic) {
    largest.emplace_back(count, topic);
  }
  std::sort(largest.begin(), largest.end(), std::greater<>{});
  std::string names;
  for (std::size_t i = 0; i < largest.size() && i < 3; ++i) {
    if (i != 0) names += ", ";
    names += largest[i].second + " (" + std::to_string(largest[i].first) + ")";
  }
  throw std::runtime_error(
      "telemetry: " + std::to_string(total) + " channels, over the " +
      std::to_string(telemetry_wire::kMaxChannels) +
      "-channel cap; largest sources: " + names +
      "; exclude channels in telemetry.toml" + std::string{kReadmeHint});
}

bool MatchesAnyGlob(std::string_view name,
                    const std::vector<std::string>& patterns) {
  const std::string unslashed{
      !name.empty() && name.front() == '/' ? name.substr(1) : name};
  const std::string full_str = "/" + unslashed;
  for (const auto& pattern : patterns) {
    if (::fnmatch(pattern.c_str(), full_str.c_str(), 0) == 0 ||
        ::fnmatch(pattern.c_str(), unslashed.c_str(), 0) == 0) {
      return true;
    }
  }
  return false;
}

bool MatchesGlobs(std::string_view topic,
                  const std::vector<std::string>& includes,
                  const std::vector<std::string>& excludes) {
  return !MatchesAnyGlob(topic, excludes) && MatchesAnyGlob(topic, includes);
}

std::vector<LabelSpec> ParseLabelSpecs(const config::RobotConfig& config,
                                       std::string_view subsystem,
                                       const LabelParamHooks& hooks) {
  const toml::table* table = config.GetSubsystemTable(std::string{subsystem});
  if (table == nullptr || !table->contains("labels")) {
    if (hooks.on_own_string) {
      hooks.on_own_string("labels[0].channel");
      hooks.on_own_string("labels[0].from");
    }
    return {};
  }

  const toml::node* labels_node = table->get("labels");
  const toml::array* labels_array =
      labels_node != nullptr ? labels_node->as_array() : nullptr;
  if (labels_array == nullptr) {
    throw std::invalid_argument("telemetry: labels must be an array of tables" +
                                std::string{kReadmeHint});
  }
  if (labels_array->empty()) {
    if (hooks.on_own_string) {
      hooks.on_own_string("labels[0].channel");
      hooks.on_own_string("labels[0].from");
    }
    return {};
  }

  std::vector<LabelSpec> specs;
  specs.reserve(labels_array->size());
  for (std::size_t i = 0; i < labels_array->size(); ++i) {
    const std::string key = "labels[" + std::to_string(i) + "]";
    const toml::table* entry = (*labels_array)[i].as_table();
    if (entry == nullptr) {
      throw std::invalid_argument("telemetry: " + key + " must be a table" +
                                  std::string{kReadmeHint});
    }
    for (const auto& [field_key, ignored] : *entry) {
      static_cast<void>(ignored);
      const std::string_view field = field_key.str();
      if (field != "channel" && field != "from") {
        RefuseLabel(key, field, "is an unknown key (known: channel, from)");
      }
    }

    const toml::node* channel_node = entry->get("channel");
    if (channel_node == nullptr) RefuseLabel(key, "channel", "is missing");
    const auto channel_opt = channel_node->value_exact<std::string>();
    if (!channel_opt) RefuseLabel(key, "channel", "must be a string");
    const std::string& channel = *channel_opt;
    if (channel.empty()) RefuseLabel(key, "channel", "is missing");

    const toml::node* from_node = entry->get("from");
    if (from_node == nullptr) RefuseLabel(key, "from", "is missing");
    const auto from_opt = from_node->value_exact<std::string>();
    if (!from_opt) RefuseLabel(key, "from", "must be a string");
    const std::string& from = *from_opt;
    if (from.empty()) RefuseLabel(key, "from", "is missing");

    const std::size_t dot = from.find('.');
    if (dot == 0 || dot == std::string::npos || dot + 1 == from.size()) {
      RefuseLabel(key, "from", "'" + from + "' is not <subsystem>.<key>");
    }
    const std::string src_subsystem = from.substr(0, dot);
    const std::string src_key = from.substr(dot + 1);

    const toml::table* src_table = config.GetSubsystemTable(src_subsystem);
    if (src_table == nullptr) {
      RefuseLabel(key, "from",
                  "'" + from + "': no [subsystems." + src_subsystem +
                      "] in the config");
    }
    const toml::node_view<const toml::node> src_val =
        src_table->at_path(src_key);
    if (!src_val) {
      RefuseLabel(key, "from",
                  "'" + from + "': parameter '" + src_key + "' is missing");
    }
    const toml::array* src_array = src_val.as_array();
    if (src_array == nullptr) {
      RefuseLabel(key, "from", "'" + from + "': must be an array of strings");
    }
    std::vector<std::string> labels;
    labels.reserve(src_array->size());
    for (const toml::node& element : *src_array) {
      const auto text = element.value_exact<std::string>();
      if (!text) {
        RefuseLabel(key, "from", "'" + from + "': must be an array of strings");
      }
      labels.push_back(*text);
    }
    if (labels.empty()) {
      RefuseLabel(key, "from", "'" + from + "' holds no labels");
    }
    if (labels.size() > telemetry_wire::kMaxEnumLabels) {
      RefuseLabel(key, "from",
                  "'" + from + "' holds " + std::to_string(labels.size()) +
                      " labels, over the " +
                      std::to_string(telemetry_wire::kMaxEnumLabels) +
                      "-label limit");
    }
    for (const std::string& label : labels) {
      if (label.size() > telemetry_wire::kMaxDictionaryStringBytes) {
        RefuseLabel(
            key, "from",
            "'" + from + "' holds a label of " + std::to_string(label.size()) +
                " bytes, over the " +
                std::to_string(telemetry_wire::kMaxDictionaryStringBytes) +
                "-byte dictionary string limit");
      }
    }

    if (hooks.on_own_string) {
      hooks.on_own_string(key + ".channel");
      hooks.on_own_string(key + ".from");
    }
    if (hooks.on_from_string_list) {
      hooks.on_from_string_list(src_subsystem, src_key);
    }
    specs.push_back(LabelSpec{key, channel, std::move(labels)});
  }
  return specs;
}

void ApplyLabelSpecs(std::vector<LabelSpec>& specs,
                     std::vector<ChannelDef>& dictionary) {
  for (LabelSpec& spec : specs) {
    const auto refuse = [&spec](const std::string& why) {
      throw std::invalid_argument("telemetry: " + spec.key + ".channel '" +
                                  spec.channel + "' " + why +
                                  std::string{kReadmeHint});
    };
    const auto it = std::find_if(
        dictionary.begin(), dictionary.end(),
        [&spec](const ChannelDef& def) { return def.name == spec.channel; });
    if (it == dictionary.end()) {
      refuse("is not a channel of this dictionary");
    }
    if (it->kind != Talos::Telemetry::ChannelKind_Int &&
        it->kind != Talos::Telemetry::ChannelKind_UInt) {
      refuse("is not an integer channel");
    }
    if (!it->labels.empty()) refuse("already has labels");
    it->labels = std::move(spec.labels);
  }
  specs.clear();
}

void CheckDashboardAgainstDictionary(
    const DashboardLayout& layout, const std::vector<ChannelDef>& dictionary) {
  std::set<std::string_view> present;
  for (const ChannelDef& def : dictionary) present.insert(def.name);
  const std::string file = layout.file.string();
  const auto require = [&](std::size_t index, const std::string& key,
                           const std::string& name, const std::string& what) {
    if (present.contains(name)) return;
    throw DashboardLayoutError(
        file + ": widgets[" + std::to_string(index) + "]." + key + ": '" +
        name + "'" + what +
        " is not a channel of this dictionary (excluded, or no state or "
        "status topic holds it; see talOS/telemetry/README.md)");
  };
  for (std::size_t i = 0; i < layout.widgets.size(); ++i) {
    const DashboardWidget& widget = layout.widgets[i];
    const bool fixed = widget.kind == DashboardWidgetKind::kMatch ||
                       widget.kind == DashboardWidgetKind::kPower;
    for (const DashboardChannel& channel : widget.channels) {
      if (fixed) {
        require(i, "kind", channel.name,
                ", which a " +
                    std::string(DashboardWidgetKindName(widget.kind)) +
                    " widget reads,");
      } else if (widget.kind == DashboardWidgetKind::kField &&
                 (channel.role == "x" || channel.role == "y" ||
                  channel.role == "yaw")) {
        require(i, "pose", channel.name, "");
      } else {
        require(i, channel.role, channel.name, "");
      }
    }
    for (std::size_t k = 0; k < widget.series.size(); ++k) {
      require(i, "channels[" + std::to_string(k) + "]", widget.series[k], "");
    }
    for (std::size_t j = 0; j < widget.items.size(); ++j) {
      require(i, "items[" + std::to_string(j) + "].channel",
              widget.items[j].channel, "");
    }
  }
  const DriveChannels feed = DriveFeedChannels(layout);
  const auto carried = static_cast<std::size_t>(
      std::ranges::count_if(dictionary, [&feed](const ChannelDef& def) {
        return feed.Carries(def.name,
                            def.alert != Talos::Telemetry::AlertLevel_None);
      }));
  CheckDashboardBudget(layout, carried);
}

std::vector<std::string> ReadTelemetryGlobParam(
    const config::RobotConfig& config, std::string_view subsystem,
    std::string_view key, std::vector<std::string> fallback) {
  const toml::table* table = config.GetSubsystemTable(std::string{subsystem});
  if (table == nullptr) return fallback;
  const toml::node* node = table->get(key);
  if (node == nullptr) return fallback;
  const toml::array* array = node->as_array();
  if (array == nullptr) {
    throw std::invalid_argument("telemetry: " + std::string{key} +
                                " must be an array of strings" +
                                std::string{kReadmeHint});
  }
  std::vector<std::string> result;
  result.reserve(array->size());
  for (const toml::node& element : *array) {
    const auto text = element.value_exact<std::string>();
    if (!text) {
      throw std::invalid_argument("telemetry: " + std::string{key} +
                                  " must be an array of strings" +
                                  std::string{kReadmeHint});
    }
    result.push_back(*text);
  }
  return result;
}

std::vector<ChannelDef> BuildTelemetryDictionary(
    const introspect::DeclaredGraph& graph,
    const std::vector<std::string>& include_globs,
    const std::vector<std::string>& exclude_globs) {
  std::vector<ChannelDef> dictionary;
  AppendValidatedChannel(dictionary, exclude_globs, "pose/valid",
                         Talos::Telemetry::ChannelKind_Bool);

  std::set<std::string> seen_topics;
  for (const auto& node_desc : graph) {
    for (const auto& source : node_desc.sources) {
      if (source.kind != event::SourceKind::SENDER) continue;
      if (seen_topics.contains(source.name)) continue;

      const std::string_view role = introspect::naming::Role(source.name);
      if (role != "state" && role != "status") continue;

      if (!MatchesGlobs(source.name, include_globs, exclude_globs)) continue;
      seen_topics.insert(source.name);

      if (source.name == "/hw/state/driver_station") {
        for (const Channel& channel : DriverStationChannels()) {
          AppendValidatedChannel(dictionary, exclude_globs, channel.name,
                                 channel.kind, channel.unit, channel.labels,
                                 channel.alert);
        }
      } else if (!source.bfbs.empty()) {
        FlattenOptions options;
        options.keep = [&exclude_globs](std::string_view name) {
          return !MatchesAnyGlob(name, exclude_globs);
        };
        options.max_channels = telemetry_wire::kMaxChannels + 1;
        options.truncate = true;
        std::vector<Channel> channels =
            FlattenTopic(source.name, source.schema, source.bfbs, options);
        for (Channel& channel : channels) {
          AppendValidatedChannel(dictionary, exclude_globs,
                                 std::move(channel.name), channel.kind,
                                 std::move(channel.unit),
                                 std::move(channel.labels), channel.alert);
        }
      }
    }
  }
  return dictionary;
}

}  // namespace talos::telemetry
