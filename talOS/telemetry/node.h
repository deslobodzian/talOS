#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"
#include "talOS/schema/field_path.h"
#include "talOS/schema/identity.h"
#include "talOS/telemetry/channels.h"
#include "talOS/telemetry/config_validation.h"
#include "talOS/telemetry/dashboard_layout.h"
#include "talOS/telemetry/frames.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {

// How the telemetry node opens its byte topic. The stream is for viewers,
// not a record (each node's event log is), so it overwrites: a stalled or
// stopped reader is lapped and counts the frames it missed, and never makes
// the writer refuse frames for every other reader.
inline constexpr BytePublishOptions kTopicOptions{
    .slots = telemetry_wire::slot_count,
    .drop_newest = false,
    .external = true};

class Telemetry {
 public:
  explicit Telemetry(Node& node)
      : node_{node},
        stale_after_{std::chrono::microseconds{
            node.param<std::int64_t>("stale_timeout_us", 100'000)}},
        include_globs_{node.param<std::vector<std::string>>(
            "include", std::vector<std::string>{"*"})},
        exclude_globs_{node.param<std::vector<std::string>>(
            "exclude", std::vector<std::string>{})},
        labels_{ReadLabels(node)},
        topic_{ResolveTopic(node)},
        pose_topic_{ResolvePoseTopic(node)},
        studio_{node.publish_bytes(topic_, telemetry_wire::slot_bytes,
                                   kTopicOptions)} {
    using talos::detail::NodeAccess;

    idx_pose_valid_ =
        AddChannel("pose/valid", Talos::Telemetry::ChannelKind_Bool);

    const introspect::DeclaredGraph* graph = NodeAccess::declared_graph(node);
    if (graph != nullptr) {
      BuildFromGraph(*graph);
      // The first --describe probe has no graph, so no channels to label;
      // ReadLabels has checked every `from` already.
      ApplyLabels();
    }

    CheckChannelCap();
    // The driver dashboard's layout ([studio] dashboard): the first
    // --describe probe has no dictionary, so it checks only what the file
    // alone decides.
    const std::string& dashboard =
        NodeAccess::robot_config(node).studio.dashboard;
    if (!dashboard.empty()) {
      const DashboardLayout layout = ParseDashboardLayout(dashboard);
      if (graph != nullptr) CheckDashboard(layout);
    }
    dictionary_id_ = DictionaryId(dictionary_);
    stream_id_ = RandomStreamId();
    // update() fills these without reallocating.
    tick_channels_.reserve(dictionary_.size());
    tick_values_.reserve(dictionary_.size());
    dictionary_scratch_.entries.reserve(dictionary_.size());
    dictionary_scratch_.labels.reserve(telemetry_wire::kMaxEnumLabels);

    ReportPages(graph != nullptr);
  }

  std::uint64_t stream_id() const { return stream_id_; }
  std::uint32_t dictionary_id() const { return dictionary_id_; }
  const std::vector<ChannelDef>& dictionary() const { return dictionary_; }

  void update() {
    using talos::detail::NodeAccess;

    bool pose_ok = false;
    Talos::Telemetry::Pose chassis{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (has_pose_) {
      const bool fresh =
          NodeAccess::subscription_fresh(node_, pose_sub_index_, stale_after_);
      if (fresh) {
        const auto* raw = static_cast<const std::uint8_t*>(
            NodeAccess::subscription_value(node_, pose_sub_index_));
        bool valid = true;
        if (pose_has_valid_field_) {
          valid = *(raw + pose_valid_offset_) != 0;
        }
        double x_m = 0.0;
        double y_m = 0.0;
        double yaw_rad = 0.0;
        std::memcpy(&x_m, raw + pose_x_offset_, sizeof(double));
        std::memcpy(&y_m, raw + pose_y_offset_, sizeof(double));
        std::memcpy(&yaw_rad, raw + pose_yaw_offset_, sizeof(double));
        if (valid && std::isfinite(x_m) && std::isfinite(y_m) &&
            std::isfinite(yaw_rad)) {
          pose_ok = true;
          chassis = {x_m, y_m, 0.0, 0.0, 0.0, yaw_rad};
        }
      }
    }

    bool any_source_fresh = pose_ok;
    for (const auto& topic : observed_topics_) {
      if (NodeAccess::subscription_fresh(node_, topic.sub_index,
                                         stale_after_)) {
        any_source_fresh = true;
        break;
      }
    }

    if (!any_source_fresh && !last_frame_had_fresh_) return;
    last_frame_had_fresh_ = any_source_fresh;

    tick_channels_.clear();
    tick_values_.clear();

    SetBool(idx_pose_valid_, pose_ok);

    for (const auto& topic : observed_topics_) {
      const bool fresh =
          NodeAccess::subscription_fresh(node_, topic.sub_index, stale_after_);
      if (!fresh) continue;

      const std::span<const std::byte> message{
          static_cast<const std::byte*>(
              NodeAccess::subscription_value(node_, topic.sub_index)),
          topic.message_bytes};

      if (topic.is_driver_station) {
        std::array<std::uint64_t, kDriverStationChannelCount> values{};
        if (ReadDriverStation(message, values)) {
          for (std::size_t i = 0; i < values.size(); ++i) {
            SetRaw(topic.ds_channels[i], values[i]);
          }
        }
      } else {
        for (const auto& reader : topic.readers) {
          std::uint64_t value = 0;
          if (ReadChannel(reader.channel, message, value)) {
            SetRaw(reader.channel_index, value);
          }
        }
      }
    }

    const PageHeader header{
        .stream_id = stream_id_,
        .sequence_id = sequence_,
        .timestamp_ns = static_cast<std::uint64_t>(node_.now().nanos()),
        .dictionary_id = dictionary_id_,
        .channel_count = static_cast<std::uint32_t>(dictionary_.size())};
    const auto send = [this](std::span<const std::uint8_t> page) {
      studio_.send(std::as_bytes(page));
    };
    // The dictionary on the first tick and every 100th, before its values.
    if (sequence_ % 100 == 0) {
      EncodeDictionaryPages(builder_, dictionary_scratch_, header, dictionary_,
                            send);
    }
    // A producer tick is a keyframe: a channel it does not carry is absent.
    // It cannot fail: every page is bounded (frames_test) and the dictionary
    // is within kMaxChannels (construction).
    const bool encoded = EncodeValuePages(
        builder_, header, chassis, true, tick_channels_, tick_values_, {},
        [&send](std::span<const std::uint8_t> page, const PageSpan&) {
          send(page);
        });
    static_cast<void>(encoded);
    ++sequence_;
  }

 private:
  // A dictionary channel and the scalar it reads (channels.h).
  struct FieldReader {
    std::size_t channel_index;
    Channel channel;
  };

  struct ObservedTopic {
    std::string name;
    std::size_t sub_index;
    std::size_t message_bytes{0};
    std::vector<FieldReader> readers;
    bool is_driver_station = false;
    // Each DriverStationChannels() channel's dictionary index, in order.
    std::array<std::size_t, kDriverStationChannelCount> ds_channels{};
  };

  // Every [[subsystem.labels]] entry, its `from` read with param_of via
  // ParseLabelSpecs (config_validation.h), recording parameter reads for
  // --describe / LintParameterKeys.
  static std::vector<LabelSpec> ReadLabels(Node& node) {
    using talos::detail::NodeAccess;
    return ParseLabelSpecs(
        NodeAccess::robot_config(node), node.name(),
        LabelParamHooks{
            .on_own_string =
                [&node](std::string_view key) {
                  static_cast<void>(node.param<std::string>(key, ""));
                },
            .on_from_string_list =
                [&node](std::string_view subsystem, std::string_view key) {
                  static_cast<void>(
                      node.param_of<std::vector<std::string>>(subsystem, key));
                },
        });
  }

  void ApplyLabels() { ApplyLabelSpecs(labels_, dictionary_); }

  void CheckDashboard(const DashboardLayout& layout) const {
    CheckDashboardAgainstDictionary(layout, dictionary_);
  }

  static std::string ResolveTopic(Node& node) {
    using talos::detail::NodeAccess;
    return NodeAccess::robot_config(node).studio.topic;
  }

  static std::string ResolvePoseTopic(Node& node) {
    using talos::detail::NodeAccess;
    return NodeAccess::robot_config(node).studio.pose_topic;
  }

  static std::uint64_t RandomStreamId() {
    std::random_device device;
    const std::uint64_t id =
        (static_cast<std::uint64_t>(device()) << 32) | device();
    return id != 0 ? id : 1;
  }

  // Carries channel `index` this tick; an excluded channel (index past the
  // dictionary) carries nothing.
  void SetRaw(std::size_t index, std::uint64_t raw) {
    if (index >= dictionary_.size()) return;
    tick_channels_.push_back(static_cast<std::uint32_t>(index));
    tick_values_.push_back(raw);
  }
  void SetBool(std::size_t index, bool value) { SetRaw(index, value ? 1 : 0); }

  std::size_t AddChannel(std::string name, ChannelKind kind,
                         std::string unit = "",
                         std::vector<std::string> labels = {},
                         AlertLevel alert = Talos::Telemetry::AlertLevel_None) {
    return AppendValidatedChannel(dictionary_, exclude_globs_, std::move(name),
                                  kind, std::move(unit), std::move(labels),
                                  alert);
  }

  // Studio's bridge sends a client an update only whole, so the largest one,
  // a keyframe of every channel, must fit its burst: refuse a dictionary past
  // kMaxChannels, naming the topics with the most channels to exclude.
  void CheckChannelCap() const {
    CheckDictionaryChannelCap(dictionary_, uncounted_);
  }

  // Pages the dictionary once, which throws, naming the channel, when one
  // channel's entry cannot fit a dictionary page alone. For a graph-built
  // dictionary (the launcher's --describe-only and every launch) it reports
  // the pages a tick takes, so a config author sees the cost of a channel.
  void ReportPages(bool report) const {
    const PageBudget budget = MeasurePages(dictionary_);
    if (!report) return;
    std::cerr << "telemetry: " << dictionary_.size() << " channels; "
              << budget.value_pages << " value page(s) per tick, "
              << budget.value_bytes << " B worst case (largest "
              << budget.largest_value_page << " of "
              << telemetry_wire::kMaxValueFrameBytes << " B); dictionary "
              << budget.dictionary_pages << " page(s), "
              << budget.dictionary_bytes << " B\n";
  }

  void BuildFromGraph(const introspect::DeclaredGraph& graph) {
    using talos::detail::NodeAccess;

    // Check if pose_topic is defined in graph
    if (!pose_topic_.empty()) {
      bool found_pose_topic = false;
      for (const auto& node_desc : graph) {
        for (const auto& source : node_desc.sources) {
          if (source.name == pose_topic_ &&
              source.kind == event::SourceKind::SENDER) {
            found_pose_topic = true;
            SetupPoseExtraction(source);
            break;
          }
        }
        if (has_pose_) break;
      }
      if (!found_pose_topic) {
        throw std::invalid_argument("telemetry: pose topic '" + pose_topic_ +
                                    "' is absent from graph");
      }
      if (!has_pose_) {
        throw std::invalid_argument("telemetry: pose topic '" + pose_topic_ +
                                    "' has no geometry.Pose2d field");
      }
    }

    std::set<std::string> seen_topics;
    for (const auto& node_desc : graph) {
      for (const auto& source : node_desc.sources) {
        if (source.kind != event::SourceKind::SENDER) continue;
        if (seen_topics.contains(source.name)) continue;

        const std::string_view role = introspect::naming::Role(source.name);
        if (role != "state" && role != "status") continue;

        if (!MatchesGlobs(source.name, include_globs_, exclude_globs_)) {
          continue;
        }

        seen_topics.insert(source.name);
        // Schema-less state/status packets (notably /hw/state) contribute
        // neither subscriptions nor dictionary channels.

        if (source.name == "/hw/state/driver_station") {
          ObserveDriverStation(source);
        } else if (!source.bfbs.empty()) {
          ObserveDescribedTopic(source);
        }
      }
    }
  }

  void SetupPoseExtraction(const introspect::naming::SourceShape& source) {
    using talos::detail::NodeAccess;

    if (source.bfbs.empty()) {
      throw std::invalid_argument("telemetry: pose topic '" + source.name +
                                  "' has no schema");
    }

    const reflection::Schema* schema = schema::VerifySchema(source.bfbs);
    if (schema == nullptr) {
      throw std::invalid_argument(
          "telemetry: failed to verify schema for pose topic '" + source.name +
          "'");
    }
    const reflection::Object* object =
        schema->objects()->LookupByKey(source.schema.c_str());
    if (!object || !object->is_struct() || object->fields() == nullptr) {
      throw std::invalid_argument("telemetry: schema object '" + source.schema +
                                  "' for pose topic '" + source.name +
                                  "' is missing or not a struct");
    }

    auto read_double_offset = [&](const reflection::Field& sub_field,
                                  std::size_t pose_offset,
                                  std::size_t& out) -> void {
      if (sub_field.type() == nullptr ||
          sub_field.type()->base_type() != reflection::BaseType::Double) {
        throw std::invalid_argument(
            "telemetry: pose topic '" + source.name + "' geometry.Pose2d." +
            sub_field.name()->str() + " must be a double");
      }
      const std::size_t offset = pose_offset + sub_field.offset();
      if (offset > source.message_bytes ||
          sizeof(double) > source.message_bytes - offset) {
        throw std::invalid_argument(
            "telemetry: pose topic '" + source.name + "' geometry.Pose2d." +
            sub_field.name()->str() + " exceeds message_bytes");
      }
      out = offset;
    };

    bool found_pose = false;
    std::size_t x_offset = 0;
    std::size_t y_offset = 0;
    std::size_t yaw_offset = 0;
    for (const auto* field : *object->fields()) {
      if (field == nullptr || field->type() == nullptr) continue;
      const auto& type = *field->type();
      if (type.base_type() == reflection::BaseType::Obj && type.index() >= 0 &&
          static_cast<std::size_t>(type.index()) < schema->objects()->size()) {
        const auto* sub_obj = schema->objects()->Get(
            static_cast<flatbuffers::uoffset_t>(type.index()));
        if (sub_obj != nullptr && sub_obj->is_struct() &&
            sub_obj->fields() != nullptr &&
            sub_obj->name()->string_view() == "geometry.Pose2d") {
          const std::size_t pose_offset = field->offset();
          bool found_x = false;
          bool found_y = false;
          bool found_yaw = false;
          for (const auto* sub_field : *sub_obj->fields()) {
            if (sub_field == nullptr) continue;
            const std::string_view sub_name = sub_field->name()->string_view();
            if (sub_name == "x_m") {
              read_double_offset(*sub_field, pose_offset, x_offset);
              found_x = true;
            } else if (sub_name == "y_m") {
              read_double_offset(*sub_field, pose_offset, y_offset);
              found_y = true;
            } else if (sub_name == "yaw_rad") {
              read_double_offset(*sub_field, pose_offset, yaw_offset);
              found_yaw = true;
            }
          }
          if (found_x && found_y && found_yaw) {
            found_pose = true;
            break;
          }
        }
      }
    }

    if (!found_pose) {
      throw std::invalid_argument("telemetry: pose topic '" + source.name +
                                  "' has no geometry.Pose2d field");
    }

    bool has_valid_field = false;
    std::size_t valid_offset = 0;
    for (const auto* field : *object->fields()) {
      if (field == nullptr || field->type() == nullptr) continue;
      if (field->name()->string_view() == "valid" &&
          field->type()->base_type() == reflection::BaseType::Bool) {
        const std::size_t offset = field->offset();
        if (offset > source.message_bytes ||
            sizeof(bool) > source.message_bytes - offset) {
          throw std::invalid_argument("telemetry: pose topic '" + source.name +
                                      "' valid field exceeds message_bytes");
        }
        has_valid_field = true;
        valid_offset = offset;
        break;
      }
    }

    pose_sub_index_ = NodeAccess::add_observer(
        node_, source.name, source.message_bytes, source.alignment,
        {source.schema, source.schema_hash, source.bfbs});
    pose_x_offset_ = x_offset;
    pose_y_offset_ = y_offset;
    pose_yaw_offset_ = yaw_offset;
    pose_has_valid_field_ = has_valid_field;
    pose_valid_offset_ = valid_offset;
    has_pose_ = true;
  }

  void ObserveDriverStation(const introspect::naming::SourceShape& source) {
    using talos::detail::NodeAccess;

    const std::size_t sub_idx = NodeAccess::add_observer(
        node_, source.name, source.message_bytes, source.alignment,
        {source.schema, source.schema_hash, source.bfbs});

    ObservedTopic topic;
    topic.name = source.name;
    topic.sub_index = sub_idx;
    topic.message_bytes = source.message_bytes;
    topic.is_driver_station = true;

    const std::vector<Channel> channels = DriverStationChannels();
    for (std::size_t i = 0; i < channels.size(); ++i) {
      topic.ds_channels[i] = AddChannel(channels[i].name, channels[i].kind,
                                        channels[i].unit, channels[i].labels);
    }

    observed_topics_.push_back(std::move(topic));
  }

  void ObserveDescribedTopic(const introspect::naming::SourceShape& source) {
    using talos::detail::NodeAccess;

    // Excluded channels are not flattened or counted. Past one more than a
    // dictionary holds, the rest are counted, not built, and CheckChannelCap
    // refuses the dictionary naming the topic with every channel it keeps,
    // as for any source too large to serve whole.
    FlattenOptions options;
    options.keep = [this](std::string_view name) {
      return !MatchesAnyGlob(name, exclude_globs_);
    };
    options.max_channels = telemetry_wire::kMaxChannels + 1;
    options.truncate = true;
    std::size_t kept = 0;
    std::vector<Channel> channels =
        FlattenTopic(source.name, source.schema, source.bfbs, options, &kept);
    if (kept > channels.size()) {
      std::string_view owner = source.name;
      if (owner.starts_with('/')) owner.remove_prefix(1);
      uncounted_[std::string{owner}] += kept - channels.size();
    }

    std::size_t sub_idx = 0;
    if (has_pose_ && source.name == pose_topic_) {
      sub_idx = pose_sub_index_;
    } else {
      sub_idx = NodeAccess::add_observer(
          node_, source.name, source.message_bytes, source.alignment,
          {source.schema, source.schema_hash, source.bfbs});
    }

    ObservedTopic topic;
    topic.name = source.name;
    topic.sub_index = sub_idx;
    topic.message_bytes = source.message_bytes;
    for (Channel& channel : channels) {
      const std::size_t index =
          AddChannel(channel.name, channel.kind, channel.unit, channel.labels,
                     channel.alert);
      if (index == std::numeric_limits<std::size_t>::max()) continue;
      topic.readers.push_back(FieldReader{index, std::move(channel)});
    }

    observed_topics_.push_back(std::move(topic));
  }

  Node& node_;
  Duration stale_after_;
  std::vector<std::string> include_globs_;
  std::vector<std::string> exclude_globs_;
  std::vector<LabelSpec> labels_;  // until ApplyLabels
  std::string topic_;
  std::string pose_topic_;
  BytePublisher studio_;

  std::vector<ChannelDef> dictionary_;
  // Per topic (without its leading slash), channels kept past the cap and
  // so not built: they count in CheckChannelCap's refusal.
  std::map<std::string, std::size_t> uncounted_;
  std::uint32_t dictionary_id_{0};
  std::uint64_t stream_id_{0};
  // This tick's carried channels and their raw samples, index-aligned.
  std::vector<std::uint32_t> tick_channels_;
  std::vector<std::uint64_t> tick_values_;
  flatbuffers::FlatBufferBuilder builder_{2048};
  DictionaryScratch dictionary_scratch_;
  std::uint64_t sequence_ = 0;
  bool last_frame_had_fresh_{false};

  std::size_t idx_pose_valid_{0};
  bool has_pose_{false};
  std::size_t pose_sub_index_{0};
  std::size_t pose_x_offset_{0};
  std::size_t pose_y_offset_{0};
  std::size_t pose_yaw_offset_{0};
  bool pose_has_valid_field_{false};
  std::size_t pose_valid_offset_{0};

  std::vector<ObservedTopic> observed_topics_;
};

}  // namespace talos::telemetry
