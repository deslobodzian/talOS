#include "studio/review/session.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "nlohmann/json.hpp"
#include "studio/review/hardware_channels.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/launcher/log_inputs.h"
#include "talOS/launcher/log_merge.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/unit_suffix.h"

namespace studio::review {
namespace {

namespace fs = std::filesystem;
namespace tlog = talos::event::log;
using talos::event::EventKind;
using talos::launcher::HardwareRecord;
using talos::telemetry::Channel;
using talos::telemetry::ChannelKind;

constexpr std::int64_t kNanosPerSecond = 1'000'000'000;

std::int64_t FloorSecond(std::int64_t ns) {
  std::int64_t second = ns / kNanosPerSecond;
  if (ns % kNanosPerSecond < 0) --second;
  return second;
}

std::uint64_t SourceBit(std::uint16_t source_id) {
  return std::uint64_t{1} << std::min<std::uint16_t>(source_id, 63);
}

// A value a reader received: a MESSAGE, or a SEND its transport accepted
// (a failed send is logged too, and reached no reader).
bool IsValueRecord(const tlog::RecordHeader& header) {
  return header.kind == static_cast<std::uint16_t>(EventKind::MESSAGE) ||
         tlog::IsAcceptedSend(header);
}

// A topic as the index pass learns it.
struct TopicBuild {
  std::string name;
  Decoder decoder{Decoder::kStruct};
  HardwareRecord hardware{HardwareRecord::kNone};
  std::uint64_t schema_hash{0};
  std::vector<Channel> channels;  // struct topics
  // Indices into `channels` whose alert level is not None.
  std::vector<std::uint32_t> alert_channels;
  // Hardware topics: every channel name seen, with its kind.
  std::map<std::string, ChannelKind> hardware_channels;
  std::int64_t decoded_second{std::numeric_limits<std::int64_t>::min()};
};

constexpr std::uintmax_t kMaxManifestBytes = std::uintmax_t{4} << 20;

ModeKind ModeFromDriverStation(
    const std::array<std::uint64_t,
                     talos::telemetry::kDriverStationChannelCount>& raw) {
  if (raw[5] != 0) return ModeKind::kEstop;
  if (raw[0] == 0 || raw[1] == 0) return ModeKind::kDisabled;
  if (raw[2] != 0) return ModeKind::kAuto;
  if (raw[3] != 0) return ModeKind::kTeleop;
  if (raw[4] != 0) return ModeKind::kTest;
  return ModeKind::kDisabled;
}

struct ModeStream {
  std::vector<ModeInterval> intervals;
  std::uint64_t dropped{0};
  std::int64_t last_publish_ns{std::numeric_limits<std::int64_t>::min()};
  ModeKind last_mode{ModeKind::kDisabled};
  bool last_fms{false};
  bool closed_last{false};
};

struct AlertChannelState {
  bool active{false};
  std::int64_t start_ns{0};
};

struct RawAlertInterval {
  std::int64_t start_ns{0};
  std::int64_t end_ns{0};
  std::uint32_t channel{0};
};

struct AlertStream {
  std::vector<AlertChannelState> state;
  std::vector<RawAlertInterval> intervals;
  std::uint64_t dropped{0};
  std::int64_t last_publish_ns{std::numeric_limits<std::int64_t>::min()};
};

bool ReadManifestInt(const nlohmann::json& object, std::string_view key,
                     std::int64_t& out) {
  const auto it = object.find(key);
  if (it == object.end()) return false;
  if (it->is_number_integer()) {
    out = it->get<std::int64_t>();
    return true;
  }
  if (it->is_string()) {
    const std::string& text = it->get_ref<const std::string&>();
    const auto [end, ec] =
        std::from_chars(text.data(), text.data() + text.size(), out);
    return ec == std::errc{} && end == text.data() + text.size() &&
           !text.empty();
  }
  return false;
}

void ReadManifestRuns(const fs::path& manifest_path,
                      std::int64_t origin_wall_ns,
                      std::vector<RunInterval>& runs,
                      std::uint64_t& runs_dropped) {
  std::error_code error;
  if (fs::is_symlink(manifest_path, error) ||
      !fs::is_regular_file(manifest_path, error)) {
    return;
  }
  const auto bytes = fs::file_size(manifest_path, error);
  if (error || bytes == 0 || bytes > kMaxManifestBytes) return;
  std::ifstream stream{manifest_path};
  if (!stream.is_open()) return;
  const auto manifest = nlohmann::json::parse(
      std::string{std::istreambuf_iterator<char>{stream}, {}}, nullptr,
      /*allow_exceptions=*/false);
  if (!manifest.is_object()) return;
  const auto nodes = manifest.find("nodes");
  if (nodes == manifest.end() || !nodes->is_array()) return;
  for (const auto& node : *nodes) {
    if (!node.is_object()) continue;
    const auto name_it = node.find("name");
    if (name_it == node.end() || !name_it->is_string()) continue;
    const std::string& node_name = name_it->get_ref<const std::string&>();
    std::int64_t node_dropped = 0;
    if (ReadManifestInt(node, "runs_dropped", node_dropped) &&
        node_dropped > 0) {
      runs_dropped += static_cast<std::uint64_t>(node_dropped);
    }
    const auto node_runs = node.find("runs");
    if (node_runs == node.end() || !node_runs->is_array()) continue;
    for (const auto& run : *node_runs) {
      if (!run.is_object()) continue;
      std::int64_t start_wall_ns = 0;
      if (!ReadManifestInt(run, "start_wall_ns", start_wall_ns) ||
          start_wall_ns <= 0) {
        continue;
      }
      const auto reason_it = run.find("reason");
      if (reason_it == run.end() || !reason_it->is_string()) continue;
      std::int64_t end_wall_ns = 0;
      (void)ReadManifestInt(run, "end_wall_ns", end_wall_ns);
      std::int64_t start_ns = start_wall_ns - origin_wall_ns;
      if (start_ns < 0) start_ns = 0;
      std::optional<std::int64_t> end_ns;
      if (end_wall_ns > 0) {
        end_ns = std::max(start_ns, end_wall_ns - origin_wall_ns);
      }
      runs.push_back({node_name, start_ns, end_ns,
                      reason_it->get_ref<const std::string&>()});
    }
  }
  std::sort(runs.begin(), runs.end(),
            [](const RunInterval& a, const RunInterval& b) {
              const std::int64_t a_end =
                  a.end_ns.value_or(std::numeric_limits<std::int64_t>::max());
              const std::int64_t b_end =
                  b.end_ns.value_or(std::numeric_limits<std::int64_t>::max());
              return std::tie(a.start_ns, a.node, a_end, a.reason) <
                     std::tie(b.start_ns, b.node, b_end, b.reason);
            });
  if (runs.size() > kMaxTimelineRuns) {
    runs_dropped += runs.size() - kMaxTimelineRuns;
    runs.resize(kMaxTimelineRuns);
  }
  runs.shrink_to_fit();
}

// A log file as the index pass learns it: per source id, its topic (-1 for
// none, kUnresolved before its first value record) and which value record
// kinds it holds.
constexpr std::int32_t kUnresolved = -2;
struct FileBuild {
  std::vector<std::int32_t> topic;
  std::vector<std::uint8_t> kinds;  // bit 0 SEND, bit 1 MESSAGE
};

class Indexer {
 public:
  std::vector<TopicBuild> topics;

  // The topic `source` carries, created on first sight; -1 for one that
  // holds no channel review reads, or whose schema disagrees with the one
  // the topic was first seen with.
  std::int32_t Resolve(const talos::event::Registration& source,
                       const std::string& path) {
    const auto& schema = source.schema;
    if (schema.name.empty() || schema.name == talos::schema::kBytesName) {
      return -1;
    }
    const auto found = by_name_.find(source.name);
    if (found != by_name_.end()) {
      if (found->second >= 0 &&
          topics[static_cast<std::size_t>(found->second)].schema_hash !=
              schema.hash) {
        std::cerr << "studio_review: " << path << " records " << source.name
                  << " as " << schema.name
                  << ", another log as another type; its records are "
                     "skipped\n";
        return -1;
      }
      return found->second;
    }
    TopicBuild topic;
    topic.name = source.name;
    topic.schema_hash = schema.hash;
    topic.hardware =
        talos::launcher::HardwareRecordOf(source.name, schema.name);
    std::int32_t index = -1;
    try {
      if (topic.hardware == HardwareRecord::kDriverStation) {
        topic.decoder = Decoder::kDriverStation;
        topic.channels = talos::telemetry::DriverStationChannels();
      } else if (topic.hardware != HardwareRecord::kNone) {
        topic.decoder = Decoder::kHardware;
      } else if (schema.name == talos::schema::kPacketName ||
                 schema.bfbs.empty()) {
        topic.decoder = Decoder::kStruct;  // nothing to decode
      } else if (const auto inner = talos::schema::EnvelopeInner(schema.name);
                 !inner.empty()) {
        topic.channels =
            talos::telemetry::EnvelopeChannels(source.name, inner, schema.bfbs);
      } else {
        topic.channels = talos::telemetry::FlattenTopic(
            source.name, schema.name, schema.bfbs);
      }
      if (topic.decoder == Decoder::kHardware || !topic.channels.empty()) {
        for (std::uint32_t c = 0; c < topic.channels.size(); ++c) {
          if (topic.channels[c].alert != Talos::Telemetry::AlertLevel_None) {
            topic.alert_channels.push_back(c);
          }
        }
        index = static_cast<std::int32_t>(topics.size());
        topics.push_back(std::move(topic));
      }
    } catch (const std::exception& e) {
      std::cerr << "studio_review: " << source.name << " in " << path
                << " is not reviewed: " << e.what() << '\n';
    }
    by_name_.emplace(source.name, index);
    return index;
  }

 private:
  std::unordered_map<std::string, std::int32_t> by_name_;
};

// The wall clock and monotonic time a log's header records for its start.
struct HeaderTimes {
  std::int64_t wall_ns{0};
  std::int64_t monotonic_ns{0};
};
std::optional<HeaderTimes> ReadHeaderTimes(const std::string& path) {
  std::ifstream stream{path, std::ios::binary};
  tlog::FileHeader header{};
  if (!stream.read(reinterpret_cast<char*>(&header), sizeof header)) {
    return std::nullopt;
  }
  return HeaderTimes{header.start_wall_ns, header.start_monotonic_ns};
}

}  // namespace

std::string_view ModeKindName(ModeKind mode) {
  switch (mode) {
    case ModeKind::kDisabled:
      return "disabled";
    case ModeKind::kTeleop:
      return "teleop";
    case ModeKind::kAuto:
      return "auto";
    case ModeKind::kTest:
      return "test";
    case ModeKind::kEstop:
      return "estop";
  }
  return "disabled";
}

double ValueOf(ChannelKind kind, std::uint64_t raw) {
  switch (kind) {
    case Talos::Telemetry::ChannelKind_Bool:
      return raw != 0 ? 1.0 : 0.0;
    case Talos::Telemetry::ChannelKind_Int:
      return static_cast<double>(static_cast<std::int64_t>(raw));
    case Talos::Telemetry::ChannelKind_UInt:
      return static_cast<double>(raw);
    default:
      return talos::telemetry::DoubleFromRaw(raw);
  }
}

std::unique_ptr<Session> Session::Open(const fs::path& dir) {
  const std::vector<std::string> paths =
      talos::launcher::LogInputFiles({dir.string()});
  if (paths.empty() || !fs::is_directory(dir)) {
    throw std::runtime_error("no .tlog files in " + dir.string());
  }
  std::unique_ptr<Session> session{new Session};
  Indexer indexer;
  std::vector<FileBuild> builds;
  std::unordered_map<const talos::launcher::MergedLog*, std::size_t> file_of;
  const talos::launcher::MergedLog* last_log = nullptr;
  std::size_t file = 0;
  bool any = false;
  std::vector<HardwareSample> samples;
  std::map<std::pair<std::string, bool>, ModeStream> mode_streams;
  std::map<std::tuple<std::size_t, std::string, bool>, AlertStream>
      alert_streams;
  auto recorded = talos::launcher::RecordedProtocolVersion({dir.string()});
  if (const std::string line = recorded.Disagreement(); !line.empty()) {
    std::cerr << "studio_review: " << dir.string() << ": " << line << '\n';
  }
  talos::launcher::HardwareProtocolCheck protocol{std::move(recorded)};

  const auto visit = [&](const talos::launcher::MergedRecord& r) {
    const auto* merged = r.log.get();
    if (merged != last_log) {
      const auto [it, inserted] =
          file_of.emplace(merged, session->files_.size());
      if (inserted) {
        session->files_.push_back({merged->path, merged->process_name, {}});
        builds.emplace_back();
      }
      last_log = merged;
      file = it->second;
    }
    LogFile& log_file = session->files_[file];
    if (const auto gap = tlog::GapOf(r.header, r.payload)) {
      // No checkpoint and no value: only its span, in the bounds too.
      session->gaps_.push_back(
          {gap->start_ns, gap->resumed_ns, gap->detail.records,
           gap->detail.dispatches, gap->detail.sends,
           static_cast<std::uint32_t>(file),
           (gap->detail.flags & tlog::GAP_CLOSED_AT_FLUSH) != 0});
      const std::int64_t low = std::min(gap->start_ns, gap->resumed_ns);
      const std::int64_t high = std::max(gap->start_ns, gap->resumed_ns);
      if (!any || low < session->start_ns_) session->start_ns_ = low;
      if (!any || high > session->end_ns_) session->end_ns_ = high;
      any = true;
      return true;
    }
    const std::int64_t event_ns = r.header.event_time_ns;
    const std::int64_t second = FloorSecond(event_ns);
    if (log_file.checkpoints.empty() ||
        second > log_file.checkpoints.back().second) {
      log_file.checkpoints.push_back({second, r.offset, 0});
    }
    if (!any || event_ns < session->start_ns_) session->start_ns_ = event_ns;
    if (!any || event_ns > session->end_ns_) session->end_ns_ = event_ns;
    any = true;
    if (r.header.kind == static_cast<std::uint16_t>(EventKind::EXIT)) {
      return true;
    }
    if (!IsValueRecord(r.header)) return true;
    log_file.checkpoints.back().sources |= SourceBit(r.header.source_id);
    if (r.source == nullptr || r.payload.empty()) return true;
    FileBuild& build = builds[file];
    const std::uint16_t id = r.header.source_id;
    if (id >= build.topic.size()) {
      build.topic.resize(id + 1u, kUnresolved);
      build.kinds.resize(id + 1u, 0);
    }
    if (build.topic[id] == kUnresolved) {
      build.topic[id] = indexer.Resolve(*r.source, merged->path);
    }
    if (const auto kind = talos::launcher::HardwareRecordOf(
            r.source->name, r.source->schema.name);
        kind != HardwareRecord::kNone) {
      protocol.Add(kind, r.source->name,
                   r.header.kind != static_cast<std::uint16_t>(EventKind::SEND),
                   r.payload);
    }
    if (build.topic[id] < 0) return true;
    const bool is_send =
        r.header.kind == static_cast<std::uint16_t>(EventKind::SEND);
    build.kinds[id] |= is_send ? 1 : 2;
    const auto topic_idx = static_cast<std::size_t>(build.topic[id]);
    TopicBuild& topic = indexer.topics[topic_idx];
    // A hardware topic's channels are what its packets hold: learned from
    // its first packet in each second.
    if (topic.decoder == Decoder::kHardware && topic.decoded_second != second) {
      topic.decoded_second = second;
      FlattenHardware(topic.name, topic.hardware, r.payload, samples);
      for (const HardwareSample& sample : samples) {
        if (!sample.null)
          topic.hardware_channels.emplace(sample.name, sample.kind);
      }
    } else if (topic.decoder == Decoder::kDriverStation) {
      std::array<std::uint64_t, talos::telemetry::kDriverStationChannelCount>
          values{};
      if (talos::telemetry::ReadDriverStation(r.payload, values)) {
        const std::int64_t publish_ns = r.header.publish_time_ns;
        ModeStream& stream = mode_streams[{log_file.process, is_send}];
        if (publish_ns >= stream.last_publish_ns) {
          stream.last_publish_ns = publish_ns;
          const ModeKind mode = ModeFromDriverStation(values);
          const bool fms = values[6] != 0;
          if (stream.intervals.empty()) {
            stream.intervals.push_back({publish_ns, publish_ns, mode, fms});
            stream.last_mode = mode;
            stream.last_fms = fms;
          } else if (mode != stream.last_mode || fms != stream.last_fms) {
            if (!stream.closed_last) {
              stream.intervals.back().end_ns = publish_ns;
              if (stream.intervals.size() < kMaxTimelineModes) {
                stream.intervals.push_back({publish_ns, publish_ns, mode, fms});
              } else {
                stream.closed_last = true;
                ++stream.dropped;
              }
            } else {
              ++stream.dropped;
            }
            stream.last_mode = mode;
            stream.last_fms = fms;
          }
        }
      }
    } else if (topic.decoder == Decoder::kStruct &&
               !topic.alert_channels.empty()) {
      const std::int64_t publish_ns = r.header.publish_time_ns;
      AlertStream& stream =
          alert_streams[{topic_idx, log_file.process, is_send}];
      if (stream.state.empty()) {
        stream.state.resize(topic.alert_channels.size());
      }
      if (publish_ns >= stream.last_publish_ns) {
        stream.last_publish_ns = publish_ns;
        for (std::size_t i = 0; i < topic.alert_channels.size(); ++i) {
          const std::uint32_t c = topic.alert_channels[i];
          std::uint64_t raw = 0;
          if (!talos::telemetry::ReadChannel(topic.channels[c], r.payload,
                                             raw)) {
            continue;
          }
          const bool active = raw != 0;
          AlertChannelState& st = stream.state[i];
          if (active && !st.active) {
            st.active = true;
            st.start_ns = publish_ns;
          } else if (!active && st.active) {
            st.active = false;
            if (stream.intervals.size() < kMaxTimelineAlerts) {
              stream.intervals.push_back({st.start_ns, publish_ns, c});
            } else {
              ++stream.dropped;
            }
          }
        }
      }
    }
    return true;
  };
  std::string error;
  if (!talos::launcher::MergeLogs(paths, visit, error)) {
    throw std::runtime_error(error);
  }
  if (!any) throw std::runtime_error("no records in " + dir.string());
  std::stable_sort(session->gaps_.begin(), session->gaps_.end(),
                   [&files = session->files_](const Gap& a, const Gap& b) {
                     if (a.start_ns != b.start_ns) {
                       return a.start_ns < b.start_ns;
                     }
                     return files[a.file].process < files[b.file].process;
                   });
  session->gaps_.shrink_to_fit();
  if (protocol.foreign()) {
    constexpr std::string_view kWarning = "warning: ";
    session->hardware_not_decoded_ = protocol.Warning().substr(kWarning.size());
    std::cerr << "studio_review: " << dir.string() << ": "
              << session->hardware_not_decoded_ << '\n';
  }

  // Each topic's process: the one that sent it, else the one whose log
  // path sorts first among those that received it.
  std::vector<std::size_t> topic_build_index;
  for (std::size_t t = 0; t < indexer.topics.size(); ++t) {
    TopicBuild& build = indexer.topics[t];
    std::optional<std::size_t> sender;
    std::optional<std::size_t> receiver;
    for (std::size_t f = 0; f < builds.size(); ++f) {
      const FileBuild& file_build = builds[f];
      for (std::size_t id = 0; id < file_build.topic.size(); ++id) {
        if (file_build.topic[id] != static_cast<std::int32_t>(t)) continue;
        if ((file_build.kinds[id] & 1) != 0 && !sender) sender = f;
        // The reader whose log path sorts first, as RecordedSession orders
        // logs: not the first in merge order, which depends on record times.
        if ((file_build.kinds[id] & 2) != 0 &&
            (!receiver ||
             session->files_[f].path < session->files_[*receiver].path)) {
          receiver = f;
        }
      }
    }
    if (!sender && !receiver) continue;
    if (protocol.foreign() && build.hardware != HardwareRecord::kNone) {
      continue;
    }
    Topic topic;
    topic.name = build.name;
    topic.decoder = build.decoder;
    topic.hardware = build.hardware;
    topic.kind = sender ? EventKind::SEND : EventKind::MESSAGE;
    topic.process = session->files_[sender ? *sender : *receiver].process;
    for (std::size_t f = 0; f < builds.size(); ++f) {
      if (session->files_[f].process != topic.process) continue;
      const FileBuild& file_build = builds[f];
      for (std::size_t id = 0; id < file_build.topic.size(); ++id) {
        if (file_build.topic[id] == static_cast<std::int32_t>(t)) {
          topic.sources.push_back({f, static_cast<std::uint16_t>(id)});
        }
      }
    }
    std::unordered_map<std::string, std::uint32_t> hardware_index;
    if (build.decoder == Decoder::kHardware) {
      for (const auto& [name, kind] : build.hardware_channels) {
        Channel channel;
        channel.name = name;
        const std::string_view leaf =
            std::string_view{name}.substr(name.rfind('/') + 1);
        if (kind != Talos::Telemetry::ChannelKind_Bool) {
          channel.unit = std::string{talos::schema::unit_for_field(leaf)};
        }
        channel.kind = kind;
        hardware_index.emplace(
            name, static_cast<std::uint32_t>(topic.channels.size()));
        topic.channels.push_back(std::move(channel));
      }
    } else {
      topic.channels = std::move(build.channels);
    }
    // A hardware topic none of whose packets decoded has nothing to serve.
    if (topic.channels.empty()) continue;
    session->topics_.push_back(std::move(topic));
    session->hardware_index_.push_back(std::move(hardware_index));
    topic_build_index.push_back(t);
  }
  // Topics by name, so channels.json lists them in a stable order.
  std::vector<std::size_t> order(session->topics_.size());
  for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return session->topics_[a].name < session->topics_[b].name;
  });
  std::vector<Topic> topics;
  std::vector<std::unordered_map<std::string, std::uint32_t>> hardware_index;
  std::vector<std::size_t> sorted_build_index;
  for (const std::size_t i : order) {
    topics.push_back(std::move(session->topics_[i]));
    hardware_index.push_back(std::move(session->hardware_index_[i]));
    sorted_build_index.push_back(topic_build_index[i]);
  }
  session->topics_ = std::move(topics);
  session->hardware_index_ = std::move(hardware_index);
  for (std::uint32_t t = 0; t < session->topics_.size(); ++t) {
    const auto& channels = session->topics_[t].channels;
    for (std::uint32_t c = 0; c < channels.size(); ++c) {
      session->by_name_.emplace(channels[c].name, ChannelRef{t, c});
    }
  }

  // Finalize timeline modes and alerts from each topic's canonical stream.
  std::vector<AlertInterval> combined_alerts;
  for (std::uint32_t t = 0; t < session->topics_.size(); ++t) {
    const Topic& topic = session->topics_[t];
    const bool is_send = topic.kind == EventKind::SEND;
    if (!protocol.foreign() && topic.decoder == Decoder::kDriverStation) {
      const auto it = mode_streams.find({topic.process, is_send});
      if (it != mode_streams.end()) {
        session->modes_ = std::move(it->second.intervals);
        session->modes_dropped_ = it->second.dropped;
        if (!session->modes_.empty() && !it->second.closed_last) {
          session->modes_.back().end_ns =
              std::max(session->modes_.back().start_ns, session->end_ns_);
        }
        session->modes_.shrink_to_fit();
      }
    } else if (topic.decoder == Decoder::kStruct) {
      const std::size_t build_idx = sorted_build_index[t];
      const auto it = alert_streams.find({build_idx, topic.process, is_send});
      if (it != alert_streams.end()) {
        AlertStream& stream = it->second;
        const auto& alert_channels = indexer.topics[build_idx].alert_channels;
        for (std::size_t i = 0; i < alert_channels.size(); ++i) {
          if (stream.state[i].active) {
            if (stream.intervals.size() < kMaxTimelineAlerts) {
              stream.intervals.push_back(
                  {stream.state[i].start_ns,
                   std::max(stream.state[i].start_ns, session->end_ns_),
                   alert_channels[i]});
            } else {
              ++stream.dropped;
            }
          }
        }
        session->alerts_dropped_ += stream.dropped;
        for (const RawAlertInterval& iv : stream.intervals) {
          combined_alerts.push_back(
              {iv.start_ns, iv.end_ns, ChannelRef{t, iv.channel}});
        }
      }
    }
  }
  std::sort(combined_alerts.begin(), combined_alerts.end(),
            [&session](const AlertInterval& a, const AlertInterval& b) {
              if (a.start_ns != b.start_ns) return a.start_ns < b.start_ns;
              if (a.end_ns != b.end_ns) return a.end_ns < b.end_ns;
              return session->channel(a.channel).name <
                     session->channel(b.channel).name;
            });
  if (combined_alerts.size() > kMaxTimelineAlerts) {
    session->alerts_dropped_ += combined_alerts.size() - kMaxTimelineAlerts;
    combined_alerts.resize(kMaxTimelineAlerts);
  }
  session->alerts_ = std::move(combined_alerts);
  session->alerts_.shrink_to_fit();

  std::optional<HeaderTimes> origin;
  for (const LogFile& log_file : session->files_) {
    const auto times = ReadHeaderTimes(log_file.path);
    if (times && (!origin || times->monotonic_ns < origin->monotonic_ns)) {
      origin = times;
    }
  }
  if (origin) {
    session->origin_wall_ns_ = origin->wall_ns - origin->monotonic_ns;
  }
  ReadManifestRuns(dir / "manifest.json", session->origin_wall_ns_,
                   session->runs_, session->runs_dropped_);
  return session;
}

std::optional<ChannelRef> Session::Find(std::string_view name) const {
  const auto it = by_name_.find(std::string{name});
  if (it == by_name_.end()) return std::nullopt;
  return it->second;
}

std::size_t Session::log_seconds() const {
  std::size_t seconds = 0;
  for (const LogFile& file : files_) seconds += file.checkpoints.size();
  return seconds;
}

std::size_t Session::index_bytes() const {
  return log_seconds() * sizeof(Checkpoint) + gaps_.size() * sizeof(Gap);
}

bool Session::Decode(const Topic& topic, std::span<const std::byte> payload,
                     std::vector<std::uint64_t>& raw,
                     std::vector<std::uint8_t>& present) const {
  const std::size_t count = topic.channels.size();
  raw.assign(count, 0);
  present.assign(count, 0);
  switch (topic.decoder) {
    case Decoder::kStruct: {
      bool any = false;
      for (std::size_t i = 0; i < count; ++i) {
        present[i] =
            talos::telemetry::ReadChannel(topic.channels[i], payload, raw[i])
                ? 1
                : 0;
        any = any || present[i] != 0;
      }
      return any;
    }
    case Decoder::kDriverStation: {
      std::array<std::uint64_t, talos::telemetry::kDriverStationChannelCount>
          values{};
      if (!talos::telemetry::ReadDriverStation(payload, values)) return false;
      for (std::size_t i = 0; i < count && i < values.size(); ++i) {
        raw[i] = values[i];
        present[i] = 1;
      }
      return true;
    }
    case Decoder::kHardware: {
      const auto& index =
          hardware_index_[static_cast<std::size_t>(&topic - topics_.data())];
      std::vector<HardwareSample> samples;
      FlattenHardware(topic.name, topic.hardware, payload, samples);
      for (const HardwareSample& sample : samples) {
        const auto it = index.find(sample.name);
        if (it == index.end()) continue;
        const Channel& channel = topic.channels[it->second];
        // A null is a number with no finite reading: no reading at all.
        if (sample.null || sample.kind != channel.kind) continue;
        raw[it->second] = sample.raw;
        present[it->second] = 1;
      }
      return !samples.empty();
    }
  }
  return false;
}

namespace {

// The sources of the requested topics, per file: (source id, topic index).
using FileSources =
    std::map<std::size_t, std::vector<std::pair<std::uint16_t, std::uint32_t>>>;

void Count(const tlog::LogReader::Record& record, ReadStats* stats) {
  if (stats == nullptr) return;
  ++stats->records;
  stats->bytes += sizeof(tlog::RecordHeader) + record.header.stored_bytes;
}

FileSources SourcesByFile(const std::vector<Topic>& topics,
                          const std::vector<std::uint32_t>& wanted) {
  FileSources by_file;
  for (const std::uint32_t t : wanted) {
    for (const Topic::Source& source : topics[t].sources) {
      by_file[source.file].emplace_back(source.source_id, t);
    }
  }
  return by_file;
}

}  // namespace

std::vector<Series> Session::ReadSeries(std::span<const ChannelRef> channels,
                                        std::int64_t start_ns,
                                        std::int64_t end_ns,
                                        std::uint32_t points,
                                        ReadStats* stats) const {
  if (end_ns <= start_ns || points == 0) {
    throw std::invalid_argument(
        "a series needs end_ns > start_ns and points > 0");
  }
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  std::vector<Series> out(channels.size());
  std::vector<std::vector<std::int64_t>> last_ns(channels.size());
  // Per topic: which outputs want which of its channels.
  std::map<std::uint32_t, std::vector<std::pair<std::size_t, std::uint32_t>>>
      wants;
  for (std::size_t i = 0; i < channels.size(); ++i) {
    Series& series = out[i];
    series.t_ns.assign(points, kNaN);
    series.min.assign(points, std::numeric_limits<double>::infinity());
    series.max.assign(points, -std::numeric_limits<double>::infinity());
    series.last.assign(points, kNaN);
    series.count.assign(points, 0);
    last_ns[i].assign(points, std::numeric_limits<std::int64_t>::min());
    wants[channels[i].topic].emplace_back(i, channels[i].channel);
  }
  std::vector<std::uint32_t> wanted_topics;
  wanted_topics.reserve(wants.size());
  for (const auto& [topic, unused] : wants) wanted_topics.push_back(topic);

  const std::int64_t first_second = FloorSecond(start_ns) - 1;
  const std::int64_t last_second = FloorSecond(end_ns) + 1;
  const auto span_ns = static_cast<__int128>(end_ns) - start_ns;
  std::vector<std::uint64_t> raw;
  std::vector<std::uint8_t> present;
  for (const auto& [file_index, sources] :
       SourcesByFile(topics_, wanted_topics)) {
    const LogFile& file = files_[file_index];
    const auto& checkpoints = file.checkpoints;
    const auto first = std::lower_bound(
        checkpoints.begin(), checkpoints.end(), first_second,
        [](const Checkpoint& c, std::int64_t s) { return c.second < s; });
    std::uint64_t mask = 0;
    for (const auto& [id, topic] : sources) mask |= SourceBit(id);
    bool holds = false;
    for (auto it = first; it != checkpoints.end() && it->second <= last_second;
         ++it) {
      holds = holds || (it->sources & mask) != 0;
    }
    if (!holds) continue;
    if (stats != nullptr) ++stats->seeks;
    tlog::LogVisitor visitor;
    visitor.manifest = [](std::span<const tlog::ManifestEntry>) {
      return true;
    };
    visitor.record = [&](const tlog::LogReader::Record& record) {
      const auto& header = record.header;
      if (FloorSecond(header.event_time_ns) > last_second) return false;
      Count(record, stats);
      const auto source = std::find_if(
          sources.begin(), sources.end(),
          [&](const auto& s) { return s.first == header.source_id; });
      if (source == sources.end()) return true;
      const Topic& topic = topics_[source->second];
      if (header.kind != static_cast<std::uint16_t>(topic.kind) ||
          !IsValueRecord(header)) {
        return true;
      }
      const std::int64_t publish_ns = header.publish_time_ns;
      if (publish_ns < start_ns || publish_ns >= end_ns) return true;
      if (!Decode(topic, record.payload, raw, present)) return true;
      const auto bucket = static_cast<std::size_t>(
          (static_cast<__int128>(publish_ns) - start_ns) * points / span_ns);
      for (const auto& [output, channel] : wants.at(source->second)) {
        if (present[channel] == 0) continue;
        const double value =
            ValueOf(topic.channels[channel].kind, raw[channel]);
        Series& series = out[output];
        series.min[bucket] = std::fmin(series.min[bucket], value);
        series.max[bucket] = std::fmax(series.max[bucket], value);
        ++series.count[bucket];
        if (publish_ns >= last_ns[output][bucket]) {
          last_ns[output][bucket] = publish_ns;
          series.last[bucket] = value;
          series.t_ns[bucket] = static_cast<double>(publish_ns);
        }
      }
      return true;
    };
    tlog::VisitLog(file.path, visitor, tlog::VisitOptions{first->offset});
  }
  for (Series& series : out) {
    for (std::size_t b = 0; b < points; ++b) {
      // Empty, or every sample NaN (fmin and fmax skip one, so min stays
      // +Inf above max's -Inf): no extremes. The count stays.
      if (series.count[b] == 0 || series.min[b] > series.max[b]) {
        series.min[b] = series.max[b] = kNaN;
      }
    }
  }
  return out;
}

std::vector<Sample> Session::Snapshot(std::int64_t t_ns,
                                      ReadStats* stats) const {
  struct Latest {
    std::int64_t publish_ns{std::numeric_limits<std::int64_t>::min()};
    std::vector<std::uint64_t> raw;
    std::vector<std::uint8_t> present;
  };
  std::vector<Latest> latest(topics_.size());
  std::vector<std::uint32_t> all(topics_.size());
  for (std::uint32_t t = 0; t < all.size(); ++t) all[t] = t;
  const std::int64_t last_second = FloorSecond(t_ns) + 1;
  std::vector<std::uint64_t> raw;
  std::vector<std::uint8_t> present;

  for (const auto& [file_index, sources] : SourcesByFile(topics_, all)) {
    const LogFile& file = files_[file_index];
    const auto& checkpoints = file.checkpoints;
    // The latest second at or before `from` holding source `id`, or -1.
    const auto previous = [&](std::uint16_t id, std::ptrdiff_t from) {
      for (std::ptrdiff_t c = from; c >= 0; --c) {
        const Checkpoint& checkpoint = checkpoints[static_cast<std::size_t>(c)];
        if (checkpoint.second <= last_second &&
            (checkpoint.sources & SourceBit(id)) != 0) {
          return c;
        }
      }
      return std::ptrdiff_t{-1};
    };
    struct Item {
      std::uint16_t id;
      std::uint32_t topic;
      std::ptrdiff_t candidate;
      Latest found;
    };
    std::vector<Item> items;
    for (const auto& [id, topic] : sources) {
      items.push_back(
          {id,
           topic,
           previous(id, static_cast<std::ptrdiff_t>(checkpoints.size()) - 1),
           {}});
    }
    // Seconds latest first, each read once, for every item whose candidate
    // it is; an item that finds nothing there moves to its previous second.
    while (true) {
      std::ptrdiff_t second = -1;
      for (const Item& item : items) second = std::max(second, item.candidate);
      if (second < 0) break;
      const auto at = static_cast<std::size_t>(second);
      const std::uint64_t stop =
          at + 1 < checkpoints.size()
              ? checkpoints[at + 1].offset
              : std::numeric_limits<std::uint64_t>::max();
      if (stats != nullptr) ++stats->seeks;
      tlog::LogVisitor visitor;
      visitor.manifest = [](std::span<const tlog::ManifestEntry>) {
        return true;
      };
      visitor.record = [&](const tlog::LogReader::Record& record) {
        if (record.offset >= stop) return false;
        Count(record, stats);
        const auto& header = record.header;
        for (Item& item : items) {
          if (item.candidate != second || item.id != header.source_id) continue;
          const Topic& topic = topics_[item.topic];
          if (header.kind != static_cast<std::uint16_t>(topic.kind) ||
              !IsValueRecord(header) || header.publish_time_ns > t_ns ||
              header.publish_time_ns < item.found.publish_ns) {
            continue;
          }
          if (!Decode(topic, record.payload, raw, present)) continue;
          item.found.publish_ns = header.publish_time_ns;
          item.found.raw = raw;
          item.found.present = present;
        }
        return true;
      };
      tlog::VisitLog(file.path, visitor,
                     tlog::VisitOptions{checkpoints[at].offset});
      for (Item& item : items) {
        if (item.candidate != second) continue;
        item.candidate =
            item.found.raw.empty() ? previous(item.id, second - 1) : -1;
      }
    }
    for (Item& item : items) {
      if (!item.found.raw.empty() &&
          item.found.publish_ns > latest[item.topic].publish_ns) {
        latest[item.topic] = std::move(item.found);
      }
    }
  }
  std::vector<Sample> samples;
  for (std::uint32_t t = 0; t < latest.size(); ++t) {
    const Latest& value = latest[t];
    for (std::uint32_t c = 0; c < value.raw.size(); ++c) {
      samples.push_back(
          {{t, c}, value.raw[c], value.publish_ns, value.present[c] != 0});
    }
  }
  return samples;
}

}  // namespace studio::review
