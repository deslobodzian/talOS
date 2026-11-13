#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/names.h"
#include "talOS/launcher/hardware_record.h"
#include "talOS/launcher/log_inputs.h"
#include "talOS/launcher/log_merge.h"
#include "talOS/schema/decode.h"

namespace {

using talos::event::EventKind;

double ToMillis(std::int64_t start_ns, std::int64_t ns) {
  return static_cast<double>(ns - start_ns) / 1e6;
}

const char* EventKindName(std::uint16_t kind) {
  return talos::event::to_string(static_cast<EventKind>(kind));
}

struct Options {
  std::vector<std::string> nodes;
  std::vector<std::string> topics;
  std::vector<std::string> paths;
  std::string config;
  std::string format{"text"};
  double since_ms{-std::numeric_limits<double>::infinity()};
  double until_ms{std::numeric_limits<double>::infinity()};
};

void AddPatterns(std::vector<std::string>& patterns, const std::string& value) {
  std::size_t start = 0;
  do {
    const auto end = value.find(',', start);
    patterns.push_back(value.substr(start, end - start));
    if (end == std::string::npos) break;
    start = end + 1;
  } while (true);
}

bool ParseOptions(int argc, char** argv, Options& options, std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--") {
      while (++i < argc) options.paths.emplace_back(argv[i]);
      break;
    }
    if (!arg.starts_with("--")) {
      options.paths.push_back(arg);
      continue;
    }
    if (arg != "--node" && arg != "--topic" && arg != "--since-ms" &&
        arg != "--until-ms" && arg != "--format" && arg != "--config") {
      error = "unknown flag: " + arg;
      return false;
    }
    if (++i == argc) {
      error = "missing value for " + arg;
      return false;
    }
    const std::string value = argv[i];
    if (arg == "--node") {
      AddPatterns(options.nodes, value);
    } else if (arg == "--topic") {
      AddPatterns(options.topics, value);
    } else if (arg == "--config") {
      options.config = value;
    } else if (arg == "--format") {
      if (value != "text" && value != "jsonl" && value != "csv" &&
          value != "summary") {
        error = "invalid --format: " + value;
        return false;
      }
      options.format = value;
    } else {
      char* end = nullptr;
      const double ms = std::strtod(value.c_str(), &end);
      if (end == value.c_str() || *end != '\0' || !std::isfinite(ms)) {
        error = "invalid finite number for ";
        error.append(arg).append(": ").append(value);
        return false;
      }
      (arg == "--since-ms" ? options.since_ms : options.until_ms) = ms;
    }
  }
  if (options.paths.empty()) {
    error = "a log file or directory is required";
    return false;
  }
  return true;
}

bool Matches(const Options& options, const talos::launcher::MergedRecord& r,
             std::int64_t base_ns) {
  if (!options.nodes.empty() &&
      std::find(options.nodes.begin(), options.nodes.end(), r.process_name()) ==
          options.nodes.end()) {
    return false;
  }
  if (!options.topics.empty() &&
      !std::any_of(options.topics.begin(), options.topics.end(),
                   [&](const std::string& pattern) {
                     return pattern.ends_with('*')
                                ? r.source_name().starts_with(
                                      std::string_view{pattern}.substr(
                                          0, pattern.size() - 1))
                                : r.source_name() == pattern;
                   })) {
    return false;
  }
  const double ms = ToMillis(base_ns, r.header.event_time_ns);
  return ms >= options.since_ms && ms <= options.until_ms;
}

std::string JsonString(std::string_view value) {
  std::string result{"\""};
  constexpr char hex[] = "0123456789abcdef";
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') {
      result += '\\';
      result += static_cast<char>(c);
    } else if (c < 0x20) {
      result += "\\u00";
      result += hex[c >> 4];
      result += hex[c & 15];
    } else {
      result += static_cast<char>(c);
    }
  }
  result += '"';
  return result;
}

std::string CsvString(std::string_view value) {
  if (value.find_first_of(",\"\r\n") == std::string_view::npos) {
    return std::string{value};
  }
  std::string result{"\""};
  for (const char c : value) {
    if (c == '"') result += '"';
    result += c;
  }
  result += '"';
  return result;
}

void ExportPayloadHex(std::span<const std::byte> payload) {
  constexpr char hex[] = "0123456789abcdef";
  std::cout << '"';
  for (const auto byte : payload) {
    const auto value = std::to_integer<unsigned char>(byte);
    std::cout << hex[value >> 4] << hex[value & 15];
  }
  std::cout << '"';
}

// The hardware packet topics' decoded JSON object (hardware_record.h), or
// empty for any other record.
std::string HardwareJson(const talos::launcher::MergedRecord& r,
                         const talos::launcher::DeviceNames* names) {
  if (r.source == nullptr || r.payload.empty()) return "";
  const auto kind =
      talos::launcher::HardwareRecordOf(r.source_name(), r.source->schema.name);
  if (kind == talos::launcher::HardwareRecord::kNone) return "";
  return talos::launcher::HardwareRecordJson(kind, r.payload, names);
}

// Defined after DecodedFields, which it walks.
std::string AlertsJson(std::string_view topic, const std::string& decoded);

void ExportRecord(const talos::launcher::MergedRecord& r, std::int64_t base_ns,
                  bool json, const talos::launcher::DeviceNames* names) {
  const std::string schema = r.source == nullptr ? "" : r.source->schema.name;
  const std::string hardware = HardwareJson(r, names);
  const std::string decoded =
      !schema.empty() && !r.payload.empty()
          ? talos::schema::DecodeToText(r.source->schema, r.payload)
          : "";
  const auto quote = json ? JsonString : CsvString;
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
  if (json) {
    std::cout << "{\"process\":" << quote(r.process_name())
              << ",\"dispatch_index\":" << r.header.dispatch_index
              << ",\"kind\":" << quote(EventKindName(r.header.kind))
              << ",\"source_id\":" << r.header.source_id
              << ",\"topic\":" << quote(r.source_name())
              << ",\"event_ms\":" << ToMillis(base_ns, r.header.event_time_ns)
              << ",\"now_ms\":" << ToMillis(base_ns, r.header.now_ns)
              << ",\"publish_ms\":"
              << ToMillis(base_ns, r.header.publish_time_ns)
              << ",\"sequence\":" << r.header.sequence
              << ",\"aux\":" << r.header.aux
              << ",\"payload_bytes\":" << r.header.payload_bytes
              << ",\"schema\":" << quote(schema) << ",\"payload_hex\":";
    ExportPayloadHex(r.payload);
    std::cout << ",\"decoded\":" << quote(decoded);
    if (const auto alerts = AlertsJson(r.source_name(), decoded);
        !alerts.empty()) {
      std::cout << ",\"alerts\":" << alerts;
    }
    if (!hardware.empty()) std::cout << ",\"hardware\":" << hardware;
    if (r.source_name() == talos::hardware::kDriverStationTopic &&
        r.payload.size() == sizeof(talos::hardware::Packet)) {
      talos::hardware::Packet packet{};
      std::memcpy(&packet, r.payload.data(), sizeof(packet));
      talos::driver_station::DriverStationData ds;
      if (talos::driver_station::Decode(packet.bytes(), ds)) {
        std::cout << ",\"ds_enabled\":"
                  << ((ds.fms.flags & talos::driver_station::kEnabled) != 0
                          ? "true"
                          : "false");
      }
    }
    std::cout << "}\n";
  } else {
    std::cout << quote(r.process_name()) << ',' << r.header.dispatch_index
              << ',' << quote(EventKindName(r.header.kind)) << ','
              << r.header.source_id << ',' << quote(r.source_name()) << ','
              << ToMillis(base_ns, r.header.event_time_ns) << ','
              << ToMillis(base_ns, r.header.now_ns) << ','
              << ToMillis(base_ns, r.header.publish_time_ns) << ','
              << r.header.sequence << ',' << r.header.aux << ','
              << r.header.payload_bytes << ',' << quote(schema) << ','
              << quote(decoded) << ',' << quote(hardware) << "\r\n";
  }
}

// The "config_path" string of a session's manifest.json (session.cc
// SessionManifest::ToJson, escaped by introspect::AppendJsonString), or empty.
std::string ManifestConfigPath(const std::filesystem::path& session) {
  std::ifstream stream{session / "manifest.json"};
  const std::string text{std::istreambuf_iterator<char>{stream}, {}};
  constexpr std::string_view kKey = "\"config_path\"";
  auto at = text.find(kKey);
  if (at == std::string::npos) return "";
  at = text.find('"', text.find(':', at + kKey.size()));
  if (at == std::string::npos) return "";
  std::string value;
  for (std::size_t i = at + 1; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '"') return value;
    if (c != '\\') {
      value += c;
    } else if (i + 1 < text.size() && text[i + 1] != 'u') {
      value += text[++i];
    } else if (i + 5 < text.size()) {
      value += static_cast<char>(
          std::strtol(text.substr(i + 2, 4).c_str(), nullptr, 16));
      i += 5;
    }
  }
  return "";
}

// Every scalar of a DecodeToText string (talOS/schema/decode.h) with its
// field path: `{a: 1, b: {c: [2, 3]}}` gives a=1, b.c[0]=2 and b.c[1]=3. A
// `<...>` (a payload the schema could not decode) is one scalar. Text of any
// other shape ends the walk early; it never reads past the string.
class DecodedFields {
 public:
  using Visit = std::function<void(const std::string&, std::string_view)>;

  static void Walk(std::string_view text, const Visit& visit) {
    DecodedFields walker{text, visit};
    std::string path;
    walker.Value(path);
  }

 private:
  DecodedFields(std::string_view text, const Visit& visit)
      : text_{text}, visit_{visit} {}

  // Past the separator after a member; false when nothing was consumed
  // since `before`, which only malformed text does.
  bool Next(std::size_t before) {
    if (text_.substr(at_).starts_with(", ")) at_ += 2;
    return at_ > before;
  }

  void Value(std::string& path) {
    if (at_ >= text_.size()) return;
    const std::size_t length = path.size();
    if (text_[at_] == '{') {
      ++at_;
      while (at_ < text_.size() && text_[at_] != '}') {
        const std::size_t before = at_;
        const std::size_t colon = text_.find(": ", at_);
        if (colon == std::string_view::npos) {
          at_ = text_.size();
          return;
        }
        if (length != 0) path += '.';
        path.append(text_.substr(at_, colon - at_));
        at_ = colon + 2;
        Value(path);
        path.resize(length);
        if (!Next(before)) {
          at_ = text_.size();
          return;
        }
      }
      ++at_;
    } else if (text_[at_] == '[') {
      ++at_;
      for (std::size_t i = 0; at_ < text_.size() && text_[at_] != ']'; ++i) {
        const std::size_t before = at_;
        path += "[" + std::to_string(i) + "]";
        Value(path);
        path.resize(length);
        if (!Next(before)) {
          at_ = text_.size();
          return;
        }
      }
      ++at_;
    } else {
      std::size_t end = text_[at_] == '<' ? text_.find('>', at_)
                                          : text_.find_first_of(",}]", at_);
      if (end == std::string_view::npos) {
        end = text_.size();
      } else if (text_[at_] == '<') {
        ++end;
      }
      visit_(path, text_.substr(at_, end - at_));
      at_ = end;
    }
  }

  std::string_view text_;
  const Visit& visit_;
  std::size_t at_{0};
};

const char* AlertLevelName(talos::introspect::naming::AlertLevel level) {
  using talos::introspect::naming::AlertLevel;
  switch (level) {
    case AlertLevel::kError:
      return "error";
    case AlertLevel::kWarning:
      return "warning";
    case AlertLevel::kInfo:
      return "info";
    case AlertLevel::kNone:
      break;
  }
  return "none";
}

// The alerts of a record of a `status` topic (NAMING.md): each bool field,
// nested ones included, whose own name ParseAlertField reads, as a JSON array
// of {name: its field path, level, active}; empty for any other record, or a
// status message with no alert field.
std::string AlertsJson(std::string_view topic, const std::string& decoded) {
  const auto segments = talos::introspect::naming::detail::Segments(topic);
  if (segments.size() < 2 || segments[1] != "status" || decoded.empty()) {
    return "";
  }
  std::string alerts;
  DecodedFields::Walk(
      decoded, [&alerts](const std::string& path, std::string_view token) {
        if (token != "true" && token != "false") return;  // Not a bool.
        std::string_view field{path};
        if (const auto dot = field.rfind('.'); dot != std::string_view::npos) {
          field.remove_prefix(dot + 1);
        }
        const auto alert = talos::introspect::naming::ParseAlertField(field);
        if (!alert) return;
        alerts += alerts.empty() ? "[" : ",";
        alerts.append("{\"name\":")
            .append(JsonString(path))
            .append(",\"level\":\"")
            .append(AlertLevelName(alert->level))
            .append("\",\"active\":")
            .append(token)
            .append("}");
      });
  if (!alerts.empty()) alerts += ']';
  return alerts;
}

// --format summary: one topic's records folded in as they stream past, so a
// session of any length takes memory in proportion to its topics and their
// fields, not its records.
class TopicSummary {
 public:
  void Add(const talos::launcher::MergedRecord& r, std::int64_t base_ns,
           const talos::launcher::DeviceNames* names) {
    ++records_[{r.process_name(), EventKindName(r.header.kind)}];
    // A timer's record has no publication; its event time stands in.
    const std::int64_t published = r.header.publish_time_ns != 0
                                       ? r.header.publish_time_ns
                                       : r.header.event_time_ns;
    first_ms_ = std::min(first_ms_, ToMillis(base_ns, published));
    last_ms_ = std::max(last_ms_, ToMillis(base_ns, published));
    if (r.source == nullptr || r.payload.empty()) return;
    if (const auto hardware = HardwareJson(r, names); !hardware.empty()) {
      hardware_ = hardware;
    }
    if (r.source->schema.name.empty()) return;
    schema_ = r.source->schema.name;
    latest_ = talos::schema::DecodeToText(r.source->schema, r.payload);
    if (latest_.starts_with("bytes=")) return;  // A byte layout: no fields.
    DecodedFields::Walk(
        latest_, [this](const std::string& path, std::string_view token) {
          Widen(path, token);
        });
  }

  void Print(const std::string& topic) const {
    std::printf("topic %s%s%s\n", topic.c_str(), schema_.empty() ? "" : " ",
                schema_.c_str());
    for (const auto& [key, count] : records_) {
      std::printf("  records %s %s %zu\n", key.first.c_str(),
                  key.second.c_str(), count);
    }
    std::printf("  first_ms %.3f\n  last_ms %.3f\n", first_ms_, last_ms_);
    if (!latest_.empty()) std::printf("  latest %s\n", latest_.c_str());
    if (!hardware_.empty()) std::printf("  hardware %s\n", hardware_.c_str());
    for (const auto& range : ranges_) {
      std::printf("  range %s %s %s\n", range.path.c_str(),
                  range.min_text.c_str(), range.max_text.c_str());
    }
  }

 private:
  struct Range {
    std::string path;
    double min;
    double max;
    std::string min_text;  // As decoded, so no digit is lost in a double.
    std::string max_text;
  };

  // A number or a bool (false below true) widens its field's range; an
  // enum's name or an undecodable payload has none.
  void Widen(const std::string& path, std::string_view token) {
    double value = 0;
    if (token == "true" || token == "false") {
      value = token == "true" ? 1 : 0;
    } else {
      // strtod, not from_chars: libc++ has no floating-point from_chars
      // before macOS 26.
      const std::string text{token};
      char* end = nullptr;
      value = std::strtod(text.c_str(), &end);
      if (text.empty() || end != text.c_str() + text.size() ||
          std::isnan(value)) {
        return;
      }
    }
    const auto [at, added] = range_index_.try_emplace(path, ranges_.size());
    if (added) {
      ranges_.push_back(
          {path, value, value, std::string{token}, std::string{token}});
      return;
    }
    Range& range = ranges_[at->second];
    if (value < range.min) {
      range.min = value;
      range.min_text = token;
    }
    if (value > range.max) {
      range.max = value;
      range.max_text = token;
    }
  }

  std::map<std::pair<std::string, std::string>, std::size_t> records_;
  double first_ms_{std::numeric_limits<double>::infinity()};
  double last_ms_{-std::numeric_limits<double>::infinity()};
  std::string schema_;
  std::string latest_;
  std::string hardware_;
  std::vector<Range> ranges_;  // In the order their fields first appeared.
  std::map<std::string, std::size_t> range_index_;
};

// A relative config path is the launcher's or the user's, relative to where
// they ran: under `bazel run` that is BUILD_WORKING_DIRECTORY, since this
// binary then runs in its runfiles directory; otherwise the working directory.
std::string ResolveConfigPath(const std::string& path) {
  const char* base = std::getenv("BUILD_WORKING_DIRECTORY");
  if (path.empty() || std::filesystem::path{path}.is_absolute() ||
      base == nullptr || *base == '\0') {
    return path;
  }
  return (std::filesystem::path{base} / path).string();
}

// Device names for the hardware packets: from --config, else from the config
// a session directory's manifest names, when that file can still be read.
// Without one, devices are identified by logical id only.
std::optional<talos::launcher::DeviceNames> LoadDeviceNames(
    const Options& options) {
  std::string path = options.config;
  if (path.empty()) {
    for (const auto& input : options.paths) {
      std::error_code ec;
      if (!std::filesystem::is_directory(input, ec)) continue;
      path = ManifestConfigPath(input);
      if (!path.empty()) break;
    }
    if (path.empty()) return std::nullopt;
  }
  path = ResolveConfigPath(path);
  try {
    return talos::launcher::DeviceNames::FromConfig(
        talos::config::ParseRobotConfig(path).hardware);
  } catch (const std::exception& e) {
    std::fprintf(stderr,
                 "log_dump: hardware devices by id only, config %s: %s\n",
                 path.c_str(), e.what());
    return std::nullopt;
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  std::string error;
  if (!ParseOptions(argc, argv, options, error)) {
    std::fprintf(stderr,
                 "log_dump: %s\nusage: %s [--node name[,name...]] "
                 "[--topic pattern[,pattern...]] [--since-ms ms] "
                 "[--until-ms ms] [--format text|jsonl|csv|summary] "
                 "[--config robot.toml] "
                 "<log-file-or-dir> [more-log-files...]\n",
                 error.c_str(), argv[0]);
    return 1;
  }

  const std::vector<std::string> log_files =
      talos::launcher::LogInputFiles(options.paths);
  if (log_files.empty()) {
    std::fprintf(stderr, "log_dump: no .tlog files found\n");
    return 1;
  }

  const auto device_names = LoadDeviceNames(options);
  const talos::launcher::DeviceNames* names =
      device_names ? &*device_names : nullptr;

  // The session is streamed twice, a fixed window per log, never held: once
  // for the baseline every time is relative to (the earliest event time of
  // any record, not necessarily the first merged) and the record count,
  // warning about damage then, before any output; once to print.
  using talos::launcher::MergedRecord;
  std::int64_t base_ns = 0;
  std::size_t total_count = 0;
  if (!talos::launcher::MergeLogs(
          log_files,
          [&](const MergedRecord& r) {
            if (total_count++ == 0 || r.header.event_time_ns < base_ns) {
              base_ns = r.header.event_time_ns;
            }
            return true;
          },
          error)) {
    std::fprintf(stderr, "log_dump error: %s\n", error.c_str());
    return 1;
  }
  const auto print = [&](const std::function<void(const MergedRecord&)>& each) {
    if (!talos::launcher::MergeLogs(
            log_files,
            [&](const MergedRecord& r) {
              if (Matches(options, r, base_ns)) each(r);
              return true;
            },
            error, talos::launcher::MergeWarnings::kQuiet)) {
      std::fprintf(stderr, "log_dump error: %s\n", error.c_str());
      return false;
    }
    return true;
  };

  if (options.format == "summary") {
    std::map<std::string, TopicSummary, std::less<>> topics;
    const bool read = print([&](const MergedRecord& r) {
      if (r.header.kind == static_cast<std::uint16_t>(EventKind::EXIT)) return;
      auto at = topics.find(r.source_name());
      if (at == topics.end()) {
        at = topics.emplace(std::string{r.source_name()}, TopicSummary{}).first;
      }
      at->second.Add(r, base_ns, names);
    });
    if (!read) return 1;
    const char* separator = "";
    for (const auto& [topic, summary] : topics) {
      std::printf("%s", separator);
      separator = "\n";
      summary.Print(topic);
    }
    return 0;
  }

  if (options.format != "text") {
    if (options.format == "csv") {
      std::cout
          << "process,dispatch_index,kind,source_id,topic,event_ms,now_ms,"
             "publish_ms,sequence,aux,payload_bytes,schema,decoded,"
             "hardware\r\n";
    }
    return print([&](const MergedRecord& r) {
      ExportRecord(r, base_ns, options.format == "jsonl", names);
    })
               ? 0
               : 1;
  }

  std::printf("merged log dump: %zu files, %zu total records\n",
              log_files.size(), total_count);
  for (std::size_t i = 0; i < log_files.size(); ++i) {
    std::printf("  [%zu] %s\n", i, log_files[i].c_str());
  }
  std::printf("\nrecords (monotonic time ordered):\n");

  std::map<std::string, std::size_t> counts_by_process;
  std::map<std::uint16_t, std::size_t> counts_by_kind;
  std::size_t matched_count = 0;

  const bool printed = print([&](const MergedRecord& r) {
    ++matched_count;
    ++counts_by_process[r.process_name()];
    ++counts_by_kind[r.header.kind];

    const bool has_source =
        r.header.kind != static_cast<std::uint16_t>(EventKind::EXIT);

    const std::string source_id =
        has_source ? std::to_string(r.header.source_id) : std::string{"-"};
    const std::string source_name =
        has_source ? std::string{r.source_name()} : std::string{"-"};

    std::printf(
        "  [%-12s] #%-6llu %-8s src=%-3s (%-16s) event=%9.3fms now=%9.3fms "
        "publish=%9.3fms seq=%-6llu aux=%-6u payload=%uB\n",
        r.process_name().c_str(),
        static_cast<unsigned long long>(r.header.dispatch_index),
        EventKindName(r.header.kind), source_id.c_str(), source_name.c_str(),
        ToMillis(base_ns, r.header.event_time_ns),
        ToMillis(base_ns, r.header.now_ns),
        ToMillis(base_ns, r.header.publish_time_ns),
        static_cast<unsigned long long>(r.header.sequence), r.header.aux,
        r.header.payload_bytes);
    // The topic's schema travels in the log (talOS/events/log/format.h), so
    // every payload decodes without the build that recorded it.
    if (r.source != nullptr && !r.source->schema.name.empty() &&
        !r.payload.empty()) {
      std::printf(
          "      %s %s\n", r.source->schema.name.c_str(),
          talos::schema::DecodeToText(r.source->schema, r.payload).c_str());
    }
    if (const auto hardware = HardwareJson(r, names); !hardware.empty()) {
      std::printf("      hardware %s\n", hardware.c_str());
    }
  });
  if (!printed) return 1;

  std::printf("\nsummary: %zu records across %zu processes\n", matched_count,
              counts_by_process.size());
  for (const auto& [proc, count] : counts_by_process) {
    std::printf("  process '%s': %zu records\n", proc.c_str(), count);
  }
  for (const auto& [kind, count] : counts_by_kind) {
    std::printf("  kind '%s': %zu records\n", EventKindName(kind), count);
  }

  return 0;
}
