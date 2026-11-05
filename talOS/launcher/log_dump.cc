#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/launcher/launcher.h"
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
        arg != "--until-ms" && arg != "--format") {
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
    } else if (arg == "--format") {
      if (value != "text" && value != "jsonl" && value != "csv") {
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
      std::find(options.nodes.begin(), options.nodes.end(), r.process_name) ==
          options.nodes.end()) {
    return false;
  }
  if (!options.topics.empty() &&
      !std::any_of(options.topics.begin(), options.topics.end(),
                   [&](const std::string& pattern) {
                     return pattern.ends_with('*')
                                ? r.source_name.starts_with(
                                      std::string_view{pattern}.substr(
                                          0, pattern.size() - 1))
                                : r.source_name == pattern;
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

void ExportRecord(const talos::launcher::MergedRecord& r, std::int64_t base_ns,
                  bool json) {
  const std::string schema = r.source == nullptr ? "" : r.source->schema.name;
  const std::string decoded =
      !schema.empty() && !r.payload.empty()
          ? talos::schema::DecodeToText(r.source->schema, r.payload)
          : "";
  const auto quote = json ? JsonString : CsvString;
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
  if (json) {
    std::cout << "{\"process\":" << quote(r.process_name)
              << ",\"dispatch_index\":" << r.header.dispatch_index
              << ",\"kind\":" << quote(EventKindName(r.header.kind))
              << ",\"source_id\":" << r.header.source_id
              << ",\"topic\":" << quote(r.source_name)
              << ",\"event_ms\":" << ToMillis(base_ns, r.header.event_time_ns)
              << ",\"now_ms\":" << ToMillis(base_ns, r.header.now_ns)
              << ",\"sequence\":" << r.header.sequence
              << ",\"aux\":" << r.header.aux
              << ",\"payload_bytes\":" << r.header.payload_bytes
              << ",\"schema\":" << quote(schema)
              << ",\"decoded\":" << quote(decoded) << "}\n";
  } else {
    std::cout << quote(r.process_name) << ',' << r.header.dispatch_index << ','
              << quote(EventKindName(r.header.kind)) << ','
              << r.header.source_id << ',' << quote(r.source_name) << ','
              << ToMillis(base_ns, r.header.event_time_ns) << ','
              << ToMillis(base_ns, r.header.now_ns) << ',' << r.header.sequence
              << ',' << r.header.aux << ',' << r.header.payload_bytes << ','
              << quote(schema) << ',' << quote(decoded) << "\r\n";
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
                 "[--until-ms ms] [--format text|jsonl|csv] "
                 "<log-file-or-dir> [more-log-files...]\n",
                 error.c_str(), argv[0]);
    return 1;
  }

  std::vector<std::string> log_files;
  for (const auto& path : options.paths) {
    auto found = talos::event::log::FindLogFiles(path);
    log_files.insert(log_files.end(), found.begin(), found.end());
  }

  if (log_files.empty()) {
    std::fprintf(stderr, "log_dump: no .tlog files found\n");
    return 1;
  }

  std::vector<talos::launcher::MergedRecord> records;
  if (!talos::launcher::MergeLogs(log_files, records, error)) {
    std::fprintf(stderr, "log_dump error: %s\n", error.c_str());
    return 1;
  }

  // Find baseline earliest monotonic timestamp across records
  int64_t base_ns = records.empty() ? 0 : records.front().header.event_time_ns;
  for (const auto& r : records) {
    if (r.header.event_time_ns < base_ns) {
      base_ns = r.header.event_time_ns;
    }
  }

  if (options.format != "text") {
    if (options.format == "csv") {
      std::cout
          << "process,dispatch_index,kind,source_id,topic,event_ms,now_ms,"
             "sequence,aux,payload_bytes,schema,decoded\r\n";
    }
    for (const auto& r : records) {
      if (Matches(options, r, base_ns)) {
        ExportRecord(r, base_ns, options.format == "jsonl");
      }
    }
    return 0;
  }

  std::printf("merged log dump: %zu files, %zu total records\n",
              log_files.size(), records.size());
  for (std::size_t i = 0; i < log_files.size(); ++i) {
    std::printf("  [%zu] %s\n", i, log_files[i].c_str());
  }
  std::printf("\nrecords (monotonic time ordered):\n");

  std::map<std::string, std::size_t> counts_by_process;
  std::map<std::uint16_t, std::size_t> counts_by_kind;
  std::size_t matched_count = 0;

  for (const auto& r : records) {
    if (!Matches(options, r, base_ns)) continue;
    ++matched_count;
    ++counts_by_process[r.process_name];
    ++counts_by_kind[r.header.kind];

    const bool has_source =
        r.header.kind != static_cast<std::uint16_t>(EventKind::EXIT);

    const std::string source_id =
        has_source ? std::to_string(r.header.source_id) : std::string{"-"};
    const std::string source_name =
        has_source ? r.source_name : std::string{"-"};

    std::printf(
        "  [%-12s] #%-6llu %-8s src=%-3s (%-16s) event=%9.3fms now=%9.3fms "
        "seq=%-6llu aux=%-6u payload=%uB\n",
        r.process_name.c_str(),
        static_cast<unsigned long long>(r.header.dispatch_index),
        EventKindName(r.header.kind), source_id.c_str(), source_name.c_str(),
        ToMillis(base_ns, r.header.event_time_ns),
        ToMillis(base_ns, r.header.now_ns),
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
  }

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
