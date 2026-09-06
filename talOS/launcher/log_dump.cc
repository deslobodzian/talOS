#include <cstdint>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/launcher/launcher.h"

namespace {

using talos::event::EventKind;

double ToMillis(std::int64_t start_ns, std::int64_t ns) {
  return static_cast<double>(ns - start_ns) / 1e6;
}

const char* EventKindName(std::uint16_t kind) {
  return talos::event::to_string(static_cast<EventKind>(kind));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <log-file-or-dir> [more-log-files...]\n",
                 argv[0]);
    return 1;
  }

  std::vector<std::string> log_files;
  for (int i = 1; i < argc; ++i) {
    auto found = talos::launcher::FindLogFiles(argv[i]);
    log_files.insert(log_files.end(), found.begin(), found.end());
  }

  if (log_files.empty()) {
    std::fprintf(stderr, "log_dump: no .tlog files found\n");
    return 1;
  }

  std::vector<talos::launcher::MergedRecord> records;
  std::string error;
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

  std::printf("merged log dump: %zu files, %zu total records\n",
              log_files.size(), records.size());
  for (std::size_t i = 0; i < log_files.size(); ++i) {
    std::printf("  [%zu] %s\n", i, log_files[i].c_str());
  }
  std::printf("\nrecords (monotonic time ordered):\n");

  std::map<std::string, std::size_t> counts_by_process;
  std::map<std::uint16_t, std::size_t> counts_by_kind;

  for (const auto& r : records) {
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
  }

  std::printf("\nsummary: %zu records across %zu processes\n", records.size(),
              counts_by_process.size());
  for (const auto& [proc, count] : counts_by_process) {
    std::printf("  process '%s': %zu records\n", proc.c_str(), count);
  }
  for (const auto& [kind, count] : counts_by_kind) {
    std::printf("  kind '%s': %zu records\n", EventKindName(kind), count);
  }

  return 0;
}
