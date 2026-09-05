#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>

#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/manifest.h"

namespace {

using talos::event::EventKind;
using talos::event::Manifest;
using talos::event::log::LogReader;

double ToMillis(std::int64_t start_ns, std::int64_t ns) {
  return static_cast<double>(ns - start_ns) / 1e6;
}

std::string ResolveSourceName(const Manifest& manifest,
                              std::uint16_t source_id) {
  for (const auto& reg : manifest) {
    if (reg.id == source_id) {
      return reg.name;
    }
  }
  return "?";
}

const char* EventKindName(std::uint16_t kind) {
  return talos::event::to_string(static_cast<EventKind>(kind));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <log-path>\n", argv[0]);
    return 1;
  }

  std::unique_ptr<LogReader> reader;
  try {
    reader = std::make_unique<LogReader>(argv[1]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "log_dump: failed to open %s: %s\n", argv[1],
                 e.what());
    return 1;
  }

  const std::int64_t start_ns = reader->start_time().nanos();

  std::printf("process: %s\n", reader->process_name().c_str());
  std::printf("format version: %u\n", reader->format_version());
  std::printf("start monotonic: %lld ns\n", static_cast<long long>(start_ns));
  std::printf("start wall: %lld ns\n",
              static_cast<long long>(reader->start_wall_ns()));

  std::printf("\nmanifest (%zu sources):\n", reader->manifest().size());
  std::printf("  %-5s %-9s %-40s %-9s %s\n", "id", "kind", "name", "msg_bytes",
              "period_ns");
  for (const auto& reg : reader->manifest()) {
    std::printf("  %-5u %-9s %-40s %-9u %lld\n", reg.id,
                talos::event::to_string(reg.kind), reg.name.c_str(),
                reg.message_bytes, static_cast<long long>(reg.period_ns));
  }

  std::printf("\nrecords:\n");
  std::unordered_map<std::uint16_t, std::size_t> counts_by_kind;
  std::size_t total = 0;

  LogReader::Record record;
  while (reader->next(record)) {
    ++total;
    ++counts_by_kind[record.header.kind];
    // An exit record belongs to the loop, not to a source, so resolving its
    // id would name an unrelated topic.
    const bool has_source =
        record.header.kind != static_cast<std::uint16_t>(EventKind::EXIT);

    const std::string source_name =
        has_source
            ? ResolveSourceName(reader->manifest(), record.header.source_id)
            : std::string{"-"};

    const std::string source_id =
        has_source ? std::to_string(record.header.source_id) : std::string{"-"};

    std::printf(
        "  #%-6llu %-8s src=%-3s (%-16s) event=%9.3fms now=%9.3fms "
        "seq=%-6llu aux=%-6u payload=%uB\n",
        static_cast<unsigned long long>(record.header.dispatch_index),
        EventKindName(record.header.kind), source_id.c_str(),
        source_name.c_str(), ToMillis(start_ns, record.header.event_time_ns),
        ToMillis(start_ns, record.header.now_ns),
        static_cast<unsigned long long>(record.header.sequence),
        record.header.aux, record.header.payload_bytes);
  }

  std::printf("\nsummary: %zu records", total);
  for (const auto& [kind, count] : counts_by_kind) {
    std::printf(", %s=%zu", EventKindName(kind), count);
  }
  std::printf(", truncated=%s\n", reader->truncated() ? "yes" : "no");

  return 0;
}
