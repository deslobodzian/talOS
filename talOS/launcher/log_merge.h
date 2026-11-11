#pragma once

// Merging one session's event logs into one time-ordered stream (log_dump).

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/format.h"

namespace talos::launcher {

struct MergedRecord {
  std::string process_name;
  std::string source_name;
  // The record's source in its log's manifest, schema included (format.h);
  // null for an EXIT record or an id the manifest lacks.
  std::shared_ptr<const event::Registration> source;
  event::log::RecordHeader header;
  std::vector<std::byte> payload;
};

// `source_id`'s entry in `manifest`, sharing its ownership; null if none.
std::shared_ptr<const event::Registration> ResolveSource(
    const std::shared_ptr<const event::Manifest>& manifest,
    std::uint16_t source_id);

// Merges logs of one session by event time, then now, then dispatch index.
bool MergeLogs(const std::vector<std::string>& log_paths,
               std::vector<MergedRecord>& out_records, std::string& error);

}  // namespace talos::launcher
