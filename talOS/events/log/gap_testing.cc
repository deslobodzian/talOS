#include "talOS/events/log/gap_testing.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/format.h"

namespace talos::event::log {

namespace {

bool Is(const RecordHeader& header, EventKind kind) {
  return header.kind == static_cast<std::uint16_t>(kind);
}

std::vector<char> ReadBytes(const std::string& path) {
  std::ifstream in{path, std::ios::binary};
  if (!in) throw std::runtime_error("CopyWithGap: cannot open " + path);
  return {std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{}};
}

}  // namespace

LogGap CopyWithGap(const std::string& from, const std::string& to,
                   std::size_t first, std::size_t end) {
  const std::vector<char> bytes = ReadBytes(from);
  struct Kept {
    RecordHeader header;
    std::uint64_t offset;
  };
  std::vector<Kept> records;
  {
    LogReader reader{from};
    LogReader::Record record;
    while (reader.next(record))
      records.push_back({record.header, record.offset});
  }
  const std::size_t n = records.size();
  const std::string range =
      "records [" + std::to_string(first) + ", " + std::to_string(end) + ")";
  if (first >= end || end > n) {
    throw std::invalid_argument("CopyWithGap: " + range + " of a log of " +
                                std::to_string(n) + " records");
  }
  // An EXIT that finds no room is held, never dropped, and written after the
  // GAP when the writer is flushed.
  const bool held_exit = end == n && Is(records[n - 1].header, EventKind::EXIT);
  const std::size_t dropped_end = held_exit ? n - 1 : end;
  if (first >= dropped_end) {
    throw std::invalid_argument("CopyWithGap: " + range +
                                " holds only the EXIT, which is never dropped");
  }
  for (std::size_t i = first; i < dropped_end; ++i) {
    if (Is(records[i].header, EventKind::EXIT)) {
      throw std::invalid_argument("CopyWithGap: record " + std::to_string(i) +
                                  " is an EXIT that is not the last record");
    }
  }
  // Capture resumes only at a record of no dispatch or of another dispatch
  // than the last dropped one.
  if (end < n && records[end].header.dispatch_index != 0 &&
      records[end].header.dispatch_index ==
          records[end - 1].header.dispatch_index) {
    throw std::invalid_argument(
        "CopyWithGap: record " + std::to_string(end) + " does not start a " +
        "dispatch (dispatch " +
        std::to_string(records[end].header.dispatch_index) +
        " has a dropped record)");
  }

  GapRecord detail{};
  detail.cause = static_cast<std::uint16_t>(GapCause::POOL_FULL);
  detail.first_lost_ns = records[first].header.now_ns;
  detail.last_lost_ns = std::numeric_limits<std::int64_t>::min();
  std::int64_t start = std::numeric_limits<std::int64_t>::max();
  for (std::size_t i = first; i < dropped_end; ++i) {
    const RecordHeader& h = records[i].header;
    ++detail.records;
    detail.bytes += sizeof(RecordHeader) + h.payload_bytes;
    const bool opens = Is(h, EventKind::TIMER) || Is(h, EventKind::MESSAGE);
    const bool send = Is(h, EventKind::SEND);
    if (opens) ++detail.dispatches;
    if (send) ++detail.sends;
    if (opens || send) start = std::min(start, h.event_time_ns);
    if (Is(h, EventKind::MESSAGE) || send) {
      start = std::min(start, h.publish_time_ns);
    }
    detail.last_lost_ns = std::max(detail.last_lost_ns, h.now_ns);
    detail.last_dispatch_index = h.dispatch_index;
  }
  if (start == std::numeric_limits<std::int64_t>::max()) {
    start = detail.first_lost_ns;
  }
  const std::uint64_t first_dispatch = records[first].header.dispatch_index;
  if (first_dispatch != 0 && first > 0 &&
      records[first - 1].header.dispatch_index == first_dispatch) {
    detail.flags |= GAP_SPLITS_DISPATCH;
  }
  if (end == n) detail.flags |= GAP_CLOSED_AT_FLUSH;

  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(EventKind::GAP);
  header.source_id = GAP_SOURCE_ID;
  header.dispatch_index = first_dispatch;
  header.event_time_ns = start;
  if (end < n) {
    header.now_ns = records[end].header.now_ns;
  } else if (held_exit) {
    header.now_ns = records[n - 1].header.now_ns;
  } else {
    header.now_ns = detail.last_lost_ns;
  }
  const std::span<const std::byte> stored = SealRecord(
      header, std::span<const std::byte>{
                  reinterpret_cast<const std::byte*>(&detail), sizeof(detail)});

  // Where the copy resumes: record `end`, the held EXIT, or what follows the
  // last record (a damaged tail).
  std::uint64_t resume_at = 0;
  if (end < n) {
    resume_at = records[end].offset;
  } else if (held_exit) {
    resume_at = records[n - 1].offset;
  } else {
    resume_at = records[n - 1].offset + sizeof(RecordHeader) +
                records[n - 1].header.stored_bytes;
  }

  std::ofstream out{to, std::ios::binary | std::ios::trunc};
  out.write(bytes.data(), static_cast<std::streamsize>(records[first].offset));
  out.write(reinterpret_cast<const char*>(&header), sizeof(header));
  out.write(reinterpret_cast<const char*>(stored.data()),
            static_cast<std::streamsize>(stored.size()));
  out.write(bytes.data() + resume_at,
            static_cast<std::streamsize>(bytes.size() - resume_at));
  out.close();
  if (!out) throw std::runtime_error("CopyWithGap: cannot write " + to);

  return LogGap{.first_dispatch_index = first_dispatch,
                .start_ns = start,
                .resumed_ns = header.now_ns,
                .detail = detail,
                .detailed = true};
}

}  // namespace talos::event::log
