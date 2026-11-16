#pragma once

#include <cstddef>
#include <string>

#include "talOS/events/log/log_reader.h"

// Tests only: makes a log with a GAP record (format.h) out of one without,
// for the readers, replay and the tools to be tested on.
namespace talos::event::log {

// Copies the log at `from` to `to` with records [first, end) (0-based) replaced
// by the one GAP record the writer would write for them (cause POOL_FULL,
// closed at flush when `end` is past the last record). Throws
// std::invalid_argument when record `end` does not start a dispatch.
//
// As the writer does: a final EXIT in the range is not dropped but held, and
// follows the GAP, which takes its now_ns; the gap is flagged
// GAP_SPLITS_DISPATCH when record `first - 1` has record `first`'s nonzero
// dispatch index; and its start is the earliest event time of a dropped
// TIMER, MESSAGE or SEND and publish time of a dropped MESSAGE or SEND
// (first_lost_ns when only operation records were dropped). The bytes before
// record `first`, and from record `end` (or the held EXIT) on, are copied
// as they are, damage included. Also throws std::invalid_argument for an
// empty range, one past the log's records, or one holding an EXIT that is
// not the log's last record.
LogGap CopyWithGap(const std::string& from, const std::string& to,
                   std::size_t first, std::size_t end);

}  // namespace talos::event::log
