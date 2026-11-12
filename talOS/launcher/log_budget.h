#pragma once

// How a session's event-log budget is divided among its members: each
// member's expected stored bytes per second, from what it declared
// (--describe) and the config, shares proportional to them, and the window
// of record time those shares hold, after which every capped log ends a
// segment, so every capped member keeps a tail of the session of about the
// same length (REQ-OPS-32).

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/launcher/describe_graph.h"

namespace talos::launcher {

// A topic's publications per second and the payload bytes a log stores for
// each (record header excluded).
struct TopicLoad {
  double per_second{0};
  double stored_bytes{0};
};
using TopicLoads = std::map<std::string, TopicLoad, std::less<>>;

// A member's tick: its subsystem's `period_us`, else `[robot] period_us`,
// else 5000.
int64_t MemberPeriodUs(const config::RobotConfig& config,
                       std::string_view member);

// Every declared topic's load: a user topic at its writer's 1e6 / period_us
// and message_bytes; /hw/state once per gateway tick ([robot] period_us),
// /hw/state/driver_station and /hw/status at 50 Hz; /hw/request/<node> at
// <node>'s rate; /hw/command at the sum of its actuator owners' rates. A
// hardware packet stores 4 + its encoded size from `config`'s devices (a
// Driver Station packet: one joystick at its largest). A topic nobody in the
// session writes has no publications.
TopicLoads DeclaredTopicLoads(const std::vector<NodeDescription>& described,
                              const config::RobotConfig& config);

// A member's expected event-log bytes per second: per declared source,
// records per second x (sizeof(event::log::RecordHeader) + stored bytes).
// Timers and senders at the member's rate (the hardware bridge: /hw/state
// and /hw/command only, the two it logs); a watcher at its topic's rate, at
// most the member's rate when latest-only; one READ per tick when it reads
// its own target lanes. Byte topics are never logged. An estimate to weight
// shares by, not a bound: a callback can send more often, an idle node less.
double ExpectedLogRate(const introspect::Description& declared,
                       int64_t period_us, bool hardware_bridge,
                       const TopicLoads& loads);

// The smallest log cap at which a member's log rotates rather than stops at
// the cap: two segments, each its log's prefix (file header, manifest and
// schemas) and its largest record (LogWriter's segment rule).
uint64_t SmallestRotatingShare(const introspect::Description& declared);

// Shares of `budget` and the window they hold (see SplitLogBudget).
struct LogSplit {
  std::vector<uint64_t> shares;
  // The record time each member not held at a floor keeps at its expected
  // rate: the budget left after the floors over those members' summed
  // weights, less what each one's segments spend on their prefix (half its
  // floor per segment, up to kLogSegments segments), the least over them.
  // Zero when the floors do not fit, when every logging member is held at
  // its floor, or when no logging member's rate is known.
  std::chrono::microseconds window{0};
};

// Shares of `budget` proportional to `rates` (negative: unknown, weighted as
// the mean of the known), each at least floors[i] while the floors fit in the
// budget (else in proportion to the floors); non-logging members get 0 and
// every logging member at least 1 byte; and the window those shares hold.
// The shares never exceed the budget unless it is smaller than one byte per
// logging member. Each member's log ends a segment at least every window / n
// (LogWriterOptions::window), so a member storing no more than its rate keeps
// between (n - 1) / n of the window and the window.
LogSplit SplitLogBudget(uint64_t budget, const std::vector<bool>& logs,
                        const std::vector<double>& rates,
                        const std::vector<uint64_t>& floors);

}  // namespace talos::launcher
