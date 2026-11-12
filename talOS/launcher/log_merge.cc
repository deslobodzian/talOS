#include "talOS/launcher/log_merge.h"

#include <exception>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <tuple>
#include <utility>

#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"

namespace talos::launcher {

const event::Registration* MergedLog::source(std::uint16_t source_id) const {
  for (const auto& reg : manifest) {
    if (reg.id == source_id) return &reg;
  }
  return nullptr;
}

bool MergeLogs(const std::vector<std::string>& log_paths,
               const std::function<bool(const MergedRecord&)>& visit,
               std::string& error, MergeWarnings warnings) {
  error.clear();
  if (log_paths.empty()) {
    error = "no log files provided";
    return false;
  }
  const bool warn = warnings == MergeWarnings::kPrint;

  // One open log: its reader's fixed window and the record at its head.
  struct Member {
    event::log::WindowedLogReader reader;
    MergedRecord current;
  };
  std::vector<Member> members;
  members.reserve(log_paths.size());
  // Reads the member's next intact record. A log the reader skipped damage
  // in still merges, and its damage is named once, when it ends.
  const auto advance = [warn](Member& member) {
    event::log::LogReader::Record record;
    if (!member.reader.next(record)) {
      if (const std::string damage =
              event::log::DescribeDamage(member.reader.damage());
          warn && !damage.empty()) {
        std::cerr << "warning: damaged log " << member.current.log->path << ": "
                  << damage << "\n";
      }
      return false;
    }
    MergedRecord& current = member.current;
    current.header = record.header;
    current.payload = record.payload;
    current.source =
        record.header.kind == static_cast<std::uint16_t>(event::EventKind::EXIT)
            ? nullptr
            : current.log->source(record.header.source_id);
    return true;
  };

  for (const auto& path : log_paths) {
    std::optional<event::log::WindowedLogReader> reader;
    try {
      reader.emplace(path);
    } catch (const event::log::IncompleteLogHeader& e) {
      // A log cut while it was being created holds no records.
      if (warn) {
        std::cerr << "warning: skipping " << path << ": " << e.what() << "\n";
      }
      continue;
    } catch (const std::exception& e) {
      error = "failed to open " + path + ": " + e.what();
      return false;
    }
    if (!members.empty() &&
        reader->session_id() != members.front().reader.session_id()) {
      error = "session_id mismatch: file '" + path + "' has session_id " +
              std::to_string(reader->session_id()) + ", expected " +
              std::to_string(members.front().reader.session_id());
      return false;
    }
    auto log = std::make_shared<MergedLog>();
    log->path = path;
    log->process_name = reader->process_name();
    log->manifest = reader->manifest();
    members.push_back(Member{std::move(*reader), MergedRecord{.log = log}});
  }

  // The members with a record at their head, the earliest on top: by event
  // time, now, dispatch index, then input order.
  using Key =
      std::tuple<std::int64_t, std::int64_t, std::uint64_t, std::size_t>;
  const auto key = [&](std::size_t index) {
    const auto& header = members[index].current.header;
    return Key{header.event_time_ns, header.now_ns, header.dispatch_index,
               index};
  };
  std::priority_queue<Key, std::vector<Key>, std::greater<>> heads;
  for (std::size_t i = 0; i < members.size(); ++i) {
    if (advance(members[i])) heads.push(key(i));
  }
  while (!heads.empty()) {
    const std::size_t index = std::get<3>(heads.top());
    heads.pop();
    Member& member = members[index];
    if (!visit(member.current)) return true;
    if (advance(member)) heads.push(key(index));
  }
  return true;
}

}  // namespace talos::launcher
