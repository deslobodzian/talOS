#include "talOS/launcher/log_merge.h"

#include <exception>
#include <iostream>
#include <tuple>
#include <utility>

#include "talOS/events/log/log_reader.h"

namespace talos::launcher {

std::shared_ptr<const event::Registration> ResolveSource(
    const std::shared_ptr<const event::Manifest>& manifest,
    std::uint16_t source_id) {
  for (const auto& reg : *manifest) {
    if (reg.id == source_id) return {manifest, &reg};
  }
  return nullptr;
}

bool MergeLogs(const std::vector<std::string>& log_paths,
               std::vector<MergedRecord>& out_records, std::string& error) {
  out_records.clear();
  error.clear();
  if (log_paths.empty()) {
    error = "no log files provided";
    return false;
  }

  struct ReaderState {
    std::string path;
    std::unique_ptr<event::log::LogReader> reader;
    std::shared_ptr<const event::Manifest> manifest;
    event::log::LogReader::Record current;
    bool has_record{false};
  };
  std::vector<ReaderState> states;
  states.reserve(log_paths.size());
  // Reads the next intact record. A log the reader skipped damage in still
  // merges, and its damage is named once, when it ends; session_report
  // parses this line.
  const auto advance = [](ReaderState& state) {
    state.has_record = state.reader->next(state.current);
    if (state.has_record) return;
    if (const std::string damage =
            event::log::DescribeDamage(state.reader->damage());
        !damage.empty()) {
      std::cerr << "warning: damaged log " << state.path << ": " << damage
                << "\n";
    }
  };

  for (const auto& path : log_paths) {
    ReaderState state;
    try {
      state.reader = std::make_unique<event::log::LogReader>(path);
    } catch (const event::log::IncompleteLogHeader& e) {
      // A log cut while it was being created holds no records.
      std::cerr << "warning: skipping " << path << ": " << e.what() << "\n";
      continue;
    } catch (const std::exception& e) {
      error = "failed to open " + path + ": " + e.what();
      return false;
    }
    if (!states.empty() &&
        state.reader->session_id() != states.front().reader->session_id()) {
      error = "session_id mismatch: file '" + path + "' has session_id " +
              std::to_string(state.reader->session_id()) + ", expected " +
              std::to_string(states.front().reader->session_id());
      return false;
    }
    state.manifest =
        std::make_shared<const event::Manifest>(state.reader->manifest());
    state.path = path;
    advance(state);
    states.push_back(std::move(state));
  }

  const auto key = [](const ReaderState& state) {
    const auto& header = state.current.header;
    return std::tuple{header.event_time_ns, header.now_ns,
                      header.dispatch_index};
  };
  for (;;) {
    ReaderState* best = nullptr;
    for (auto& state : states) {
      if (state.has_record && (best == nullptr || key(state) < key(*best))) {
        best = &state;
      }
    }
    if (best == nullptr) return true;

    MergedRecord record;
    record.process_name = best->reader->process_name();
    if (best->current.header.kind !=
        static_cast<std::uint16_t>(event::EventKind::EXIT)) {
      record.source =
          ResolveSource(best->manifest, best->current.header.source_id);
    }
    record.source_name = record.source ? record.source->name : "?";
    record.header = best->current.header;
    record.payload.assign(best->current.payload.begin(),
                          best->current.payload.end());
    out_records.push_back(std::move(record));
    advance(*best);
  }
}

}  // namespace talos::launcher
