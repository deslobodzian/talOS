#include "CanBusHealth.h"

#include <cstdio>
#include <string>
#include <system_error>
#include <utility>

#ifndef _WIN32
#include <pthread.h>
#endif
#ifdef __linux__
#include <sched.h>
#endif

namespace talos::rio {

void CanBusSampler::Run(const std::shared_ptr<CanBusHealth::Board>& board,
                        const Read& read, Clock clock, uint64_t period_us,
                        const std::shared_ptr<StuckReaders>& stuck) {
#ifdef __linux__
  pthread_setname_np(pthread_self(), "talos-canbus");
  const sched_param param{};
  pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
#elif defined(__APPLE__)
  pthread_setname_np("talos-canbus");
#endif
  // Never let a vendor-read exception escape the thread and invoke
  // std::terminate() mid-match; report it as a failed bus read instead.
  const auto safe_read = [&read](std::string_view bus) noexcept {
    try {
      return read(bus);
    } catch (...) {
      CanBusHealth::Sample failed{};
      failed.status = "exception in CAN bus status read";
      return failed;
    }
  };
  const std::chrono::microseconds period{period_us};
  while (board->SampleNext(clock(), safe_read) && board->WaitPeriod(period)) {
  }
  if (board->Exit()) stuck->Exited();
}

CanBusSampler::CanBusSampler(CanBusHealth& health, Read read, Clock clock,
                             Report report, uint64_t period_us,
                             std::chrono::microseconds stop_bound,
                             std::shared_ptr<StuckReaders> stuck)
    : health_{health},
      read_{std::move(read)},
      clock_{clock},
      report_{std::move(report)},
      period_us_{period_us},
      stop_bound_{stop_bound},
      stuck_{std::move(stuck)} {}

CanBusSampler::~CanBusSampler() {
  Stop();
  for (auto& slot : abandoned_) {
    if (!slot.thread.joinable()) continue;
    // One that has exited is past its last use of anything shared; one still
    // blocked owns what it touches (see the header).
    if (slot.board->exited()) {
      slot.thread.join();
    } else {
      slot.thread.detach();
    }
  }
}

void CanBusSampler::Reap() {
  for (auto& slot : abandoned_) {
    if (slot.thread.joinable() && slot.board->exited()) {
      slot.thread.join();
      slot.board.reset();
    }
  }
}

void CanBusSampler::Reconfigure(const hardware::Config& config,
                                bool simulation) {
  Stop();
  Reap();
  health_.Configure(config, simulation);
  std::shared_ptr<CanBusHealth::Board> board = health_.board();
  if (!board) return;
  const int stuck = stuck_->count();
  if (stuck >= kMaxStuckReaders) {
    char line[160];
    std::snprintf(line, sizeof line,
                  "[phoenix] CAN bus readers: %d abandoned readers still "
                  "blocked in a status read; none started, so every CAN bus "
                  "reads stale",
                  stuck);
    if (report_) report_(line);
    return;
  }
  try {
    thread_ = std::thread{
        [board, read = read_, clock = clock_, period_us = period_us_,
         stuck = stuck_] { Run(board, read, clock, period_us, stuck); }};
    board_ = std::move(board);
  } catch (const std::system_error&) {  // NOLINT(bugprone-empty-catch)
    // No reads: Poll reports every bus stale (see the header).
  }
}

void CanBusSampler::Stop() {
  if (!thread_.joinable()) return;
  std::string in_flight;
  const bool exited =
      board_->StopAndWait(stop_bound_, [&](std::string_view bus) {
        stuck_->Add();
        in_flight.assign(bus);
      });
  if (exited) {
    thread_.join();
    board_.reset();
    return;
  }
  Reap();
  bool parked = false;
  for (auto& slot : abandoned_) {
    if (slot.thread.joinable()) continue;
    slot.thread = std::move(thread_);
    slot.board = std::move(board_);
    parked = true;
    break;
  }
  if (!parked) {
    thread_.detach();
    board_.reset();
  }
  const auto bound_ms = static_cast<unsigned long long>(
      std::chrono::duration_cast<std::chrono::milliseconds>(stop_bound_)
          .count());
  char line[160];
  if (in_flight.empty()) {
    std::snprintf(line, sizeof line,
                  "[phoenix] CAN bus reader did not stop within %llu ms with "
                  "no read in flight; it was abandoned",
                  bound_ms);
  } else {
    std::snprintf(line, sizeof line,
                  "[phoenix] CAN bus %.63s status read did not return within "
                  "%llu ms; its reader was abandoned",
                  in_flight.c_str(), bound_ms);
  }
  if (report_) report_(line);
}

}  // namespace talos::rio
