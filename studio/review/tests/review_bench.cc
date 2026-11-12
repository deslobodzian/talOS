// Open time and peak resident memory of studio_review's session reader on a
// generated session of at least 200 MB of .tlog (README.md, "Reviewing a
// session"). Not a test: run by hand, on an idle host.
//
//   bazel run -c opt //studio/review:review_bench
//
// The session is generated, then this binary re-executes itself on it, so
// the measured process holds only what review holds: it opens the session
// (the index pass), reads a series over the whole session for one channel
// of each member, and a snapshot at its middle, and reports its own peak
// resident set (ru_maxrss).

#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "review_test/review_message_descriptor.h"
#include "review_test/review_message_generated.h"
#include "studio/review/session.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_writer.h"

namespace {

constexpr std::uint64_t kSessionId = 0x5E5511;
constexpr int kMembers = 8;
constexpr std::uint64_t kSessionBytes = 210ull << 20;
constexpr std::int64_t kStartNs = 1'000'000'000'000;

// One member's log: a review_test.Shot SEND on /member<k>/state every
// millisecond until the member holds its share of the session.
void WriteMember(const std::string& directory, int member) {
  using D = talos::Descriptor<review_test::Shot>;
  const std::string name = "member" + std::to_string(member);
  talos::event::Registration source;
  source.id = 1;
  source.kind = talos::event::SourceKind::SENDER;
  source.name = "/" + name + "/state";
  source.message_bytes = sizeof(review_test::Shot);
  source.schema = {std::string{D::kName}, D::kHash,
                   std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                               D::kBfbs.size()}};
  talos::event::log::LogWriter writer{
      directory + "/" + name + ".tlog", name,
      talos::event::log::LogWriterOptions{1u << 20, 4, false}, kSessionId};
  writer.start({source}, talos::event::MonotonicTime::from_nanos(kStartNs));
  const std::uint64_t records =
      kSessionBytes / kMembers /
      (sizeof(talos::event::log::RecordHeader) + sizeof(review_test::Shot));
  for (std::uint64_t i = 0; i < records; ++i) {
    const review_test::Shot shot{static_cast<double>(i) * 0.001,
                                 static_cast<std::uint32_t>(i),
                                 review_test::Mode::SPINNING, (i & 1) != 0, i};
    talos::event::Context ctx{};
    ctx.kind = talos::event::EventKind::TIMER;
    ctx.dispatch_index = i + 1;
    ctx.event_time = talos::event::MonotonicTime::from_nanos(
        kStartNs + static_cast<std::int64_t>(i) * 1'000'000 + member * 1'000);
    ctx.now = ctx.event_time;
    writer.send(ctx, 1, std::as_bytes(std::span{&shot, 1}), 0, i + 1);
  }
  writer.flush();
  if (writer.failed()) {
    std::fprintf(stderr, "review_bench: %s\n", writer.error().c_str());
    std::exit(1);
  }
}

double Seconds(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - since)
      .count();
}

int Review(const std::string& directory) {
  const auto open_start = std::chrono::steady_clock::now();
  const auto session = studio::review::Session::Open(directory);
  const double open_s = Seconds(open_start);
  std::vector<studio::review::ChannelRef> channels;
  for (int member = 0; member < kMembers; ++member) {
    channels.push_back(*session->Find("member" + std::to_string(member) +
                                      "/state/position_m"));
  }
  const auto series_start = std::chrono::steady_clock::now();
  studio::review::ReadStats stats;
  const auto series = session->ReadSeries(channels, session->start_ns(),
                                          session->end_ns() + 1, 1000, &stats);
  const double series_s = Seconds(series_start);
  const auto snapshot_start = std::chrono::steady_clock::now();
  const auto snapshot =
      session->Snapshot((session->start_ns() + session->end_ns()) / 2, nullptr);
  const double snapshot_s = Seconds(snapshot_start);
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
#if defined(__APPLE__)
  const double peak_mib = static_cast<double>(usage.ru_maxrss) / (1 << 20);
#else
  const double peak_mib = static_cast<double>(usage.ru_maxrss) / (1 << 10);
#endif
  std::printf(
      "open (index pass): %.2f s; index %zu B for %zu log-file seconds "
      "(%.1f B each)\n"
      "series, %zu channels x 1000 buckets over the whole session: %.2f s, "
      "%llu records\n"
      "snapshot at the middle: %.3f s, %zu values\n"
      "peak resident: %.1f MiB\n",
      open_s, session->index_bytes(), session->log_seconds(),
      static_cast<double>(session->index_bytes()) /
          static_cast<double>(session->log_seconds()),
      series.size(), series_s, static_cast<unsigned long long>(stats.records),
      snapshot_s, snapshot.size(), peak_mib);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 3 && std::string{argv[1]} == "--review") return Review(argv[2]);
  std::string pattern =
      (std::filesystem::temp_directory_path() / "review_bench.XXXXXX").string();
  if (::mkdtemp(pattern.data()) == nullptr) {
    std::perror("review_bench: mkdtemp");
    return 1;
  }
  const std::string directory = pattern;
  for (int member = 0; member < kMembers; ++member) {
    WriteMember(directory, member);
  }
  std::uint64_t bytes = 0;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    bytes += entry.file_size();
  }
  std::printf("session: %d logs, %.1f MiB\n", kMembers,
              static_cast<double>(bytes) / (1 << 20));
  std::fflush(stdout);
  const pid_t child = fork();
  if (child == 0) {
    execl("/proc/self/exe", argv[0], "--review", directory.c_str(),
          static_cast<char*>(nullptr));
    execl(argv[0], argv[0], "--review", directory.c_str(),
          static_cast<char*>(nullptr));
    std::_Exit(127);
  }
  int status = 0;
  waitpid(child, &status, 0);
  std::filesystem::remove_all(directory);
  return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : 1;
}
