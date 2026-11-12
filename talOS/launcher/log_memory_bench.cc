// Peak resident memory of the session readers (log_dump, tlog_to_wpilog) on
// a synthetic session of at least 200 MB of .tlog, generated here and
// removed after; README.md, "Filtering and exporting logs". Not a test: run by
// hand, on an idle host.
//
//   bazel run -c opt //talOS/launcher:log_memory_bench
//
// Each tool runs as a child with its output discarded; its peak resident
// set is the child's ru_maxrss.

#include <fcntl.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_writer.h"

namespace {

constexpr std::uint64_t kSessionId = 0x5E5510;
constexpr int kMembers = 8;
constexpr std::size_t kPayloadBytes = 64;  // 128 B a record with its header
constexpr std::uint64_t kSessionBytes = 210ull << 20;

// One member's log: a SEND of a schema-less topic every millisecond,
// members interleaved, until the member holds its share of the session.
std::uint64_t WriteMember(const std::string& directory, int member) {
  const std::string name = "member" + std::to_string(member);
  talos::event::Registration source;
  source.id = 1;
  source.kind = talos::event::SourceKind::SENDER;
  source.name = "/" + name + "/state";
  source.message_bytes = kPayloadBytes;
  talos::event::log::LogWriter writer{
      directory + "/" + name + ".tlog", name,
      talos::event::log::LogWriterOptions{1u << 20, 4, false}, kSessionId};
  writer.start({source}, talos::event::MonotonicTime::from_nanos(0));
  std::array<std::byte, kPayloadBytes> payload{};
  const std::uint64_t records =
      kSessionBytes / kMembers /
      (sizeof(talos::event::log::RecordHeader) + kPayloadBytes);
  for (std::uint64_t i = 0; i < records; ++i) {
    for (std::size_t b = 0; b < payload.size(); ++b) {
      payload[b] = static_cast<std::byte>(i + b + 1);
    }
    talos::event::Context ctx{};
    ctx.kind = talos::event::EventKind::SEND;
    ctx.source_id = 1;
    ctx.dispatch_index = i + 1;
    ctx.event_time =
        talos::event::MonotonicTime::from_nanos(static_cast<std::int64_t>(
            i * 1'000'000 + static_cast<std::uint64_t>(member) * 1'000));
    ctx.now = ctx.event_time;
    ctx.publish_time = ctx.event_time;
    ctx.sequence = i + 1;
    writer.dispatch(ctx, payload);
  }
  writer.flush();
  if (writer.failed()) {
    std::fprintf(stderr, "log_memory_bench: %s\n", writer.error().c_str());
    std::exit(1);
  }
  return records;
}

// The child's peak resident set in MiB, or -1 if it failed.
double PeakMiB(const std::vector<std::string>& argv) {
  const pid_t pid = ::fork();
  if (pid == 0) {
    const int null = ::open("/dev/null", O_WRONLY);
    ::dup2(null, STDOUT_FILENO);
    std::vector<char*> args;
    args.reserve(argv.size() + 1);
    for (const auto& arg : argv) args.push_back(const_cast<char*>(arg.c_str()));
    args.push_back(nullptr);
    ::execv(args[0], args.data());
    std::_Exit(127);
  }
  int status = 0;
  struct rusage usage {};
  if (::wait4(pid, &status, 0, &usage) != pid || !WIFEXITED(status) ||
      WEXITSTATUS(status) != 0) {
    return -1;
  }
#if defined(__APPLE__)
  return static_cast<double>(usage.ru_maxrss) / (1 << 20);  // bytes
#else
  return static_cast<double>(usage.ru_maxrss) / (1 << 10);  // KiB
#endif
}

}  // namespace

int main(int argc, char** argv) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::Create(
          argv[0], BAZEL_CURRENT_REPOSITORY, &error));
  if (runfiles == nullptr) {
    std::fprintf(stderr, "log_memory_bench: %s\n", error.c_str());
    return 1;
  }
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <log_dump> <tlog_to_wpilog>\n", argv[0]);
    return 1;
  }
  const std::string log_dump = runfiles->Rlocation(argv[1]);
  const std::string tlog_to_wpilog = runfiles->Rlocation(argv[2]);

  std::string pattern =
      (std::filesystem::temp_directory_path() / "log_memory_bench.XXXXXX")
          .string();
  if (::mkdtemp(pattern.data()) == nullptr) {
    std::perror("log_memory_bench: mkdtemp");
    return 1;
  }
  const std::string directory = pattern;
  std::uint64_t records = 0;
  for (int member = 0; member < kMembers; ++member) {
    records += WriteMember(directory, member);
  }
  std::uint64_t bytes = 0;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    bytes += entry.file_size();
  }
  std::printf("session: %d logs, %.1f MiB, %llu records\n", kMembers,
              static_cast<double>(bytes) / (1 << 20),
              static_cast<unsigned long long>(records));

  const double dump = PeakMiB({log_dump, directory});
  std::printf("log_dump peak RSS: %.1f MiB\n", dump);
  const double jsonl = PeakMiB({log_dump, "--format", "jsonl", directory});
  std::printf("log_dump --format jsonl peak RSS: %.1f MiB\n", jsonl);
  const double wpilog =
      PeakMiB({tlog_to_wpilog, directory, "-o", directory + "/out.wpilog"});
  std::printf("tlog_to_wpilog peak RSS: %.1f MiB\n", wpilog);

  std::filesystem::remove_all(directory);
  return dump < 0 || jsonl < 0 || wpilog < 0 ? 1 : 0;
}
