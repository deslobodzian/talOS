// Minimal Studio bridge test fixture for //talOS/build's talos_sim_test
// studio=True checks (kept inside talOS/ so talOS/ never depends on studio/,
// RULES.md 5). Speaks the launcher's studio_bridge contract: --describe,
// ephemeral TCP port bind + `listening <port>`, live registry row with
// observer tier + matching watcher endpoint, and clean SIGINT/SIGTERM stop.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/manifest.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/process/process.h"
#include "talOS/process/realtime_outcome.h"
#include "talOS/schema/identity.h"
#include "talOS/telemetry/wire.h"

namespace {

constexpr std::string_view kTarget =
    "//talOS/build/testdata/sim_check:studio_bridge";

int Describe(int argc, char** argv) {
  const char* config_path = nullptr;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string_view{argv[i]} == "--config") config_path = argv[i + 1];
  }
  std::string topic = "/talos/telemetry";
  if (config_path != nullptr) {
    try {
      topic = talos::config::ParseRobotConfig(config_path).studio.topic;
    } catch (const std::exception& e) {
      std::cerr << "stub_studio_bridge --describe: " << e.what() << '\n';
    }
  }
  talos::introspect::Description description;
  description.name = "studio_bridge";
  description.target = std::string{kTarget};
  talos::introspect::naming::SourceShape source{
      talos::event::SourceKind::WATCHER, topic,
      static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes), 0};
  const talos::schema::Identity bytes = talos::schema::Bytes();
  source.schema = bytes.name;
  source.schema_hash = bytes.hash;
  source.alignment = 8;
  source.slots = static_cast<std::uint32_t>(talos::telemetry_wire::slot_count);
  description.sources.push_back(std::move(source));
  std::cout << talos::introspect::DescribeToJson(description);
  return 0;
}

std::optional<std::uint64_t> ParseU64(std::string_view text) {
  std::uint64_t value = 0;
  const auto* end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  if (text.empty() || ec != std::errc{} || ptr != end) return std::nullopt;
  return value;
}

int BindLoopback(std::uint16_t requested_port, std::uint16_t& bound_port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return -1;
  const int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(requested_port);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0 ||
      ::listen(fd, 1) != 0) {
    ::close(fd);
    return -1;
  }
  socklen_t len = sizeof(addr);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
    ::close(fd);
    return -1;
  }
  bound_port = ntohs(addr.sin_port);
  return fd;
}

}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string_view{argv[i]} == "--describe") return Describe(argc, argv);
  }

  std::vector<std::string_view> positional;
  std::uint64_t session_id = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};
    if (arg == "--session-id" && i + 1 < argc) {
      const auto parsed = ParseU64(argv[++i]);
      if (!parsed) return 2;
      session_id = *parsed;
    } else if ((arg == "--config" || arg == "--declared" ||
                arg == "--manifest") &&
               i + 1 < argc) {
      ++i;
    } else if (!arg.starts_with("--")) {
      positional.push_back(arg);
    }
  }

  const std::string topic =
      !positional.empty() ? std::string{positional[0]} : "/talos/telemetry";
  std::uint16_t requested_port = 5800;
  if (positional.size() >= 2) {
    const auto parsed = ParseU64(positional[1]);
    if (!parsed || *parsed > 65535) return 2;
    requested_port = static_cast<std::uint16_t>(*parsed);
  }

  talos::process::InstallStopHandlers();

  std::uint16_t bound_port = 0;
  const int listen_fd = BindLoopback(requested_port, bound_port);
  if (listen_fd < 0) {
    std::cerr << "cannot listen on port " << requested_port << ": "
              << std::strerror(errno) << '\n';
    return 1;
  }

  std::optional<talos::introspect::NodeRegistration> row;
  if (session_id != 0) {
    row.emplace(talos::introspect::NodeRegistration::Identity{
        .name = "studio_bridge",
        .target = std::string{kTarget},
        .session_id = session_id,
        .flags = 0,
    });
    talos::event::Registration watcher;
    watcher.id = 0;
    watcher.kind = talos::event::SourceKind::WATCHER;
    watcher.name = topic;
    watcher.message_bytes =
        static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes);
    watcher.alignment = 8;
    watcher.schema = talos::schema::Bytes();
    row->publish(talos::event::Manifest{watcher});
    row->set_realtime({.tier = talos::process::LoopTier::kObserver});
  }

  std::uint64_t dispatch = 1;
  const auto beat = [&] {
    if (!row) return;
    row->begin_sample();
    row->heartbeat();
    row->set_dispatch_count(dispatch++);
    row->end_sample();
  };
  beat();

  std::cout << "listening " << bound_port << '\n' << std::flush;

  pollfd pfd{};
  pfd.fd = talos::process::StopFd();
  pfd.events = POLLIN;
  while (!talos::process::stop_requested.load(std::memory_order_acquire)) {
    ::poll(&pfd, 1, 100);
    beat();
  }

  ::close(listen_fd);
  return 0;
}
