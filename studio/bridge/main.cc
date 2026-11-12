#include <App.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "system.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/process/process.h"
#include "talOS/rtms/rtms.h"
#include "talOS/rtms/wake.h"
#include "talOS/telemetry/wire.h"
#include "telemetry.h"
// uSockets' callback poll (`us_internal_callback_t`, POLL_TYPE_CALLBACK): how
// its own timers and UDP sockets put a descriptor in the loop. Its public API
// has no other way to watch a descriptor it did not open.
extern "C" {
#include "internal/internal.h"
}
namespace fs = std::filesystem;
namespace {
using studio::EgressBudget;
struct Client {
  EgressBudget budget;
  // What this client has been sent of the graph and the telemetry; its
  // updates carry only what changed.
  studio::ClientFeeds feeds;
};
// The id of the segment the shared-memory name `segment` holds now, or
// nullopt while there is none or its creator has not finished initializing
// it. Maps only the header, read-only, and keeps nothing open.
std::optional<std::uint64_t> SegmentId(const std::string& segment) {
  const int fd = ::shm_open(segment.c_str(), O_RDONLY, 0);
  if (fd < 0) return std::nullopt;
  std::optional<std::uint64_t> id;
  struct stat status {};
  // A segment still being sized is shorter than its header; mapping past
  // its end would fault on the first read.
  if (::fstat(fd, &status) == 0 &&
      status.st_size >= static_cast<off_t>(sizeof(RTMSHeader))) {
    void* mapped =
        ::mmap(nullptr, sizeof(RTMSHeader), PROT_READ, MAP_SHARED, fd, 0);
    if (mapped != MAP_FAILED) {
      const auto* header = static_cast<const RTMSHeader*>(mapped);
      if (header->magic.load(std::memory_order_acquire) == kRtmsMagic) {
        id = header->segment_id;
      }
      ::munmap(mapped, sizeof(RTMSHeader));
    }
  }
  ::close(fd);
  return id;
}
using Socket = uWS::WebSocket<false, true, Client>;
struct Bridge {
  std::string topic;
  // The telemetry topic, once its publisher's segment exists. The bridge
  // serves HTTP and the system graph without it, waits for the segment
  // rather than refusing to start, and re-attaches when the name comes to
  // hold a different segment (a new launcher session, or one created after
  // the last holder of the old one closed it).
  std::string segment;
  std::optional<RTMSQueue> queue;
  std::size_t reader = 0;
  std::uint64_t segment_id = 0;
  bool announced_wait = false;
  // The wake channel (talOS/rtms README "Wake channel"): the reader slot
  // names this receiver, so a publication wakes the loop. Empty when none
  // could be made; the loop then reads every WakeSchedule::kFallbackNs.
  std::optional<talos::WakeReceiver> wake;
  // The last read stopped at its page budget, or could not arm the wake:
  // there may be pages to read, so the loop comes back in kFallbackNs.
  bool backlog = false;
  // Every duty the loop wakes for on its own (next_wake_ns).
  studio::WakeSchedule schedule;
  std::string attach_error;
  // Every send to a client leaves in one of its passes (pass_clients).
  studio::ClientRotation<Socket*> clients;
  us_listen_socket_t* listener = nullptr;
  // ws_drops counts messages the socket refused; that client is then
  // resynchronized (the whole graph, a telemetry keyframe). A graph or
  // telemetry update the budget defers is not counted: the client's next
  // update carries its changes. forwarded counts complete producer ticks.
  std::uint64_t invalid = 0, ws_drops = 0, forwarded = 0;
  // Frames the publisher overwrote before this bridge read them: it never
  // waits for the bridge, so a stalled bridge is lapped instead.
  std::uint64_t lapped = 0;
  // Client passes held because the client still had output buffered: a
  // client that stopped reading costs the others no turn and no budget.
  std::uint64_t held = 0;
  // read_next() copies each page here and validates the copy before
  // ingesting commits side effects (client sends, counters).
  std::vector<std::byte> frame =
      std::vector<std::byte>(talos::telemetry_wire::slot_bytes);
  std::size_t max_clients = 4;
  std::uint64_t refused = 0, egress_bytes = 0;
  EgressBudget budget;
  // Returns from the event loop's wait (every one, whatever woke it), and of
  // them the ones a telemetry publication or a deadline timer woke.
  std::uint64_t wakes = 0, publication_wakes = 0, deadline_wakes = 0;

  Bridge() = default;
  Bridge(const Bridge&) = delete;
  Bridge& operator=(const Bridge&) = delete;
  ~Bridge() { detach(); }

  void detach() {
    if (!queue) return;
    queue->release_reader(reader);
    queue.reset();
  }

  // Attaches to segment `id` of the topic. False, logged once per distinct
  // error, when the segment cannot be used yet; the next check retries.
  bool attach(std::uint64_t id) {
    try {
      // A reader's policy only decides whether it may take the slot the
      // writer is about to reuse; overwrite is right for any publisher.
      RTMSQueue next(topic, talos::telemetry_wire::slot_bytes, 8,
                     talos::schema::Bytes(), talos::telemetry_wire::slot_count,
                     {OverflowPolicy::OVERWRITE_OLDEST, ReadMode::SEQUENCE});
      // Replaced while attaching: which one `next` mapped is unknown.
      if (SegmentId(segment) != id) return false;
      const auto claimed = next.register_reader();
      if (!claimed) throw std::runtime_error("RTMS reader slots exhausted");
      detach();
      queue.emplace(std::move(next));
      reader = *claimed;
      if (wake) queue->set_reader_wake(reader, wake->id());
      segment_id = id;
      attach_error.clear();
      std::cerr << "attached to " << topic << '\n';
      return true;
    } catch (const std::exception& e) {
      if (attach_error != e.what()) {
        attach_error = e.what();
        std::cerr << "cannot attach to " << topic << " (" << e.what()
                  << "); retrying\n";
      }
      return false;
    }
  }

  // Attaches when the segment appears, re-attaches when the name holds a
  // different one. A segment whose name was unlinked is kept: nothing
  // replaces it until a new one appears.
  void check_segment() {
    const auto id = SegmentId(segment);
    if (!id) {
      if (!queue && !announced_wait) {
        announced_wait = true;
        std::cerr << "waiting for " << topic << " (start its publisher)\n";
      }
      return;
    }
    if (queue && *id == segment_id) return;
    if (queue) std::cerr << topic << " was replaced; re-attaching\n";
    attach(*id);
  }

  // Checks every 100 ms while waiting, every second once attached (a
  // replacement is rare; each check is a few syscalls).
  static constexpr std::int64_t kWaitingCheckNs = 100'000'000;
  static constexpr std::int64_t kAttachedCheckNs = 1'000'000'000;
  void watch(std::int64_t now_ns) {
    if (now_ns < schedule.check_due_ns) return;
    check_segment();
    schedule.check_due_ns =
        now_ns + (queue ? kAttachedCheckNs : kWaitingCheckNs);
  }

  // Each client's own refill rate, B/s.
  double client_rate() const {
    return EgressBudget::kBytesPerSecond / static_cast<double>(max_clients);
  }

  // Whether `framed_bytes` fit the client's and the shared bucket now, and
  // the client has nothing buffered. Spends nothing. A send the budget
  // defers is due again when both buckets will hold it (schedule); one
  // deferred for buffered output is retried by the socket's drain handler.
  // `now` is the pass's instant, not each client's: every client in a pass
  // sees the buckets at one level, so a refill landing between two clients
  // cannot hand the turn of one that could not yet afford a message to a
  // later one with a message of the same size.
  bool affordable(Socket* client, std::size_t framed_bytes, std::int64_t now) {
    auto& local = client->getUserData()->budget;
    const double rate = client_rate();
    const bool own = local.available(framed_bytes, rate, now);
    const bool shared =
        budget.available(framed_bytes, EgressBudget::kBytesPerSecond, now);
    if (!own || !shared) {
      schedule.Deferred(
          now, std::max(local.wait_ns(framed_bytes, rate),
                        budget.wait_ns(framed_bytes,
                                       EgressBudget::kBytesPerSecond)));
      return false;
    }
    if (client->getBufferedAmount() != 0) {
      ++held;
      return false;
    }
    return true;
  }

  // Sends one message the caller found affordable and charges it.
  bool send_now(Socket* client, std::string_view bytes, uWS::OpCode opcode) {
    if (client->send(bytes, opcode) == Socket::DROPPED) {
      ++ws_drops;
      return false;
    }
    const auto size = EgressBudget::framed(bytes.size());
    client->getUserData()->budget.spend(size);
    budget.spend(size);
    egress_bytes += size;
    return true;
  }

  // Brings one client toward the latest graph and tick: each whole on join,
  // then change-only updates, the graph first (studio::ClientFeeds).
  // Returns whether it sent the client anything. Only a pass calls it, with
  // the pass's instant.
  bool pump(Socket* client, std::int64_t now_ns) {
    // Every successful send is charged to egress_bytes (send_now).
    const std::uint64_t charged = egress_bytes;
    client->getUserData()->feeds.Pump(
        system_graph, telemetry_state, now_ns, client_rate(),
        [this, client, now_ns](std::size_t framed_bytes) {
          return affordable(client, framed_bytes, now_ns);
        },
        [this, client](std::string_view text) {
          return send_now(client, text, uWS::OpCode::TEXT);
        },
        [this, client](std::string_view page) {
          return send_now(client, page, uWS::OpCode::BINARY);
        });
    return egress_bytes != charged;
  }

  // Offers every client what is due, in rotation (studio::ClientRotation).
  void pass_clients() {
    const std::int64_t now_ns = talos::introspect::monotonic_now_ns();
    clients.Pass(
        [this, now_ns](Socket* client) { return pump(client, now_ns); });
  }

  // A client's output drained: a pass, which offers it its turn
  // (studio::ClientRotation::Drained).
  void drained(Socket* client) {
    const std::int64_t now_ns = talos::introspect::monotonic_now_ns();
    clients.Drained(client, [this, now_ns](Socket* offered) {
      return pump(offered, now_ns);
    });
  }

  // The decoded feed: latest dictionary and latest complete tick, backing
  // /telemetry.json, /channels.json and every client's updates.
  studio::TelemetryState telemetry_state;

  // Opened lazily: the bridge may well start before any node, and a viewer
  // that refuses to run until the robot is up is a viewer you cannot use to
  // watch the robot come up.
  std::optional<talos::introspect::RegistryReader> registry;
  // The latest refresh; empty until the first, so no client is sent "{}".
  studio::SystemGraphDocument system_graph;
  bool system_json_oversized = false;
  std::int64_t started_wall_ns = talos::introspect::wall_now_ns();

  void refresh_system() {
    if (!registry) {
      registry = talos::introspect::RegistryReader::open();
    }

    studio::SystemGraphInput input;
    input.bridge = {.topic = topic,
                    .clients = clients.clients().size(),
                    .frames_forwarded = forwarded,
                    .invalid = invalid,
                    .ws_drops = ws_drops,
                    .refused = refused,
                    .egress_bytes = egress_bytes};
    input.wall_ns = talos::introspect::wall_now_ns();
    input.monotonic_ns = talos::introspect::monotonic_now_ns();
    if (registry) {
      input.registry_available = true;
      input.node_capacity = registry->node_capacity();
      input.source_capacity = registry->source_capacity();
      input.nodes = registry->nodes();
    }

    // The bridge is a participant, not a neutral observer: it holds a reader
    // slot on the telemetry topic. Leaving itself out made that topic report
    // "published but nobody reads it", which is exactly the fault the topic
    // view exists to surface -- a false one is worse than none.
    input.nodes.push_back(self());

    system_graph = studio::BuildSystemGraph(input);

    // A send must fit the egress burst, so a larger graph is never delivered
    // whole: a client that needs it gets nothing, and Studio marks its last
    // graph stale. Logged once per crossing, not on every refresh.
    const bool oversized =
        static_cast<double>(EgressBudget::framed(system_graph.json.size())) >
        EgressBudget::kBurstBytes;
    if (oversized && !system_json_oversized) {
      std::cerr << "system graph is " << system_graph.json.size()
                << " B, over the " << EgressBudget::kBurstBytes
                << " B WebSocket egress burst; clients will not receive it "
                   "(/system.json still serves it)\n";
    }
    system_json_oversized = oversized;
    // Clients are sent it by pump(), as text on the WebSocket, so a client
    // tells the two message families apart by frame type.
  }

  // This process as the registry would have described it, had it a slot.
  talos::introspect::NodeSnapshot self() const {
    talos::introspect::NodeSnapshot node;
    node.name = "studio_bridge";
    node.target = "//studio/bridge:studio_bridge";
    node.pid = static_cast<std::uint64_t>(::getpid());
    node.start_wall_ns = started_wall_ns;
    node.heartbeat_monotonic_ns = talos::introspect::monotonic_now_ns();
    node.alive = true;
    node.dispatch_count = forwarded;
    node.declared_source_count = 1;

    talos::introspect::SourceSnapshot source;
    // A watcher, not a fetcher: the reader is registered with
    // ReadMode::SEQUENCE, so it sees every page in order rather than only the
    // newest, unless the publisher laps it. Lapped pages and the ticks they
    // left incomplete count as dropped; a deferred client update loses
    // nothing, so it does not.
    source.kind = talos::event::SourceKind::WATCHER;
    source.name = topic;
    source.message_bytes =
        static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes);
    source.events = forwarded;
    source.dropped = lapped + telemetry_state.incomplete_ticks;
    source.sequence = queue ? queue->writer_sequence() : 0;
    node.sources.push_back(std::move(source));
    return node;
  }

  // Reads what the topic holds, up to a page budget (`backlog` when it stops
  // there), and brings every client toward it.
  void poll() {
    backlog = false;
    if (queue) {
      // Bound work so HTTP and client socket processing cannot starve.
      for (unsigned count = 0;; ++count) {
        if (count == 256) {
          backlog = true;
          break;
        }
        MessageInfo info;
        const auto result = queue->read_next(reader, frame, info);
        if (result == ReadResult::TORN) {
          ++invalid;
          continue;
        }
        if (result != ReadResult::OK) break;
        lapped += info.dropped;
        auto bytes = talos::telemetry_wire::payload(frame);
        if (bytes.empty()) {
          ++invalid;
          continue;
        }
        const bool had_frame = telemetry_state.has_frame;
        const std::uint64_t sequence = telemetry_state.sequence_id;
        if (!studio::IngestTelemetry(telemetry_state, bytes,
                                     talos::introspect::monotonic_now_ns())) {
          ++invalid;
          continue;
        }
        if (telemetry_state.has_frame &&
            (!had_frame || telemetry_state.sequence_id != sequence)) {
          ++forwarded;
        }
      }
    }
    // Every pass, not only on a new tick: a pass woken by the budget's refill
    // or a client's heartbeat (next_wake_ns) sends what is due.
    pass_clients();
  }

  // One pass of the loop, whatever woke it: the wake channel's protocol
  // (drain, read until empty, arm, and read again while an arm finds a
  // page), the segment check, the graph refresh, and every client's due
  // sends. Each client is sent its changed counters at most every
  // GraphFeed::kUpdateNs (or a deferral notice), so Studio keeps measuring
  // node rates from successive graphs' counters and never marks a merely
  // deferred graph stale.
  void service(std::int64_t now_ns) {
    if (wake) wake->drain();
    watch(now_ns);
    schedule.BeginPass();
    // Refreshed before the clients are pumped, so a graph update due with
    // the refresh goes out on this pass, not on the next wake.
    if (schedule.RefreshDue(now_ns)) {
      schedule.refreshed_ns = now_ns;
      refresh_system();
    }
    poll();
    if (!queue || !wake) return;
    // Three tries: a slot this queue no longer holds never arms, and must
    // not make the loop spin; it falls back to reading every kFallbackNs.
    for (int attempt = 0; !backlog; ++attempt) {
      if (queue->arm_wake(reader)) return;
      if (attempt == 2) {
        backlog = true;
        return;
      }
      poll();
    }
  }

  // When the loop must next run with no socket event or publication
  // (studio::WakeSchedule::NextWakeNs).
  std::int64_t next_wake_ns(std::int64_t now_ns) const {
    return schedule.NextWakeNs(now_ns, !wake || backlog, system_graph,
                               [this](const auto& visit) {
                                 for (Socket* client : clients.clients())
                                   visit(client->getUserData()->feeds);
                               });
  }
};

// Puts `fd` in the uSockets loop as a callback poll that runs `on_ready`
// while it is readable, leaving the descriptor for the caller to drain. The
// poll's extension holds `user`.
us_poll_t* WatchReadable(us_loop_t* loop, int fd, void* user,
                         void (*on_ready)(us_internal_callback_t*)) {
  auto* poll =
      us_create_poll(loop, 0, sizeof(us_internal_callback_t) + sizeof(user));
  us_poll_init(poll, fd, POLL_TYPE_CALLBACK);
  auto* callback = reinterpret_cast<us_internal_callback_t*>(poll);
  callback->loop = loop;
  callback->cb_expects_the_loop = 0;
  callback->leave_poll_ready = 1;
  callback->cb = on_ready;
  const auto bytes = std::bit_cast<std::array<std::byte, sizeof(user)>>(user);
  std::memcpy(callback + 1, bytes.data(), bytes.size());
  us_poll_start(poll, loop, LIBUS_SOCKET_READABLE);
  return poll;
}

// The `user` WatchReadable stored with the poll.
template <class T>
T* WatchedUser(us_internal_callback_t* callback) {
  std::array<std::byte, sizeof(void*)> bytes{};
  std::memcpy(bytes.data(), callback + 1, bytes.size());
  return static_cast<T*>(std::bit_cast<void*>(bytes));
}
std::string mime(const fs::path& p) {
  auto ext = p.extension().string();
  if (ext == ".html") return "text/html; charset=utf-8";
  if (ext == ".js") return "text/javascript";
  if (ext == ".css") return "text/css";
  if (ext == ".json") return "application/json";
  if (ext == ".svg") return "image/svg+xml";
  if (ext == ".wasm") return "application/wasm";
  return "application/octet-stream";
}
// Static files and the manifest are read synchronously on the loop thread
// that also polls RTMS, so each is capped. The static cap is a few times the
// built Studio bundle's largest asset; the manifest cap is Studio's own
// MAX_MANIFEST_BYTES (studio/src/data/manifest.ts), which rejects anything
// larger anyway.
constexpr std::uintmax_t kMaxStaticFileBytes = std::uintmax_t{4} * 1024 * 1024;
constexpr std::uintmax_t kMaxManifestBytes = std::uintmax_t{1024} * 1024;
// uWebSockets' idle timeout for a client, in seconds. With T - m seconds
// gone without data from the client (or a drain of its backlog) it pings,
// and it closes the client if nothing arrives in the next m: m is 4, 8 or
// 16 s, 8 at T = 16 (WebSocketContextData::calculateIdleTimeoutCompnents).
// uSockets checks timeouts every 4 s, so the ping can go up to 4 s early,
// and a client that stopped answering loses its slot 12 to 16 s after the
// last data it sent. 16 is the smallest T with an 8 s pong window, which
// covers five retransmissions of a lost pong at TCP's 200 ms minimum RTO
// (sent by 6.2 s). The bound and its reason: docs/DATA_FLOW.md "Telemetry,
// Studio, and the radio boundary". uWebSockets refuses T below 8 or above
// 960.
constexpr unsigned kIdleTimeoutS = 16;
constexpr unsigned kMinIdleTimeoutS = 8;
constexpr unsigned kMaxIdleTimeoutS = 960;
}  // namespace
int main(int argc, char** argv) try {
  // `--describe`: answer what this process is without touching shared memory,
  // sockets, or the HTTP port, so the launcher can probe this binary like any
  // other node and carry it in the declared graph. `--config` and `--sim` are
  // accepted and otherwise ignored, because the launcher probes every binary
  // with the same flags.
  for (int i = 1; i < argc; ++i) {
    if (std::string_view(argv[i]) != "--describe") continue;
    const char* config_path = nullptr;
    for (int j = 1; j + 1 < argc; ++j) {
      if (std::string_view(argv[j]) == "--config") config_path = argv[j + 1];
    }
    // The watched topic is read from [studio] topic in robot configuration.
    std::string topic = "/talos/telemetry";
    if (config_path != nullptr) {
      try {
        const auto robot = talos::config::ParseRobotConfig(config_path);
        topic = robot.studio.topic;
      } catch (const std::exception& e) {
        std::cerr << "studio_bridge --describe: cannot parse " << config_path
                  << " (" << e.what() << "); describing the default topic\n";
      }
    }
    talos::introspect::Description description;
    description.name = "studio_bridge";
    description.target = "//studio/bridge:studio_bridge";
    // Every topic end names its schema and ring layout (the launcher creates
    // the segment from them): the telemetry byte topic's, as its producer
    // declares it and as the queue below attaches.
    talos::introspect::naming::SourceShape source{
        talos::event::SourceKind::WATCHER, topic,
        static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes), 0};
    const talos::schema::Identity bytes = talos::schema::Bytes();
    source.schema = bytes.name;
    source.schema_hash = bytes.hash;
    source.alignment = 8;
    source.slots =
        static_cast<std::uint32_t>(talos::telemetry_wire::slot_count);
    description.sources.push_back(std::move(source));
    std::cout << talos::introspect::DescribeToJson(description);
    return 0;
  }
  // `--declared` and `--manifest` are lifted out before anything reads a
  // position, so either can be passed anywhere on the line without shifting
  // the positional arguments.
  std::vector<const char*> args;
  std::string declared_path;
  std::string manifest_path;
  std::string config_path;
  unsigned idle_timeout_s = kIdleTimeoutS;
  bool bad_idle_timeout = false;
  for (int i = 0; i < argc; ++i) {
    const std::string_view argument = argv[i];
    static constexpr std::string_view kDeclared = "--declared";
    static constexpr std::string_view kManifest = "--manifest";
    if (argument == "--config" && i + 1 < argc) {
      config_path = argv[++i];
      continue;
    }
    if (argument == "--idle-timeout-s" && i + 1 < argc) {
      const std::string_view value = argv[++i];
      const auto [end, error] = std::from_chars(
          value.data(), value.data() + value.size(), idle_timeout_s);
      bad_idle_timeout = error != std::errc{} ||
                         end != value.data() + value.size() ||
                         idle_timeout_s < kMinIdleTimeoutS ||
                         idle_timeout_s > kMaxIdleTimeoutS;
      continue;
    }
    if (argument == kDeclared && i + 1 < argc) {
      declared_path = argv[++i];
      continue;
    }
    if (argument.starts_with(kDeclared) && argument.size() > kDeclared.size() &&
        argument[kDeclared.size()] == '=') {
      declared_path = argument.substr(kDeclared.size() + 1);
      continue;
    }
    if (argument == kManifest && i + 1 < argc) {
      manifest_path = argv[++i];
      continue;
    }
    if (argument.starts_with(kManifest) && argument.size() > kManifest.size() &&
        argument[kManifest.size()] == '=') {
      manifest_path = argument.substr(kManifest.size() + 1);
      continue;
    }
    args.push_back(argv[i]);
  }
  const std::size_t count = args.size();
  const auto at = [&args](std::size_t i) { return args[i]; };

  const bool unknown_option =
      std::any_of(args.begin() + 1, args.end(), [](const char* argument) {
        return std::string_view(argument).starts_with("--");
      });
  if (count > 4 || unknown_option || bad_idle_timeout) {
    std::cerr
        << "Usage: studio_bridge "
           "[topic=/talos/telemetry] [HTTP=5800] [webroot=studio/dist]\n"
           "       [--declared PATH]   the launcher's graph.json, served at "
           "/declared.json\n"
           "       [--manifest PATH]   the launcher's manifest.json, served at "
           "/manifest.json\n"
           "       [--config PATH]     reads [studio] max_clients\n"
           "       [--idle-timeout-s S]  client idle timeout, 8 to 960 "
           "(default 16)\n"
           "       [--describe]          print the --describe envelope and "
           "exit\n";
    return 2;
  }
  // SIGINT and SIGTERM request a stop and make the stop pipe readable; the
  // loop watches it, so a stop wakes the loop wherever it waits.
  talos::process::InstallStopHandlers();
  const char* topic = count > 1 ? at(1) : "/talos/telemetry";

  // The declared graph: what the launcher probed with `--describe` before it
  // spawned anything.
  //
  // This is what lets Studio tell "a node crashed before it registered" from
  // "a node is not part of this session". The registry only ever holds what did
  // start, so a name's absence from it says nothing on its own; the declared
  // graph is the other half of that comparison.
  //
  // Served verbatim and never parsed. The launcher already wrote JSON, there is
  // no JSON parser in this process, and re-emitting a document byte for byte is
  // the one transformation that cannot be subtly wrong.
  std::string declared_json;
  if (!declared_path.empty()) {
    std::ifstream declared(declared_path, std::ios::binary);
    if (declared) {
      declared_json.assign(std::istreambuf_iterator<char>(declared),
                           std::istreambuf_iterator<char>());
    } else {
      // Logged once, then the bridge carries on. A viewer that cannot show
      // declared-against-actual is a much smaller failure than a bridge that
      // will not start. Under the launcher this file is written atomically
      // before anything spawns, so this path means a standalone run with a
      // wrong --declared path.
      std::cerr << "declared graph unreadable at " << declared_path
                << "; /declared.json will report it absent\n";
    }
  }
  // Port 0 asks the OS for a free port; the bound port is reported on stdout
  // once listening, so a test never hands the bridge a port it closed first.
  auto port = [](const char* s) {
    auto p = std::stoul(s);
    if (p > 65535) throw std::runtime_error("invalid port");
    return static_cast<std::uint16_t>(p);
  };
  const auto root = fs::canonical(count > 3 ? at(3) : "studio/dist");
  Bridge bridge;
  bridge.topic = topic;
  // Watched by segment name, not by topic. A topic like /talos/telemetry has
  // an interior slash, which POSIX does not allow in an shm name, so RTMS maps
  // it (`ShmObjectName`) before touching the OS. The bridge attaches only
  // once the segment exists: an empty ring of its own would wait for a
  // publisher that might disagree with its layout.
  bridge.segment = ShmObjectName(topic);
  try {
    bridge.wake.emplace();
  } catch (const std::exception& e) {
    std::cerr << "no wake channel (" << e.what()
              << "); reading the topic every 1 ms\n";
  }
  if (!config_path.empty()) {
    bridge.max_clients =
        talos::config::ParseRobotConfig(config_path).studio.max_clients;
  }
  uWS::App app;
  uWS::App::WebSocketBehavior<Client> behavior{};
  behavior.maxPayloadLength = 1024;
  behavior.maxBackpressure = 512 * 1024;
  // A client that stopped answering (a laptop that dropped off the network)
  // gives up its slot within the idle timeout (kIdleTimeoutS); one that only
  // receives keeps it by answering the pings, as browsers do.
  behavior.idleTimeout = static_cast<unsigned short>(idle_timeout_s);
  behavior.sendPingsAutomatically = true;
  behavior.upgrade = [&](auto* response, auto* request, auto* context) {
    if (bridge.clients.clients().size() >= bridge.max_clients) {
      ++bridge.refused;
      response->writeStatus("503 Studio client limit")
          ->writeHeader("Retry-After", "5")
          ->end("Studio client limit reached");
      return;
    }
    response->template upgrade<Client>(
        {}, request->getHeader("sec-websocket-key"),
        request->getHeader("sec-websocket-protocol"),
        request->getHeader("sec-websocket-extensions"), context);
  };
  behavior.open = [&](auto* ws) {
    bridge.clients.Add(ws);
    const int fd = static_cast<int>(
        reinterpret_cast<std::uintptr_t>(ws->getNativeHandle()));
    const int send_buffer =
        studio::ClientSendBuffer::Bytes(bridge.client_rate());
    if (::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &send_buffer,
                     sizeof send_buffer) != 0) {
      std::cerr << "client send buffer not set: " << std::strerror(errno)
                << '\n';
    }
    // The current graph at once, so the client does not open on an empty
    // system view, then the dictionary and a keyframe of the latest tick, so
    // it starts with every channel instead of waiting for the next dictionary
    // the producer schedules. That tick is as old as the last telemetry
    // received -- possibly from before telemetry stopped -- so the client
    // treats its first update as of unknown age.
    // It is offered all that in this pass, in its turn.
    bridge.pass_clients();
  };
  // A client held back by buffered output is offered what is due as soon as
  // its socket drains, in a pass: behind any client owed a turn before it,
  // so a client whose writes keep draining is not sent to twice in a turn.
  behavior.drain = [&](auto* ws) { bridge.drained(ws); };
  behavior.close = [&](auto* ws, int, std::string_view) {
    bridge.clients.Remove(ws);
  };
  app.ws<Client>("/telemetry", std::move(behavior));
  app.get("/telemetry.json", [&](auto* res, auto*) {
    // 503 until a dictionary and an aligned frame have both arrived: an
    // empty or half-described document would read as "the robot is doing
    // nothing", which is a different statement from "not known yet".
    if (!bridge.telemetry_state.has_dictionary ||
        !bridge.telemetry_state.has_frame) {
      res->writeStatus("503 Service Unavailable")->end();
      return;
    }
    res->writeHeader("Content-Type", "application/json")
        ->writeHeader("X-Content-Type-Options", "nosniff")
        ->end(studio::TelemetryJson(bridge.telemetry_state,
                                    talos::introspect::monotonic_now_ns()));
  });
  app.get("/channels.json", [&](auto* res, auto*) {
    if (!bridge.telemetry_state.has_dictionary) {
      res->writeStatus("503 Service Unavailable")->end();
      return;
    }
    res->writeHeader("Content-Type", "application/json")
        ->writeHeader("X-Content-Type-Options", "nosniff")
        ->end(studio::ChannelsJson(bridge.telemetry_state));
  });
  app.get("/system.json", [&](auto* res, auto*) {
    res->writeHeader("Content-Type", "application/json")
        ->writeHeader("X-Content-Type-Options", "nosniff")
        ->end(bridge.system_graph.json);
  });
  app.get("/declared.json", [&](auto* res, auto*) {
    // 404 rather than an empty document: "no declared graph was given" and "the
    // declared graph is empty" are different statements, and a viewer that
    // cannot tell them apart would draw a session with no nodes in it.
    if (declared_json.empty()) {
      res->writeStatus("404 Not Found")->end();
      return;
    }
    res->writeHeader("Content-Type", "application/json")
        ->writeHeader("X-Content-Type-Options", "nosniff")
        ->end(declared_json);
  });
  app.get("/manifest.json", [&](auto* res, auto*) {
    // Re-read on every request: the launcher atomically replaces the manifest
    // as nodes start, restart, and exit, so /manifest.json is the live view of
    // restart counts and exit status. 404 when no path was given or the file
    // is unreadable, so "no manifest" is not confused with "an empty manifest".
    if (manifest_path.empty()) {
      res->writeStatus("404 Not Found")->end();
      return;
    }
    std::error_code error;
    const std::uintmax_t size = fs::file_size(manifest_path, error);
    if (error) {
      res->writeStatus("404 Not Found")->end();
      return;
    }
    if (size > kMaxManifestBytes) {
      res->writeStatus("413 Content Too Large")->end();
      return;
    }
    std::ifstream input(manifest_path, std::ios::binary);
    if (!input) {
      res->writeStatus("404 Not Found")->end();
      return;
    }
    std::string body((std::istreambuf_iterator<char>(input)), {});
    res->writeHeader("Content-Type", "application/json")
        ->writeHeader("X-Content-Type-Options", "nosniff")
        ->end(body);
  });
  app.get("/*", [&](auto* res, auto* req) {
    auto url = std::string(req->getUrl());
    // Reject encoded/ambiguous paths; serve only canonical descendants.
    if (url.find('%') != url.npos || url.find('\\') != url.npos) {
      res->writeStatus("400 Bad Request")->end();
      return;
    }
    std::error_code error;
    auto file = fs::weakly_canonical(
        root / (url == "/" ? "index.html" : url.substr(1)), error);
    if (error) {
      res->writeStatus("404 Not Found")->end();
      return;
    }
    auto relative = file.lexically_relative(root);
    if (relative.empty() || *relative.begin() == ".." ||
        !fs::is_regular_file(file, error)) {
      res->writeStatus("404 Not Found")->end();
      return;
    }
    // Files are bounded; uWS owns buffered response bytes after end().
    if (fs::file_size(file, error) > kMaxStaticFileBytes) {
      res->writeStatus("413 Content Too Large")->end();
      return;
    }
    std::ifstream input(file, std::ios::binary);
    if (!input) {
      res->writeStatus("404 Not Found")->end();
      return;
    }
    std::string body((std::istreambuf_iterator<char>(input)), {});
    res->writeHeader("Content-Type", mime(file))
        ->writeHeader("X-Content-Type-Options", "nosniff")
        ->end(body);
  });
  bool listening = false;
  app.listen(count > 2 ? port(at(2)) : 5800, [&](auto* token) {
    listening = token != nullptr;
    bridge.listener = token;
  });
  if (!listening) throw std::runtime_error("HTTP listen failed");
  std::cout << "listening "
            << us_socket_local_port(
                   0, reinterpret_cast<us_socket_t*>(bridge.listener))
            << '\n'
            << std::flush;
  // The loop sleeps until a socket event, a publication on the topic (the
  // wake channel's descriptor) or the timer, which is set before each wait
  // to the bridge's next duty (Bridge::next_wake_ns). Every return from the
  // wait ends in the post handler below, which makes the bridge's pass.
  auto* loop = reinterpret_cast<us_loop_t*>(uWS::Loop::get());
  auto* timer = us_create_timer(loop, 0, sizeof(Bridge*));
  if (!timer) throw std::runtime_error("deadline timer allocation failed");
  *static_cast<Bridge**>(us_timer_ext(timer)) = &bridge;
  const auto on_deadline = [](us_timer_t* t) {
    ++(*static_cast<Bridge**>(us_timer_ext(t)))->deadline_wakes;
  };
  us_poll_t* wake_poll = nullptr;
  if (bridge.wake) {
    wake_poll = WatchReadable(
        loop, bridge.wake->fd(), &bridge, [](us_internal_callback_t* callback) {
          ++WatchedUser<Bridge>(callback)->publication_wakes;
        });
  }
  // The stop pipe: readable from the moment a stop is requested, so a signal
  // landing after the pass's check and before the wait still ends the wait.
  // Level-triggered: the pass that sees the stop removes it.
  us_poll_t* stop_poll = nullptr;
  if (const int stop_fd = talos::process::StopFd(); stop_fd >= 0) {
    stop_poll =
        WatchReadable(loop, stop_fd, &bridge, [](us_internal_callback_t*) {});
  }
  std::int64_t timer_due_ns = 0;
  bool stopping = false;
  const auto pass = [&] {
    if (stopping) return;
    ++bridge.wakes;
    if (talos::process::stop_requested.load()) {
      stopping = true;
      us_listen_socket_close(0, bridge.listener);
      auto clients = bridge.clients.clients();
      for (auto* ws : clients) ws->close();
      us_timer_close(timer);
      for (us_poll_t* poll : {wake_poll, stop_poll}) {
        if (poll == nullptr) continue;
        us_poll_stop(poll, loop);
        us_poll_free(poll, loop);
      }
      return;
    }
    bridge.service(talos::introspect::monotonic_now_ns());
    const std::int64_t now_ns = talos::introspect::monotonic_now_ns();
    const std::int64_t next_ns = bridge.next_wake_ns(now_ns);
    // Set again only when it would fire too late, or has fired: one that
    // fires early costs a pass that finds nothing due.
    if (timer_due_ns <= now_ns || next_ns < timer_due_ns) {
      // Whole milliseconds, rounded up, so it never fires before the duty.
      const auto ms =
          static_cast<int>((next_ns - now_ns + 999'999) / 1'000'000);
      us_timer_set(timer, on_deadline, ms, 0);
      timer_due_ns = now_ns + std::int64_t{ms} * 1'000'000;
    }
  };
  pass();
  // A stop makes the stop pipe readable, which ends the wait in this handler.
  uWS::Loop::get()->addPostHandler(&bridge, [&pass](uWS::Loop*) { pass(); });
  // Normal process termination is handled below; signal exit avoids freeing a
  // running loop.
  app.run();
  uWS::Loop::get()->removePostHandler(&bridge);
  std::cerr << "forwarded=" << bridge.forwarded << " invalid=" << bridge.invalid
            << " lapped=" << bridge.lapped << " held=" << bridge.held
            << " ws_drops=" << bridge.ws_drops << " refused=" << bridge.refused
            << " egress_bytes=" << bridge.egress_bytes
            << " wakes=" << bridge.wakes
            << " publication_wakes=" << bridge.publication_wakes
            << " deadline_wakes=" << bridge.deadline_wakes << '\n';
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
