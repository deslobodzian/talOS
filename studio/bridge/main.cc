#include <App.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "system.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/rtms/rtms.h"
#include "talOS/telemetry/wire.h"
#include "telemetry.h"
namespace fs = std::filesystem;
namespace {
volatile std::sig_atomic_t stop = 0;
void signal_handler(int) { stop = 1; }
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
  unsigned watch_ticks = 0;
  bool announced_wait = false;
  std::string attach_error;
  std::vector<Socket*> clients;
  // Where the next fan-out starts. The shared bucket goes to whichever client
  // asks first, and every client's own burst equals the shared one, so in a
  // fixed order the clients ahead of the last would spend the shared bucket
  // and starve it for seconds; starting one place later each time gives every
  // client first claim in turn.
  std::size_t first_client = 0;
  us_listen_socket_t* listener = nullptr;
  // ws_drops counts messages the socket refused; that client is then
  // resynchronized (the whole graph, a telemetry keyframe). A graph or
  // telemetry update the budget defers is not counted: the client's next
  // update carries its changes. forwarded counts complete producer ticks.
  std::uint64_t invalid = 0, ws_drops = 0, forwarded = 0;
  // Frames the publisher overwrote before this bridge read them: it never
  // waits for the bridge, so a stalled bridge is lapped instead.
  std::uint64_t lapped = 0;
  // read_next() copies each page here and validates the copy before
  // ingesting commits side effects (client sends, counters).
  std::vector<std::byte> frame =
      std::vector<std::byte>(talos::telemetry_wire::slot_bytes);
  std::size_t max_clients = 4;
  std::uint64_t refused = 0, egress_bytes = 0;
  EgressBudget budget;

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

  // Called every poll tick: checks every 100 ticks while waiting, every
  // 1000 once attached (a replacement is rare; each check is a few syscalls).
  void watch() {
    if (++watch_ticks < (queue ? 1000u : 100u)) return;
    watch_ticks = 0;
    check_segment();
  }

  // Each client's own refill rate, B/s.
  double client_rate() const {
    return EgressBudget::kBytesPerSecond / static_cast<double>(max_clients);
  }

  // Whether `framed_bytes` fit the client's and the shared bucket now, and
  // the client has nothing buffered. Spends nothing.
  bool affordable(Socket* client, std::size_t framed_bytes) {
    const std::int64_t now = talos::introspect::monotonic_now_ns();
    auto& local = client->getUserData()->budget;
    if (client->getBufferedAmount() != 0 ||
        !local.available(framed_bytes, client_rate(), now) ||
        !budget.available(framed_bytes, EgressBudget::kBytesPerSecond, now)) {
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
  void pump(Socket* client) {
    client->getUserData()->feeds.Pump(
        system_graph, telemetry_state, talos::introspect::monotonic_now_ns(),
        client_rate(),
        [this, client](std::size_t framed_bytes) {
          return affordable(client, framed_bytes);
        },
        [this, client](std::string_view text) {
          return send_now(client, text, uWS::OpCode::TEXT);
        },
        [this, client](std::string_view page) {
          return send_now(client, page, uWS::OpCode::BINARY);
        });
  }

  template <typename Send>
  void for_each_client(Send&& send_to) {
    const std::size_t count = clients.size();
    if (count == 0) return;
    const std::size_t first = first_client++ % count;
    for (std::size_t i = 0; i < count; ++i) {
      send_to(clients[(first + i) % count]);
    }
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
                    .clients = clients.size(),
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

  void poll() {
    if (queue) {
      // Bound work so HTTP and client socket processing cannot starve.
      for (unsigned count = 0; count < 256; ++count) {
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
    // Every poll, not only on a new tick: an update the budget deferred goes
    // out as soon as the budget refills, and a quiet stream still sends its
    // heartbeat.
    for_each_client([this](Socket* client) { pump(client); });
  }
};
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
  for (int i = 0; i < argc; ++i) {
    const std::string_view argument = argv[i];
    static constexpr std::string_view kDeclared = "--declared";
    static constexpr std::string_view kManifest = "--manifest";
    if (argument == "--config" && i + 1 < argc) {
      config_path = argv[++i];
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
  if (count > 4 || unknown_option) {
    std::cerr
        << "Usage: studio_bridge "
           "[topic=/talos/telemetry] [HTTP=5800] [webroot=studio/dist]\n"
           "       [--declared PATH]   the launcher's graph.json, served at "
           "/declared.json\n"
           "       [--manifest PATH]   the launcher's manifest.json, served at "
           "/manifest.json\n"
           "       [--config PATH]     reads [studio] max_clients\n"
           "       [--describe]          print the --describe envelope and "
           "exit\n";
    return 2;
  }
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
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
  auto port = [](const char* s) {
    auto p = std::stoul(s);
    if (p == 0 || p > 65535) throw std::runtime_error("invalid port");
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
  bridge.check_segment();
  if (!config_path.empty()) {
    bridge.max_clients =
        talos::config::ParseRobotConfig(config_path).studio.max_clients;
  }
  uWS::App app;
  uWS::App::WebSocketBehavior<Client> behavior{};
  behavior.maxPayloadLength = 1024;
  behavior.maxBackpressure = 512 * 1024;
  behavior.upgrade = [&](auto* response, auto* request, auto* context) {
    if (bridge.clients.size() >= bridge.max_clients) {
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
    bridge.clients.push_back(ws);
    // The current graph at once, so the client does not open on an empty
    // system view, then the dictionary and a keyframe of the latest tick, so
    // it starts with every channel instead of waiting for the next dictionary
    // the producer schedules. That tick is as old as the last telemetry
    // received -- possibly from before telemetry stopped -- so the client
    // treats its first update as of unknown age.
    bridge.pump(ws);
  };
  behavior.close = [&](auto* ws, int, std::string_view) {
    std::erase(bridge.clients, ws);
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
  auto* timer = us_create_timer(reinterpret_cast<us_loop_t*>(uWS::Loop::get()),
                                0, sizeof(Bridge*));
  if (!timer) throw std::runtime_error("poll timer allocation failed");
  *static_cast<Bridge**>(us_timer_ext(timer)) = &bridge;
  // One timer at the telemetry rate; the system graph is structure, not
  // signal, so it refreshes every 250 ms instead, by the clock: under load a
  // 1 ms timer fires late, and counting ticks stretched the refresh past
  // GRAPH_STALE_AFTER_MS (studio/src/data/freshness.ts). Each client is sent
  // its changed counters at most every GraphFeed::kUpdateNs (or a deferral
  // notice), so Studio keeps measuring node rates from successive graphs'
  // counters and never marks a merely deferred graph stale.
  static constexpr std::int64_t kSystemRefreshNs = 250'000'000;
  static std::int64_t refreshed_ns = talos::introspect::monotonic_now_ns();
  bridge.refresh_system();
  us_timer_set(
      timer,
      [](us_timer_t* t) {
        auto* b = *static_cast<Bridge**>(us_timer_ext(t));
        if (stop) {
          us_listen_socket_close(0, b->listener);
          auto clients = b->clients;
          for (auto* ws : clients) ws->close();
          us_timer_close(t);
          return;
        }
        b->watch();
        b->poll();
        const std::int64_t now_ns = talos::introspect::monotonic_now_ns();
        if (now_ns - refreshed_ns >= kSystemRefreshNs) {
          refreshed_ns = now_ns;
          b->refresh_system();
        }
      },
      1, 1);
  // Normal process termination is handled below; signal exit avoids freeing a
  // running loop.
  app.run();
  std::cerr << "forwarded=" << bridge.forwarded << " invalid=" << bridge.invalid
            << " lapped=" << bridge.lapped << " ws_drops=" << bridge.ws_drops
            << " refused=" << bridge.refused
            << " egress_bytes=" << bridge.egress_bytes << '\n';
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
