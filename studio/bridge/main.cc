#include <App.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
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
struct EgressBudget {
  static constexpr double kBytesPerSecond = 160000;
  static constexpr double kBurstBytes = 64000;
  // WebSocket framing is charged too, conservatively: a 2-byte header below
  // 126 payload bytes, otherwise 10.
  static constexpr std::size_t framed(std::size_t bytes) {
    return bytes + (bytes < 126 ? 2 : 10);
  }
  double tokens = kBurstBytes;
  std::chrono::steady_clock::time_point updated =
      std::chrono::steady_clock::now();

  bool available(std::size_t bytes, double rate) {
    const auto now = std::chrono::steady_clock::now();
    tokens = std::min(
        kBurstBytes,
        tokens + std::chrono::duration<double>(now - updated).count() * rate);
    updated = now;
    return tokens >= static_cast<double>(bytes);
  }
};
struct Client {
  EgressBudget budget;
  std::uint64_t stream_id = 0;
  std::uint32_t dictionary_id = 0;
  bool has_dictionary = false;
};
struct ReaderLease {
  RTMSQueue& queue;
  std::size_t id;
  ~ReaderLease() { queue.release_reader(id); }
};
using Socket = uWS::WebSocket<false, true, Client>;
struct Bridge {
  RTMSQueue& queue;
  std::size_t reader;
  std::string topic;
  std::vector<Socket*> clients;
  // Where the next fan-out starts. The shared bucket goes to whichever client
  // asks first, and every client's own burst equals the shared one, so in a
  // fixed order the clients ahead of the last would spend the shared bucket
  // and starve it for seconds; starting one place later each time gives every
  // client first claim in turn.
  std::size_t first_client = 0;
  us_listen_socket_t* listener = nullptr;
  std::uint64_t invalid = 0, ws_drops = 0, telemetry_ws_drops = 0,
                forwarded = 0;
  // read_next() copies each frame here first: forwarding commits side effects
  // (client sends, counters), which a zero-copy read() callback must not.
  std::vector<std::byte> frame =
      std::vector<std::byte>(talos::telemetry_wire::slot_bytes);
  std::size_t max_clients = 4;
  std::uint64_t refused = 0, egress_bytes = 0;
  EgressBudget budget;

  bool send(Socket* client, std::string_view bytes, uWS::OpCode opcode) {
    // Charge WebSocket framing too. Drop rather than queue old observations.
    const auto size = EgressBudget::framed(bytes.size());
    auto& local = client->getUserData()->budget;
    if (client->getBufferedAmount() != 0 ||
        !local.available(size, EgressBudget::kBytesPerSecond /
                                   static_cast<double>(max_clients)) ||
        !budget.available(size, EgressBudget::kBytesPerSecond)) {
      ++ws_drops;
      if (opcode == uWS::OpCode::BINARY) ++telemetry_ws_drops;
      return false;
    }
    if (client->send(bytes, opcode) == Socket::DROPPED) {
      ++ws_drops;
      if (opcode == uWS::OpCode::BINARY) ++telemetry_ws_drops;
      return false;
    }
    local.tokens -= static_cast<double>(size);
    budget.tokens -= static_cast<double>(size);
    egress_bytes += size;
    return true;
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

  // The decoded v2 feed: latest dictionary and latest aligned frame,
  // backing /telemetry.json, /channels.json and the dictionary a newly
  // connected WebSocket client is handed on open.
  studio::TelemetryState telemetry_state;

  void send_dictionary(Socket* client) {
    if (send(client, telemetry_state.dictionary_frame, uWS::OpCode::BINARY)) {
      auto& state = *client->getUserData();
      state.stream_id = telemetry_state.stream_id;
      state.dictionary_id = telemetry_state.dictionary_id;
      state.has_dictionary = true;
    }
  }

  // Opened lazily: the bridge may well start before any node, and a viewer
  // that refuses to run until the robot is up is a viewer you cannot use to
  // watch the robot come up.
  std::optional<talos::introspect::RegistryReader> registry;
  std::string system_json = "{}";
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

    system_json = studio::SystemGraphJson(input);

    // A send must fit the egress burst, so a larger graph is never delivered:
    // every attempt counts as a ws_drop and Studio marks its last graph stale.
    // Logged once per crossing, not on every refresh.
    const bool oversized =
        static_cast<double>(EgressBudget::framed(system_json.size())) >
        EgressBudget::kBurstBytes;
    if (oversized && !system_json_oversized) {
      std::cerr << "system graph is " << system_json.size() << " B, over the "
                << EgressBudget::kBurstBytes
                << " B WebSocket egress burst; clients will not receive it "
                   "(/system.json still serves it)\n";
    }
    system_json_oversized = oversized;

    // Text on the WebSocket, so a client tells the two message families apart
    // by frame type and needs no sniffing.
    for_each_client([this](Socket* client) {
      send(client, system_json, uWS::OpCode::TEXT);
    });
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
    // ReadMode::SEQUENCE, so it sees every frame in order rather than only the
    // newest.
    source.kind = talos::event::SourceKind::WATCHER;
    source.name = topic;
    source.message_bytes =
        static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes);
    source.events = forwarded;
    source.dropped = telemetry_ws_drops;
    source.sequence = queue.writer_sequence();
    node.sources.push_back(std::move(source));
    return node;
  }

  void poll() {
    // Bound work so HTTP and client socket processing cannot starve.
    for (unsigned count = 0; count < 256; ++count) {
      MessageInfo info;
      const auto result = queue.read_next(reader, frame, info);
      if (result == ReadResult::TORN) {
        ++invalid;
        continue;
      }
      if (result != ReadResult::OK) break;
      auto bytes = talos::telemetry_wire::payload(frame);
      if (bytes.empty()) {
        ++invalid;
        continue;
      }
      if (!studio::IngestTelemetry(telemetry_state, bytes,
                                   talos::introspect::monotonic_now_ns())) {
        ++invalid;
        continue;
      }
      // Forward only frames a client can decode: one aligned with the cached
      // dictionary (a dictionary frame aligns with itself). A value frame that
      // arrives before any dictionary is ingested above but not sent, so the
      // first binary frame every client sees is a dictionary frame.
      const auto* decoded =
          Talos::Telemetry::GetSizePrefixedFrame(bytes.data());
      const bool aligned =
          telemetry_state.has_dictionary && decoded != nullptr &&
          decoded->stream_id() == telemetry_state.stream_id &&
          decoded->dictionary_id() == telemetry_state.dictionary_id &&
          decoded->values() != nullptr &&
          decoded->values()->size() == telemetry_state.dictionary.size();
      if (!aligned) continue;
      ++forwarded;
      for_each_client([this, bytes](Socket* client) {
        const auto& state = *client->getUserData();
        if (!state.has_dictionary ||
            state.stream_id != telemetry_state.stream_id ||
            state.dictionary_id != telemetry_state.dictionary_id) {
          // A budget drop must not strand a viewer without its dictionary.
          // Withhold values so its bucket can refill enough to retry.
          send_dictionary(client);
          return;
        }
        send(client, bytes, uWS::OpCode::BINARY);
      });
    }
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

  if (count < 2 || std::string_view(at(1)) != "--drop-newest-publisher") {
    std::cerr
        << "Usage: studio_bridge --drop-newest-publisher "
           "[topic=/talos/telemetry] [HTTP=5800] [webroot=studio/dist]\n"
           "       [--declared PATH]   the launcher's graph.json, served at "
           "/declared.json\n"
           "       [--manifest PATH]   the launcher's manifest.json, served at "
           "/manifest.json\n"
           "       [--config PATH]     reads [studio] max_clients\n"
           "       [--describe]          print the --describe envelope and "
           "exit\n"
           "The publisher MUST use DROP_NEWEST; RTMS does not store this "
           "policy in shared memory.\n";
    return 2;
  }
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  const char* topic = count > 2 ? at(2) : "/talos/telemetry";

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
  // Refuse a missing publisher instead of silently creating an empty ring.
  //
  // Opened by segment name, not by topic. A topic like /talos/telemetry has an
  // interior slash, which POSIX does not allow in an shm name, so RTMS maps it
  // (`ShmObjectName`) before touching the OS. Checking the raw topic here
  // looked for a segment that never exists and reported a healthy publisher as
  // missing -- on macOS, where the unmapped name happens to be legal, this
  // failed only once the mapping was introduced.
  const std::string segment = ShmObjectName(topic);
  int existing = shm_open(segment.c_str(), O_RDWR, 0);
  if (existing < 0) {
    throw std::runtime_error("start the telemetry publisher before the bridge");
  }
  close(existing);
  RTMSQueue queue(topic, talos::telemetry_wire::slot_bytes, 8,
                  talos::schema::Bytes(), talos::telemetry_wire::slot_count,
                  {OverflowPolicy::DROP_NEWEST, ReadMode::SEQUENCE});
  auto reader = queue.register_reader();
  if (!reader) throw std::runtime_error("RTMS reader slots exhausted");
  ReaderLease lease{queue, *reader};
  auto port = [](const char* s) {
    auto p = std::stoul(s);
    if (p == 0 || p > 65535) throw std::runtime_error("invalid port");
    return static_cast<std::uint16_t>(p);
  };
  const auto root = fs::canonical(count > 4 ? at(4) : "studio/dist");
  Bridge bridge{queue, *reader, topic};
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
    // Send the current graph immediately: a client that has to wait for the
    // next refresh opens on an empty system view.
    bridge.send(ws, bridge.system_json, uWS::OpCode::TEXT);
    // Then the cached dictionary frame, so the client's first value frame
    // is already decodable instead of waiting up to a second for the next
    // dictionary the producer schedules. Its values are as old as the last
    // telemetry received -- possibly from before telemetry stopped -- so the
    // client treats this first binary message as of unknown age.
    if (bridge.telemetry_state.has_dictionary) {
      bridge.send_dictionary(ws);
    }
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
        ->end(bridge.system_json);
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
  app.listen(count > 3 ? port(at(3)) : 5800, [&](auto* token) {
    listening = token != nullptr;
    bridge.listener = token;
  });
  if (!listening) throw std::runtime_error("HTTP listen failed");
  auto* timer = us_create_timer(reinterpret_cast<us_loop_t*>(uWS::Loop::get()),
                                0, sizeof(Bridge*));
  if (!timer) throw std::runtime_error("poll timer allocation failed");
  *static_cast<Bridge**>(us_timer_ext(timer)) = &bridge;
  // One timer at the telemetry rate; the system graph is structure, not
  // signal, so it refreshes every 250th tick instead. Each refresh is re-sent
  // even when the structure is unchanged: Studio marks the graph stale when
  // none arrives within GRAPH_STALE_AFTER_MS (studio/src/data/freshness.ts),
  // and measures node rates from successive graphs' counters.
  static constexpr unsigned kSystemRefreshTicks = 250;
  static unsigned ticks = 0;
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
        b->poll();
        if (++ticks >= kSystemRefreshTicks) {
          ticks = 0;
          b->refresh_system();
        }
      },
      1, 1);
  // Normal process termination is handled below; signal exit avoids freeing a
  // running loop.
  app.run();
  std::cerr << "forwarded=" << bridge.forwarded << " invalid=" << bridge.invalid
            << " ws_drops=" << bridge.ws_drops << " refused=" << bridge.refused
            << " egress_bytes=" << bridge.egress_bytes << '\n';
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
