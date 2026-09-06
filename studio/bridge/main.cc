#include <App.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "system.h"
#include "talOS/introspection/registry.h"
#include "talOS/rtms/rtms.h"
#include "studio/schema/wire.h"
namespace fs = std::filesystem;
namespace {
volatile std::sig_atomic_t stop = 0;
void signal_handler(int) { stop = 1; }
struct Client {};
struct ReaderLease {
  RTMSQueue& queue;
  std::size_t id;
  ~ReaderLease() { queue.release_reader(id); }
};
struct FileDescriptor {
  int value;
  ~FileDescriptor() {
    if (value >= 0) close(value);
  }
};
using Socket = uWS::WebSocket<false, true, Client>;
struct Bridge {
  RTMSQueue& queue;
  std::size_t reader;
  int udp;
  sockaddr_in destination;
  std::string topic;
  std::unordered_set<Socket*> clients;
  us_listen_socket_t* listener = nullptr;
  std::uint64_t invalid = 0, udp_drops = 0, ws_drops = 0, forwarded = 0;

  // Opened lazily: the bridge may well start before any node, and a viewer
  // that refuses to run until the robot is up is a viewer you cannot use to
  // watch the robot come up.
  std::optional<talos::introspect::RegistryReader> registry;
  std::string system_json = "{}";

  // Identifies one document across its chunks, so a receiver never splices
  // halves of two different refreshes together.
  std::uint16_t system_document = 0;

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
                    .udp_drops = udp_drops,
                    .ws_drops = ws_drops};
    input.wall_ns = talos::introspect::wall_now_ns();
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

    // Text on the WebSocket, so a client tells the two message families apart
    // by frame type and needs no sniffing. The UDP path has no such framing,
    // hence the explicit tag.
    for (auto* client : clients) {
      if (client->getBufferedAmount() > 256 * 1024) {
        ++ws_drops;
        continue;
      }
      if (client->send(system_json, uWS::OpCode::TEXT) == Socket::DROPPED) {
        ++ws_drops;
      }
    }

    // Chunked: a whole graph exceeds the platform's maximum datagram. See
    // SystemDatagrams for why sending it whole fails invisibly.
    for (const std::string& datagram :
         studio::SystemDatagrams(system_json, ++system_document)) {
      if (sendto(udp, datagram.data(), datagram.size(), 0,
                 reinterpret_cast<sockaddr*>(&destination),
                 sizeof(destination)) < 0) {
        ++udp_drops;
      }
    }
  }

  // This process as the registry would have described it, had it a slot.
  talos::introspect::NodeSnapshot self() const {
    talos::introspect::NodeSnapshot node;
    node.name = "studio_bridge";
    node.target = "//studio/bridge:studio_bridge";
    node.pid = static_cast<std::uint64_t>(::getpid());
    node.start_wall_ns = started_wall_ns;
    node.heartbeat_wall_ns = talos::introspect::wall_now_ns();
    node.alive = true;
    node.dispatch_count = forwarded;
    node.declared_source_count = 1;

    talos::introspect::SourceSnapshot source;
    // A watcher, not a fetcher: the reader is registered with
    // ReadMode::SEQUENCE, so it sees every frame in order rather than only the
    // newest.
    source.kind = talos::event::SourceKind::WATCHER;
    source.name = topic;
    source.message_bytes = static_cast<std::uint32_t>(studio::slot_bytes);
    source.events = forwarded;
    source.dropped = ws_drops + udp_drops;
    source.sequence = queue.writer_sequence();
    node.sources.push_back(std::move(source));
    return node;
  }

  void poll() {
    // Bound work so HTTP and client socket processing cannot starve.
    for (unsigned count = 0; count < 256; ++count) {
      if (!queue.read(reader, [&](std::span<const std::byte> slot) {
            auto bytes = studio::payload(slot);
            if (bytes.empty()) {
              ++invalid;
              return;
            }
            ++forwarded;
            if (sendto(udp, bytes.data(), bytes.size(), 0,
                       reinterpret_cast<sockaddr*>(&destination),
                       sizeof(destination)) < 0)
              ++udp_drops;
            for (auto* client : clients) {
              if (client->getBufferedAmount() > 256 * 1024) {
                ++ws_drops;
                continue;
              }
              if (client->send(bytes, uWS::OpCode::BINARY) == Socket::DROPPED)
                ++ws_drops;
            }
          }))
        break;
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
}  // namespace
int main(int argc, char** argv) try {
  // `--declared` is lifted out before anything reads a position, so it can be
  // passed anywhere on the line without shifting the positional arguments that
  // every existing script and the launcher already pass.
  std::vector<const char*> args;
  std::string declared_path;
  for (int i = 0; i < argc; ++i) {
    const std::string_view argument = argv[i];
    static constexpr std::string_view kDeclared = "--declared";
    if (argument == kDeclared && i + 1 < argc) {
      declared_path = argv[++i];
      continue;
    }
    if (argument.starts_with(kDeclared) && argument.size() > kDeclared.size() &&
        argument[kDeclared.size()] == '=') {
      declared_path = argument.substr(kDeclared.size() + 1);
      continue;
    }
    args.push_back(argv[i]);
  }
  const std::size_t count = args.size();
  const auto at = [&args](std::size_t i) { return args[i]; };

  if (count < 2 || std::string_view(at(1)) != "--drop-newest-publisher") {
    std::cerr
        << "Usage: studio_bridge --drop-newest-publisher "
           "[topic=/talos/telemetry] "
           "[IP=127.0.0.1] [UDP=5801] [HTTP=5800] [webroot=studio/dist]\n"
           "       [--declared PATH]   the launcher's graph.json, served at "
           "/declared.json\n"
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
      // will not start, and the launcher writes this file concurrently with
      // spawning us.
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
  if (existing < 0)
    throw std::runtime_error("start the telemetry publisher before the bridge");
  close(existing);
  RTMSQueue queue(topic, studio::slot_bytes, 8, studio::slot_count,
                  {OverflowPolicy::DROP_NEWEST, ReadMode::SEQUENCE});
  auto reader = queue.register_reader();
  if (!reader) throw std::runtime_error("RTMS reader slots exhausted");
  ReaderLease lease{queue, *reader};
  int udp = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp < 0) throw std::runtime_error("UDP socket failed");
  FileDescriptor udp_owner{udp};
  const int flags = fcntl(udp, F_GETFL, 0);
  if (flags < 0 || fcntl(udp, F_SETFL, flags | O_NONBLOCK) < 0)
    throw std::runtime_error("UDP nonblocking setup failed");
  int broadcast = 1;
  setsockopt(udp, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
  auto port = [](const char* s) {
    auto p = std::stoul(s);
    if (p == 0 || p > 65535) throw std::runtime_error("invalid port");
    return static_cast<unsigned short>(p);
  };
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(count > 4 ? port(at(4)) : 5801);
  if (inet_pton(AF_INET, count > 3 ? at(3) : "127.0.0.1", &dest.sin_addr) != 1)
    throw std::runtime_error("invalid IPv4 address");
  const auto root = fs::canonical(count > 6 ? at(6) : "studio/dist");
  Bridge bridge{queue, *reader, udp, dest, topic};
  uWS::App app;
  uWS::App::WebSocketBehavior<Client> behavior{};
  behavior.maxPayloadLength = 1024;
  behavior.maxBackpressure = 512 * 1024;
  behavior.open = [&](auto* ws) {
    bridge.clients.insert(ws);
    // Send the current graph immediately: a client that has to wait for the
    // next refresh opens on an empty system view.
    ws->send(bridge.system_json, uWS::OpCode::TEXT);
  };
  behavior.close = [&](auto* ws, int, std::string_view) {
    bridge.clients.erase(ws);
  };
  app.ws<Client>("/telemetry", std::move(behavior));
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
    if (fs::file_size(file, error) > 32 * 1024 * 1024) {
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
  app.listen(count > 5 ? port(at(5)) : 5800, [&](auto* token) {
    listening = token != nullptr;
    bridge.listener = token;
  });
  if (!listening) throw std::runtime_error("HTTP listen failed");
  auto* timer = us_create_timer(reinterpret_cast<us_loop_t*>(uWS::Loop::get()),
                                0, sizeof(Bridge*));
  if (!timer) throw std::runtime_error("poll timer allocation failed");
  *static_cast<Bridge**>(us_timer_ext(timer)) = &bridge;
  // One timer at the telemetry rate; the system graph is structure, not
  // signal, so it refreshes every 250th tick instead.
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
            << " udp_drops=" << bridge.udp_drops
            << " ws_drops=" << bridge.ws_drops << '\n';
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
