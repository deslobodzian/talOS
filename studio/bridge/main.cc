#include <App.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_set>

#include "talOS/rtms/rtms.h"
#include "wire.h"
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
  std::unordered_set<Socket*> clients;
  us_listen_socket_t* listener = nullptr;
  std::uint64_t invalid = 0, udp_drops = 0, ws_drops = 0;
  void poll() {
    // Bound work so HTTP and client socket processing cannot starve.
    for (unsigned count = 0; count < 256; ++count) {
      if (!queue.read(reader, [&](std::span<const std::byte> slot) {
            auto bytes = studio::payload(slot);
            if (bytes.empty()) {
              ++invalid;
              return;
            }
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
  if (argc < 2 || std::string_view(argv[1]) != "--drop-newest-publisher") {
    std::cerr
        << "Usage: studio_bridge --drop-newest-publisher [topic=/talos_studio] "
           "[IP=127.0.0.1] [UDP=5801] [HTTP=5800] [webroot=studio/dist]\n"
           "The publisher MUST use DROP_NEWEST; RTMS does not store this "
           "policy in shared memory.\n";
    return 2;
  }
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  const char* topic = argc > 2 ? argv[2] : "/talos_studio";
  // Refuse a missing publisher instead of silently creating an empty ring.
  int existing = shm_open(topic, O_RDWR, 0);
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
  dest.sin_port = htons(argc > 4 ? port(argv[4]) : 5801);
  if (inet_pton(AF_INET, argc > 3 ? argv[3] : "127.0.0.1", &dest.sin_addr) != 1)
    throw std::runtime_error("invalid IPv4 address");
  const auto root = fs::canonical(argc > 6 ? argv[6] : "studio/dist");
  Bridge bridge{queue, *reader, udp, dest};
  uWS::App app;
  uWS::App::WebSocketBehavior<Client> behavior{};
  behavior.maxPayloadLength = 1024;
  behavior.maxBackpressure = 512 * 1024;
  behavior.open = [&](auto* ws) { bridge.clients.insert(ws); };
  behavior.close = [&](auto* ws, int, std::string_view) {
    bridge.clients.erase(ws);
  };
  app.ws<Client>("/telemetry", std::move(behavior));
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
  app.listen(argc > 5 ? port(argv[5]) : 5800, [&](auto* token) {
    listening = token != nullptr;
    bridge.listener = token;
  });
  if (!listening) throw std::runtime_error("HTTP listen failed");
  auto* timer = us_create_timer(reinterpret_cast<us_loop_t*>(uWS::Loop::get()),
                                0, sizeof(Bridge*));
  if (!timer) throw std::runtime_error("poll timer allocation failed");
  *static_cast<Bridge**>(us_timer_ext(timer)) = &bridge;
  us_timer_set(
      timer,
      [](us_timer_t* t) {
        if (stop) {
          auto* b = *static_cast<Bridge**>(us_timer_ext(t));
          us_listen_socket_close(0, b->listener);
          auto clients = b->clients;
          for (auto* ws : clients) ws->close();
          us_timer_close(t);
          return;
        }
        (*static_cast<Bridge**>(us_timer_ext(t)))->poll();
      },
      1, 1);
  // Normal process termination is handled below; signal exit avoids freeing a
  // running loop.
  app.run();
  std::cerr << "invalid=" << bridge.invalid << " udp_drops=" << bridge.udp_drops
            << " ws_drops=" << bridge.ws_drops << '\n';
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
