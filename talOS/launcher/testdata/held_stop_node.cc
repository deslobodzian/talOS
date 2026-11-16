// A node binary whose runtime, like CPython's, has a start-up of its own
// before the node framework runs, during which a SIGINT ends it with an
// ordinary exit 1 (CPython raises KeyboardInterrupt; "Fatal Python error:
// init_import_site" while it imports `site`). Once its handler is in it
// creates `<--log value>.started`, for the test to send the stop then, and
// its start-up lasts until a SIGINT is pending, so the stop always lands
// inside it: a stop delivered there ends the process 1, and one the launcher
// held (blocked) stays pending and is then taken by the framework's stop
// handlers, in talos::Run, as a stop during construction (exit 0).
// --describe skips it, and answers through the framework, holds_stop
// included.

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <string_view>
#include <thread>

#include "talOS/node/node.h"

namespace {

class HeldStopNode {
 public:
  explicit HeldStopNode(talos::Node& /*node*/) {}
  void update() {}
};

void ExitOne(int /*signal*/) { ::_exit(1); }

// The runtime's start-up: SIGINT exits 1 until a SIGINT is pending. The bound
// only keeps a launch that never sends the stop from hanging here.
void RuntimeStartUp(const std::string& started) {
  struct sigaction action {};
  action.sa_handler = ExitOne;
  sigemptyset(&action.sa_mask);
  ::sigaction(SIGINT, &action, nullptr);
  const int marker = ::open(started.c_str(), O_CREAT | O_WRONLY, 0644);
  if (marker >= 0) ::close(marker);
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds{20};
  while (std::chrono::steady_clock::now() < give_up) {
    sigset_t pending;
    sigemptyset(&pending);
    if (::sigpending(&pending) == 0 && sigismember(&pending, SIGINT) == 1)
      return;
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
}

}  // namespace

int main(int argc, char** argv) {
  bool describing = false;
  std::string log;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};
    if (arg == "--describe") describing = true;
    if (arg == "--log" && i + 1 < argc) log = argv[i + 1];
  }
  if (!describing) RuntimeStartUp(log + ".started");
  return talos::Run<HeldStopNode>(argc, argv);
}
