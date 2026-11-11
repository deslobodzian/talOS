// A node whose constructor blocks in read(2) on stdin until EOF, standing in
// for one stuck in a slow system call (a camera waiting on the Metal compiler
// cache lock). node_test stops it there. The read must restart after the stop
// signal: an interrupted read fails construction (exit 1). For each byte it
// reads, it prints whether the stop has landed ("stop requested" or
// "running"), which is how the test knows the signal arrived while the
// constructor was still blocked.

#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include "talOS/node/node.h"
#include "talOS/process/process.h"

class Blocked {
 public:
  explicit Blocked(talos::Node& /*node*/) {
    std::printf("constructing\n");
    std::fflush(stdout);
    char byte = 0;
    for (;;) {
      const ssize_t got = ::read(STDIN_FILENO, &byte, 1);
      if (got == 0) return;
      if (got < 0) {
        throw std::runtime_error(std::string{"read: "} + std::strerror(errno));
      }
      std::printf("%s\n", talos::process::stop_requested.load()
                              ? "stop requested"
                              : "running");
      std::fflush(stdout);
    }
  }

  void update() {}
};

TALOS_NODE(Blocked)
