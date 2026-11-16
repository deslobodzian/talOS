// A node that prints Node::describing() from its constructor, for the test
// that only the --describe probe builds a node with it set.

#include <cstdio>

#include "talOS/node/node.h"

class Describing {
 public:
  explicit Describing(talos::Node& node) {
    std::fprintf(stderr, "describing=%d\n", node.describing() ? 1 : 0);
  }

  void update() {}
};

TALOS_NODE(Describing)
