#include <chrono>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "talOS/node/node.h"

namespace {
bool describing = false;
class ConstructionNode {
 public:
  explicit ConstructionNode(talos::Node& node) {
    if (describing) return;
    const int delay = node.param<int>("construction_ms", 6000);
    if (delay < 0) throw std::runtime_error("construction fixture failure");
    std::this_thread::sleep_for(std::chrono::milliseconds{delay});
  }
  void update() {}
};
}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string_view{argv[i]} == "--describe") describing = true;
  }
  return talos::Run<ConstructionNode>(argc, argv);
}
