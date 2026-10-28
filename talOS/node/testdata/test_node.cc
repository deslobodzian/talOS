// A node exercising every generic registration: an optional subscription, a
// publication, an extra timer and a parameter. node_test runs it for
// --describe and for a logged --sim run replayed without divergence.

#include <chrono>
#include <cstdint>

#include "node_test/node_test_message_generated.h"
#include "talOS/node/node.h"

class Counter {
 public:
  explicit Counter(talos::Node& node)
      : node_{node},
        step_{node.param<std::int64_t>("step")},
        input_{node.subscribe<node_test::Count>("/test_node/target",
                                                {.optional = true})},
        output_{node.publish<node_test::Count>("/test_node/state")} {
    node.every(std::chrono::milliseconds{50}, [this] { ++beats_; });
  }

  void update() {
    const std::int64_t extra = input_.has_value() ? input_->value() : 0;
    output_.send(node_test::Count{
        static_cast<std::int64_t>(node_.tick()) * step_ + beats_ + extra,
        node_.now().nanos()});
  }

 private:
  talos::Node& node_;
  std::int64_t step_;
  talos::Subscription<node_test::Count> input_;
  talos::Publisher<node_test::Count> output_;
  std::int64_t beats_ = 0;
};

TALOS_NODE(Counter)
