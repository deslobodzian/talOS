// A node that reads the declared graph: it observes every `state` topic a
// node in the graph publishes, as the telemetry node does. node_test runs it
// for --describe without and with --graph, and for real without one.

#include <string_view>

#include "talOS/introspection/describe.h"
#include "talOS/node/node.h"

class Observer {
 public:
  explicit Observer(talos::Node& node) {
    using talos::detail::NodeAccess;
    const talos::detail::DeclaredGraph* graph =
        NodeAccess::declared_graph(node);
    if (graph == nullptr) return;  // the launcher's first --describe pass
    for (const auto& described : *graph) {
      for (const auto& source : described.sources) {
        if (source.kind != talos::event::SourceKind::SENDER ||
            !std::string_view{source.name}.ends_with("/state")) {
          continue;
        }
        NodeAccess::add_observer(
            node, source.name, source.message_bytes, source.alignment,
            {source.schema, source.schema_hash, source.bfbs});
      }
    }
  }

  void update() {}
};

TALOS_NODE(Observer)
