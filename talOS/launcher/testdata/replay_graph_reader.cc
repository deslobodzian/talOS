// replay_inputs_test's graph reader, a node whose endpoints come from the
// declared graph, as telemetry's and teleop's do. It counts the `state` topics
// the other nodes publish and sends that count each tick. Under a session
// directory it keeps a per-session record there, as the 2026 autonomous node
// keeps its routine selection and progress.

#include <cstdint>
#include <fstream>
#include <string_view>

#include "replay_inputs_test/replay_message_generated.h"
#include "talOS/introspection/describe.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"

class ReplayGraphReader {
 public:
  explicit ReplayGraphReader(talos::Node& node)
      : out_{node.publish<replay_inputs_test::Summary>("state")} {
    using talos::detail::NodeAccess;
    if (const auto session = node.session_dir()) {
      std::ofstream{*session / "replay_graph_reader.record"} << "kept\n";
    }
    const talos::detail::DeclaredGraph* graph =
        NodeAccess::declared_graph(node);
    if (graph == nullptr) return;  // the launcher's first --describe pass
    for (const auto& described : *graph) {
      for (const auto& source : described.sources) {
        if (source.kind == talos::event::SourceKind::SENDER &&
            std::string_view{source.name}.ends_with("/state")) {
          ++state_topics_;
        }
      }
    }
  }

  void update() { out_.send(replay_inputs_test::Summary{state_topics_, 0.0F}); }

 private:
  talos::Publisher<replay_inputs_test::Summary> out_;
  std::uint32_t state_topics_ = 0;
};

TALOS_NODE(ReplayGraphReader)
