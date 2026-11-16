// continuity_test's publisher. Its first run (no `marker` file yet) writes
// the marker on its first tick -- not in the constructor, which the
// launcher's --describe probe runs too -- publishes kMessages counts and dies
// by SIGKILL, which the launcher restarts; the restart publishes kMessages
// more and stops itself with SIGTERM, a clean exit the launcher does not
// restart because the test's config marks this node non-critical.

#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include "continuity_test/continuity_message_generated.h"
#include "talOS/node/node.h"

class Source {
 public:
  static constexpr std::uint32_t kMessages = 60;

  explicit Source(talos::Node& node)
      : marker_{node.param<std::string>("marker")},
        run_{std::filesystem::exists(marker_) ? 1u : 0u},
        out_{node.publish<continuity_test::Count>("state")} {}

  void update() {
    if (sent_ == kMessages) return;
    if (sent_ == 0 && run_ == 0) std::ofstream{marker_} << "first run\n";
    out_.send(continuity_test::Count{run_, sent_});
    if (++sent_ < kMessages) return;
    std::raise(run_ == 0 ? SIGKILL : SIGTERM);
  }

 private:
  std::string marker_;
  std::uint32_t run_;
  talos::Publisher<continuity_test::Count> out_;
  std::uint32_t sent_ = 0;
};

TALOS_NODE(Source)
