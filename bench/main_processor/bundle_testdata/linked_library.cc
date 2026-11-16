// bundle_test only: a C++ node whose binary needs libfixture_library.so, a
// shared library the bundle stages in lib/ and the node finds there through
// talos_cc_node's rpath. Not a robot node: it owns no device and publishes
// nothing.
#include "bench/main_processor/bundle_testdata/fixture_library.h"
#include "talOS/node/node.h"

namespace {

class LinkedLibrary {
 public:
  // Calls into the library, so the link keeps it as a needed library.
  explicit LinkedLibrary(talos::Node& /*node*/)
      : answer_{bundle_testdata::FixtureLibraryAnswer()} {}
  void update() { static_cast<void>(answer_); }

 private:
  int answer_;
};

}  // namespace

TALOS_NODE(LinkedLibrary)
