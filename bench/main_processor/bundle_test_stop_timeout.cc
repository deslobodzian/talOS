// Prints the launcher's stop timeout (kStopTimeout) in whole seconds, the
// bound bundle_test holds the boot service's stop timeout above, so the test
// reads it from the launcher's own constant.

#include <chrono>
#include <cstdio>

#include "talOS/launcher/launcher.h"

int main() {
  std::printf("%lld\n",
              static_cast<long long>(std::chrono::ceil<std::chrono::seconds>(
                                         talos::launcher::kStopTimeout)
                                         .count()));
  return 0;
}
