// For session_report_test: naming::IsTargetLane, the authority, on each line
// of stdin, printed as one line of "1" (a target lane) or "0", so the test
// can hold session_report.py's copy of the grammar to it.
#include <iostream>
#include <string>

#include "talOS/introspection/names.h"

int main() {
  std::string topic;
  while (std::getline(std::cin, topic)) {
    std::cout << (talos::introspect::naming::IsTargetLane(topic) ? "1" : "0")
              << '\n';
  }
  return 0;
}
