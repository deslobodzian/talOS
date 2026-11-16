// For shm_name_test: prints ShmObjectName(topic, namespace) for
// `shm_name_probe <namespace> <topic>`.
#include <iostream>

#include "talOS/memory/shared_memory_ptr.h"

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::cout << ShmObjectName(argv[2], argv[1]) << '\n';
  return 0;
}
