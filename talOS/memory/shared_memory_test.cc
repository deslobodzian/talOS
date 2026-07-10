#include "shared_memory_ptr.h"

#include <gtest/gtest.h>

TEST(SharedMemory, SharedMemoryPtr) {
#if defined(__linux__)
    auto path = "rtms_test";
#else
    auto path = "/tmp/rtms/test";
#endif
    {
        SharedMemoryPtr test{path, 100};
    }
}
