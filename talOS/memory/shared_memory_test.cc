#include "shared_memory_ptr.h"

#include <gtest/gtest.h>

TEST(SharedMemory, SharedMemoryPtr) {
    {
        SharedMemoryPtr test("/tmp/rtms_test", 100);
    }
}
