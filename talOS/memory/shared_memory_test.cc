#include "shared_memory_ptr.h"

#include <gtest/gtest.h>

TEST(SharedMemory, SharedMemoryPtr) {
    {
        SharedMemoryPtr test = SharedMemoryPtr::create("/tmp/rtms_test", 100);
    }
}
