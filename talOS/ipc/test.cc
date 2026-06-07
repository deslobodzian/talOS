#include <gtest/gtest.h>
#include "shared_memory.h"

TEST(OpenSHMMAP, SharedMemory) {
    open_shmmap("/test", 64);
}
