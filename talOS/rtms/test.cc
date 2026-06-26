#include <gtest/gtest.h>
#include "rtms.h"

TEST(RTMSTest, RTMS) {
    std::printf("rtms_queue_t size: %li", sizeof(rtms_queue_t));
    EXPECT_EQ(5, 5);
}
