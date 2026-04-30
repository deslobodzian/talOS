#include <gtest/gtest.h>

TEST(Message, MessageTest) {
    EXPECT_STRNE("hello", "world");
    EXPECT_EQ(6 * 7, 42);
}
