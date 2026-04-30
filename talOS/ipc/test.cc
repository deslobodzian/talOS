#include <gtest/gtest.h>

TEST(HelloTest, Assertion) {
    EXPECT_STRNE("hello", "world");
    EXPECT_EQ(6 * 7, 42);
}
