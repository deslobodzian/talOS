#include <gtest/gtest.h>
#include "configuration.h"

TEST(ConfigurationTest, Parsing) {
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(6 * 7, 42);
  test();
}
