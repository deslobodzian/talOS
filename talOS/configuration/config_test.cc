#include <gtest/gtest.h>
#include <filesystem>
#include <iostream>
#include "config_parser.h"

TEST(ConfigurationTest, Parsing) {
    auto config = talos::config::ParseSubsystemToml("talOS/configuration/subsystem.toml");

    EXPECT_EQ(config.inverted, false);
    EXPECT_EQ(config.stator_current, 20);
    EXPECT_EQ(config.supply_current, 20);
}
