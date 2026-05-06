#pragma once

#include <iostream>
#include <toml.hpp>

inline void test() {
    constexpr std::string_view source = R"(
        [app]
        name = "demo"
        threads = 4
    )";
    toml::table config = toml::parse(source);
    std::cout << "name = "
              << config["app"]["name"].value_or("unknown")
              << "\n";

    std::cout << "threads = "
              << config["app"]["threads"].value_or(1)
              << "\n";
}
