load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

cc_library(
    name = "launcher_lib",
    hdrs = ["launcher.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/configuration",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/rtms",
        "//talOS/schema:unit_suffix",
        "@tomlplusplus//:toml++",
    ],
)

cc_binary(
    name = "launcher",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [":launcher_lib"],
)

cc_binary(
    name = "log_dump",
    srcs = ["log_dump.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":launcher_lib",
        "//talOS/events:events_core",
        "//talOS/schema:decode",
    ],
)

cc_test(
    name = "launcher_test",
    size = "small",
    srcs = ["launcher_test.cc"],
    deps = [
        ":launcher_lib",
        "//talOS/events:events_log",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "supervision_test",
    size = "small",
    srcs = ["supervision_test.cc"],
    deps = [
        ":launcher_lib",
        "@googletest//:gtest_main",
    ],
)

# A publisher that kills itself (SIGKILL) on its first run and exits cleanly
# on its second, and a subscriber that logs what it receives: the launcher's
# session-owned segments keep the subscriber's topic across both.
talos_messages(
    name = "continuity_message",
    srcs = ["testdata/continuity_message.fbs"],
    include_prefix = "continuity_test",
    visibility = ["//visibility:private"],
)

talos_cc_node(
    name = "continuity_source",
    srcs = ["testdata/continuity_source.cc"],
    node_name = "continuity_source",
    visibility = ["//visibility:private"],
    deps = [":continuity_message"],
)

talos_cc_node(
    name = "continuity_sink",
    srcs = ["testdata/continuity_sink.cc"],
    node_name = "continuity_sink",
    visibility = ["//visibility:private"],
    deps = [":continuity_message"],
)

cc_test(
    name = "continuity_test",
    size = "small",
    srcs = ["continuity_test.cc"],
    data = [
        ":continuity_sink",
        ":continuity_source",
    ],
    env = {
        "TALOS_SOURCE_NODE": "$(rlocationpath :continuity_source)",
        "TALOS_SINK_NODE": "$(rlocationpath :continuity_sink)",
    },
    deps = [
        ":continuity_message",
        ":launcher_lib",
        "//talOS/events:events_log",
        "//talOS/rtms",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
