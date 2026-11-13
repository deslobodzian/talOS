load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# The naming protocol, as code: the topic grammar, the closed role vocabulary
# and the graph-level checks. Uses the transport's reader-slot limit so launch
# validation stays consistent with runtime registration, and reads each
# writer's message schema for the alert-field grammar (names.cc).
cc_library(
    name = "names",
    srcs = ["names.cc"],
    hdrs = ["names.h"],
    deps = [
        "//talOS/events:events_core",
        "//talOS/hardware",
        "//talOS/rtms",
        "@flatbuffers//:runtime_cc",
    ],
)

# What a node's shape is before it runs: `--describe` writes it, the launcher
# reads it.
cc_library(
    name = "describe",
    hdrs = ["describe.h"],
    deps = [
        ":names",
        "//talOS/events:events_core",
    ],
)

# The live registry: what is running, and which topics each node publishes and
# subscribes to. Depends only on the manifest types, so a reader (Studio's
# bridge, a command-line tool) does not pull in an event loop.
cc_library(
    name = "registry",
    hdrs = ["registry.h"],
    deps = [
        ":lock_file",
        ":names",
        "//talOS/events:events_core",
        "//talOS/memory:shared_memory",
        "//talOS/process:realtime_outcome",
        "//talOS/rtms:process_liveness",
    ],
)

# Lock files beside a shared-memory namespace: the registry's writer lock and
# the launcher's one-session-per-namespace lock.
cc_library(
    name = "lock_file",
    hdrs = ["lock_file.h"],
)

# Publishes a running loop into the registry. Separate target because a reader
# never needs it.
cc_library(
    name = "reporter",
    hdrs = ["reporter.h"],
    deps = [
        ":describe",
        ":registry",
    ],
)

cc_test(
    name = "names_test",
    size = "small",
    srcs = ["names_test.cc"],
    deps = [
        ":names",
        "//talOS/schema:identity",
        "//talOS/schema:test_messages",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "describe_test",
    size = "small",
    srcs = ["describe_test.cc"],
    deps = [
        ":describe",
        "//talOS/events",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "registry_test",
    size = "small",
    srcs = ["registry_test.cc"],
    deps = [
        ":names",
        ":registry",
        ":reporter",
        "//talOS/events",
        "//talOS/rtms",
        "//talOS/rtms:wake",
        "@googletest//:gtest_main",
    ],
)
