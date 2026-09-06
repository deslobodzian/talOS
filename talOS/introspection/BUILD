load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# The naming protocol, as code: the topic grammar, the closed role vocabulary
# and the graph-level checks. No transport and no shared memory, so the launcher
# can validate a graph that has not started and the registry can validate one
# that has, from the same source of truth.
cc_library(
    name = "names",
    hdrs = ["names.h"],
    deps = ["//talOS/events:events_core"],
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
        ":names",
        "//talOS/events:events_core",
    ],
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
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "describe_test",
    size = "small",
    srcs = ["describe_test.cc"],
    deps = [
        ":describe",
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
        "//talOS/events:events",
        "@googletest//:gtest_main",
    ],
)
