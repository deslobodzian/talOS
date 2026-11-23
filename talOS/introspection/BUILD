load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:private"])

exports_files(
    ["names.h"],
    visibility = ["//tools:__pkg__"],
)

_FRAMEWORK_VISIBILITY = [
    "//studio:__subpackages__",
    "//talOS:__subpackages__",
    "//tools:__pkg__",
]

# The naming protocol, as code: the topic grammar, the closed role vocabulary
# and the graph-level checks. Pins kMaxTopicReaders to RTMS's MAX_READERS in
# names_test.cc so :names stays independent of //talOS/rtms, and reads each
# writer's message schema for the alert-field grammar (names.cc).
cc_library(
    name = "names",
    srcs = ["names.cc"],
    hdrs = ["names.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/events:events_core",
        "//talOS/schema:verify",
        "@flatbuffers//:runtime_cc",
    ],
)

# Advisory hardware-config lint (CAN bus load, sensor velocity limits), run by
# the launcher beside LintGraph. Separate from :names so talOS/node never
# reaches talOS/hardware through talOS/introspection.
cc_library(
    name = "hardware_lint",
    hdrs = ["hardware_lint.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":names",
        "//talOS/hardware:config",
    ],
)

# What a node's shape is before it runs: `--describe` writes it, the launcher
# reads it.
cc_library(
    name = "describe",
    hdrs = ["describe.h"],
    visibility = ["//visibility:public"],
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
    srcs = ["registry.cc"],
    hdrs = ["registry.h"],
    visibility = _FRAMEWORK_VISIBILITY,
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
    visibility = ["//visibility:public"],
)

# Publishes a running loop into the registry. Separate target because a reader
# never needs it.
cc_library(
    name = "reporter",
    hdrs = ["reporter.h"],
    visibility = _FRAMEWORK_VISIBILITY,
    deps = [
        ":describe",
        ":registry",
        "//talOS/memory:residency",
    ],
)

cc_test(
    name = "names_test",
    size = "small",
    srcs = ["names_test.cc"],
    deps = [
        ":hardware_lint",
        ":names",
        "//talOS/hardware",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "//talOS/schema:test_messages",
        "@flatbuffers//:runtime_cc",
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
        "//talOS/events:events_log",
        "//talOS/rtms",
        "//talOS/rtms:wake",
        "@googletest//:gtest_main",
    ],
)
