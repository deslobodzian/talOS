load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# Types shared by every loop: time, dispatch context, callbacks, the source
# manifest, the timer queue, the recorder policy and the delivery policy.
# Header only, no OS calls.
cc_library(
    name = "events_core",
    hdrs = [
        "callback.h",
        "context.h",
        "delivery.h",
        "dispatch_observer.h",
        "manifest.h",
        "recorder.h",
        "scheduler.h",
        "time.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["//talOS/schema:identity"],
)

cc_test(
    name = "scheduler_test",
    size = "small",
    srcs = ["scheduler_test.cc"],
    deps = [
        ":events_core",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# Dispatch log: on-disk format, the recorder that writes it, the reader that
# replays it.
cc_library(
    name = "events_log",
    srcs = [
        "log/log_reader.cc",
        "log/log_writer.cc",
    ],
    hdrs = [
        "log/format.h",
        "log/log_reader.h",
        "log/log_writer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":events_core",
        "//talOS/process:durable_sync",
    ],
)

cc_test(
    name = "log_test",
    size = "small",
    srcs = ["log/log_test.cc"],
    deps = [
        ":events_log",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# Platform readiness backend. kqueue on macOS, epoll_pwait2 on Linux; the
# concrete type is picked at compile time so there is no virtual dispatch.
cc_library(
    name = "events_os",
    srcs = select({
        "@platforms//os:macos": ["os/kqueue_poller.cc"],
        "@platforms//os:linux": ["os/epoll_poller.cc"],
    }),
    hdrs = select({
        "@platforms//os:macos": ["os/kqueue_poller.h"],
        "@platforms//os:linux": ["os/epoll_poller.h"],
    }) + ["os/poller.h"],
    visibility = ["//visibility:public"],
    deps = [":events_core"],
)

cc_test(
    name = "poller_test",
    size = "small",
    srcs = ["os/poller_test.cc"],
    deps = [
        ":events_os",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# Loop messages carry a schema descriptor, so the test messages come from
# talos_messages like every node's.
talos_messages(
    name = "events_test_message",
    srcs = ["events_test_message.fbs"],
    include_prefix = "events_test",
    visibility = ["//visibility:private"],
)

# The loops themselves.
cc_library(
    name = "events",
    hdrs = [
        "event_loop_base.h",
        "handles.h",
        "realtime_event_loop.h",
        "replay_event_loop.h",
        "simulated_event_loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":events_core",
        ":events_log",
        ":events_os",
        "//talOS/ipc",
        "//talOS/rtms",
        "//talOS/rtms:wake",
        "@flatbuffers",
    ],
)

# Drives a program that was not in a recorded session from that session's
# logged topics, on a simulated loop (input_replay.h).
cc_library(
    name = "input_replay",
    srcs = ["input_replay.cc"],
    hdrs = ["input_replay.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":events",
        ":events_core",
        ":events_log",
        "//talOS/schema:identity",
    ],
)

cc_test(
    name = "input_replay_test",
    size = "small",
    srcs = ["input_replay_test.cc"],
    deps = [
        ":events",
        ":events_log",
        ":events_test_message",
        ":input_replay",
        ":test_robot",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# A tiny robot built on the loop API, shared by the determinism tests.
cc_library(
    name = "test_robot",
    testonly = True,
    hdrs = ["test_robot.h"],
    deps = [
        ":events",
        ":events_test_message",
    ],
)

cc_test(
    name = "simulated_event_loop_test",
    size = "small",
    srcs = ["simulated_event_loop_test.cc"],
    deps = [
        ":events",
        ":events_test_message",
        ":test_robot",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "replay_test",
    size = "small",
    srcs = ["replay_test.cc"],
    deps = [
        ":events",
        ":events_test_message",
        ":test_robot",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "realtime_event_loop_test",
    size = "small",
    srcs = ["realtime_event_loop_test.cc"],
    deps = [
        ":events",
        ":events_test_message",
        ":test_robot",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
