load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# Shared ground-truth library for non-C++ nodes. New package only; no
# existing target is modified.
cc_library(
    name = "node_api",
    srcs = ["node_api.cc"],
    hdrs = ["node_api.h"],
    deps = [
        "//talOS/events:events_core",
        "//talOS/introspection:describe",
        "//talOS/ipc:ipc",
        "//talOS/rtms:rtms",
    ],
)

# Shared object loaded via ctypes/cffi. linkshared produces libtalos_node.so.
cc_binary(
    name = "libtalos_node.so",
    srcs = [
        "node_api.cc",
        "node_api.h",
    ],
    linkshared = True,
    deps = [
        "//talOS/events:events_core",
        "//talOS/introspection:describe",
        "//talOS/ipc:ipc",
        "//talOS/rtms:rtms",
    ],
)

cc_test(
    name = "node_api_test",
    size = "small",
    srcs = ["node_api_test.cc"],
    deps = [
        ":node_api",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
