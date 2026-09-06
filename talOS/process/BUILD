load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "process",
    hdrs = ["process.h"],
    visibility = ["//visibility:public"],
)

# Shared runner for loop-node binaries. Header-only; parameterized by a
# factory lambda because node constructors vary. talOS/bridge and
# talOS/launcher mains are excluded (not loop nodes).
cc_library(
    name = "node_main",
    hdrs = ["node_main.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":process",
        "//talOS/events:events",
        "//talOS/introspection:describe",
        "//talOS/introspection:reporter",
    ],
)
