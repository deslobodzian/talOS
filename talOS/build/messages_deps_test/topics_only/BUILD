load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages")

# A topics-only package: no schema of its own, its state type comes from
# `deps` (as a pose estimator publishes geometry.PoseEstimate). The targets
# still carry the topic names and make the dep's types reachable.
talos_messages(
    name = "messages",
    srcs = [],
    include_prefix = "deps_topics_only",
    topics = {
        "state": "/deps_topics_only/state",
    },
    deps = ["//talOS/build/messages_deps_test/geometry:messages"],
)

cc_test(
    name = "messages_cc_test",
    size = "small",
    srcs = ["messages_test.cc"],
    deps = [
        ":messages",
        "@googletest//:gtest_main",
    ],
)

py_test(
    name = "messages_py_test",
    size = "small",
    srcs = ["messages_test.py"],
    main = "messages_test.py",
    deps = [
        ":messages_py",
        "//talOS/python:talos",
    ],
)
