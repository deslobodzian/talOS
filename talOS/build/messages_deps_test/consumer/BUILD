load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages")

# Consumer messages: embeds deps_geometry.Pose2d through `deps`. The .fbs
# spells the dep as include "deps_geometry/geometry.fbs" (the dep's include
# prefix, not its package path), so this keeps working when the dep moves to
# another repo; only the label in `deps` changes.
talos_messages(
    name = "messages",
    srcs = ["consumer.fbs"],
    include_prefix = "deps_consumer",
    topics = {
        "state": "/deps_consumer/state",
    },
    deps = ["//talOS/build/messages_deps_test/geometry:messages"],
)

cc_test(
    name = "messages_cc_test",
    size = "small",
    srcs = ["messages_test.cc"],
    deps = [
        ":messages",
        "//talOS/build/messages_deps_test/geometry:messages",
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
        "//talOS/build/messages_deps_test/geometry:messages_py",
        "//talOS/python:talos",
    ],
)
