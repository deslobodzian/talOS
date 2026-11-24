load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

package(default_visibility = ["//visibility:public"])

# Shared geometry and control message contracts (ROS *_msgs style), so
# subsystems, target producers, odometry, and operator interfaces can
# interoperate without depending on each other's packages or repos.
talos_messages(
    name = "geometry",
    srcs = ["geometry.fbs"],
    include_prefix = "geometry",
)

talos_messages(
    name = "control",
    srcs = ["control.fbs"],
    include_prefix = "control",
)

# Compile-time checks: building and running this test verifies every shared
# message's sizeof/alignof and talos::Message contract (layout_asserts.cc).
cc_test(
    name = "layout_asserts_test",
    size = "small",
    srcs = ["layout_asserts.cc"],
    deps = [
        ":control",
        ":geometry",
        "//talOS/node",
    ],
)
