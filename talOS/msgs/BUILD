load("@rules_cc//cc:cc_library.bzl", "cc_library")
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

# Compile-time checks: building it is the check (layout_asserts.cc).
cc_library(
    name = "layout_asserts",
    testonly = True,
    srcs = ["layout_asserts.cc"],
    deps = [
        ":control",
        ":geometry",
        "//talOS/node",
    ],
)
