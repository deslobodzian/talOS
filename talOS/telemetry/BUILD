load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

package(default_visibility = ["//visibility:private"])

_FRAMEWORK_VISIBILITY = [
    "//studio:__subpackages__",
    "//talOS:__subpackages__",
    "//tools:__pkg__",
]

exports_files(
    ["telemetry.fbs"],
    visibility = ["//visibility:public"],
)

# The Studio feed contract: the telemetry frame FlatBuffers schema plus the
# slot-geometry wire header. Its only dependency is flatbuffers, so robot
# nodes produce telemetry and Studio consumes it without either depending on
# the other.
flatbuffer_cc_library(
    name = "schema",
    srcs = ["telemetry.fbs"],
    visibility = ["//visibility:public"],
)

# The slot geometry is a contract between the bridge and whatever produces
# telemetry, so producers outside the bridge package need it too.
cc_library(
    name = "wire",
    hdrs = ["wire.h"],
    visibility = ["//visibility:public"],
)

# How the dashboard layout and the Studio bridge write a JSON string.
cc_library(
    name = "json_string",
    hdrs = ["json_string.h"],
    visibility = ["//studio/bridge:__pkg__"],
)

# Page encoding: the producer, the Studio bridge and the mock build every
# TLM3 page through it.
cc_library(
    name = "frames",
    hdrs = ["frames.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":schema",
        ":wire",
        "@flatbuffers//:runtime_cc",
    ],
)

cc_test(
    name = "frames_test",
    size = "small",
    srcs = ["tests/frames_test.cc"],
    deps = [
        ":frames",
        ":schema",
        ":wire",
        "@flatbuffers//:runtime_cc",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "wire_test",
    srcs = ["tests/wire_test.cc"],
    deps = [":wire"],
)

# The channel model: a topic's channels from its schema (names by field
# path, kinds, units, enum labels) and each one's value from a message's
# bytes, plus the Driver Station and target-envelope channels. The node
# builds its dictionary with it, and a reader of recorded messages names
# channels the same way.
cc_library(
    name = "channels",
    srcs = ["channels.cc"],
    hdrs = ["channels.h"],
    visibility = _FRAMEWORK_VISIBILITY,
    deps = [
        ":frames",
        ":schema",
        ":wire",
        "//talOS/driver_station",
        "//talOS/hardware:codecs",
        "//talOS/introspection:names",
        "//talOS/schema:field_path",
        "//talOS/schema:identity",
        "//talOS/schema:unit_suffix",
        "@flatbuffers//:runtime_cc",
    ],
)

cc_test(
    name = "channels_test",
    size = "small",
    srcs = ["tests/channels_test.cc"],
    deps = [
        ":channels",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:sim_messages",
        "//talOS/schema:field_path",
        "//talOS/schema:identity",
        "//talOS/schema:test_messages",
        "@flatbuffers//:runtime_cc",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "config_validation",
    srcs = ["config_validation.cc"],
    hdrs = ["config_validation.h"],
    visibility = _FRAMEWORK_VISIBILITY,
    deps = [
        ":channels",
        ":dashboard_layout",
        ":frames",
        ":schema",
        ":wire",
        "//talOS/configuration",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "@tomlplusplus//:toml++",
    ],
)

cc_library(
    name = "telemetry",
    hdrs = ["node.h"],
    visibility = _FRAMEWORK_VISIBILITY,
    deps = [
        ":channels",
        ":config_validation",
        ":dashboard_layout",
        ":frames",
        ":schema",
        ":wire",
        "//talOS/configuration",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/node",
        "//talOS/node:access",
        "//talOS/schema:field_path",
        "//talOS/schema:identity",
        "@flatbuffers//:runtime_cc",
    ],
)

# Test helper for robot/bench Sim tests to add the telemetry node without
# exposing talOS/telemetry/node.h or talOS/node/access.h.
cc_library(
    name = "testing",
    testonly = True,
    srcs = ["testing.cc"],
    hdrs = ["testing.h"],
    implementation_deps = [
        ":telemetry",
        "//talOS/node",
        "//talOS/node:testing",
    ],
    visibility = ["//visibility:public"],
)

talos_cc_node(
    name = "node",
    srcs = ["node.cc"],
    node_name = "telemetry",
    visibility = ["//visibility:public"],
    deps = [":telemetry"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":frames",
        ":telemetry",
        ":wire",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:sim_messages",
        "//talOS/msgs:geometry",
        "//talOS/node:access",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "//talOS/schema:test_messages",
        "@googletest//:gtest_main",
    ],
)

# The driver dashboard's layout ([studio] dashboard): parsed and validated
# without the telemetry dictionary, its channel list and worst-case feed
# bytes, and the JSON the Studio bridge serves as /drive/layout.json.
cc_library(
    name = "dashboard_layout",
    srcs = ["dashboard_layout.cc"],
    hdrs = ["dashboard_layout.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":json_string",
        ":wire",
        "//talOS/introspection:names",
        "@tomlplusplus//:toml++",
    ],
)

cc_test(
    name = "dashboard_layout_test",
    size = "small",
    srcs = ["tests/dashboard_layout_test.cc"],
    deps = [
        ":dashboard_layout",
        ":wire",
        "@googletest//:gtest_main",
    ],
)
