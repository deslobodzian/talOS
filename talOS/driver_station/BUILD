load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# The Driver Station wire codec (the bridge's packet payload).
cc_library(
    name = "driver_station",
    srcs = ["driver_station.cc"],
    hdrs = ["driver_station.h"],
    deps = [":little_endian"],
)

# The bounded little-endian payload cursors this codec and the hardware
# Command and State codecs (//talOS/hardware:codecs) derive theirs from.
# Header only, so the RoboRIO program's GradleRIO build compiles it too.
cc_library(
    name = "little_endian",
    hdrs = ["little_endian.h"],
    visibility = ["//talOS/hardware:__pkg__"],
)

# The simulators' scripted mode and joystick schedules. Header only, so the
# RoboRIO program's GradleRIO build compiles it too.
cc_library(
    name = "schedule",
    hdrs = ["schedule.h"],
    deps = [":driver_station"],
)

cc_test(
    name = "schedule_test",
    size = "small",
    srcs = ["schedule_test.cc"],
    deps = [
        ":schedule",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "driver_station_test",
    size = "small",
    srcs = ["driver_station_test.cc"],
    deps = [
        ":driver_station",
        "@googletest//:gtest_main",
    ],
)

# The Driver Station packet's bytes of the current hardware protocol: an
# encoding change fails it until kProtocolVersion is bumped (REQ-OPS-51).
cc_test(
    name = "wire_golden_test",
    size = "small",
    srcs = ["wire_golden_test.cc"],
    deps = [
        ":driver_station",
        "//talOS/protocol",
        "@googletest//:gtest_main",
    ],
)

cc_binary(
    name = "send_joystick",
    srcs = ["send_joystick.cc"],
    deps = [
        ":driver_station",
        "//talOS/events:events_os",
        "//talOS/hardware",
        "//talOS/ipc",
        "//talOS/process",
    ],
)

cc_test(
    name = "codec_fuzz_test",
    size = "small",
    srcs = ["codec_fuzz_test.cc"],
    deps = [
        ":driver_station",
        "//talOS/protocol:fuzz_corpus",
        "@googletest//:gtest_main",
    ],
)
