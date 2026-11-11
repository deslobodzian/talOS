load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_cc_shared_node", "talos_messages", "talos_py_node")

talos_messages(
    name = "messages",
    srcs = ["motor_message.fbs"],
    topics = {"state": "/motor/state"},
)

cc_library(
    name = "motor",
    hdrs = ["motor.h"],
    include_prefix = "motor",
    strip_include_prefix = ".",
    deps = [
        ":messages",
        "//talOS/bridge:messages",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["motor.cc"],
    node_name = "motor",
    deps = [":motor"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//bench/main_processor/configuration:robot_config"],
    deps = [
        ":motor",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "//talOS/teleop",
        "@googletest//:gtest_main",
    ],
)

# The same node in Python (motor.py), which
# //bench/main_processor/configuration:robot_py.toml launches.
talos_py_node(
    name = "node_py",
    class_name = "BenchMotor",
    main = "motor.py",
    node_name = "motor",
    deps = [
        ":messages_py",
        "//talOS/bridge:messages_py",
    ],
)

# The C++ node for talos.cpp_node, so node_py_test runs both nodes on one
# input script and compares what they publish and request.
talos_cc_shared_node(
    name = "libbench_motor.so",
    testonly = True,
    srcs = ["motor_abi.cc"],
    deps = [":motor"],
)

py_test(
    name = "node_py_test",
    size = "small",
    srcs = ["node_py_test.py"],
    data = [
        "motor.py",
        ":libbench_motor.so",
        "//bench/main_processor/configuration:robot_py.toml",
        "//bench/main_processor/configuration:robot_py_config",
    ],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath //bench/main_processor/configuration:robot_py.toml)"},
    legacy_create_init = False,
    # ctypes cannot initialize sanitizer runtimes inside the Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":messages_py",
        "//talOS/bridge:messages_py",
        "//talOS/python:teleop",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
