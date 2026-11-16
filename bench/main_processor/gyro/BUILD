load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

talos_messages(
    name = "messages",
    srcs = ["gyro_message.fbs"],
    topics = {"state": "/gyro/state"},
)

# The bench Pigeon 2 observer, in Python only: it runs the Python devices
# layer's Gyro (ctypes over //talOS/node_api) against the bench hardware.
talos_py_node(
    name = "node",
    class_name = "BenchGyro",
    main = "gyro.py",
    node_name = "gyro",
    deps = [":messages_py"],
)

py_test(
    name = "node_py_test",
    size = "small",
    srcs = ["node_py_test.py"],
    data = [
        "gyro.py",
        "//bench/main_processor/configuration:robot.toml",
        "//bench/main_processor/configuration:robot_config",
    ],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath //bench/main_processor/configuration:robot.toml)"},
    legacy_create_init = False,
    # ctypes cannot initialize sanitizer runtimes inside the Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
