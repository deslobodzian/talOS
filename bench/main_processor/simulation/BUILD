load("@rules_python//python:defs.bzl", "py_test")
load("//bench/main_processor/configuration:robot_manifest.bzl", "CONFIG_FILES", "NODES")
load("//talOS/build:contract.bzl", "talos_graph_test")

SOAK_CONFIG_FILES = CONFIG_FILES + [
    "//bench/main_processor/configuration:robot.toml",
    "testdata/subsystems/auto.toml",
    "testdata/subsystems/python_motor.toml",
]

SOAK_NODES = NODES + [
    "//bench/main_processor/simulation/testdata/nodes:auto_producer",
    "//bench/main_processor/simulation/testdata/nodes:python_motor",
]

# Validates testdata/robot_soak.toml with launcher --describe-only on every
# build/test so graph or naming regressions fail before the manual soak lane.
talos_graph_test(
    name = "robot_soak_graph_test",
    testonly = True,
    config = "testdata/robot_soak.toml",
    data = SOAK_CONFIG_FILES,
    nodes = SOAK_NODES,
    # Launches Python nodes (gyro, python_motor); ctypes cannot load sanitizer
    # runtimes in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

# Minutes of the bench's simulated soak graph, testdata/robot_soak.toml, with
# two 5 ms C++ actuator owners (swerve_module and motor), a 20 ms Python
# actuator owner (python_motor), auto_producer, and the Studio bridge
# (--start-studio-bridge and the [studio] defaults) under SIGKILL/SIGSTOP
# chaos; see soak_test.py.
# Manual: `bazel test //...` skips it, and the CI soak job runs it on its own
# (TALOS_SOAK_SECONDS sets the enabled soak length).
py_test(
    name = "soak_test",
    size = "large",
    timeout = "long",
    srcs = [
        "soak_test.py",
        "support.py",
    ],
    data = [
        "testdata/robot_soak.toml",
        # The bridge refuses to start without its webroot, the [studio]
        # default bazel-bin/studio/dist.
        "//studio:web_dist",
        "//studio/bridge:studio_bridge",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
        "//talOS/simulation:sim_gateway",
    ] + SOAK_CONFIG_FILES + SOAK_NODES,
    imports = ["."],
    # Launches a Python node (gyro); ctypes cannot load sanitizer runtimes
    # in Python.
    tags = [
        "exclusive",
        "manual",
        "no_asan",
        "no_tsan",
    ],
    deps = [
        "//talOS/build:udp_ports",
        "//talOS/launcher:tlog_read",
    ],
)

py_test(
    name = "soak_sigterm_test",
    size = "small",
    srcs = [
        "soak_sigterm_test.py",
        "soak_test.py",
        "support.py",
    ],
    imports = ["."],
    main = "soak_sigterm_test.py",
    deps = [
        "//talOS/build:udp_ports",
        "//talOS/launcher:tlog_read",
    ],
)
