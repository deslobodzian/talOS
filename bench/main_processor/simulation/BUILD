load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_test")
load("//bench/main_processor/configuration:robot_manifest.bzl", "CONFIG_FILES", "NODES")
load("//talOS/build:contract.bzl", "talos_graph_test")

SOAK_CONFIG_FILES = CONFIG_FILES + [
    "//bench/main_processor/configuration:robot.toml",
    "testdata/subsystems/auto.toml",
]

SOAK_NODES = NODES + [
    ":auto_producer",
]

# Test-only autonomous target producer for the bench soak graph
# (testdata/robot_soak.toml). Declared as a testonly cc_binary rather than
# talos_cc_node so bench/main_processor/simulation remains a simulation
# harness package (launching the bench's own nodes in soak_test and
# robot_soak_graph_test) rather than a robot subsystem package under
# tools/check_layers.py.
cc_binary(
    name = "auto_producer",
    testonly = True,
    srcs = ["auto_producer.cc"],
    local_defines = [
        'TALOS_NODE_NAME=\\"auto_producer\\"',
        'TALOS_NODE_TARGET=\\"//bench/main_processor/simulation:auto_producer\\"',
    ],
    deps = [
        "//bench/main_processor/motor:messages",
        "//bench/main_processor/swerve_module:messages",
        "//talOS/devices",
        "//talOS/node",
    ],
)

# Validates testdata/robot_soak.toml with launcher --describe-only on every
# build/test so graph or naming regressions fail before the manual soak lane.
talos_graph_test(
    name = "robot_soak_graph_test",
    testonly = True,
    config = "testdata/robot_soak.toml",
    data = SOAK_CONFIG_FILES,
    nodes = SOAK_NODES,
    # Launches a Python node (gyro); ctypes cannot load sanitizer runtimes
    # in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

# Minutes of the bench's simulated soak graph, testdata/robot_soak.toml, with
# two 5 ms C++ actuator owners (swerve_module and motor), auto_producer, and
# the Studio bridge (--start-studio-bridge and the [studio] defaults) under
# SIGKILL/SIGSTOP chaos; see soak_test.py.
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
