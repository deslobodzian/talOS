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

CPP_SIM_CONFIG_FILES = [
    "//bench/main_processor/configuration:subsystems/motor.toml",
    "//bench/main_processor/configuration:subsystems/swerve_module.toml",
    "//bench/main_processor/configuration:subsystems/telemetry.toml",
    "//bench/main_processor/configuration:subsystems/teleop.toml",
]

CPP_SIM_NODES = [
    "//bench/main_processor/motor:node",
    "//bench/main_processor/swerve_module:node",
    "//talOS/telemetry:node",
    "//talOS/teleop:node",
]

# Validates testdata/robot_cpp_sim.toml with launcher --describe-only under
# plain, ASan, and TSan (every member is a C++ binary, no Python ctypes load).
talos_graph_test(
    name = "robot_cpp_sim_graph_test",
    testonly = True,
    config = "testdata/robot_cpp_sim.toml",
    data = CPP_SIM_CONFIG_FILES,
    nodes = CPP_SIM_NODES,
    # Spawns five instrumented C++ nodes for --describe; macOS TSan adds ~100x
    # fork/exec overhead on 3-vCPU hosted runners (TODO.md CI-13). Linux
    # sanitizer lanes run this test.
    tags = ["no_macos_sanitizer"],
)

# e2e (REQ-OPS-30, REQ-SAFE-31): pure-C++ full-graph simulation
# (launcher + hardware_node + sim_gateway --physics + studio_bridge +
# swerve_module + motor + teleop + telemetry) running disabled -> teleop
# driving -> disabled cleanly under plain, ASan, and TSan (TODO.md CI-3).
# Runs ~3.2 s of scripted session plus startup/shutdown; e2e lane.
py_test(
    name = "cpp_full_graph_sim_test",
    size = "medium",
    srcs = [
        "cpp_full_graph_sim_test.py",
        "support.py",
        "//talOS/build:sim_check.py",
    ],
    data = [
        "testdata/robot_cpp_sim.toml",
        "//studio:web_dist",
        "//studio/bridge:studio_bridge",
        "//talOS/bridge:hardware_node",
        "//talOS/build:registry_unlink",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
        "//talOS/launcher:session_report",
        "//talOS/simulation:sim_gateway",
    ] + CPP_SIM_CONFIG_FILES + CPP_SIM_NODES,
    env = {
        "CONFIG": "$(rlocationpath testdata/robot_cpp_sim.toml)",
        "LAUNCHER": "$(rlocationpath //talOS/launcher)",
        "LOG_DUMP": "$(rlocationpath //talOS/launcher:log_dump)",
        "REGISTRY_UNLINK": "$(rlocationpath //talOS/build:registry_unlink)",
    },
    imports = ["."],
    legacy_create_init = False,
    main = "cpp_full_graph_sim_test.py",
    # Launches eight instrumented C++ processes; macOS TSan fork/exec overhead
    # overruns the ready wait on 3-vCPU hosted runners (TODO.md CI-13). Both
    # Linux sanitizer lanes (tools/sanitizer_lane.sh asan|tsan) run this test.
    tags = [
        "e2e",
        "exclusive",
        "no_macos_sanitizer",
    ],
    deps = [
        "//talOS/build:udp_ports",
        "//talOS/launcher:tlog_read",
        "@rules_python//python/runfiles",
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
