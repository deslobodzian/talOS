load("@rules_python//python:defs.bzl", "py_test")
load("//bench/main_processor/configuration:robot_physics_sim_manifest.bzl", "CONFIG_FILES", "NODES")

# Minutes of the bench's simulated graph, robot_physics_sim.toml, with the
# Studio bridge (--start-studio-bridge and the [studio] defaults) under
# SIGKILL/SIGSTOP chaos; see soak_test.py. Its graph test and manifest test
# are robot_physics_sim.toml's (//bench/main_processor/configuration).
# Manual: `bazel test //...` skips it, and the CI soak job runs it on its own
# (TALOS_SOAK_SECONDS sets the teleop length).
py_test(
    name = "soak_test",
    size = "large",
    timeout = "long",
    srcs = [
        "soak_test.py",
        "support.py",
    ],
    data = [
        "//bench/main_processor/configuration:robot_physics_sim.toml",
        # The bridge refuses to start without its webroot, the [studio]
        # default bazel-bin/studio/dist.
        "//studio:web_dist",
        "//studio/bridge:studio_bridge",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
        "//talOS/simulation:sim_gateway",
    ] + CONFIG_FILES + NODES,
    imports = ["."],
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
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
