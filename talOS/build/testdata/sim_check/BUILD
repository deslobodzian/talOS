load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_binary")

# sim_check_no_state_test and sim_check_studio_*'s session fixtures
# (//talOS/build).
package(default_visibility = ["//talOS/build:__pkg__"])

exports_files([
    "robot.toml",
    "studio_robot.toml",
])

# Named as the roster names the simulator, so the node-name rule holds.
py_binary(
    name = "sim_gateway",
    testonly = True,
    srcs = ["misaddressed_gateway.py"],
    data = ["//talOS/simulation:sim_gateway"],
    legacy_create_init = False,
    main = "misaddressed_gateway.py",
    deps = ["@rules_python//python/runfiles"],
)

# Named as the roster names Studio's bridge, so the node-name rule holds.
cc_binary(
    name = "studio_bridge",
    testonly = True,
    srcs = ["stub_studio_bridge.cc"],
    deps = [
        "//talOS/configuration",
        "//talOS/events:events_core",
        "//talOS/introspection:describe",
        "//talOS/introspection:registry",
        "//talOS/process",
        "//talOS/process:realtime_outcome",
        "//talOS/schema:identity",
        "//talOS/telemetry:wire",
    ],
)
