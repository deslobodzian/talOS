load("@rules_python//python:defs.bzl", "py_binary")

# sim_check_no_state_test's session (//talOS/build): talOS's telemetry node
# with a simulator that sends its State to the wrong port.
package(default_visibility = ["//talOS/build:__pkg__"])

exports_files(["robot.toml"])

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
