load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "sim_gateway",
    srcs = ["sim_gateway.cc"],
    deps = [
        "//talOS/configuration:configuration",
        "//talOS/hardware:hardware",
        "//talOS/introspection:describe",
        "//talOS/process:process",
    ],
)

# Fail-fast startup contract: no --config must exit nonzero (an unconfigured
# gateway stamps config_id 0 and the graph runs dead), --describe must
# answer. Hermetic: neither path touches sockets or shared memory.
py_test(
    name = "startup_test",
    srcs = ["startup_test.py"],
    data = [":sim_gateway"],
)
