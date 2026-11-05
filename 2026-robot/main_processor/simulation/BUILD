load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "sim_gateway",
    srcs = ["sim_gateway.cc"],
    deps = [
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/process",
    ],
)

# Startup and scripted Driver Station contract, including UDP mode generations.
py_test(
    name = "startup_test",
    srcs = ["startup_test.py"],
    data = [":sim_gateway"],
    tags = ["exclusive"],
)
