load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "flywheel_example",
    srcs = ["flywheel_example.py"],
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = ["//2026-robot/main_processor/intake:intake_py"],
)

py_test(
    name = "flywheel_example_test",
    size = "small",
    srcs = ["flywheel_example_test.py"],
    legacy_create_init = False,
    deps = [
        ":flywheel_example",
        "//2026-robot/main_processor/intake:intake_py",
    ],
)

exports_files(["subsystem.toml"])
