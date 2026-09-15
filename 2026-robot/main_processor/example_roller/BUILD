load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "roller_example",
    srcs = ["roller_example.py"],
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = ["//2026-robot/main_processor/intake:intake_py"],
)

py_test(
    name = "roller_example_test",
    size = "small",
    srcs = ["roller_example_test.py"],
    legacy_create_init = False,
    deps = [
        ":roller_example",
        "//2026-robot/main_processor/intake:intake_py",
    ],
)

exports_files(["subsystem.toml"])
