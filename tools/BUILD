load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

# manifest_sync_test (sync_manifest.bzl) runs the script as its main.
exports_files(["sync_manifest.py"])

# bazel run //tools:new_subsystem -- <name> --lang cpp|py [--robot R] [--standalone DIR] [--dry-run]
# Writes a node package under <robot>/main_processor/<name>/, adds it to that
# robot's robot.toml and regenerates its manifest, or scaffolds a standalone
# Bzlmod repo.
py_binary(
    name = "new_subsystem",
    srcs = ["new_subsystem.py"],
    deps = [":sync_manifest_lib"],
)

py_test(
    name = "new_subsystem_test",
    size = "small",
    srcs = [
        "new_subsystem.py",
        "new_subsystem_test.py",
    ],
    data = [
        "//tools/scaffold_testdata/gripper:golden",
        "//tools/scaffold_testdata/localizer:golden",
        "//tools/scaffold_testdata/py_localizer:golden",
        "//tools/scaffold_testdata/scaffold_py:golden",
    ],
    legacy_create_init = False,
    main = "new_subsystem_test.py",
    deps = [":sync_manifest_lib"],
)

# bazel run //tools:sync_manifest -- [--check] <robot.toml>...
# Writes <stem>_manifest.bzl (NODES, CONFIG_FILES) beside each robot config.
py_binary(
    name = "sync_manifest",
    srcs = ["sync_manifest.py"],
)

py_library(
    name = "sync_manifest_lib",
    srcs = ["sync_manifest.py"],
)

py_test(
    name = "sync_manifest_test",
    size = "small",
    srcs = ["sync_manifest_test.py"],
    data = ["//talOS/configuration:resolution_cases"],
    legacy_create_init = False,
    deps = [":sync_manifest_lib"],
)
