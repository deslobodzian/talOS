load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

# manifest_sync_test (sync_manifest.bzl) runs the script as its main;
# //2026-robot/main_processor:bundle_test installs bundles with deploy.sh.
exports_files([
    "deploy.sh",
    "sync_manifest.py",
])

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
        "//talOS/introspection:names.h",
        "//tools/scaffold_testdata/gripper:golden",
        "//tools/scaffold_testdata/localizer:golden",
        "//tools/scaffold_testdata/position_cpp:golden",
        "//tools/scaffold_testdata/position_py:golden",
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
    data = ["//talOS/configuration:resolve"],
    visibility = ["//talOS/build:__pkg__"],
    deps = ["@rules_python//python/runfiles"],
)

py_library(
    name = "sync_manifest_lib",
    srcs = ["sync_manifest.py"],
    data = ["//talOS/configuration:resolve"],
    deps = ["@rules_python//python/runfiles"],
)

py_test(
    name = "sync_manifest_test",
    size = "small",
    srcs = ["sync_manifest_test.py"],
    data = ["//talOS/configuration:resolution_cases"],
    legacy_create_init = False,
    # Runs the resolver binary once per case (about 65 runs); under macOS
    # TSan each start costs about a second, past the small timeout. The
    # resolver starts no threads, so TSan finds nothing here; ASan still
    # runs it.
    tags = ["no_tsan"],
    deps = [":sync_manifest_lib"],
)

# bazel run //tools:check_node_matrix -- [PATH...]
# Reports each node test's docs/TESTING.md §7 matrix cases: present, n/a
# with its reason, or missing (exit 1).
py_binary(
    name = "check_node_matrix",
    srcs = ["check_node_matrix.py"],
)

py_test(
    name = "check_node_matrix_test",
    size = "small",
    srcs = [
        "check_node_matrix.py",
        "check_node_matrix_test.py",
        "new_subsystem.py",
    ],
    legacy_create_init = False,
    main = "check_node_matrix_test.py",
    deps = [":sync_manifest_lib"],
)
