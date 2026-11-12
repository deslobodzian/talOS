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
        "//talOS/configuration:config_parser.h",
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

# bazel run //tools:new_robot -- <name> --mechanism M --lang cpp|py [--dir DIR]
# Writes a robot beside 2026-robot/: its configuration (teleop, telemetry),
# generated manifest, graph test and bundle, and its first mechanism through
# new_subsystem.
py_binary(
    name = "new_robot",
    srcs = [
        "new_robot.py",
        "new_subsystem.py",
    ],
    deps = [":sync_manifest_lib"],
)

py_test(
    name = "new_robot_test",
    size = "small",
    srcs = [
        "new_robot.py",
        "new_robot_test.py",
        "new_subsystem.py",
    ],
    data = [
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/telemetry:node",
        "//talOS/teleop:node",
        # new_subsystem's C++ velocity and position mechanisms, built.
        "//tools/scaffold_testdata/gripper:node",
        "//tools/scaffold_testdata/position_cpp:node",
    ],
    legacy_create_init = False,
    main = "new_robot_test.py",
    deps = [
        ":sync_manifest_lib",
        "@rules_python//python/runfiles",
    ],
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

# python3 tools/check_layers.py (CI: the Linux job, after the build)
# Checks the Bazel graph against ARCHITECTURE.md's layers with bazel query.
py_test(
    name = "check_layers_test",
    size = "small",
    srcs = [
        "check_layers.py",
        "check_layers_test.py",
    ],
    legacy_create_init = False,
    main = "check_layers_test.py",
)

# python3 tools/lint_test_time.py (CI: the lint job)
# No wall clock in tests or node code, no test retries; exceptions with their
# reasons in lint_test_time_allowlist.txt.
py_test(
    name = "lint_test_time_test",
    size = "small",
    srcs = [
        "lint_test_time.py",
        "lint_test_time_test.py",
    ],
    legacy_create_init = False,
    main = "lint_test_time_test.py",
)

# python3 tools/check_links.py (CI: the lint job)
# Every relative markdown link and #anchor in the tracked files resolves.
py_test(
    name = "check_links_test",
    size = "small",
    srcs = [
        "check_links.py",
        "check_links_test.py",
    ],
    legacy_create_init = False,
    main = "check_links_test.py",
)

# python3 tools/trace.py --check (CI: the lint job)
# Every SAFE, CONTRACT and P0 requirement has a tagged test or a cited
# verification; gaps are listed with their TODO item in trace_waivers.txt.
py_test(
    name = "trace_test",
    size = "small",
    srcs = [
        "trace.py",
        "trace_test.py",
    ],
    legacy_create_init = False,
    main = "trace_test.py",
)

# python3 tools/slow_tests.py [--threshold 5] [--check] (not a CI check yet)
# Lists the tests over a time budget, slowest first, from bazel-testlogs.
py_test(
    name = "slow_tests_test",
    size = "small",
    srcs = [
        "slow_tests.py",
        "slow_tests_test.py",
    ],
    legacy_create_init = False,
    main = "slow_tests_test.py",
)
