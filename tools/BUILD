load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

# manifest_sync_test (sync_manifest.bzl) runs the script as its main;
# //bench/main_processor:bundle_test, :install_check_test and :warm_up_test
# install bundles with deploy.sh; a robot
# repository's trace_test runs trace.py as its main
# (examples/robot_repo/BUILD.bazel).
exports_files([
    "deploy.sh",
    "sync_manifest.py",
    "trace.py",
])

# bazel run //tools:new_subsystem -- <name> --lang cpp|py [--type T [--raw]] (--robot R | --standalone DIR) [--dry-run]
# Writes a node package under <robot>/main_processor/<name>/, adds it to that
# robot's robot.toml and regenerates its manifest, or scaffolds a standalone
# Bzlmod repo. From a robot repository, `bazel run @talos//tools:new_subsystem
# -- <name> --lang cpp|py [--config PATH] [--package-dir DIR]` does the same
# there, naming talOS by @talos//.
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
        "//tools/scaffold_testdata/position_raw_cpp:golden",
        "//tools/scaffold_testdata/position_raw_py:golden",
        "//tools/scaffold_testdata/py_localizer:golden",
        "//tools/scaffold_testdata/scaffold_py:golden",
    ],
    legacy_create_init = False,
    main = "new_subsystem_test.py",
    # Its only instrumented code is the configuration resolver, which it
    # starts about 70 times (three per scaffold's plan). Under TSan on the
    # hosted macOS runner each start costs most of a second, past the small
    # timeout (TODO.md CI-10); plain it takes 0.2 s. The resolver starts no
    # thread; its library runs under TSan in //talOS/configuration:test and
    # :config_fuzz_test.
    tags = ["no_tsan"],  # ~70 resolver starts; single-threaded; see above
    deps = [":sync_manifest_lib"],
)

# bazel run //tools:new_robot -- <name> --standalone DIR
# Creates DIR, outside this checkout, as a robot repository depending on
# talOS: a copy of examples/robot_repo as new_robot.py's TEMPLATE places it.
# bazel run //tools:new_robot -- <name> --mechanism M --lang cpp|py [--dir DIR]
# Writes a robot directory at the workspace root, like bench/: its
# configuration (teleop, telemetry),
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
        # Its message modules, under the generated Python node.
        "//tools/scaffold_testdata/scaffold_py:node",
    ],
    legacy_create_init = False,
    main = "new_robot_test.py",
    # The launcher runs a Python node, which cannot load the sanitizer
    # runtimes through ctypes.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":sync_manifest_lib",
        "//talOS/python:talos",
        "@rules_python//python/runfiles",
    ],
)

# A scaffolded velocity and position mechanism, edited as their `Next:` says,
# reach their teleop goals under sim_gateway --physics, and a --no-mechanism
# robot runs there with no alert, reject or fault. Over 5 s, e2e lane: the
# launcher runs the graphs for `Next:`'s 8 s mode and joystick schedule and
# 3 s more in real time (the gateway has no faster-than-real-time mode).
py_test(
    name = "scaffold_sim_test",
    size = "medium",
    timeout = "short",
    srcs = [
        "new_robot.py",
        "new_robot_test.py",
        "new_subsystem.py",
        "scaffold_sim_test.py",
    ],
    data = [
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
        "//talOS/simulation:sim_gateway",
        "//talOS/telemetry:node",
        "//talOS/teleop:node",
        "//tools/scaffold_testdata/gripper:node",
        "//tools/scaffold_testdata/position_cpp:node",
    ],
    legacy_create_init = False,
    main = "scaffold_sim_test.py",
    tags = [
        "e2e",
        "exclusive",
    ],
    deps = [
        ":sync_manifest_lib",
        "@rules_python//python/runfiles",
    ],
)

# ./sim.sh --config: which file a path names, and its refusals (missing,
# outside the repository, ambiguous, stale manifest), with a stub bazel and
# resolver so nothing builds or starts.
py_test(
    name = "sim_sh_test",
    size = "small",
    srcs = ["sim_sh_test.py"],
    data = ["//:sim.sh"],
    legacy_create_init = False,
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

# bazel run //tools:check_node_matrix -- [--baseline FILE] [PATH...]
# Reports each node test's docs/TESTING.md §7 matrix cases: present, n/a
# with its reason, covered by a parity run, or missing (exit 1). CI (the lint job) runs it with
# --baseline check_node_matrix_baseline.txt, the missing cases that may only
# shrink.
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

# python3 tools/slow_tests.py [--threshold 5] [--check] (CI: the Linux job,
# after the suite) Lists the tests over a time budget, slowest first, from
# bazel-testlogs or a build event file; --check fails on a lane budget.
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

# tools/prepush.sh [--ref C] [--keep] [--full] [--skip-gradle] [--skip-studio]
# (docs/DEVELOPMENT.md §5 rung 3) RULES.md 11's pre-push checks on a clean,
# read-only worktree of the commit; tools/make_sources_read_only.py is CI's
# read-only step, and tools/check_host_includes.py runs after the build. The
# test runs it with a stub bazel against a throwaway repository.
py_test(
    name = "prepush_test",
    size = "small",
    srcs = ["prepush_test.py"],
    data = [
        "check_host_includes.py",
        "make_sources_read_only.py",
        "prepush.sh",
        "prepush_robot_repository.sh",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# tools/check_robot_repository.sh PATH [BAZEL_FLAG...] (tools/prepush.sh,
# when TALOS_ROBOT_REPOSITORY is set) builds and tests a copy of a robot
# repository's committed tree against this talOS checkout. The test runs it
# with a stub bazel against a throwaway repository.
py_test(
    name = "check_robot_repository_test",
    size = "small",
    srcs = ["check_robot_repository_test.py"],
    data = [
        "check_copy_lock.sh",
        "check_robot_repository.sh",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# tools/prepush.sh sources prepush_robot_repository.sh for whether its
# robot-repository step runs, is skipped or fails (TALOS_ROBOT_REPOSITORY).
py_test(
    name = "prepush_robot_repository_test",
    size = "small",
    srcs = ["prepush_robot_repository_test.py"],
    data = ["prepush_robot_repository.sh"],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# python3 tools/test_rio_in_the_loop.py: its narrowing of the bench config
# to the subject's subsystem (subject_config), pure text in and out.
py_test(
    name = "test_rio_in_the_loop_test",
    size = "small",
    srcs = [
        "test_rio_in_the_loop.py",
        "test_rio_in_the_loop_test.py",
    ],
    legacy_create_init = False,
    main = "test_rio_in_the_loop_test.py",
)

# python3 tools/check_host_includes.py DIR (tools/prepush.sh, after the build)
# No compile read a header from the host's include tree but the C library's,
# the kernel's and the C++ standard library's.
py_test(
    name = "check_host_includes_test",
    size = "small",
    srcs = [
        "check_host_includes.py",
        "check_host_includes_test.py",
    ],
    legacy_create_init = False,
    main = "check_host_includes_test.py",
)
