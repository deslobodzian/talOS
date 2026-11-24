load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
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
    "workspace_status.sh",
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
    # timeout; plain it takes 0.2 s. The resolver starts no
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
# robot runs there with no alert, reject or fault. Over 5 s, e2e lane: once
# each session is up, the test runs it for `Next:`'s 8 s mode and joystick
# schedule and 3 s more in real time (the gateway has no
# faster-than-real-time mode), timed from the schedules' start.
py_test(
    name = "scaffold_sim_test",
    size = "medium",
    timeout = "short",
    srcs = [
        "new_robot.py",
        "new_robot_test.py",
        "new_subsystem.py",
        "scaffold_sim_test.py",
        # Its wait for a session to be up, and its ordered stop.
        "//talOS/build:sim_check.py",
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
    # Launches six instrumented processes per session (two fork()s + exec per
    # spawn); under macOS TSan on a 3-vCPU hosted runner start-up takes ~1 s
    # per member and overruns the 15 s ready wait (TODO.md CI-13). The Linux
    # sanitizer legs still run this test.
    tags = [
        "e2e",
        "exclusive",
        "no_macos_sanitizer",
    ],
    deps = [
        ":sync_manifest_lib",
        "//talOS/memory:shm_name",
        "@rules_python//python/runfiles",
    ],
)

# A scaffolded node's binary replays its own logged simulated run without
# divergence: one target per language, so the generated node tests spawn no
# binary.
py_test(
    name = "scaffold_replay_cc_test",
    size = "small",
    srcs = ["scaffold_replay_test.py"],
    data = [
        "//tools/scaffold_testdata/gripper:golden",
        "//tools/scaffold_testdata/gripper:node",
    ],
    env = {"NODE_BINARY": "$(rlocationpath //tools/scaffold_testdata/gripper:node)"},
    legacy_create_init = False,
    main = "scaffold_replay_test.py",
    deps = ["@rules_python//python/runfiles"],
)

py_test(
    name = "scaffold_replay_py_test",
    size = "small",
    srcs = ["scaffold_replay_test.py"],
    data = [
        "//tools/scaffold_testdata/scaffold_py:golden",
        "//tools/scaffold_testdata/scaffold_py:node",
    ],
    env = {"NODE_BINARY": "$(rlocationpath //tools/scaffold_testdata/scaffold_py:node)"},
    legacy_create_init = False,
    main = "scaffold_replay_test.py",
    # The node's ctypes host cannot initialize sanitizer runtimes.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = ["@rules_python//python/runfiles"],
)

# Shared unittest sharding loader for sharded py_test targets under tools/.
py_library(
    name = "sharded_test_loader",
    srcs = ["sharded_test_loader.py"],
)

# ./sim.sh --config: which file a path names, and its refusals (missing,
# outside the repository, ambiguous, stale manifest), with a stub bazel and
# resolver so nothing builds or starts. Sharded across 4 actions to stay well
# under the fast lane's 5 s budget on CI.
py_test(
    name = "sim_sh_test",
    size = "small",
    srcs = ["sim_sh_test.py"],
    data = ["//:sim.sh"],
    legacy_create_init = False,
    shard_count = 4,
    deps = [
        ":sharded_test_loader",
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

# The `bazel query --keep_going` exit-3 classifier check_layers.py and
# slow_tests.py share.
py_library(
    name = "bazel_query",
    srcs = ["bazel_query.py"],
)

# python3 tools/check_layers.py (CI: the Linux job, after the build) and
# python3 tools/check_layers.py --source-only (tools/source_lints.sh)
# Checks the Bazel graph against ARCHITECTURE.md's layers with bazel query,
# and scans C++ node sources outside talOS/ for RULES.md 2 bypasses.
py_test(
    name = "check_layers_test",
    size = "small",
    srcs = [
        "check_layers.py",
        "check_layers_test.py",
    ],
    legacy_create_init = False,
    main = "check_layers_test.py",
    deps = [":bazel_query"],
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

# python3 tools/check_testing_rules.py (CI: the lint job)
# Enforces docs/TESTING.md §6.7 (perf targets depend unconditionally on
# //talOS/process:host_qualification, use StallMeter, and record
# host_unqualified/[HOST_UNQUALIFIED]) and §8.2 (e2e/soak targets and
# talos_sim_test preserve failure artifacts under TEST_UNDECLARED_OUTPUTS_DIR).
py_test(
    name = "check_testing_rules_test",
    size = "small",
    srcs = [
        "check_testing_rules.py",
        "check_testing_rules_test.py",
    ],
    legacy_create_init = False,
    main = "check_testing_rules_test.py",
)

# tools/format.sh --check (tools/prepush.sh; CI: the clang-format job)
# lists the C/C++ files changed since the base, untracked ones included. The
# test runs it on a throwaway repository with a stub clang-format.
py_test(
    name = "format_test",
    size = "small",
    srcs = ["format_test.py"],
    data = [
        "clang18.sh",
        "format.sh",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
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

# python3 tools/check_deps_listed.py (CI: the lint job)
# docs/METHODOLOGY.md §13's "Bazel module" table lists exactly MODULE.bazel's
# bazel_deps.
py_test(
    name = "check_deps_listed_test",
    size = "small",
    srcs = [
        "check_deps_listed.py",
        "check_deps_listed_test.py",
    ],
    legacy_create_init = False,
    main = "check_deps_listed_test.py",
)

# bazel run //tools:data_flow_sizes
# Prints the sizes docs/DATA_FLOW.md states (frames, payloads, device caps,
# shared-memory slots, log records, telemetry budgets) as `name value`, from
# the headers that define and assert them.
cc_binary(
    name = "data_flow_sizes",
    srcs = ["data_flow_sizes.cc"],
    deps = [
        "//talOS/bridge:messages",
        "//talOS/driver_station",
        "//talOS/events:events_log",
        "//talOS/hardware",
        "//talOS/introspection:registry",
        "//talOS/msgs:geometry",
        "//talOS/protocol",
        "//talOS/rtms",
        "//talOS/telemetry:wire",
    ],
)

# RULES.md 9: every `<!-- size: name -->` number in docs/DATA_FLOW.md equals
# what :data_flow_sizes prints, and every printed size has a marker. Reads
# docs/DATA_FLOW.md from the source tree beside this file (the root package
# exports no docs), which is not a Bazel input, so it is never cached
# (`external`); well under a second.
py_test(
    name = "check_data_flow_sizes_test",
    size = "small",
    srcs = [
        "check_data_flow_sizes.py",
        "check_data_flow_sizes_test.py",
    ],
    data = [":data_flow_sizes"],
    env = {"DATA_FLOW_SIZES": "$(rootpath :data_flow_sizes)"},
    legacy_create_init = False,
    main = "check_data_flow_sizes_test.py",
    tags = ["external"],
)

# python3 tools/check_commissioning.py --base <rev> (CI: the lint job)
# RULES.md 4: no commit since the merge base sets `commissioned = true` or
# changes a CAN id in a robot or bench config without the person's
# `Commissioned-By: <name>` trailer.
# Fast-lane unit tests (no git subprocesses) for REQ-DEV-33.
py_test(
    name = "check_commissioning_unit_test",
    size = "small",
    srcs = [
        "check_commissioning.py",
        "check_commissioning_unit_test.py",
    ],
    legacy_create_init = False,
    main = "check_commissioning_unit_test.py",
)

# Over 5 s on 4-vCPU CI (6.8 s), integration lane: spawns dozens of real git
# subprocesses (init, add, commit, diff-tree, merge) on scratch repositories.
py_test(
    name = "check_commissioning_test",
    size = "small",
    srcs = [
        "check_commissioning.py",
        "check_commissioning_test.py",
    ],
    legacy_create_init = False,
    main = "check_commissioning_test.py",
    tags = ["integration"],
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
    deps = [":bazel_query"],
)

# tools/prepush.sh [--ref C] [--keep] [--full] [--skip-gradle] [--skip-studio]
# (docs/DEVELOPMENT.md §5 rung 3) RULES.md 11's pre-push checks on a clean,
# read-only worktree of the commit; tools/make_sources_read_only.py is CI's
# read-only step, and tools/check_host_includes.py runs after the build. The
# test runs it with a stub bazel against a throwaway repository.
# Over 5 s: spawns temporary git repositories and full prepush.sh worktree
# checks (sharded across 4 actions to stay well under the 60 s small timeout
# on 3-vCPU macOS CI); integration lane.
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
    shard_count = 4,
    tags = ["integration"],
    deps = [
        ":sharded_test_loader",
        "@rules_python//python/runfiles",
    ],
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

# REQ-DEV-29: talOS fetches patched toml++ through a module-visible
# http_archive (not a root-only override), dependent repositories
# (examples/robot_repo, examples/wrist_repo, and standalone scaffolds) carry
# no override or patch file for it, and CI/prepush run
# @talos//talOS/configuration:config_fuzz_test in both dependents. Reads
# source-tree files (.bazelignore hides examples/*_repo from Bazel labels),
# so it is never cached (`external`); well under a second.
py_test(
    name = "dependent_toml_test",
    size = "small",
    srcs = [
        "dependent_toml_test.py",
        "new_subsystem.py",
    ],
    legacy_create_init = False,
    main = "dependent_toml_test.py",
    tags = ["external"],
    deps = [":sync_manifest_lib"],
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

# python3 tools/run_clang_tidy.py BASE (CI and tools/tidy.sh)
# Selects changed C++ sources and includer TUs for changed headers, falling
# back to test TUs when no production TU includes a changed header.
py_test(
    name = "run_clang_tidy_test",
    size = "small",
    srcs = [
        "run_clang_tidy.py",
        "run_clang_tidy_test.py",
    ],
    legacy_create_init = False,
    main = "run_clang_tidy_test.py",
)

# tools/tidy.sh (CI's clang-tidy selection locally, with Linux x86_64 GCC 14
# libstdc++ fallback when clang 18 cannot parse the host standard library).
py_test(
    name = "tidy_test",
    size = "small",
    srcs = ["tidy_test.py"],
    data = [
        "clang18.sh",
        "run_clang_tidy.py",
        "tidy.sh",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# python3 tools/run_rio_clang_tidy.py (CI's rio-adapter job)
# Sanitizes GradleRIO's compile_commands.json and runs clang-tidy-18 over
# controller_processor/rio/src/main/cpp and src/main/include (CI-12).
py_test(
    name = "run_rio_clang_tidy_test",
    size = "small",
    srcs = [
        "run_rio_clang_tidy.py",
        "run_rio_clang_tidy_test.py",
    ],
    legacy_create_init = False,
    main = "run_rio_clang_tidy_test.py",
)

# python3 tools/retry_unacquired.py (.github/workflows/retry-unacquired.yaml)
# Fail-closed retry decider for unacquired hosted macOS runners (CI-11).
py_test(
    name = "retry_unacquired_test",
    size = "small",
    srcs = [
        "retry_unacquired.py",
        "retry_unacquired_test.py",
    ],
    legacy_create_init = False,
    main = "retry_unacquired_test.py",
)

# python3 tools/check_runfiles_collisions.py (CI: the Linux job, after the build)
# Fails on empty/real runfiles collisions, generated _main/**/__init__.py, or
# zero manifests found.
py_test(
    name = "check_runfiles_collisions_test",
    size = "small",
    srcs = [
        "check_runfiles_collisions.py",
        "check_runfiles_collisions_test.py",
    ],
    legacy_create_init = False,
    main = "check_runfiles_collisions_test.py",
)

# tools/deploy.sh argument validation, prefix checks, and rollback safety
# (REQ-OPS-50, REQ-OPS-54) with stub bundles.
py_test(
    name = "deploy_test",
    size = "small",
    srcs = ["deploy_test.py"],
    data = ["deploy.sh"],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# tools/setup.sh --check (REQ-DEV-12): syntax check and component detection
# (passing, missing, wrong-version, --no-studio/--no-rio/--no-lint) with a
# stubbed PATH so nothing installs or depends on host tools.
py_test(
    name = "setup_test",
    size = "small",
    srcs = ["setup_test.py"],
    data = [
        "clang18.sh",
        "setup.sh",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)
