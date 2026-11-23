load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("//bench/main_processor/bundle_testdata:robot_manifest.bzl", RELOCATION_CONFIG_FILES = "CONFIG_FILES", RELOCATION_NODES = "NODES")
load("//bench/main_processor/configuration:robot_bench_manifest.bzl", "CONFIG_FILES", "NODES")
load("//bench/main_processor/configuration:robot_py_manifest.bzl", PY_CONFIG_FILES = "CONFIG_FILES", PY_NODES = "NODES")
load("//talOS/build:contract.bzl", "talos_bundle", "talos_graph_test")

package(default_visibility = ["//visibility:public"])

talos_bundle(
    name = "bench_bundle",
    config = "//bench/main_processor/configuration:robot_bench.toml",
    config_files = CONFIG_FILES,
    nodes = ["//talOS/bridge:hardware_node"] + NODES,
    # Studio for `run.sh --start-studio-bridge` (docs/DEPLOYMENT.md).
    studio_bridge = "//studio/bridge:studio_bridge",
    studio_webroot = "//studio:web_dist",
)

# bundle_test only: the Python bench robot (robot_py.toml), two Python nodes
# (motor_py, gyro) sharing one bundled interpreter, without Studio.
talos_bundle(
    name = "bench_py_bundle",
    testonly = True,
    config = "//bench/main_processor/configuration:robot_py.toml",
    config_files = PY_CONFIG_FILES,
    nodes = ["//talOS/bridge:hardware_node"] + PY_NODES,
)

# bundle_test only: a config reaching its files through `include`,
# parent-directory `defaults`, a custom `path` and an entry `node`.
talos_bundle(
    name = "relocation_bundle",
    testonly = True,
    config = "//bench/main_processor/bundle_testdata:robot.toml",
    config_files = RELOCATION_CONFIG_FILES,
    nodes = ["//talOS/bridge:hardware_node"] + RELOCATION_NODES,
)

# relocation_bundle's config, described and linted like every robot
# config's: a graph error in it would otherwise surface only as an install
# failure inside bundle_test.
talos_graph_test(
    name = "relocation_graph_test",
    config = "//bench/main_processor/bundle_testdata:robot.toml",
    data = RELOCATION_CONFIG_FILES,
    nodes = RELOCATION_NODES,
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

# bundle_test only: prints the launcher's stop timeout in seconds.
cc_binary(
    name = "bundle_test_stop_timeout",
    testonly = True,
    srcs = ["bundle_test_stop_timeout.cc"],
    deps = ["//talOS/launcher:launcher_lib"],
)

# warm_up_test only: a native library whose load leaves a mark (the
# warm-up's macOS native-library pass). Never instrumented: deploy.sh loads
# it into an uninstrumented python3, and on macOS a sanitizer's runtime is a
# dylib that aborts ("Interceptors are not working") when dlopen'd after
# start, so under --config=asan or tsan the load would kill the warm-up.
# -fno-sanitize=all leaves it no sanitizer call; the global --linkopt still
# names the runtime after this rule's linkopts, so ld64 drops the dylibs
# nothing references.
cc_binary(
    name = "bundle_test_native_mark.so",
    testonly = True,
    srcs = ["bundle_test_native_mark.cc"],
    copts = ["-fno-sanitize=all"],
    linkopts = select({
        "@platforms//os:macos": ["-Wl,-dead_strip_dylibs"],
        "//conditions:default": [],
    }),
    linkshared = True,
)

# The deploy.sh that bundle_test, install_check_test and warm_up_test
# install with.
#
# Each of the three carries "cpu:7", more than half of a 12-CPU Mac mini or
# an 8-core workstation (a smaller host runs it when it has no other test):
# Bazel then never runs two of them at once, nor much else beside one.
# Every install copies a bundle as new files, and macOS charges each new
# file's first exec about 100 ms one file at a time host-wide (MAC-2): three
# installers at once under a full suite (load about 9) took over 60 s where
# each alone takes under 30 s.
py_library(
    name = "deploy_test_support",
    testonly = True,
    srcs = ["deploy_test_support.py"],
    data = ["//tools:deploy.sh"],
    deps = ["@rules_python//python/runfiles"],
)

# Over 5 s: three real installs, each paying deploy.sh's warm-up and graph
# check for every member (on macOS, a new file's first exec and native
# library load: about 27 s there, docs/DEPLOYMENT.md "Warm-up"; 5.4-12.2 s on
# Linux CI); integration lane.
py_test(
    name = "bundle_test",
    size = "small",
    srcs = ["bundle_test.py"],
    data = [
        ":bench_bundle",
        ":bench_py_bundle",
        ":bundle_test_stop_timeout",
        ":relocation_bundle",
    ],
    legacy_create_init = False,
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "cpu:7",
        "integration",
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":deploy_test_support",
        "@rules_python//python/runfiles",
    ],
)

# Over 5 s: three real installs of bench_bundle, each paying deploy.sh's
# graph check without its warm-up (about 12 s on macOS, 5.4-12.2 s on Linux
# CI); integration lane.
py_test(
    name = "install_check_test",
    size = "small",
    srcs = ["install_check_test.py"],
    data = [":bench_bundle"],
    legacy_create_init = False,
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "cpu:7",
        "integration",
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":deploy_test_support",
        "@rules_python//python/runfiles",
    ],
)

# Over 5 s: installs and rollbacks of stub bundles (a stop during the graph
# check and rollback's graph check among them), one of which waits out a
# 3 s warm-up bound for a member that never exits (about 9.5 s on macOS,
# 5.4-12.2 s on Linux CI); integration lane. Runs only stubs and deploy.sh,
# no node: no sanitizer tag.
py_test(
    name = "warm_up_test",
    size = "small",
    srcs = ["warm_up_test.py"],
    data = [":bundle_test_native_mark.so"],
    legacy_create_init = False,
    tags = [
        "cpu:7",
        "integration",
    ],
    deps = [
        ":deploy_test_support",
        "@rules_python//python/runfiles",
    ],
)

# REQ-RT-07: update() callback entry-to-exit p99 wall-clock benchmark for
# representative C++ bench and framework nodes (motor, swerve_module, teleop,
# telemetry) against docs/METHODOLOGY.md §10's 10% period budget, with
# StallMeter host qualification (docs/TESTING.md §6.7).
cc_test(
    name = "update_p99_perf_test",
    size = "small",
    srcs = ["update_p99_perf_test.cc"],
    data = ["//bench/main_processor/configuration:robot_config"],
    tags = [
        "exclusive",
        "perf",
    ],
    deps = [
        "//bench/main_processor/motor",
        "//bench/main_processor/swerve_module",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/node",
        "//talOS/node:testing",
        "//talOS/process:host_qualification",
        "//talOS/telemetry:testing",
        "//talOS/teleop:testing",
        "@googletest//:gtest_main",
    ],
)
