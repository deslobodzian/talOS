load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_test")
load("//bench/main_processor/bundle_testdata:robot_manifest.bzl", RELOCATION_CONFIG_FILES = "CONFIG_FILES", RELOCATION_NODES = "NODES")
load("//bench/main_processor/configuration:robot_bench_manifest.bzl", "CONFIG_FILES", "NODES")
load("//bench/main_processor/configuration:robot_py_manifest.bzl", PY_CONFIG_FILES = "CONFIG_FILES", PY_NODES = "NODES")
load("//talOS/build:contract.bzl", "talos_bundle")

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

# bundle_test only: prints the launcher's stop timeout in seconds.
cc_binary(
    name = "bundle_test_stop_timeout",
    testonly = True,
    srcs = ["bundle_test_stop_timeout.cc"],
    deps = ["//talOS/launcher:launcher_lib"],
)

# bundle_test only: a native library whose load leaves a mark (the
# warm-up's macOS native-library pass).
cc_binary(
    name = "bundle_test_native_mark.so",
    testonly = True,
    srcs = ["bundle_test_native_mark.cc"],
    linkshared = True,
)

py_test(
    name = "bundle_test",
    size = "small",
    srcs = ["bundle_test.py"],
    data = [
        ":bench_bundle",
        ":bench_py_bundle",
        ":bundle_test_native_mark.so",
        ":bundle_test_stop_timeout",
        ":relocation_bundle",
        "//tools:deploy.sh",
    ],
    legacy_create_init = False,
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = ["@rules_python//python/runfiles"],
)
