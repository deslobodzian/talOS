load("//talOS/build:contract.bzl", "talos_py_node")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES")

package(default_visibility = ["//bench/main_processor:__pkg__"])

exports_files([
    "robot.toml",
    "mechanisms/motor.toml",
    "mechanisms/native_wheel.toml",
])

# bundle_test only: a Python node importing numpy, a native wheel, so
# relocation_bundle carries a wheel's extension modules and the shared
# libraries it bundles.
talos_py_node(
    name = "native_wheel",
    class_name = "NativeWheel",
    main = "native_wheel.py",
    node_name = "native_wheel",
    deps = ["@pypi//numpy"],
)

manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)
