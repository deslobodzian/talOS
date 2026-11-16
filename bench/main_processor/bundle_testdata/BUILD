load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_shared_library.bzl", "cc_shared_library")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_py_node")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES")

package(default_visibility = ["//bench/main_processor:__pkg__"])

exports_files([
    "robot.toml",
    "mechanisms/linked_library.toml",
    "mechanisms/motor.toml",
    "mechanisms/native_wheel.toml",
])

# bundle_test only: a Python node importing native wheels (numpy, and the
# gtsam and robotpy-wpimath that talOS pins for robot repositories and no
# other talOS target loads), so relocation_bundle carries wheels' extension
# modules and the shared libraries they bundle. testonly: no robot bundle
# can ship it, and check_node_matrix skips it as a fixture.
talos_py_node(
    name = "native_wheel",
    testonly = True,
    class_name = "NativeWheel",
    main = "native_wheel.py",
    node_name = "native_wheel",
    deps = [
        "@pypi//gtsam",
        "@pypi//numpy",
        "@pypi//robotpy_wpimath",
    ],
)

# bundle_test only: a C++ node whose binary links a shared library
# dynamically, so relocation_bundle stages that library in lib/ and the
# installed node must load it from there (talos_cc_node's rpath); the bench
# robot's C++ nodes link everything statically. testonly, as native_wheel.
cc_library(
    name = "fixture_library_objects",
    testonly = True,
    srcs = ["fixture_library.cc"],
    hdrs = ["fixture_library.h"],
)

cc_shared_library(
    name = "libfixture_library.so",
    testonly = True,
    features = ["set_soname"],
    shared_lib_name = "libfixture_library.so",
    deps = [":fixture_library_objects"],
)

talos_cc_node(
    name = "linked_library",
    testonly = True,
    srcs = ["linked_library.cc"],
    dynamic_deps = [":libfixture_library.so"],
    node_name = "linked_library",
    deps = [":fixture_library_objects"],
)

manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)
