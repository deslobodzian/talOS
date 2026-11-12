load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_shared_library.bzl", "cc_shared_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_cc_shared_node")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "internal",
    hdrs = ["internal.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":node_api_h",
        "//talOS/node",
    ],
)

cc_library(
    name = "node_api_h",
    hdrs = ["node_api.h"],
    visibility = ["//visibility:private"],
)

# C ABI over talos::Node, the devices layer and talOS/mechanisms, for nodes
# written in other languages (//talOS/python:talos).
cc_library(
    name = "node_api",
    srcs = [
        "mechanisms_api.cc",
        "node_api.cc",
    ],
    hdrs = ["node_api.h"],
    deps = [
        ":internal",
        "//talOS/devices",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/mechanisms",
        "//talOS/msgs:control",
        "//talOS/node",
    ],
    # Every exported symbol must reach the shared objects below, even though
    # nothing in them references them.
    alwayslink = True,
)

# The Sim / SimHardware section of node_api.h.
cc_library(
    name = "node_api_testing",
    testonly = True,
    srcs = ["node_api_testing.cc"],
    deps = [
        ":internal",
        ":node_api",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:testing",
    ],
    alwayslink = True,
)

# Loaded via ctypes by the `talos` Python package: the process's one copy of
# the framework. Every C++ library hosted in the same process (the testing
# library below, talos_cc_shared_node) links it dynamically instead of
# carrying its own node, devices and transport code; the exports_filter
# claims the closure it links, so such a library cannot link any of it again.
#
# A library in `deps` is linked whole; the rest of the closure only as far as
# node_api reaches into it. So `deps` lists every framework library with
# sources in that closure: a hosted node may call any of their functions
# (Gateway::Reconfigure, say) that node_api never does. A library missing
# here fails the hosted library's link (an undefined symbol), not its load.
# The third-party closure is header-only except @flatbuffers//src:flatbuffers
# (reflection, the schema parser), linked only as far as the framework uses it.
#
# The soname (Linux; macOS sets the install name @rpath/libtalos_node.so
# already) is how the loader matches a hosted library's DT_NEEDED to the image
# Python loaded by path. talos_bundle copies the runfiles, so the library's
# runfiles path and its _solib path become two files; without a soname a
# hosted library would load the second one, a second framework.
cc_shared_library(
    name = "libtalos_node.so",
    exports_filter = [
        "//talOS:__subpackages__",
        "@flatbuffers//:__subpackages__",
        "@tomlplusplus//:__subpackages__",
    ],
    features = ["set_soname"],
    shared_lib_name = "libtalos_node.so",
    deps = [
        ":node_api",
        "//talOS/devices",
        "//talOS/driver_station",
        "//talOS/events:events_log",
        "//talOS/events:events_os",
        "//talOS/events:input_replay",
        "//talOS/hardware",
        "//talOS/hardware:config",
        "//talOS/mechanisms",
        "//talOS/memory:residency",
        "//talOS/node",
        "//talOS/process",
        "//talOS/protocol",
        "//talOS/rtms",
        "//talOS/rtms:process_liveness",
        "//talOS/schema:schema_hash",
    ],
)

cc_shared_library(
    name = "libtalos_node_testing.so",
    testonly = True,
    dynamic_deps = [":libtalos_node.so"],
    features = ["set_soname"],
    shared_lib_name = "libtalos_node_testing.so",
    deps = [":node_api_testing"],
)

cc_test(
    name = "node_api_test",
    size = "small",
    srcs = ["node_api_test.cc"],
    deps = [
        ":node_api_testing",
        "@googletest//:gtest_main",
    ],
)

# An exported C++ node for cpp_node_test.
talos_cc_shared_node(
    name = "libframework_probe.so",
    testonly = True,
    srcs = ["testdata/framework_probe.cc"],
    deps = [
        "//talOS/devices",
        "//talOS/hardware",
    ],
)

# talos.cpp_node hosts a talos_cc_shared_node library on the process's one
# libtalos_node: the library's node, devices and hardware calls land in that
# image, both under talos.testing and with TALOS_NODE_LIB naming a copy of
# libtalos_node.so (a bundle's lib/).
py_test(
    name = "cpp_node_test",
    size = "small",
    srcs = ["cpp_node_test.py"],
    data = [
        ":libframework_probe.so",
        ":libtalos_node.so",
    ],
    env = {
        "TALOS_FRAMEWORK_LIB": "$(rlocationpath :libtalos_node.so)",
        "TALOS_PROBE_LIB": "$(rlocationpath :libframework_probe.so)",
    },
    legacy_create_init = False,
    # ctypes cannot initialize sanitizer runtimes inside the Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
