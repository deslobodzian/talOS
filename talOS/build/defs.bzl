"""Talos SDK build rules and macros."""

load(
    "//talOS/build:contract.bzl",
    _TalosNodeInfo = "TalosNodeInfo",
    _flatbuffer_py_library = "flatbuffer_py_library",
    _talos_bundle = "talos_bundle",
    _talos_cc_node = "talos_cc_node",
    _talos_contract_cc = "talos_contract_cc",
    _talos_node = "talos_node",
    _talos_py_node = "talos_py_node",
)

TalosNodeInfo = _TalosNodeInfo
flatbuffer_py_library = _flatbuffer_py_library
talos_contract_cc = _talos_contract_cc
talos_node = _talos_node
talos_cc_node = _talos_cc_node
talos_py_node = _talos_py_node
talos_bundle = _talos_bundle
