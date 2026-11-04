load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("//talOS/build:contract.bzl", "talos_messages")

package(default_visibility = ["//visibility:public"])

# talos::Descriptor<M>, specialized by talos_messages for each struct.
cc_library(
    name = "descriptor",
    hdrs = ["descriptor.h"],
)

# talos::schema::Identity: a topic's schema name, hash and .bfbs, plus the
# framework layouts' identities (talos.Bytes, talos.hardware.Packet,
# talos.TargetEnvelope<M>).
cc_library(
    name = "identity",
    hdrs = ["identity.h"],
    deps = [":descriptor"],
)

# talos::schema::DecodeToText: a payload as named fields from its Identity's
# .bfbs (flatbuffers/reflection.h), for log_dump.
cc_library(
    name = "decode",
    srcs = ["decode.cc"],
    hdrs = ["decode.h"],
    deps = [
        ":identity",
        "@flatbuffers//:runtime_cc",
    ],
)

cc_test(
    name = "decode_test",
    size = "small",
    srcs = ["decode_test.cc"],
    deps = [
        ":decode",
        ":test_messages",
        "@googletest//:gtest_main",
    ],
)

# The METHODOLOGY §6 unit suffixes the field-name lint checks against.
cc_library(
    name = "unit_suffix",
    hdrs = ["unit_suffix.h"],
)

cc_test(
    name = "unit_suffix_test",
    size = "small",
    srcs = ["unit_suffix_test.cc"],
    deps = [
        ":unit_suffix",
        "@googletest//:gtest_main",
    ],
)

# talos::schema::struct_hash over upstream flatbuffers/reflection.h types.
cc_library(
    name = "schema_hash",
    srcs = ["schema_hash.cc"],
    hdrs = ["schema_hash.h"],
    deps = ["@flatbuffers//:runtime_cc"],
)

# Python classes for upstream reflection.fbs (`import reflection`): the
# flatbuffers wheel does not ship them. One module (and .pyi stub) per
# reflection.fbs type, as flatc --python names them.
_REFLECTION_TYPES = [
    "AdvancedFeatures",
    "BaseType",
    "Enum",
    "EnumVal",
    "Field",
    "KeyValue",
    "Object",
    "RPCCall",
    "Schema",
    "SchemaFile",
    "Service",
    "Type",
]

genrule(
    name = "reflection_py_gen",
    srcs = ["@flatbuffers//reflection:reflection_fbs_schema"],
    outs = ["reflection/__init__.py"] + [
        "reflection/%s.%s" % (t, ext)
        for t in _REFLECTION_TYPES
        for ext in ("py", "pyi")
    ],
    cmd = "$(execpath @flatbuffers//:flatc) -p --python-typing -o $(RULEDIR) $(SRCS)",
    tools = ["@flatbuffers//:flatc"],
    visibility = ["//visibility:private"],
)

py_library(
    name = "reflection_py",
    srcs = [":reflection_py_gen"],
    imports = ["."],
    deps = ["@pypi//flatbuffers"],
)

# `import talos_schema`: struct_hash from .bfbs bytes, equal to C++'s.
py_library(
    name = "talos_schema",
    srcs = ["talos_schema/__init__.py"],
    imports = ["."],
    deps = [":reflection_py"],
)

# Descriptor fixtures: nested structs, fixed arrays, enums of two widths and
# mixed alignment. include_prefix keeps them out of the `schema` namespace.
talos_messages(
    name = "test_messages",
    srcs = ["testdata/shapes.fbs"],
    include_prefix = "schema_test",
    visibility = [
        "//talOS/python:__pkg__",
        "//talOS/telemetry:__pkg__",
    ],
)

# schema_variant (an enum, a struct using it, a struct nesting that) in five
# spellings compiled separately, so the struct names stay the same: a
# same-size field type change, a field rename, an enum renumbering, and a
# comment/whitespace change of the same schema.
_VARIANTS = [
    "float_level",
    "float_level_reformatted",
    "int_level",
    "renamed_level",
    "renumbered_kind",
]

genrule(
    name = "variants",
    srcs = ["testdata/variants/%s.fbs" % v for v in _VARIANTS],
    outs = ["variants/%s.bfbs" % v for v in _VARIANTS],
    cmd = "$(execpath @flatbuffers//:flatc) --binary --schema -o $(RULEDIR)/variants $(SRCS)",
    tools = ["@flatbuffers//:flatc"],
    # rtms_test attaches the float/int variants to prove the attach check.
    visibility = ["//talOS/rtms:__pkg__"],
)

# TALOS_VARIANT_<NAME>: the runfile of variants/<name>.bfbs.
_VARIANT_ENV = {
    "TALOS_VARIANT_" + v.upper(): "$(rlocationpath variants/%s.bfbs)" % v
    for v in _VARIANTS
}

cc_test(
    name = "schema_hash_test",
    size = "small",
    srcs = ["schema_hash_test.cc"],
    data = ["variants/%s.bfbs" % v for v in _VARIANTS],
    env = _VARIANT_ENV,
    deps = [
        ":descriptor",
        ":schema_hash",
        ":test_messages",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Python's struct_hash against the C++-generated TALOS_SCHEMA_HASH.
py_test(
    name = "schema_hash_py_test",
    size = "small",
    srcs = ["schema_hash_py_test.py"],
    data = ["variants/%s.bfbs" % v for v in _VARIANTS],
    env = _VARIANT_ENV,
    legacy_create_init = False,
    main = "schema_hash_py_test.py",
    deps = [
        ":talos_schema",
        ":test_messages_py",
        "//talOS/build/messages_deps_test/consumer:messages_py",
        "//talOS/build/messages_deps_test/geometry:messages_py",
        "@rules_python//python/runfiles",
    ],
)
