"""Talos contract build rules and macros for FlatBuffers and multi-language messages."""

load("@aspect_bazel_lib//lib:stamping.bzl", "STAMP_ATTRS", "maybe_stamp")
load("@flatbuffers//:build_defs.bzl", "DEFAULT_INCLUDE_PATHS", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_shared_library.bzl", "cc_shared_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("@rules_python//python:py_info.bzl", "PyInfo")
load("//tools:sync_manifest.bzl", "manifest_freshness_stamp")

_TalosMessagesInfo = provider(
    doc = "The schema include tree of a talos_messages target.",
    fields = {
        "tree": "directory holding <include_prefix>/<file>.fbs for the target and all its deps",
    },
)

def _talos_schemas_impl(ctx):
    # One include root for the whole closure: each tree copies its deps'
    # trees, so `include "<prefix>/<file>.fbs";` resolves with a single -I and
    # a macro never has to name a transitive dep's output path.
    tree = ctx.actions.declare_directory(ctx.label.name)
    dep_trees = [dep[_TalosMessagesInfo].tree for dep in ctx.attr.deps]
    ctx.actions.run_shell(
        inputs = ctx.files.srcs + dep_trees,
        outputs = [tree],
        # chmod: copied trees are read-only, and a diamond copies a file twice.
        command = (
            'out="$1"; prefix="$2"; n="$3"; shift 3; ' +
            'for dep in "${@:1:$n}"; do ' +
            'cp -R "$dep/." "$out" && chmod -R u+w "$out" || exit 1; done; ' +
            'shift "$n"; mkdir -p "$out/$prefix" && ' +
            '{ [ "$#" -eq 0 ] || cp "$@" "$out/$prefix"; }'
        ),
        arguments = [tree.path, ctx.attr.include_prefix, str(len(dep_trees))] +
                    [t.path for t in dep_trees] + [f.path for f in ctx.files.srcs],
        mnemonic = "TalosSchemas",
        progress_message = "Laying out schemas of %{label}",
    )
    return [DefaultInfo(files = depset([tree])), _TalosMessagesInfo(tree = tree)]

_talos_schemas = rule(
    implementation = _talos_schemas_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = [".fbs"], mandatory = True),
        "include_prefix": attr.string(mandatory = True),
        "deps": attr.label_list(providers = [_TalosMessagesInfo]),
    },
)

_TalosBfbsInfo = provider(
    doc = "The binary schemas (.bfbs) of a talos_messages target.",
    fields = {
        "schemas": "list of struct(bfbs = File, file = declaration file " +
                   "'//<include_prefix>/<name>.fbs', stem = '<name>')",
    },
)

_DESCRIPTOR_GEN = Label("//talOS/build:descriptor_gen")

def _talos_bfbs_impl(ctx):
    # One reflection.Schema per schema, compiled from the include tree so
    # flatc records each type's declaration file as //<prefix>/<name>.fbs and
    # the generators can tell this package's structs from included ones. The
    # field-name lint runs in the same action: no .bfbs, so no C++ header
    # or Python module, while a numeric field lacks a unit suffix. flatc
    # records paths relative to the realpath of --bfbs-filenames, and the
    # sandbox symlinks each input file elsewhere, so it compiles a real copy.
    tree = ctx.attr.schemas[_TalosMessagesInfo].tree
    schemas = []
    for src in ctx.files.srcs:
        stem = src.basename.removesuffix(".fbs")
        out = ctx.actions.declare_file("%s/%s.bfbs" % (ctx.label.name, stem))
        file = "//%s/%s" % (ctx.attr.include_prefix, src.basename)
        ctx.actions.run_shell(
            inputs = [tree],
            tools = [ctx.executable._flatc, ctx.executable._gen],
            outputs = [out],
            command = (
                'tmp="$(mktemp -d)" && trap \'rm -rf "$tmp"\' EXIT && ' +
                'cp -RL "$2/." "$tmp" && ' +
                '"$1" --binary --schema --bfbs-filenames "$tmp" -I "$tmp" ' +
                '-o "$(dirname "$4")" "$tmp/$3" && "$5" lint "$4" "//$3"'
            ),
            arguments = [
                ctx.executable._flatc.path,
                tree.path,
                file.removeprefix("//"),
                out.path,
                ctx.executable._gen.path,
            ],
            mnemonic = "TalosBfbs",
            progress_message = "Compiling and linting schema %s of %%{label}" % src.basename,
        )
        schemas.append(struct(bfbs = out, file = file, stem = stem))
    return [
        DefaultInfo(files = depset([s.bfbs for s in schemas])),
        _TalosBfbsInfo(schemas = schemas),
    ]

_talos_bfbs = rule(
    implementation = _talos_bfbs_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = [".fbs"], mandatory = True),
        "schemas": attr.label(mandatory = True, providers = [_TalosMessagesInfo]),
        "include_prefix": attr.string(mandatory = True),
        "_flatc": attr.label(
            default = Label("@flatbuffers//:flatc"),
            executable = True,
            cfg = "exec",
        ),
        "_gen": attr.label(default = _DESCRIPTOR_GEN, executable = True, cfg = "exec"),
    },
)

def _talos_cc_descriptors_impl(ctx):
    # <stem>_descriptor.h beside flatc's <stem>_generated.h, so both include
    # as "<prefix>/<stem>_...h" through the talos_messages cc_library; flatc
    # writes `#include "<prefix>/<stem>_descriptor.h"` into every generated
    # header (--cpp-include), so a struct's user always sees Descriptor<M>.
    headers = []
    for schema in ctx.attr.bfbs[_TalosBfbsInfo].schemas:
        out = ctx.actions.declare_file(schema.stem + "_descriptor.h")
        ctx.actions.run(
            executable = ctx.executable._gen,
            inputs = [schema.bfbs],
            outputs = [out],
            arguments = [
                "cc",
                schema.bfbs.path,
                schema.file,
                out.path,
                ctx.attr.messages_label,
            ],
            mnemonic = "TalosDescriptorCc",
            progress_message = "Generating C++ descriptors of %s" % schema.file,
        )
        headers.append(out)
    return [DefaultInfo(files = depset(headers))]

_talos_cc_descriptors = rule(
    implementation = _talos_cc_descriptors_impl,
    attrs = {
        "bfbs": attr.label(mandatory = True, providers = [_TalosBfbsInfo]),
        "messages_label": attr.string(mandatory = True),
        "_gen": attr.label(default = _DESCRIPTOR_GEN, executable = True, cfg = "exec"),
    },
)

def _flatc_py_tree_impl(ctx):
    # One directory output, so the generated modules (one per struct, plus a
    # package __init__.py per namespace) need not be listed: what flatc emits
    # is decided by the schema, which a macro cannot read. descriptor_gen
    # then appends each struct's TALOS_SCHEMA_* attributes to its module.
    out = ctx.actions.declare_directory(ctx.label.name)
    schemas = ctx.attr.schemas[_TalosMessagesInfo].tree
    bfbs = ctx.attr.bfbs[_TalosBfbsInfo].schemas
    inputs = ctx.files.srcs + [schemas] + [s.bfbs for s in bfbs]

    # A topics-only package (no srcs) has no flatc output: its namespace
    # package is just __init__.py and topics.py.
    command = (
        'flatc="$1" out="$2" tree="$3" gen="$4" ns="$5" topics="$6" n="$7"; ' +
        'shift 7; if [ "$n" -gt 0 ]; then "$flatc" -p --python-typing ' +
        '-I "$tree" -o "$out" "${@:1:$n}" || exit 1; else ' +
        'mkdir -p "$out/$ns" && : > "$out/$ns/__init__.py"; fi; shift "$n"; ' +
        'while [ "$#" -gt 0 ]; do "$gen" py "$1" "$2" "$out" || exit 1; ' +
        "shift 2; done"
    )
    topics = ""
    if ctx.file.topics:
        # topics.py joins the namespace's package, so it imports beside the
        # structs: `from <namespace> import topics`.
        inputs.append(ctx.file.topics)
        topics = ctx.file.topics.path
        command += (
            '; if [ ! -f "$out/$ns/__init__.py" ]; then ' +
            'echo "talos_messages: topics need the schema namespace to be ' +
            'the include prefix, $ns" >&2; exit 1; fi && ' +
            'cp "$topics" "$out/$ns/topics.py"'
        )
    arguments = [
        ctx.executable._flatc.path,
        out.path,
        schemas.path,
        ctx.executable._gen.path,
        ctx.attr.namespace,
        topics,
        str(len(ctx.files.srcs)),
    ] + [f.path for f in ctx.files.srcs]
    for s in bfbs:
        arguments += [s.bfbs.path, s.file]
    ctx.actions.run_shell(
        inputs = inputs,
        tools = [ctx.executable._flatc, ctx.executable._gen],
        outputs = [out],
        command = command,
        arguments = arguments,
        mnemonic = "FlatcPython",
        progress_message = "Generating Python flatbuffers for %{label}",
    )
    import_path = ctx.workspace_name + "/" + out.short_path
    if out.short_path.startswith("../"):
        import_path = out.short_path[3:]
    return [
        DefaultInfo(files = depset([out]), runfiles = ctx.runfiles(files = [out])),
        PyInfo(
            transitive_sources = depset([out]),
            imports = depset([import_path]),
        ),
    ]

_flatc_py_tree = rule(
    implementation = _flatc_py_tree_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = [".fbs"], mandatory = True),
        "schemas": attr.label(mandatory = True, providers = [_TalosMessagesInfo]),
        "bfbs": attr.label(mandatory = True, providers = [_TalosBfbsInfo]),
        "topics": attr.label(allow_single_file = [".py"]),
        "namespace": attr.string(),
        "_flatc": attr.label(
            default = Label("@flatbuffers//:flatc"),
            executable = True,
            cfg = "exec",
        ),
        "_gen": attr.label(default = _DESCRIPTOR_GEN, executable = True, cfg = "exec"),
    },
)

# The topic grammar of talOS/NAMING.md, mirroring naming::CheckTopicName in
# talOS/introspection/names.h (the authority). //talOS/build:topic_grammar_test
# checks that the two agree on a set of names and that these tables hold
# exactly names.h's roles, abbreviations and direction suffixes.
_MAX_TOPIC_BYTES = 31
_MAX_SEGMENT_BYTES = 24
TALOS_TOPIC_ROLES = ["state", "status", "event", "telemetry", "target", "request", "command"]
TALOS_TOPIC_ABBREVIATIONS = {
    "acc": "acceleration",
    "cfg": "config",
    "cmd": "command",
    "cnt": "count",
    "drive": "drivetrain",
    "ds": "driver_station",
    "dt": "drivetrain",
    "idx": "index",
    "msg": "message",
    "odom": "odometry",
    "pkt": "packet",
    "pos": "position",
    "req": "request",
    "st": "state",
    "stat": "status",
    "tgt": "target",
    "tlm": "telemetry",
    "vel": "velocity",
}
TALOS_TOPIC_DIRECTION_SUFFIXES = ["_in", "_out", "_pub", "_sub", "_tx", "_rx", "_topic", "_send", "_recv"]
_LOWERCASE = "abcdefghijklmnopqrstuvwxyz"
_SEGMENT_CHARACTERS = _LOWERCASE + "0123456789_"
_SEGMENT_KINDS = ["owner", "role", "qualifier", "qualifier"]

def _segment_problem(segment, what):
    quoted = "%s '%s'" % (what, segment)
    if not segment:
        return what + " is empty"
    if len(segment) > _MAX_SEGMENT_BYTES:
        return "%s is %d characters; the limit is %d" % (quoted, len(segment), _MAX_SEGMENT_BYTES)
    if segment[0] not in _LOWERCASE.elems():
        return quoted + " must start with a lowercase letter"
    for c in segment.elems():
        if c not in _SEGMENT_CHARACTERS.elems():
            return quoted + " contains '%s'; segments are lowercase letters, digits and underscores" % c
    if "__" in segment:
        return quoted + " has a doubled underscore"
    if segment.endswith("_"):
        return quoted + " ends with an underscore"
    if segment in TALOS_TOPIC_ABBREVIATIONS:
        return quoted + " is an abbreviation; use '%s'" % TALOS_TOPIC_ABBREVIATIONS[segment]
    for suffix in TALOS_TOPIC_DIRECTION_SUFFIXES:
        if len(segment) > len(suffix) and segment.endswith(suffix):
            return quoted + " encodes direction in '%s'; direction belongs to the endpoint, not the name" % suffix
    return None

def talos_topic_problem(topic):
    """Why `topic` breaks the naming grammar (talOS/NAMING.md), or None.

    Args:
      topic: a topic name such as "/drivetrain/target/teleop".

    Returns:
      A message naming the broken rule, or None for a valid name.
    """
    quoted = "topic '%s'" % topic
    if not topic:
        return "topic name is empty"
    if not topic.startswith("/"):
        return quoted + " must start with '/'"
    if len(topic) > _MAX_TOPIC_BYTES:
        return "%s is %d characters; the limit is %d" % (quoted, len(topic), _MAX_TOPIC_BYTES)
    if topic.endswith("/"):
        return quoted + " ends with '/'"
    if "//" in topic:
        return quoted + " has an empty segment"
    segments = topic[1:].split("/")
    if len(segments) < 2 or len(segments) > 4:
        return quoted + " has %d segments; the form is /owner/role[/qualifier[/qualifier]]" % len(segments)
    for i, segment in enumerate(segments):
        problem = _segment_problem(segment, _SEGMENT_KINDS[i])
        if problem:
            return quoted + ": " + problem
    if segments[1] not in TALOS_TOPIC_ROLES:
        return quoted + ": role '%s' is not one of %s" % (segments[1], ", ".join(TALOS_TOPIC_ROLES))
    return None

def _is_lower_snake_case(key):
    if not key or key[0] not in _LOWERCASE.elems() or key.endswith("_") or "__" in key:
        return False
    for c in key.elems():
        if c not in _SEGMENT_CHARACTERS.elems():
            return False
    return True

def _check_topics(topics):
    seen_cc = {}
    for key, topic in topics.items():
        if not _is_lower_snake_case(key):
            fail("talos_messages: topic key %r must be a lower_snake_case name" % key)
        cc_name = "k" + "".join([part.capitalize() for part in key.split("_")])
        if cc_name in seen_cc:
            fail("talos_messages: topic keys %r and %r collide as C++ constant %s" % (seen_cc[cc_name], key, cc_name))
        seen_cc[cc_name] = key
        problem = talos_topic_problem(topic)
        if problem:
            fail("talos_messages: %s (talOS/NAMING.md)" % problem)

def _heredoc_genrule(name, out, content):
    native.genrule(
        name = name,
        outs = [out],
        cmd = "cat << 'EOF' > $@\n" + content + "EOF\n",
        visibility = ["//visibility:private"],
    )

def _dep_target(dep, suffix):
    # Maps a talos_messages label to a sibling target in the same package:
    # "//a/b:messages" + "_py" -> "//a/b:messages_py", ":messages" likewise.
    # A bare "//a/b" means target "b".
    if ":" in dep:
        return dep + suffix
    return dep + ":" + dep.split("/")[-1] + suffix

def talos_messages(
        name,
        srcs,
        topics = {},
        include_prefix = None,
        deps = [],
        visibility = ["//visibility:public"]):
    """A package's FlatBuffers messages and topic names, for C++ and Python.

    Provides:
      :<name>     cc_library. #include "<prefix>/<fbs_basename>_generated.h";
                  with `topics`, also "<prefix>/topics.h".
                  <prefix> defaults to the package's basename.
                  "<prefix>/<fbs_basename>_descriptor.h" specializes
                  talos::Descriptor<M> for each struct the file declares
                  (talOS/schema/descriptor.h).
      :<name>_py  py_library whose modules import as <namespace>.<Struct>;
                  with `topics`, also <namespace>.topics. Each struct class
                  carries TALOS_SCHEMA_NAME, TALOS_SCHEMA_HASH and
                  TALOS_SCHEMA_BFBS, the values Descriptor<M> holds.
      :<name>_bfbs  one <fbs_basename>.bfbs (reflection.Schema) per schema.

    Every numeric struct field must end in a unit suffix from
    docs/METHODOLOGY.md §6 (talOS/schema/unit_suffix.h); bools, enums and
    structs need none. A field without one fails the build.

    Messages are .fbs structs with `namespace <package>;` -- never talos.*,
    which would collide with the `talos` framework package in Python.

    Topic names are spelled once, in `topics`, and read the same way in both
    languages. topics = {"teleop_target": "/intake/target/teleop"} gives
      C++:    #include "intake/topics.h"   intake::topics::kTeleopTarget
      Python: from intake import topics     topics.TELEOP_TARGET
    The topics namespace is the include prefix, which must then be the
    schema's namespace. One topics dict per package.

    A schema can use another package's types: list that package's
    talos_messages target in `deps` and write
    `include "<dep_prefix>/<file>.fbs";` in the .fbs, where <dep_prefix> is
    the dependency's include prefix. C++ then includes
    "<dep_prefix>/<file>_generated.h" (resolved through the dep's cc_library)
    and Python imports the dep's namespace (resolved through its _py library).
    Deps are transitive.

    At a repository root the package path is empty, so pass `include_prefix`
    explicitly there (the scaffolder always does).

    Args:
      name: the cc_library; the py_library is name + "_py".
      srcs: .fbs schemas. Empty for a topics-only package whose messages
        all come from `deps` (a pose estimator publishing
        geometry.PoseEstimate): `:<name>` is then topics.h plus the deps and
        `:<name>_py` is `<prefix>.topics`.
      topics: lower_snake_case name -> topic.
      include_prefix: overrides the include prefix.
      deps: other talos_messages targets this package's schemas include.
      visibility: of both libraries.
    """
    if include_prefix == None:
        include_prefix = native.package_name().split("/")[-1]
    if not include_prefix:
        fail("talos_messages: cannot default include_prefix at a repository " +
             "root (empty package path); pass include_prefix explicitly, " +
             "usually the subsystem name")
    _check_topics(topics)
    label = str(native.package_relative_label(name))
    hdrs = []
    topics_py = None
    if topics:
        cc_lines = [
            "// Topic names of %s, generated by talos_messages." % label,
            "#pragma once",
            "",
            "namespace %s::topics {" % include_prefix,
        ]
        py_lines = [
            '"""Topic names of %s, generated by talos_messages."""' % label,
            "",
        ]
        for key, topic in topics.items():
            cc_name = "k" + "".join([part.capitalize() for part in key.split("_")])
            cc_lines.append('inline constexpr const char* %s = "%s";' % (cc_name, topic))
            py_lines.append('%s = "%s"' % (key.upper(), topic))
        cc_lines.append("}  // namespace %s::topics" % include_prefix)
        _heredoc_genrule(name + "_topics_h", "topics.h", "\n".join(cc_lines) + "\n")
        hdrs = [":topics.h"]
        topics_py = name + "_topics.py"
        _heredoc_genrule(name + "_topics_py", topics_py, "\n".join(py_lines) + "\n")
    schemas_name = name + "_schemas"
    _talos_schemas(
        name = schemas_name,
        srcs = srcs,
        include_prefix = include_prefix,
        deps = [_dep_target(dep, "_schemas") for dep in deps],
        visibility = visibility,
    )
    bfbs_name = name + "_bfbs"
    _talos_bfbs(
        name = bfbs_name,
        srcs = srcs,
        schemas = ":" + schemas_name,
        include_prefix = include_prefix,
        visibility = visibility,
    )
    _talos_messages_py(name, srcs, schemas_name, bfbs_name, topics_py, include_prefix, deps, visibility)
    if not srcs:
        # Topics only: every message the package uses comes from `deps` (a
        # pose estimator publishing geometry.PoseEstimate), so flatc has
        # nothing to compile and the library is topics.h plus the deps.
        if not topics:
            fail("talos_messages: %s has neither srcs nor topics" % label)
        cc_library(
            name = name,
            hdrs = hdrs,
            strip_include_prefix = ".",
            include_prefix = include_prefix,
            deps = list(deps),
            visibility = visibility,
        )
        return
    fbs_name = name + "_fbs"

    # --no-includes suppresses #include of dep schemas; drop it once this
    # package has deps, and keep each include's prefix so the generated
    # #include names the dep's include prefix (resolved through its cc_library).
    # Every generated header includes the package's descriptor headers, which
    # only forward-declare the structs they describe.
    descriptor_includes = [
        "--cpp-include %s/%s_descriptor.h" % (
            include_prefix,
            src.split("/")[-1].split(":")[-1].removesuffix(".fbs"),
        )
        for src in srcs
    ]
    flatc_args = [
        "--gen-object-api",
        "--gen-compare",
        "--gen-mutable",
        "--reflect-names",
        "--cpp-ptr-type flatbuffers::unique_ptr",
        "--scoped-enums",
        "--keep-prefix",
    ] + descriptor_includes
    if not deps:
        flatc_args = [
            "--gen-object-api",
            "--gen-compare",
            "--no-includes",
            "--gen-mutable",
            "--reflect-names",
            "--cpp-ptr-type flatbuffers::unique_ptr",
            "--scoped-enums",
        ] + descriptor_includes
    flatbuffer_cc_library(
        name = fbs_name,
        srcs = srcs,
        includes = [":" + schemas_name] if deps else [],
        include_paths = DEFAULT_INCLUDE_PATHS + ["$(location :%s)" % schemas_name] if deps else None,
        # flatbuffers' DEFAULT_FLATC_ARGS plus --scoped-enums: a schema enum
        # `enum IntakeCommand : ubyte { Idle, Intake }` is a C++ enum class,
        # spelled IntakeCommand::Intake.
        flatc_args = flatc_args,
        visibility = ["//visibility:private"],
    )
    _talos_cc_descriptors(
        name = name + "_descriptors",
        bfbs = ":" + bfbs_name,
        messages_label = label,
        visibility = ["//visibility:private"],
    )
    cc_library(
        name = name,
        hdrs = hdrs + [":" + fbs_name + "_srcs", ":" + name + "_descriptors"],
        strip_include_prefix = ".",
        include_prefix = include_prefix,
        deps = [":" + fbs_name, str(Label("//talOS/schema:descriptor"))] + list(deps),
        visibility = visibility,
    )

def _talos_messages_py(name, srcs, schemas_name, bfbs_name, topics_py, include_prefix, deps, visibility):
    _flatc_py_tree(
        name = name + "_py_gen",
        srcs = srcs,
        schemas = ":" + schemas_name,
        bfbs = ":" + bfbs_name,
        topics = topics_py,
        namespace = include_prefix,
        visibility = ["//visibility:private"],
    )
    py_library(
        name = name + "_py",
        deps = [":" + name + "_py_gen", Label("@pypi//flatbuffers")] + [_dep_target(dep, "_py") for dep in deps],
        visibility = visibility,
    )

TalosNodeInfo = provider(
    doc = "Talos node metadata and runtime information",
    fields = {
        "node_name": "logical name of the node (string)",
        "node_target": "full Bazel target label (string)",
        "executable": "File representing the main executable",
        "runfiles": "Runfiles object containing the runtime closure",
        "language": "'cpp' or 'python'",
    },
)

def _talos_node_impl(ctx):
    binary_info = ctx.attr.binary[DefaultInfo]
    out_exec = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.symlink(
        output = out_exec,
        target_file = ctx.executable.binary,
        is_executable = True,
    )
    runfiles = ctx.runfiles(transitive_files = binary_info.default_runfiles.files).merge(binary_info.default_runfiles)
    node_info = TalosNodeInfo(
        node_name = ctx.attr.node_name,
        node_target = ctx.attr.node_target,
        executable = out_exec,
        runfiles = runfiles,
        language = ctx.attr.language,
    )
    return [
        DefaultInfo(
            executable = out_exec,
            runfiles = runfiles,
        ),
        node_info,
    ]

talos_node = rule(
    implementation = _talos_node_impl,
    executable = True,
    attrs = {
        "binary": attr.label(
            mandatory = True,
            executable = True,
            cfg = "target",
        ),
        "node_name": attr.string(mandatory = True),
        "node_target": attr.string(mandatory = True),
        "language": attr.string(mandatory = True, values = ["cpp", "python"]),
    },
)

def talos_cc_node(
        name,
        node_name,
        srcs,
        deps = [],
        node_target = None,
        visibility = None,
        testonly = None,
        **kwargs):
    """A C++ talOS node: sources ending in TALOS_NODE(Class).

    TALOS_NODE_NAME and TALOS_NODE_TARGET reach the sources as string-literal
    local_defines, so the node's identity is spelled once, here. The binary
    depends on //talOS/node and is wrapped in talos_node (TalosNodeInfo) for
    bundles.

    Args:
      name: target name; the talos_node rule. The binary is name + "_bin".
      node_name: the node's subsystem name (its [subsystems.<name>] key).
      srcs: C++ sources, one of which uses TALOS_NODE.
      deps: libraries the sources use (messages, talOS/devices, ...).
      node_target: defaults to this label.
      visibility: of the talos_node target (public by default).
      testonly: of every rule generated: True for a test fixture, which only
        a testonly bundle or a test can carry. check_node_matrix skips a
        package whose node rules all set it.
      **kwargs: passed to the cc_binary.
    """

    # A bundle copies the binary to bin/<name> and its shared libraries
    # to lib/ (talos_bundle), so the binary also searches
    # ../lib beside Bazel's own runfiles rpaths.
    linkopts = kwargs.pop("linkopts", []) + select({
        Label("@platforms//os:macos"): ["-Wl,-rpath,@loader_path/../lib"],
        "//conditions:default": ["-Wl,-rpath,$$ORIGIN/../lib"],
    })
    if not node_target:
        node_target = str(native.package_relative_label(name))
        if node_target.startswith("@@//"):
            node_target = node_target[2:]
    caller_local_defines = kwargs.pop("local_defines", None)
    local_defines = [
        'TALOS_NODE_NAME=\\"%s\\"' % node_name,
        'TALOS_NODE_TARGET=\\"%s\\"' % node_target,
    ]
    if caller_local_defines != None:
        local_defines = caller_local_defines + local_defines
    node_dep = Label("//talOS/node")
    bin_name = name + "_bin"
    cc_binary(
        name = bin_name,
        srcs = srcs,
        deps = deps + [node_dep],
        linkopts = linkopts,
        local_defines = local_defines,
        testonly = testonly,
        visibility = ["//visibility:private"],
        **kwargs
    )
    talos_node(
        name = name,
        binary = ":" + bin_name,
        node_name = node_name,
        node_target = node_target,
        language = "cpp",
        testonly = testonly,
        visibility = visibility if visibility != None else ["//visibility:public"],
    )

def talos_cc_shared_node(
        name,
        srcs,
        deps = [],
        visibility = None,
        testonly = None,
        tags = None,
        **kwargs):
    """A shared library of C++ nodes for talos.cpp_node.

    The sources export node classes with TALOS_EXPORT_NODE(name, Class). The
    library links the talOS framework dynamically, from
    //talOS/node_api:libtalos_node.so: Python loads that library first, so the
    hosted classes run on the process's one copy of Node, the devices layer
    and their statics and extension types. A library that linked its own
    copy would run that copy against libtalos_node's Node. Anything else in
    `deps` is linked into the library.

    On Linux the library carries its file name as its soname and links with
    -z defs, so a symbol neither it nor libtalos_node.so defines fails the
    link instead of the load (macOS's linker refuses one already).

    Args:
      name: the library's file name, e.g. "libmechanisms.so"; talos.cpp_node
        takes "<package>/<name>".
      srcs: C++ sources using TALOS_EXPORT_NODE.
      deps: libraries the sources use (the node classes, messages, ...).
      visibility: of the shared library.
      testonly: of both rules.
      tags: of both rules.
      **kwargs: passed to the cc_library that compiles `srcs` (copts,
        defines, hdrs, ...).
    """
    objects = name.removesuffix(".so") + "_exports"
    cc_library(
        name = objects,
        srcs = srcs,
        deps = deps + [Label("//talOS/node")],
        # The talos_cpp_node_<name> entry points are referenced by nothing
        # but dlsym.
        alwayslink = True,
        testonly = testonly,
        tags = tags,
        visibility = ["//visibility:private"],
        **kwargs
    )
    cc_shared_library(
        name = name,
        shared_lib_name = name,
        dynamic_deps = [Label("//talOS/node_api:libtalos_node.so")],
        features = ["set_soname"],
        user_link_flags = select({
            Label("@platforms//os:linux"): ["-Wl,-z,defs"],
            "//conditions:default": [],
        }),
        testonly = testonly,
        tags = tags,
        visibility = visibility,
        deps = [":" + objects],
    )

def talos_py_node(
        name,
        node_name,
        main,
        class_name,
        deps = [],
        data = [],
        node_target = None,
        visibility = None,
        testonly = None,
        **kwargs):
    """A Python talOS node: class `class_name` of the file `main`.

    Generates the runner that calls talos.run(Class), as a py_binary wrapped
    in talos_node (TalosNodeInfo) for bundles. The file is loaded by path
    (talos.load), so it may share its name with its message package.

    Args:
      name: target name; the talos_node rule. The binary is name + "_bin".
      node_name: the node's subsystem name (its [subsystems.<name>] key).
      main: the node's .py file, relative to this package.
      class_name: the node class in `main`.
      deps: libraries `main` imports (messages_py, ...).
      data: runtime files.
      node_target: defaults to this label.
      visibility: of the talos_node target (public by default).
      testonly: of every rule generated, as in talos_cc_node.
      **kwargs: passed to the py_binary.
    """
    if not node_target:
        node_target = str(native.package_relative_label(name))
        if node_target.startswith("@@//"):
            node_target = node_target[2:]
    runner_py = name + "_runner.py"
    content = (
        "import os, sys, talos\n" +
        "node = talos.load(os.path.join(os.path.dirname(__file__), \"%s\"))\n" % main +
        "sys.exit(talos.run(node.%s, \"%s\", \"%s\"))\n" % (class_name, node_name, node_target)
    )
    native.genrule(
        name = name + "_runner_gen",
        outs = [runner_py],
        cmd = "cat << 'EOF' > $@\n" + content + "EOF\n",
        testonly = testonly,
        visibility = ["//visibility:private"],
    )

    bin_name = name + "_bin"
    py_binary(
        name = bin_name,
        srcs = [runner_py, main],
        main = runner_py,
        legacy_create_init = False,
        deps = deps + [Label("//talOS/python:talos")],
        data = data,
        testonly = testonly,
        visibility = ["//visibility:private"],
        **kwargs
    )

    talos_node(
        name = name,
        binary = ":" + bin_name,
        node_name = node_name,
        node_target = node_target,
        language = "python",
        testonly = testonly,
        visibility = visibility if visibility != None else ["//visibility:public"],
    )

# The bridge the launcher starts unless the config's [robot] hardware_node
# names another.
_HARDWARE_NODE = Label("//talOS/bridge:hardware_node")

def _bundle_label(label):
    """`label` as a bundle manifest spells it.

    That is //pkg:name in the main repository, @@repo//pkg:name in any other.
    """
    raw = str(label)
    if raw.startswith("@@//"):
        return raw[2:]
    if raw.startswith("@//"):
        return raw[1:]
    return raw

def _resolve_config(ctx, manifest_stamp):
    """Runs the configuration resolver on ctx's config; returns its JSON.

    The JSON names every subsystem's node and holds the effective config, so
    the bundle builder can compare `nodes` with what the config launches. The
    manifest check's stamp is an input: a stale manifest reports itself, with
    the regenerate command, before the resolver meets a file it was not given.
    """
    resolved = ctx.actions.declare_file(ctx.label.name + "_resolved.json")

    # As in the manifest check, the resolver reads the config's files by
    # their paths from the root of the config's repository.
    workspace_root = ctx.attr.config.label.workspace_root
    config_path = ctx.file.config.path
    if workspace_root:
        config_path = config_path.removeprefix(workspace_root + "/")
    ctx.actions.run_shell(
        inputs = depset([ctx.file.config, manifest_stamp] + ctx.files.config_files),
        tools = [ctx.attr._resolve[DefaultInfo].files_to_run],
        outputs = [resolved],
        command = 'root="$PWD"; cd "$root/$4" && "$root/$1" "$2" > "$root/$3"',
        arguments = [ctx.executable._resolve.path, config_path, resolved.path, workspace_root],
        mnemonic = "TalosResolveConfig",
        progress_message = "Resolving the config of %{label}",
    )
    return resolved

def _talos_bundle_impl(ctx):
    bundle_dir = ctx.actions.declare_directory(ctx.label.name)
    run_script = ctx.actions.declare_file(ctx.label.name + ".sh")

    launcher_exec = ctx.executable.launcher
    launcher_runfiles = ctx.attr.launcher[DefaultInfo].default_runfiles.files

    manifest_stamp = manifest_freshness_stamp(
        ctx,
        ctx.file.config,
        ctx.file.manifest,
        ctx.files.config_files,
        ctx.attr._manifest_sync[DefaultInfo].files_to_run,
    )
    resolved = _resolve_config(ctx, manifest_stamp)
    direct_inputs = [launcher_exec, ctx.file.config, manifest_stamp, resolved]
    direct_inputs.extend(ctx.files.config_files)
    transitive_inputs = [launcher_runfiles]

    nodes_spec = []
    for node in ctx.attr.nodes:
        if TalosNodeInfo in node:
            info = node[TalosNodeInfo]
            n_name = info.node_name
            n_target = info.node_target
            n_exec = info.executable
            n_rf = info.runfiles
            n_lang = info.language
        else:
            default_info = node[DefaultInfo]
            n_exec = default_info.files_to_run.executable
            n_rf = default_info.default_runfiles
            n_target = _bundle_label(node.label)
            if node.label.name == "node":
                if node.label.package:
                    n_name = node.label.package.split("/")[-1]
                elif node.label.repo_name:
                    n_name = node.label.repo_name.split("+")[0].split("~")[0] or node.label.name
                else:
                    n_name = node.label.name
            else:
                n_name = node.label.name
            n_lang = "cpp"

        direct_inputs.append(n_exec)
        rf_list = []
        lib_list = []
        n_repo_mapping = None
        if n_lang == "python":
            ftr = node[DefaultInfo].files_to_run
            rm = ftr.repo_mapping_manifest if ftr else None
            if rm:
                direct_inputs.append(rm)
                n_repo_mapping = rm.path
        if n_rf and n_rf.files:
            transitive_inputs.append(n_rf.files)
            if n_lang == "python":
                for f in n_rf.files.to_list():
                    if f.short_path.startswith("../"):
                        rf_rel = f.short_path[3:]
                    else:
                        rf_rel = "_main/" + f.short_path
                    rf_list.append({
                        "src_path": f.path,
                        "rf_rel": rf_rel,
                    })
            else:
                # A C++ node is copied without its runfiles, so the shared
                # libraries it links go to lib/, which talos_cc_node's rpath
                # names.
                for f in n_rf.files.to_list():
                    if f.extension in ("dylib", "so") or ".so." in f.basename:
                        lib_list.append({"src_path": f.path, "name": f.basename})
        node_spec = {
            "name": n_name,
            "target": n_target,
            "label": _bundle_label(node.label),
            "language": n_lang,
            "src_path": n_exec.path,
            "runfiles": rf_list,
            "shared_libs": lib_list,
        }
        if n_repo_mapping:
            node_spec["repo_mapping"] = n_repo_mapping
        nodes_spec.append(node_spec)

    # Each config file keeps its path relative to the config, so `include`,
    # `path` and `defaults` resolve in the bundle as they do in the source tree.
    config_files_spec = [
        {"src_path": f.path, "short_path": f.short_path}
        for f in ctx.files.config_files
    ]

    # Studio: the bridge binary and the built webroot, both or neither. The
    # robot's BUILD names them; talOS names no studio/ label.
    studio_spec = None
    if bool(ctx.attr.studio_bridge) != bool(ctx.attr.studio_webroot):
        fail("talos_bundle: set studio_bridge and studio_webroot together, or neither")
    if ctx.attr.studio_bridge:
        bridge_exec = ctx.executable.studio_bridge
        webroot = ctx.file.studio_webroot
        direct_inputs.extend([bridge_exec, webroot])
        studio_spec = {
            "bridge_path": bridge_exec.path,
            "bridge_label": _bundle_label(ctx.attr.studio_bridge.label),
            "webroot_path": webroot.path,
        }

    spec = {
        "bundle_version": 1,
        "launcher_path": launcher_exec.path,
        "config": {"src_path": ctx.file.config.path, "short_path": ctx.file.config.short_path},
        "config_files": config_files_spec,
        "nodes": nodes_spec,
        "default_hardware_node": _bundle_label(_HARDWARE_NODE),
        "studio": studio_spec,
        # deploy.sh install refuses a bundle built testonly (it may carry
        # testonly nodes, test fixtures) outside a Bazel test.
        "testonly": ctx.attr.testonly,
    }

    spec_file = ctx.actions.declare_file(ctx.label.name + "_spec.json")
    ctx.actions.write(
        output = spec_file,
        content = json.encode(spec),
    )

    direct_inputs.append(spec_file)
    arguments = [
        "--spec",
        spec_file.path,
        "--resolved",
        resolved.path,
        "--out",
        bundle_dir.path,
    ]

    # Pass Bazel's `_repo_mapping` manifests (from the launcher, nodes, and
    # Studio bridge) so bundle_builder maps canonical repo names to the main
    # repository's real apparent names instead of guessing from `+` suffixes.
    repo_mapping_targets = [ctx.attr.launcher] + list(ctx.attr.nodes)
    if ctx.attr.studio_bridge:
        repo_mapping_targets.append(ctx.attr.studio_bridge)
    seen_repo_mappings = {}
    for target in repo_mapping_targets:
        ftr = target[DefaultInfo].files_to_run
        rm = ftr.repo_mapping_manifest if ftr else None
        if rm and rm not in seen_repo_mappings:
            seen_repo_mappings[rm] = True
            direct_inputs.append(rm)
            arguments += ["--repo-mapping", rm.path]

    # Also pass apparent -> canonical repo pairs extracted in the caller's
    # repository context by the talos_bundle macro: when every target in the
    # bundle lives in an external repo (e.g. @my_talos), none of their
    # repo_mapping_manifests carry `source == ""` rows.
    for apparent, canonical in sorted(ctx.attr.apparent_repos.items()):
        arguments += ["--apparent-repo", "%s=%s" % (apparent, canonical)]

    # The source revision only under --stamp (--config=release): the status
    # file changes with every commit, so an unstamped bundle never reads it
    # and stays cached.
    stamp = maybe_stamp(ctx)
    if stamp:
        direct_inputs.append(stamp.stable_status_file)
        arguments += ["--stable-status", stamp.stable_status_file.path]

    ctx.actions.run(
        executable = ctx.attr._builder[DefaultInfo].files_to_run,
        outputs = [bundle_dir],
        inputs = depset(direct = direct_inputs, transitive = transitive_inputs),
        arguments = arguments,
        mnemonic = "TalosBundle",
        progress_message = "Assembling Talos bundle %s" % ctx.label.name,
    )

    bundle_rel = bundle_dir.short_path
    bundle_name = ctx.label.name

    run_script_content = """#!/usr/bin/env bash
set -e

BUNDLE_REL="{bundle_rel}"
BUNDLE_DIR=""

if [[ -n "${{RUNFILES_DIR:-}}" ]]; then
  for d in "${{RUNFILES_DIR}}/_main/${{BUNDLE_REL}}" "${{RUNFILES_DIR}}/${{BUNDLE_REL}}" "${{RUNFILES_DIR}}"/*/"${{BUNDLE_REL}}"; do
    if [[ -d "$d" ]]; then
      BUNDLE_DIR="$d"
      break
    fi
  done
fi

if [[ -z "${{BUNDLE_DIR}}" && -d "${{0}}.runfiles" ]]; then
  for d in "${{0}}.runfiles/_main/${{BUNDLE_REL}}" "${{0}}.runfiles/${{BUNDLE_REL}}" "${{0}}.runfiles"/*/"${{BUNDLE_REL}}"; do
    if [[ -d "$d" ]]; then
      BUNDLE_DIR="$d"
      break
    fi
  done
fi

if [[ -z "${{BUNDLE_DIR}}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
  if [[ -d "${{SCRIPT_DIR}}/{bundle_name}" ]]; then
    BUNDLE_DIR="${{SCRIPT_DIR}}/{bundle_name}"
  elif [[ -d "${{SCRIPT_DIR}}/${{BUNDLE_REL}}" ]]; then
    BUNDLE_DIR="${{SCRIPT_DIR}}/${{BUNDLE_REL}}"
  fi
fi

if [[ -z "${{BUNDLE_DIR}}" || ! -d "${{BUNDLE_DIR}}" ]]; then
  echo "Error: cannot find bundle directory for {bundle_name} at ${{BUNDLE_REL}}" >&2
  exit 1
fi

exec "${{BUNDLE_DIR}}/run.sh" "$@"
""".format(
        bundle_rel = bundle_rel,
        bundle_name = bundle_name,
    )

    ctx.actions.write(
        output = run_script,
        content = run_script_content,
        is_executable = True,
    )

    return [
        DefaultInfo(
            executable = run_script,
            files = depset([bundle_dir, run_script]),
            runfiles = ctx.runfiles(files = [bundle_dir, run_script]).merge(ctx.attr.launcher[DefaultInfo].default_runfiles),
        ),
    ]

_talos_bundle = rule(
    implementation = _talos_bundle_impl,
    executable = True,
    attrs = dict({
        "nodes": attr.label_list(
            mandatory = True,
            doc = "Every node the config launches, each providing TalosNodeInfo or DefaultInfo: " +
                  "the hardware node ([robot] hardware_node, default //talOS/bridge:hardware_node) " +
                  "and the NODES of its generated manifest. The build fails on any other set.",
        ),
        "config": attr.label(
            mandatory = True,
            allow_single_file = True,
            doc = "The robot.toml config file",
        ),
        "manifest": attr.label(
            mandatory = True,
            allow_single_file = [".bzl"],
            doc = "The generated <stem>_manifest.bzl beside config",
        ),
        "config_files": attr.label_list(
            allow_files = True,
            default = [],
            doc = "Every file the config reads besides itself: the CONFIG_FILES of its generated manifest",
        ),
        "launcher": attr.label(
            default = Label("//talOS/launcher:launcher"),
            executable = True,
            cfg = "target",
            doc = "The launcher binary",
        ),
        "studio_bridge": attr.label(
            executable = True,
            cfg = "target",
            doc = "The Studio bridge binary (the robot's [studio] bridge) that " +
                  "--start-studio-bridge starts from the bundle. Set with studio_webroot.",
        ),
        "studio_webroot": attr.label(
            allow_single_file = True,
            doc = "The built Studio site, one directory holding index.html, that " +
                  "the bundled bridge serves. Set with studio_bridge.",
        ),
        "apparent_repos": attr.string_dict(
            default = {},
            doc = "Apparent-to-canonical repo names extracted in the caller's repository context.",
        ),
        "_builder": attr.label(
            default = Label("//talOS/build:bundle_builder"),
            executable = True,
            cfg = "exec",
        ),
        "_resolve": attr.label(
            default = Label("//talOS/configuration:resolve"),
            executable = True,
            cfg = "exec",
        ),
        "_manifest_sync": attr.label(
            default = Label("//tools:sync_manifest"),
            executable = True,
            cfg = "exec",
        ),
    }, **STAMP_ATTRS),
)

def _extract_apparent_repos(targets):
    """Maps apparent repo names seen in caller target strings to canonical names."""
    mapping = {}
    for item in targets:
        if not item or type(item) != "string":
            continue
        if not item.startswith("@") or item.startswith("@@"):
            continue
        if "//" in item:
            apparent = item[1:].split("//", 1)[0]
        elif ":" not in item:
            apparent = item[1:]
        else:
            continue
        if not apparent or apparent in mapping:
            continue
        canonical = native.package_relative_label(item).repo_name
        mapping[apparent] = canonical if canonical else "_main"
    return mapping

def talos_bundle(name, config, **kwargs):
    """Assembles a deployable bundle of exactly the nodes `config` launches.

    The build fails when the config's generated manifest is stale, or when
    `nodes` is not the hardware node ([robot] hardware_node, default
    //talOS/bridge:hardware_node) plus the manifest's NODES.

    bundle_manifest.toml records each node binary's SHA-256 and, in a --stamp
    build (--config=release), the source revision and dirty flag that
    tools/workspace_status.sh reports. A testonly bundle's manifest says
    `testonly = true`, and tools/deploy.sh install refuses it outside a
    Bazel test.

    Args:
      name: bundle target name.
      config: robot TOML label; its generated manifest must live beside it.
      **kwargs: passed to the bundle rule: nodes, the hardware node plus the
        manifest's NODES; config_files, its CONFIG_FILES; and, for a robot
        viewed with Studio, studio_bridge and studio_webroot (the bridge
        binary and the built site), which put Studio in the bundle; its
        run.sh then starts the bridge unless given --no-studio.
    """
    config_label = native.package_relative_label(config)
    manifest = config_label.same_package_label(
        config_label.name.removesuffix(".toml") + "_manifest.bzl",
    )
    caller_targets = list(kwargs.get("nodes", []))
    for key in ("hardware_node", "studio_bridge", "launcher"):
        if kwargs.get(key):
            caller_targets.append(kwargs[key])
    apparent_repos = _extract_apparent_repos(caller_targets)
    if "apparent_repos" in kwargs:
        apparent_repos.update(kwargs.pop("apparent_repos"))
    _talos_bundle(
        name = name,
        config = config,
        manifest = manifest,
        apparent_repos = apparent_repos,
        **kwargs
    )

# The launcher and the runner that starts it from a runfiles tree
# (launcher_runner.py), shared by talos_graph_test and talos_sim.
_LAUNCHER = Label("//talOS/launcher:launcher")
_LAUNCHER_RUNNER = Label("//talOS/build:launcher_runner.py")

def _launcher_runner_data(config, nodes, data, hardware_node = None):
    """The runner's runfiles: config, launcher, hardware node, nodes, data."""
    hw = hardware_node if hardware_node != None else _HARDWARE_NODE
    seen = {}
    labels = []
    for item in [config, _LAUNCHER, hw] + list(nodes) + list(data):
        lbl = native.package_relative_label(item)
        if lbl not in seen:
            seen[lbl] = True
            labels.append(lbl)
    return labels

def _rlocation(label):
    """`label`'s runfiles path, as $(rlocationpath) would expand it."""
    return "/".join([label.repo_name or "_main"] + ([label.package] if label.package else []) + [label.name])

def _launcher_runner_args(config):
    return [
        "--launcher",
        "$(rlocationpath %s)" % _LAUNCHER,
        "--config",
        "$(rlocationpath %s)" % native.package_relative_label(config),
    ]

def talos_graph_test(
        name,
        config,
        nodes,
        data = [],
        hardware_node = None,
        **kwargs):
    """Probes every node in `config` with `launcher --describe-only` and lints the graph.

    The graph must list the hardware node and one node per entry of `nodes`,
    each of them described. The config must be in the main repository; talOS
    may be the main repository or a dependency (@talos).

    Args:
      name: test target name.
      config: the robot.toml file or label.
      nodes: the node binaries `config` names: the NODES of its generated manifest.
      data: every file `config` reads besides itself: the manifest's CONFIG_FILES.
      hardware_node: optional label of the `[robot] hardware_node` binary
        (defaults to `//talOS/bridge:hardware_node`). Pass this when the config
        overrides `[robot] hardware_node` so `expected_nodes` and runfiles stay
        accurate.
      **kwargs: passed to py_test.
    """

    # The roster is the hardware node and one node per NODES entry: a binary
    # registers under one name and the launcher refuses two nodes of one name,
    # so no two subsystems of a passing graph share an entry.
    hw = hardware_node if hardware_node != None else _HARDWARE_NODE
    expected_nodes = len({
        native.package_relative_label(item): True
        for item in [hw] + list(nodes)
    })
    size = kwargs.pop("size", "small")
    py_test(
        name = name,
        size = size,
        srcs = [_LAUNCHER_RUNNER],
        main = _LAUNCHER_RUNNER,
        args = _launcher_runner_args(config) + ["--graph-nodes", str(expected_nodes)],
        data = _launcher_runner_data(config, nodes, data, hardware_node = hw),
        legacy_create_init = False,
        deps = [Label("@rules_python//python/runfiles")],
        **kwargs
    )

def talos_sim(
        name,
        config,
        nodes,
        data = [],
        hardware_node = None,
        **kwargs):
    """Runs `config` in simulation: `bazel run :<name> -- [launcher args]`.

    An executable whose runfiles hold the launcher, the hardware node, every
    node and `data`. It runs `launcher --config <config> --sim
    --start-sim-gateway --sim-gateway-arg --physics` and then the arguments
    given after `--` (`--duration-s 3`, `--sim-gateway-arg --mode-schedule
    --sim-gateway-arg ...`, `--start-studio-bridge`), and exits with the
    launcher's status. A relative `--output-dir` is taken from where
    `bazel run` was invoked. Nothing is copied per run.

    Args:
      name: target name.
      config: the robot.toml file or label, in the main repository.
      nodes: the node binaries `config` names: the NODES of its generated manifest.
      data: every other file the session reads: the manifest's CONFIG_FILES
        and the simulator the config's [robot] sim_gateway names.
      hardware_node: optional label of the `[robot] hardware_node` binary
        (defaults to `//talOS/bridge:hardware_node`).
      **kwargs: passed to py_binary.
    """

    # `bazel run` applies a py_binary's `args`, but a test that runs the
    # binary from its runfiles does not, so a generated main holds them.
    main_py = name + "_sim.py"
    runner_args = [
        "--launcher",
        _rlocation(_LAUNCHER),
        "--config",
        _rlocation(native.package_relative_label(config)),
        "--sim",
        "--",
    ]
    _heredoc_genrule(name + "_main_gen", main_py, "\n".join([
        "import importlib.util",
        "import sys",
        "",
        "from python.runfiles import runfiles",
        "",
        "path = runfiles.Create().Rlocation(%r, source_repo=\"\")" % _rlocation(_LAUNCHER_RUNNER),
        "spec = importlib.util.spec_from_file_location(\"launcher_runner\", path)",
        "runner = importlib.util.module_from_spec(spec)",
        "spec.loader.exec_module(runner)",
        "sys.exit(runner.main(%r + sys.argv[1:]))" % runner_args,
        "",
    ]))
    py_binary(
        name = name,
        srcs = [main_py],
        main = main_py,
        data = _launcher_runner_data(config, nodes, data, hardware_node = hardware_node) + [_LAUNCHER_RUNNER],
        legacy_create_init = False,
        deps = [Label("@rules_python//python/runfiles")],
        **kwargs
    )

# talos_sim_test's check and what it judges a session with.
_SIM_CHECK = Label("//talOS/build:sim_check.py")
_LOG_DUMP = Label("//talOS/launcher:log_dump")
_SESSION_REPORT = Label("//talOS/launcher:session_report")
_REGISTRY_UNLINK = Label("//talOS/build:registry_unlink")

def talos_sim_test(name, sim, settle_s = 1, studio = False, **kwargs):
    """Runs the talos_sim target `sim` until it is up, stops it and judges its session.

    The session runs until the bridge has accepted a State from the
    simulator and every member's loop has started (however long the members
    take to construct), then `settle_s` more, and is then stopped in order
    with SIGINT. The test fails unless that happens before a bound derived
    from Bazel's timeout (a hang, stopped with every process it started and
    reported with its output); the launch runs until stopped and exits 0;
    every member of the session's manifest ran exactly once and exited with
    code 0 and no signal; the session holds the simulator and a node besides
    it and the hardware bridge; and the bridge received State (session_report's
    "controller link" verdict). When `studio` is True, the session also starts
    Studio's bridge on an ephemeral HTTP port (`--start-studio-bridge
    --studio-http-port 0`), waits for it to bind its port and settle past the
    launcher's settle interval, and fails unless `studio_bridge` appears in the
    manifest with a valid `http_port`, 1 clean run (`exit_code == 0`,
    `exit_signal == 0`), `restart_count == 0`, and `recorder_failed == False`.
    The check is talOS/build/sim_check.py.

    Tagged `e2e` (it runs the whole config) and `exclusive` (it does, on as
    many cores as the robot uses), as Bazel's `medium` size (its `moderate`
    timeout) unless given.

    Args:
      name: test target name.
      sim: the label of a talos_sim target.
      settle_s: seconds the session runs once it is up, before the stop: a
        member that fails soon after starting fails the test. It does not
        bound startup. A value leaving less than 10 s of the timeout for
        startup, after 45 s for the stop, fails the test before it starts.
      studio: whether to start and verify the supervised Studio bridge
        (`--studio-bridge`).
      **kwargs: passed to py_test.
    """
    sim_label = native.package_relative_label(sim)
    tags = kwargs.pop("tags", [])
    args = [
        "--sim",
        "$(rlocationpath %s)" % sim_label,
        "--settle-s",
        str(settle_s),
        "--log-dump",
        "$(rlocationpath %s)" % _LOG_DUMP,
        "--session-report",
        _rlocation(_SESSION_REPORT.same_package_label("session_report.py")),
        "--registry-unlink",
        "$(rlocationpath %s)" % _REGISTRY_UNLINK,
    ]
    if studio:
        args.append("--studio-bridge")
    py_test(
        name = name,
        size = kwargs.pop("size", "medium"),
        srcs = [_SIM_CHECK],
        main = _SIM_CHECK,
        args = args,
        data = [sim_label, _LOG_DUMP, _SESSION_REPORT, _REGISTRY_UNLINK],
        legacy_create_init = False,
        tags = tags + [tag for tag in ["e2e", "exclusive"] if tag not in tags],
        deps = [Label("@rules_python//python/runfiles")],
        **kwargs
    )
