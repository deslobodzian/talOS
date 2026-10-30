"""Talos contract build rules and macros for FlatBuffers and multi-language messages."""

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@flatbuffers//:build_defs.bzl", "DEFAULT_INCLUDE_PATHS", "flatbuffer_cc_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("@rules_python//python:py_info.bzl", "PyInfo")

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
            'shift "$n"; mkdir -p "$out/$prefix" && cp "$@" "$out/$prefix"'
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

def _flatc_py_tree_impl(ctx):
    # One directory output, so the generated modules (one per struct, plus a
    # package __init__.py per namespace) need not be listed: what flatc emits
    # is decided by the schema, which a macro cannot read.
    out = ctx.actions.declare_directory(ctx.label.name)
    schemas = ctx.attr.schemas[_TalosMessagesInfo].tree
    inputs = ctx.files.srcs + [schemas]
    command = '"$1" -p --python-typing -I "$3" -o "$2" "${@:4}"'
    arguments = [ctx.executable._flatc.path, out.path, schemas.path]
    if ctx.file.topics:
        # topics.py joins the namespace's package, so it imports beside the
        # structs: `from <namespace> import topics`.
        inputs.append(ctx.file.topics)
        command = (
            '"$1" -p --python-typing -I "$3" -o "$2" "${@:6}" && ' +
            'if [ ! -f "$2/$4/__init__.py" ]; then ' +
            'echo "talos_messages: topics need the schema namespace to be ' +
            'the include prefix, $4" >&2; exit 1; fi && ' +
            'cp "$5" "$2/$4/topics.py"'
        )
        arguments += [ctx.attr.namespace, ctx.file.topics.path]
    ctx.actions.run_shell(
        inputs = inputs,
        tools = [ctx.executable._flatc],
        outputs = [out],
        command = command,
        arguments = arguments + [f.path for f in ctx.files.srcs],
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
        "topics": attr.label(allow_single_file = [".py"]),
        "namespace": attr.string(),
        "_flatc": attr.label(
            default = Label("@flatbuffers//:flatc"),
            executable = True,
            cfg = "exec",
        ),
    },
)

def _check_topics(topics):
    for key, topic in topics.items():
        if not key or not key[0].isalpha() or not key.replace("_", "a").isalnum() or key.lower() != key:
            fail("talos_messages: topic key %r must be a lower_snake_case name" % key)
        if not topic.startswith("/") or not topic.replace("/", "a").replace("_", "a").replace("-", "a").replace(".", "a").isalnum():
            fail("talos_messages: topic %r must be /-separated [A-Za-z0-9_.-] segments" % topic)

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
      :<name>_py  py_library whose modules import as <namespace>.<Struct>;
                  with `topics`, also <namespace>.topics.

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
      srcs: .fbs schemas.
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
    fbs_name = name + "_fbs"
    # --no-includes suppresses #include of dep schemas; drop it once this
    # package has deps, and keep each include's prefix so the generated
    # #include names the dep's include prefix (resolved through its cc_library).
    # Packages without deps keep the old flags, so their generated headers are
    # byte-identical.
    flatc_args = [
        "--gen-object-api",
        "--gen-compare",
        "--gen-mutable",
        "--reflect-names",
        "--cpp-ptr-type flatbuffers::unique_ptr",
        "--scoped-enums",
        "--keep-prefix",
    ]
    if not deps:
        flatc_args = [
            "--gen-object-api",
            "--gen-compare",
            "--no-includes",
            "--gen-mutable",
            "--reflect-names",
            "--cpp-ptr-type flatbuffers::unique_ptr",
            "--scoped-enums",
        ]
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
    cc_library(
        name = name,
        hdrs = hdrs + [":" + fbs_name + "_srcs"],
        strip_include_prefix = ".",
        include_prefix = include_prefix,
        deps = [":" + fbs_name] + list(deps),
        visibility = visibility,
    )
    _flatc_py_tree(
        name = name + "_py_gen",
        srcs = srcs,
        schemas = ":" + schemas_name,
        topics = topics_py,
        namespace = include_prefix,
        visibility = ["//visibility:private"],
    )
    py_library(
        name = name + "_py",
        deps = [":" + name + "_py_gen", "@pypi//flatbuffers"] + [_dep_target(dep, "_py") for dep in deps],
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
    **kwargs
):
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
      **kwargs: passed to the cc_binary.
    """
    if not node_target:
        node_target = str(native.package_relative_label(name))
        if node_target.startswith("@@//"):
            node_target = node_target[2:]
    node_dep = Label("//talOS/node")
    bin_name = name + "_bin"
    cc_binary(
        name = bin_name,
        srcs = srcs,
        deps = deps + [node_dep],
        local_defines = [
            'TALOS_NODE_NAME=\\"%s\\"' % node_name,
            'TALOS_NODE_TARGET=\\"%s\\"' % node_target,
        ],
        visibility = ["//visibility:private"],
        **kwargs
    )
    talos_node(
        name = name,
        binary = ":" + bin_name,
        node_name = node_name,
        node_target = node_target,
        language = "cpp",
        visibility = visibility if visibility != None else ["//visibility:public"],
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
    **kwargs
):
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
        visibility = ["//visibility:private"],
        **kwargs
    )

    talos_node(
        name = name,
        binary = ":" + bin_name,
        node_name = node_name,
        node_target = node_target,
        language = "python",
        visibility = visibility if visibility != None else ["//visibility:public"],
    )

def _talos_bundle_impl(ctx):
    bundle_dir = ctx.actions.declare_directory(ctx.label.name)
    run_script = ctx.actions.declare_file(ctx.label.name + ".sh")

    launcher_exec = ctx.executable.launcher
    launcher_runfiles = ctx.attr.launcher[DefaultInfo].default_runfiles.files

    direct_inputs = [launcher_exec, ctx.file.config]
    direct_inputs.extend(ctx.files.subsystems)
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
            raw_target = str(node.label)
            if raw_target.startswith("@@//"):
                n_target = raw_target[2:]
            elif raw_target.startswith("@//"):
                n_target = raw_target[1:]
            else:
                n_target = raw_target
            if node.label.name == "node":
                n_name = node.label.package.split("/")[-1]
            else:
                n_name = node.label.name
            n_lang = "cpp"

        direct_inputs.append(n_exec)
        rf_list = []
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

        raw_label = str(node.label)
        clean_label = raw_label
        if clean_label.startswith("@@//"):
            clean_label = clean_label[2:]
        elif clean_label.startswith("@//"):
            clean_label = clean_label[1:]

        nodes_spec.append({
            "name": n_name,
            "target": n_target,
            "label": clean_label,
            "language": n_lang,
            "src_path": n_exec.path,
            "runfiles": rf_list,
        })

    subsystems_spec = []
    for sub in ctx.files.subsystems:
        if sub.basename.endswith(".toml"):
            sub_name = sub.basename[:-5]
        else:
            sub_name = sub.basename
        subsystems_spec.append({
            "src_path": sub.path,
            "name": sub_name,
        })

    spec = {
        "bundle_name": ctx.label.name,
        "bundle_version": 1,
        "launcher_path": launcher_exec.path,
        "config_path": ctx.file.config.path,
        "subsystems": subsystems_spec,
        "nodes": nodes_spec,
    }

    spec_file = ctx.actions.declare_file(ctx.label.name + "_spec.json")
    ctx.actions.write(
        output = spec_file,
        content = json.encode(spec),
    )

    builder_tool = ctx.file._builder
    direct_inputs.extend([spec_file, builder_tool])

    ctx.actions.run_shell(
        outputs = [bundle_dir],
        inputs = depset(direct = direct_inputs, transitive = transitive_inputs),
        command = 'python3 "$1" --spec "$2" --out "$3"',
        arguments = [
            builder_tool.path,
            spec_file.path,
            bundle_dir.path,
        ],
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

talos_bundle = rule(
    implementation = _talos_bundle_impl,
    executable = True,
    attrs = {
        "nodes": attr.label_list(
            mandatory = True,
            doc = "List of node labels providing TalosNodeInfo or DefaultInfo",
        ),
        "config": attr.label(
            mandatory = True,
            allow_single_file = True,
            doc = "The robot.toml config file",
        ),
        "subsystems": attr.label_list(
            allow_files = True,
            default = [],
            doc = "List of subsystem TOML files",
        ),
        "launcher": attr.label(
            default = Label("//talOS/launcher:launcher"),
            executable = True,
            cfg = "target",
            doc = "The launcher binary",
        ),
        "_builder": attr.label(
            default = Label("//talOS/build:bundle_builder.py"),
            allow_single_file = True,
        ),
    },
)

def talos_graph_test(
    name,
    config,
    nodes,
    data = [],
    **kwargs
):
    """Probes every node in `config` with `launcher --describe-only` and lints the graph.

    Args:
      name: test target name.
      config: the robot.toml file or label.
      nodes: the node binaries `config` names.
      data: additional runtime files (e.g., subsystem .toml files or filegroups).
      **kwargs: passed to py_test.
    """
    config_label = native.package_relative_label(config)
    config_rel = (config_label.package + "/" + config_label.name) if config_label.package else config_label.name
    runner_py = name + "_graph_test.py"
    content = (
        "import json\n" +
        "import os\n" +
        "import subprocess\n" +
        "import tempfile\n" +
        "import unittest\n" +
        "from python.runfiles import runfiles\n\n\n" +
        "class GraphTest(unittest.TestCase):\n" +
        "    def test_graph(self):\n" +
        "        r = runfiles.Create()\n" +
        "        launcher = r.Rlocation(\"_main/talOS/launcher/launcher\")\n" +
        "        self.assertTrue(launcher and os.path.exists(launcher), f\"missing launcher: {launcher}\")\n" +
        "        runfiles_root = os.environ.get(\"RUNFILES_DIR\")\n" +
        "        if not runfiles_root or not os.path.isdir(os.path.join(runfiles_root, \"_main\")):\n" +
        "            runfiles_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launcher))))\n" +
        "        main_dir = os.path.join(runfiles_root, \"_main\")\n\n" +
        "        with tempfile.TemporaryDirectory() as tmp:\n" +
        "            work_dir = os.path.join(tmp, \"ws\")\n" +
        "            out_dir = os.path.join(tmp, \"out\")\n" +
        "            os.makedirs(work_dir)\n" +
        "            os.makedirs(out_dir)\n" +
        "            bazel_bin = os.path.join(work_dir, \"bazel-bin\")\n" +
        "            os.makedirs(bazel_bin)\n" +
        "            for entry in os.listdir(main_dir):\n" +
        "                src = os.path.join(main_dir, entry)\n" +
        "                os.symlink(src, os.path.join(work_dir, entry))\n" +
        "                os.symlink(src, os.path.join(bazel_bin, entry))\n" +
        "            if not os.path.exists(os.path.join(bazel_bin, \"external\")):\n" +
        "                os.symlink(runfiles_root, os.path.join(bazel_bin, \"external\"))\n\n" +
        "            env = dict(os.environ, **r.EnvVars())\n" +
        "            env[\"RUNFILES_DIR\"] = runfiles_root\n" +
        "            env.pop(\"PYTHONPATH\", None)\n" +
        "            env.pop(\"BUILD_WORKSPACE_DIRECTORY\", None)\n\n" +
        "            proc = subprocess.run(\n" +
        "                [launcher, \"--config\", \"%s\", \"--describe-only\", \"--output-dir\", out_dir],\n" % config_rel +
        "                cwd=work_dir,\n" +
        "                env=env,\n" +
        "                capture_output=True,\n" +
        "                text=True,\n" +
        "                timeout=120,\n" +
        "            )\n" +
        "            self.assertEqual(proc.returncode, 0, proc.stdout + \"\\n\" + proc.stderr)\n" +
        "            with open(os.path.join(out_dir, \"graph.json\"), \"r\") as f:\n" +
        "                graph = json.load(f)\n" +
        "            for node in graph.get(\"nodes\", []):\n" +
        "                self.assertTrue(\n" +
        "                    node.get(\"described\"),\n" +
        "                    f\"node {node.get('name')} ({node.get('target')}) failed --describe: {node.get('describe_error')}\",\n" +
        "                )\n\n\n" +
        "if __name__ == \"__main__\":\n" +
        "    unittest.main()\n"
    )
    _heredoc_genrule(name + "_runner_gen", runner_py, content)
    seen = {}
    all_data = []
    for item in [config, "//talOS/launcher:launcher", "//talOS/bridge:hardware_node"] + list(nodes) + list(data):
        lbl = native.package_relative_label(item)
        if lbl not in seen:
            seen[lbl] = True
            all_data.append(lbl)
    size = kwargs.pop("size", "small")
    py_test(
        name = name,
        size = size,
        srcs = [runner_py],
        main = runner_py,
        data = all_data,
        legacy_create_init = False,
        deps = [Label("@rules_python//python/runfiles")],
        **kwargs
    )


