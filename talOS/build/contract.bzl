"""Talos contract build rules and macros for FlatBuffers and multi-language messages."""

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("@rules_python//python:py_info.bzl", "PyInfo")

def _flatc_py_tree_impl(ctx):
    # One directory output, so the generated modules (one per struct, plus a
    # package __init__.py per namespace) need not be listed: what flatc emits
    # is decided by the schema, which a macro cannot read.
    out = ctx.actions.declare_directory(ctx.label.name)
    ctx.actions.run_shell(
        inputs = ctx.files.srcs,
        tools = [ctx.executable._flatc],
        outputs = [out],
        command = '"$1" -p -o "$2" "${@:3}"',
        arguments = [ctx.executable._flatc.path, out.path] +
                    [f.path for f in ctx.files.srcs],
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
        "_flatc": attr.label(
            default = Label("@flatbuffers//:flatc"),
            executable = True,
            cfg = "exec",
        ),
    },
)

def talos_messages(
    name,
    srcs,
    hdrs = [],
    include_prefix = None,
    deps = [],
    visibility = ["//visibility:public"]):
    """A package's FlatBuffers messages, for C++ and Python.

    Provides:
      :<name>     cc_library. #include "<prefix>/<fbs_basename>_generated.h",
                  plus any `hdrs` (topic constants) as "<prefix>/<hdr>".
                  <prefix> defaults to the package's basename.
      :<name>_py  py_library whose modules import as <namespace>.<Struct>.

    Messages are .fbs structs with `namespace <package>;` -- never talos.*,
    which would collide with the `talos` framework package in Python.

    Args:
      name: the cc_library; the py_library is name + "_py".
      srcs: .fbs schemas.
      hdrs: extra C++ headers shipped with the messages (topic names).
      include_prefix: overrides the include prefix.
      deps: extra cc deps for `hdrs`.
      visibility: of both libraries.
    """
    if include_prefix == None:
        include_prefix = native.package_name().split("/")[-1]
    fbs_name = name + "_fbs"
    flatbuffer_cc_library(
        name = fbs_name,
        srcs = srcs,
        # flatbuffers' DEFAULT_FLATC_ARGS plus --scoped-enums: a schema enum
        # `enum IntakeCommand : ubyte { Idle, Intake }` is a C++ enum class,
        # spelled IntakeCommand::Intake.
        flatc_args = [
            "--gen-object-api",
            "--gen-compare",
            "--no-includes",
            "--gen-mutable",
            "--reflect-names",
            "--cpp-ptr-type flatbuffers::unique_ptr",
            "--scoped-enums",
        ],
        visibility = ["//visibility:private"],
    )
    cc_library(
        name = name,
        hdrs = hdrs + [":" + fbs_name + "_srcs"],
        strip_include_prefix = ".",
        include_prefix = include_prefix,
        deps = [":" + fbs_name] + deps,
        visibility = visibility,
    )
    _flatc_py_tree(
        name = name + "_py_gen",
        srcs = srcs,
        visibility = ["//visibility:private"],
    )
    py_library(
        name = name + "_py",
        deps = [":" + name + "_py_gen", "@pypi//flatbuffers"],
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
        "contract": "optional contract target or metadata",
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
        contract = getattr(ctx.attr, "contract", None),
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
        "contract": attr.label(mandatory = False),
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
        node_target = "//" + native.package_name() + ":" + name
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
        node_target = "//" + native.package_name() + ":" + name
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
    direct_inputs.extend(ctx.files.shared_libs)
    transitive_inputs = [launcher_runfiles]

    nodes_spec = []
    for node in ctx.attr.nodes:
        n_contract = ""
        if TalosNodeInfo in node:
            info = node[TalosNodeInfo]
            n_name = info.node_name
            n_target = info.node_target
            n_exec = info.executable
            n_rf = info.runfiles
            n_lang = info.language
            if info.contract:
                n_contract = str(info.contract.label)
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
            "contract": n_contract,
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

    shared_libs_spec = [lib.path for lib in ctx.files.shared_libs]

    spec = {
        "bundle_name": ctx.label.name,
        "bundle_version": 1,
        "launcher_path": launcher_exec.path,
        "config_path": ctx.file.config.path,
        "subsystems": subsystems_spec,
        "shared_lib_paths": shared_libs_spec,
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
        "shared_libs": attr.label_list(
            allow_files = True,
            default = [Label("//talOS/node_api:libtalos_node.so")],
            doc = "Shared libraries needed at runtime",
        ),
        "_builder": attr.label(
            default = Label("//talOS/build:bundle_builder.py"),
            allow_single_file = True,
        ),
    },
)


