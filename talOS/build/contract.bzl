"""Talos contract build rules and macros for FlatBuffers and multi-language messages."""

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")

def flatbuffer_py_library(
    name,
    srcs,
    outs = [],
    visibility = None,
    deps = None,
):
    """Compiles FlatBuffers schemas to Python bindings using flatc and exports them as a py_library."""
    if not outs:
        inferred = []
        for s in srcs:
            s_base = s.split("/")[-1]
            if "intake" in s_base:
                inferred.extend([
                    "talos/intake/__init__.py",
                    "talos/intake/IntakeTarget.py",
                    "talos/intake/IntakeState.py",
                ])
            elif "drive" in s_base:
                inferred.extend([
                    "talos/drive/__init__.py",
                    "talos/drive/ChassisTarget.py",
                    "talos/drive/DrivetrainState.py",
                ])
        outs = inferred

    if deps == None:
        deps = [Label("//talOS/ipc/python:rtms_py")]

    genrule_name = name + "_flatc"
    native.genrule(
        name = genrule_name,
        srcs = srcs,
        outs = outs,
        tools = ["@flatbuffers//:flatc"],
        cmd = "$(location @flatbuffers//:flatc) -p -o $(@D) $(SRCS) && rm -f $(@D)/talos/__init__.py",
        visibility = ["//visibility:private"],
    )
    py_library(
        name = name,
        srcs = [":" + genrule_name],
        imports = ["."],
        deps = deps,
        visibility = visibility,
    )

def talos_contract_cc(
    name = "contract",
    node_name = None,
    schema = None,
    fbs_srcs = [],
    headers = [],
    packet_hdr = None,
    extra_deps = [],
    visibility = ["//visibility:public"],
    **kwargs
):
    """Standard C++ contract / messages_cc library for a talOS node."""
    all_fbs = list(fbs_srcs)
    if schema:
        if type(schema) == "list":
            all_fbs.extend(schema)
        else:
            all_fbs.append(schema)

    all_hdrs = list(headers)
    if packet_hdr:
        if packet_hdr not in all_hdrs:
            all_hdrs.append(packet_hdr)
    if not all_hdrs:
        all_hdrs = ["packet.h"]

    if not node_name:
        pkg = native.package_name()
        if pkg:
            node_name = pkg.split("/")[-1]
        elif all_fbs:
            first_fbs = all_fbs[0]
            base = first_fbs.split("/")[-1]
            for suffix in ["_message.fbs", "-message.fbs", ".fbs"]:
                if base.endswith(suffix):
                    base = base[:-len(suffix)]
                    break
            node_name = base
        else:
            node_name = name

    fbs_target_name = name + "_fbs"
    if all_fbs:
        flatbuffer_cc_library(
            name = fbs_target_name,
            srcs = all_fbs,
            visibility = ["//visibility:public"],
        )
        cc_library(
            name = name,
            hdrs = all_hdrs + [":" + fbs_target_name + "_srcs"],
            strip_include_prefix = ".",
            include_prefix = "talos_nodes/" + node_name,
            deps = [
                ":" + fbs_target_name,
                Label("//talOS/hardware:hardware"),
            ] + extra_deps,
            visibility = visibility,
            **kwargs
        )
    else:
        cc_library(
            name = name,
            hdrs = all_hdrs,
            strip_include_prefix = ".",
            include_prefix = "talos_nodes/" + node_name,
            deps = [
                Label("//talOS/hardware:hardware"),
            ] + extra_deps,
            visibility = visibility,
            **kwargs
        )

    if name != "messages_cc":
        native.alias(
            name = "messages_cc",
            actual = ":" + name,
            visibility = visibility,
        )
    if name != "contract":
        native.alias(
            name = "contract",
            actual = ":" + name,
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
    node_name = None,
    factory = None,
    hdrs = [],
    contract = None,
    deps = [],
    hardware = True,
    node_target = None,
    visibility = None,
    optional_sources = [],
    default_config_path = None,
    **kwargs
):
    """Standard C++ entry point macro for a talOS node.

    Generates a main C++ source file using a genrule and defines a cc_binary.

    optional_sources lists C++ topic expressions (e.g.
    "talos::arbiter::kAutoChassisTopic") declared as optional graph endpoints:
    the linter stays silent about their missing publisher. Only hardware-free
    nodes support it; the hardware runner takes no endpoint attributes.
    default_config_path provides a default --config for standalone modules.
    NodeSpec carries no framework default (empty means missing), so nodes
    without one fail fast unless the caller passes --config.
    """
    if not node_name:
        node_name = name
    if not node_target:
        node_target = "//" + native.package_name() + ":" + name
    if not factory:
        fail("factory must be specified for talos_cc_node")
    if not hdrs:
        hdrs = ["node.h"]
    if optional_sources and hardware:
        fail("optional_sources is only supported for hardware-free nodes")

    main_cc = name + "_main.cc"
    genrule_name = name + "_main_gen"

    includes = []
    for h in hdrs:
        hdr_path = h[1:] if h.startswith(":") else h
        if hdr_path.startswith("<") or hdr_path.startswith('"'):
            includes.append("#include " + hdr_path)
        else:
            includes.append('#include "%s"' % hdr_path)

    includes_str = "\n".join(includes)
    if includes_str:
        includes_str += "\n"

    if hardware:
        # RunHardwareNode takes no NodeSpec, so a standalone default config
        # is appended to a copy of argv: every original flag is preserved and
        # an explicit --config still wins because the default is only added
        # when no --config flag is present.
        hw_default = ""
        if default_config_path != None:
            hw_default = (
                '  bool has_config = false;\n' +
                '  for (int i = 1; i < argc; ++i) {\n' +
                '    if (std::string{argv[i]} == "--config") { has_config = true; break; }\n' +
                '  }\n' +
                '  std::vector<std::string> default_arg_storage;\n' +
                '  std::vector<char*> default_args;\n' +
                '  int use_argc = argc;\n' +
                '  char** use_argv = argv;\n' +
                '  if (!has_config) {\n' +
                '    default_arg_storage.reserve(static_cast<std::size_t>(argc) + 2);\n' +
                '    for (int i = 0; i < argc; ++i) default_arg_storage.emplace_back(argv[i]);\n' +
                '    default_arg_storage.emplace_back("--config");\n' +
                '    default_arg_storage.emplace_back("%s");\n' % default_config_path +
                '    for (auto& s : default_arg_storage) default_args.push_back(s.data());\n' +
                '    default_args.push_back(nullptr);\n' +
                '    use_argc = static_cast<int>(default_args.size()) - 1;\n' +
                '    use_argv = default_args.data();\n' +
                '  }\n'
            )
        hw_argv = "use_argv" if default_config_path != None else "argv"
        hw_argc = "use_argc" if default_config_path != None else "argc"
        hw_extra_include = '#include <vector>\n' if default_config_path != None else ""
        content = (
            includes_str +
            '#include <string>\n' + hw_extra_include +
            '#include "talOS/process/hardware_main.h"\n\n' +
            'using namespace talos;\n\n' +
            'int main(int argc, char** argv) {\n' +
            hw_default +
            '  return talos::process::RunHardwareNode(\n' +
            '      %s, %s, "%s", "%s",\n' % (hw_argc, hw_argv, node_name, node_target) +
            '      [](auto& loop, const talos::process::HardwareSubsystem& sub, bool simulation) {\n' +
            '        return %s(loop, sub, simulation);\n' % factory +
            '      });\n' +
            '}\n'
        )
    else:
        endpoints_decl = ""
        endpoints_arg = ""
        names_include = ""
        if optional_sources:
            names_include = '#include "talOS/introspection/names.h"\n'
            endpoint_rows = "\n".join(
                ["    {%s, talos::introspect::naming::kSourceFlagOptional}," % e
                 for e in optional_sources],
            )
            endpoints_decl = (
                "const std::vector<talos::introspect::EndpointAttribute> " +
                "kTalosOptionalEndpoints = {\n" + endpoint_rows + "\n};\n"
            )
            endpoints_arg = "\n          .endpoints = kTalosOptionalEndpoints,"
        default_config_arg = ""
        if default_config_path != None:
            default_config_arg = (
                '\n          .default_config_path = "%s",' % default_config_path
            )
        content = (
            includes_str +
            '#include "talOS/process/node_main.h"\n' +
            names_include + '\n' +
            'using namespace talos;\n\n' +
            endpoints_decl + '\n' +
            'int main(int argc, char** argv) {\n' +
            '  return talos::process::RunNode(\n' +
            '      argc, argv,\n' +
            '      {.node_name = "%s", .node_target = "%s",%s%s},\n'
            % (node_name, node_target, endpoints_arg, default_config_arg) +
            '      [](auto& loop, const talos::process::NodeFlags& flags) {\n' +
            '        return %s(loop, flags);\n' % factory +
            '      });\n' +
            '}\n'
        )

    native.genrule(
        name = genrule_name,
        outs = [main_cc],
        cmd = "cat << 'EOF' > $@\n" + content + "EOF\n",
        visibility = ["//visibility:private"],
    )

    all_deps = list(deps)
    if contract:
        all_deps.append(contract)
    runner_dep = "@talos//talOS/process:hardware_main" if hardware else "@talos//talOS/process:node_main"
    if runner_dep not in all_deps and runner_dep.replace("@talos//", "//") not in all_deps:
        all_deps.append(runner_dep)
    if optional_sources:
        names_dep = "@talos//talOS/introspection:names"
        if names_dep not in all_deps and "//talOS/introspection:names" not in all_deps:
            all_deps.append(names_dep)

    bin_name = name + "_bin"
    cc_binary(
        name = bin_name,
        srcs = [main_cc] + hdrs,
        deps = all_deps,
        includes = ["."] if native.package_name() else [],
        visibility = ["//visibility:private"],
        **kwargs
    )

    talos_node(
        name = name,
        binary = ":" + bin_name,
        node_name = node_name,
        node_target = node_target,
        language = "cpp",
        contract = contract,
        visibility = visibility if visibility != None else ["//visibility:public"],
    )

def talos_py_node(
    name,
    node_name = None,
    entry_point = None,
    srcs = [],
    contract = None,
    describe_sources = None,
    default_config = None,
    deps = [],
    data = [],
    node_target = None,
    visibility = None,
    **kwargs
):
    """Standard Python entry point macro for a talOS node.

    Generates a runner script via genrule and defines a py_binary.
    """
    if not node_name:
        node_name = name
    if not node_target:
        node_target = "//" + native.package_name() + ":" + name
    if not entry_point:
        fail("entry_point must be specified for talos_py_node")

    runner_py = name + "_runner.py"
    genrule_name = name + "_runner_gen"

    describe_import = ""
    describe_expr = "None"
    if describe_sources:
        if ":" in describe_sources:
            mod_part, fn_part = describe_sources.split(":")
            describe_import = "from %s import %s as _describe_sources_fn\n" % (mod_part, fn_part)
            describe_expr = "_describe_sources_fn"
        elif "." in describe_sources:
            mod_part, fn_part = describe_sources.rsplit(".", 1)
            describe_import = "from %s import %s as _describe_sources_fn\n" % (mod_part, fn_part)
            describe_expr = "_describe_sources_fn"
        else:
            describe_expr = describe_sources

    default_cfg_expr = '""'
    if default_config:
        default_cfg_expr = '"%s"' % default_config

    content = (
        'from talos_runtime import runner\n' +
        describe_import +
        'import sys\n\n' +
        'def main(argv=None):\n' +
        '    return runner.run_node_main(\n' +
        '        node_name = "%s",\n' % node_name +
        '        node_target = "%s",\n' % node_target +
        '        entry_point = "%s",\n' % entry_point +
        '        describe_sources_fn = %s,\n' % describe_expr +
        '        default_config = %s,\n' % default_cfg_expr +
        '        argv = argv,\n' +
        '    )\n\n' +
        'if __name__ == "__main__":\n' +
        '    sys.exit(main())\n'
    )

    native.genrule(
        name = genrule_name,
        outs = [runner_py],
        cmd = "cat << 'EOF' > $@\n" + content + "EOF\n",
        visibility = ["//visibility:private"],
    )

    all_deps = list(deps)
    if contract:
        all_deps.append(contract)
    runtime_dep = "@talos//talOS/ipc/python:rtms_py"
    if runtime_dep not in all_deps and runtime_dep.replace("@talos//", "//") not in all_deps:
        all_deps.append(runtime_dep)

    bin_name = name + "_bin"
    py_binary(
        name = bin_name,
        srcs = [runner_py] + srcs,
        main = runner_py,
        legacy_create_init = False,
        deps = all_deps,
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
        contract = contract,
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
        elif clean_label.startswith("@@"):
            clean_label = "@" + clean_label[2:]

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
        if sub.basename == "subsystem.toml":
            sub_name = sub.short_path.split("/")[-2]
        elif sub.basename.endswith(".toml"):
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


