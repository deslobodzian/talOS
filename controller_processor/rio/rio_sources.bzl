"""Collects the RoboRIO Gradle project and talOS sources for staging."""

RioSourcesInfo = provider(
    doc = "Source and header files required to stage controller_processor/rio.",
    fields = {"files": "depset of File objects"},
)

def _rio_sources_aspect_impl(_target, ctx):
    collected = []
    if hasattr(ctx.rule.files, "srcs"):
        collected.extend(ctx.rule.files.srcs)
    if hasattr(ctx.rule.files, "hdrs"):
        collected.extend(ctx.rule.files.hdrs)
    transitive = [
        dep[RioSourcesInfo].files
        for dep in getattr(ctx.rule.attr, "deps", [])
        if RioSourcesInfo in dep
    ]
    return [RioSourcesInfo(files = depset(collected, transitive = transitive))]

_rio_sources_aspect = aspect(
    implementation = _rio_sources_aspect_impl,
    attr_aspects = ["deps"],
)

def _rlocation_and_relpath(f):
    if f.short_path.startswith("../"):
        rlocation = f.short_path[3:]
        _, _, relpath = rlocation.partition("/")
    else:
        relpath = f.short_path
        rlocation = "_main/" + relpath
    return relpath, rlocation

def _in_talos_repo(f, module_short_path):
    if module_short_path.startswith("../"):
        prefix = module_short_path.rpartition("/")[0] + "/"
        return f.short_path.startswith(prefix)
    return not f.short_path.startswith("../")

def _rio_stage_manifest_impl(ctx):
    files = depset(
        ctx.files.srcs,
        transitive = [dep[RioSourcesInfo].files for dep in ctx.attr.talos_deps],
    )
    module_short_path = ctx.file.module_file.short_path
    _, module_rlocation = _rlocation_and_relpath(ctx.file.module_file)
    entries = {}
    talos_files = []
    for f in files.to_list():
        if not _in_talos_repo(f, module_short_path):
            continue
        talos_files.append(f)
        relpath, rlocation = _rlocation_and_relpath(f)
        entries[relpath] = rlocation
    payload = {
        "module_rlocation": module_rlocation,
        "manifest": [[relpath, entries[relpath]] for relpath in sorted(entries)],
    }
    out = ctx.actions.declare_file(ctx.label.name + ".json")
    ctx.actions.write(output = out, content = json.encode_indent(payload) + "\n")
    runfiles = ctx.runfiles(
        files = [out, ctx.file.module_file] + talos_files,
    )
    return [
        DefaultInfo(
            files = depset([out]),
            runfiles = runfiles,
        ),
    ]

rio_stage_manifest = rule(
    implementation = _rio_stage_manifest_impl,
    attrs = {
        "module_file": attr.label(
            allow_single_file = True,
            doc = "The talOS repository's MODULE.bazel.",
            mandatory = True,
        ),
        "srcs": attr.label_list(
            allow_files = True,
            doc = "Files under controller_processor/rio to stage.",
        ),
        "talos_deps": attr.label_list(
            aspects = [_rio_sources_aspect],
            doc = "talOS cc_library targets whose srcs/hdrs GradleRIO compiles.",
        ),
    },
)
