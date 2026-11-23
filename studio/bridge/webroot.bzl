"""Studio's pages for a repository that depends on talOS.

Studio's pages (`index.html` at `/`, `drive.html`, the driver dashboard, at
`/drive`, and their assets) are built by `@talos//studio:web_dist`, which
needs rules_js, a development dependency of talOS that a dependent module
does not get. A robot repository therefore takes the folder that build
leaves in its talOS checkout, read from outside Bazel's package tree:

    # MODULE.bazel
    studio_web = use_repo_rule("@talos//studio/bridge:webroot.bzl", "studio_web")
    studio_web(name = "studio_web", path = "talos/bazel-bin/studio/dist")

    # BUILD.bazel
    load("@talos//studio/bridge:webroot.bzl", "studio_webroot")
    studio_webroot(name = "studio_webroot", srcs = ["@studio_web//:files"])

`//:studio_webroot` is then one directory, `bazel-bin/studio_webroot`, for a
`talos_sim`'s `data` and `talos_bundle`'s `studio_webroot`, and the robot's
`[studio] webroot` names it. `//studio:web_dist` writes `STUDIO_STAMP.txt`
holding the SHA-256 of Studio's web inputs and `talOS/telemetry/telemetry.fbs`;
when built pages are present with a missing or mismatched stamp, `studio_web`
emits `STALE.txt` and `studio_webroot`'s build action fails closed naming the
expected and actual stamps and the rebuild command. How a robot repository uses
it: examples/robot_repo/README.md, "Studio".
"""

# Written into the directory while the pages are not built: Bazel leaves an
# empty directory out of a target's runfiles, and the bridge must still be
# given a folder that exists (to answer its pages with this text, which it
# reads from there). studio_web writes it naming the talOS checkout its
# `path` lies in; this one, naming none, only when srcs bring no such file.
_NOT_BUILT = "NOT_BUILT.txt"
_NOT_BUILT_TEXT = ("build them with `bazel build //studio:web_dist` in the talOS " +
                   "checkout this repository uses (see MODULE.bazel's talos override)")

# Content stamp written by //studio:web_dist and checked by studio_web against
# the pinned @talos checkout.
_STAMP = "STUDIO_STAMP.txt"

# Written by studio_web instead of symlinking `dist` when built pages exist but
# STUDIO_STAMP.txt is missing or does not match the pinned @talos inputs.
# studio_webroot's build action fails closed printing this file.
_STALE = "STALE.txt"

# What `//studio:web_dist` leaves in a talOS checkout, below its root.
_DIST = "/bazel-bin/studio/dist"

def _not_built_text(rctx, path):
    """The build command naming the talOS checkout that writes `path`.

    The checkout is named when `path` is an existing checkout's
    bazel-bin/studio/dist; otherwise the text points at MODULE.bazel.
    """
    if path.endswith(_DIST):
        checkout = rctx.path(path[:-len(_DIST)])
        if checkout.exists:
            root = str(checkout.realpath)
            return ("build them with `bazel build //studio:web_dist` in the talOS " +
                    "checkout at %s: `(cd %s && bazel build //studio:web_dist)`") % (root, root)
    return _NOT_BUILT_TEXT + ", which writes %s" % path

def _rebuild_command(rctx, path, talos_root):
    """The exact shell command to rebuild Studio's pages."""
    if path.endswith(_DIST):
        checkout = rctx.path(path[:-len(_DIST)])
        if checkout.exists:
            return "(cd %s && bazel build //studio:web_dist)" % str(checkout.realpath)
    return "(cd %s && bazel build //studio:web_dist)" % str(talos_root.realpath)

def _parse_stamp_line(text):
    """Return the first non-empty stripped line of a stamp file, or empty."""
    for line in text.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped
    return ""

def _watch_and_stamp_talos(rctx, talos_root, stamp_tool):
    """Watch Studio's web inputs under `talos_root` and return their SHA-256."""
    python = rctx.which("python3") or rctx.which("python")
    if not python:
        fail("studio_web: python3 not found on PATH to compute Studio webroot stamp")
    rctx.watch(stamp_tool)
    listed = rctx.execute([python, str(stamp_tool), "--root", str(talos_root), "--list"])
    if listed.return_code != 0:
        fail("studio_web: failed to list Studio inputs under %s: %s" % (talos_root, listed.stderr))

    watched_dirs = {"studio/src": True}
    for rel in listed.stdout.splitlines():
        if not rel:
            continue
        rctx.watch(talos_root.get_child(rel))
        if rel.startswith("studio/src/"):
            parts = rel.split("/")
            for i in range(2, len(parts)):
                watched_dirs["/".join(parts[:i])] = True
    for rel_dir in watched_dirs:
        rctx.watch(talos_root.get_child(rel_dir))

    stamped = rctx.execute([python, str(stamp_tool), "--root", str(talos_root), "--hash-only"])
    if stamped.return_code != 0:
        fail("studio_web: failed to compute Studio stamp under %s: %s" % (talos_root, stamped.stderr))
    expected = _parse_stamp_line(stamped.stdout)
    if len(expected) != 64:
        fail("studio_web: unexpected stamp from %s: %r" % (stamp_tool, stamped.stdout))
    return expected

def _studio_web_impl(rctx):
    talos_root = rctx.path(rctx.attr._talos_module).dirname
    stamp_tool = rctx.path(rctx.attr._stamp_tool)
    expected_stamp = _watch_and_stamp_talos(rctx, talos_root, stamp_tool)

    path = rctx.attr.path
    if not path.startswith("/"):
        path = str(rctx.workspace_root) + "/" + path
    folder = rctx.path(path)
    pages = [folder.get_child("index.html"), folder.get_child("drive.html")]
    stamp_file = folder.get_child(_STAMP)

    # Fetched again when a page or stamp appears, changes or goes: a build in
    # the talOS checkout rewrites index.html, drive.html and STUDIO_STAMP.txt.
    for page in pages:
        rctx.watch(page)
    rctx.watch(stamp_file)
    built = all([page.exists for page in pages])
    if not built:
        rctx.file("dist/" + _NOT_BUILT, _not_built_text(rctx, path) + "\n")
        state = ", not built yet"
    else:
        actual_stamp = _parse_stamp_line(rctx.read(stamp_file)) if stamp_file.exists else ""
        if actual_stamp == expected_stamp:
            rctx.symlink(folder, "dist")
            state = ""
        else:
            actual_desc = actual_stamp if actual_stamp else "(missing %s)" % _STAMP
            rebuild = _rebuild_command(rctx, path, talos_root)
            rctx.file("dist/" + _STALE, "\n".join([
                "studio_webroot: Studio's built pages at %s do not match pinned @talos (%s)." % (
                    path,
                    str(talos_root.realpath),
                ),
                "  expected stamp: %s" % expected_stamp,
                "  actual stamp:   %s" % actual_desc,
                "  rebuild with:   %s" % rebuild,
                "",
            ]))
            state = ", stale for pinned @talos"
    rctx.file("BUILD.bazel", "\n".join([
        "# The pages at %s%s." % (path, state),
        "filegroup(",
        "    name = \"files\",",
        "    srcs = glob([\"dist/**\"]),",
        "    visibility = [\"//visibility:public\"],",
        ")",
        "",
    ]))

studio_web = repository_rule(
    implementation = _studio_web_impl,
    attrs = {
        "path": attr.string(
            mandatory = True,
            doc = "The built pages' folder (what `//studio:web_dist` leaves in " +
                  "a talOS checkout, `bazel-bin/studio/dist` there), absolute " +
                  "or relative to the root module's directory.",
        ),
        "_stamp_tool": attr.label(
            default = Label("//studio/bridge:webroot_stamp.py"),
        ),
        "_talos_module": attr.label(
            default = Label("//:MODULE.bazel"),
        ),
    },
    doc = "The folder of Studio's built pages as `:files`; until both " +
          "index.html and drive.html exist there, only NOT_BUILT.txt, naming " +
          "the talOS checkout to build them in; when built pages exist with a " +
          "missing or mismatched STUDIO_STAMP.txt, only STALE.txt so " +
          "studio_webroot fails closed.",
)

def _studio_webroot_impl(ctx):
    out = ctx.actions.declare_directory(ctx.label.name)
    args = ctx.actions.args()
    args.add(out.path)
    for f in ctx.files.srcs:
        prefix = f.owner.workspace_root + "/dist/"
        if not f.path.startswith(prefix):
            fail("studio_webroot: %s is not a studio_web repository's page" % f.path)
        args.add(f.path)
        args.add(f.path[len(prefix):])
    ctx.actions.run_shell(
        inputs = ctx.files.srcs,
        outputs = [out],
        arguments = [args],
        command = "out=\"$1\"; shift; mkdir -p \"$out\"; " +
                  "while [ $# -gt 0 ]; do " +
                  ("if [ \"$2\" = \"{s}\" ]; then cat \"$1\" >&2; exit 1; fi; ").format(s = _STALE) +
                  "mkdir -p \"$out/$(dirname \"$2\")\" && cp \"$1\" \"$out/$2\" || exit 1; " +
                  "shift 2; done; " +
                  ("[ -e \"$out/index.html\" ] || [ -e \"$out/{f}\" ] || " +
                   "echo \"{t}\" > \"$out/{f}\"").format(f = _NOT_BUILT, t = _NOT_BUILT_TEXT.replace("`", "\\`")),
        mnemonic = "StudioWebroot",
        progress_message = "Collecting Studio's pages into %{output}",
    )
    return [DefaultInfo(
        files = depset([out]),
        runfiles = ctx.runfiles(files = [out]),
    )]

studio_webroot = rule(
    implementation = _studio_webroot_impl,
    attrs = {
        "srcs": attr.label_list(
            allow_files = True,
            mandatory = True,
            doc = "A studio_web repository's `:files`.",
        ),
    },
    doc = "Studio's pages as one directory named after the target (holding " +
          "only NOT_BUILT.txt while they are not built, and failing the " +
          "build when STUDIO_STAMP.txt is stale), the webroot the Studio " +
          "bridge serves.",
)

def _stamped_web_dist_impl(ctx):
    out = ctx.actions.declare_directory(ctx.attr.out_dir)
    ctx.actions.run_shell(
        inputs = [ctx.file.dist, ctx.file.stamp],
        outputs = [out],
        arguments = [ctx.file.dist.path, ctx.file.stamp.path, out.path],
        command = "mkdir -p \"$3\" && cp -RL \"$1/.\" \"$3/\" && cp -L \"$2\" \"$3/" + _STAMP + "\"",
        mnemonic = "StudioWebDistStamp",
        progress_message = "Stamping Studio's built pages into %{output}",
    )
    return [DefaultInfo(
        files = depset([out]),
        runfiles = ctx.runfiles(files = [out]),
    )]

stamped_web_dist = rule(
    implementation = _stamped_web_dist_impl,
    attrs = {
        "dist": attr.label(
            allow_single_file = True,
            mandatory = True,
            doc = "The Vite-built Studio output directory.",
        ),
        "out_dir": attr.string(
            default = "dist",
            doc = "Output directory name (default: dist).",
        ),
        "stamp": attr.label(
            allow_single_file = True,
            mandatory = True,
            doc = "The STUDIO_STAMP.txt file produced by webroot_stamp.py.",
        ),
    },
    doc = "Combines Studio's Vite output directory with STUDIO_STAMP.txt into one output directory.",
)
