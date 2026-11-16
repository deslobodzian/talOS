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
`[studio] webroot` names it. How a robot repository uses it:
examples/robot_repo/README.md, "Studio".
"""

# Written into the directory while the pages are not built: Bazel leaves an
# empty directory out of a target's runfiles, and the bridge must still be
# given a folder that exists (to answer its pages with this text, which it
# reads from there). studio_web writes it naming the talOS checkout its
# `path` lies in; this one, naming none, only when srcs bring no such file.
_NOT_BUILT = "NOT_BUILT.txt"
_NOT_BUILT_TEXT = ("build them with `bazel build //studio:web_dist` in the talOS " +
                   "checkout this repository uses (see MODULE.bazel's talos override)")

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

def _studio_web_impl(rctx):
    path = rctx.attr.path
    if not path.startswith("/"):
        path = str(rctx.workspace_root) + "/" + path
    folder = rctx.path(path)
    pages = [folder.get_child("index.html"), folder.get_child("drive.html")]

    # Fetched again when a page appears, changes or goes: a build in the
    # talOS checkout rewrites index.html with its new asset names.
    for page in pages:
        rctx.watch(page)
    built = all([page.exists for page in pages])
    if built:
        rctx.symlink(folder, "dist")
    else:
        rctx.file("dist/" + _NOT_BUILT, _not_built_text(rctx, path) + "\n")
    rctx.file("BUILD.bazel", "\n".join([
        "# The pages at %s%s." % (path, "" if built else ", not built yet"),
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
    },
    doc = "The folder of Studio's built pages as `:files`; until both " +
          "index.html and drive.html exist there, only NOT_BUILT.txt, naming " +
          "the talOS checkout to build them in.",
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
          "only NOT_BUILT.txt while they are not built), the webroot the " +
          "Studio bridge serves.",
)
