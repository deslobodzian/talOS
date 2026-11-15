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
# given a folder that exists (to answer its pages with this command).
_NOT_BUILT = "NOT_BUILT.txt"
_NOT_BUILT_TEXT = "The Studio pages are not built: cd talos && bazel build //studio:web_dist"

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
    rctx.file("BUILD.bazel", "\n".join([
        "# The pages at %s%s." % (path, "" if built else ", not built yet"),
        "filegroup(",
        "    name = \"files\",",
        "    srcs = %s," % ("glob([\"dist/**\"])" if built else "[]"),
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
    doc = "The folder of Studio's built pages as `:files`, empty until both " +
          "index.html and drive.html exist there.",
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
                  ("[ -e \"$out/index.html\" ] || echo '%s' > \"$out/%s\"" % (_NOT_BUILT_TEXT, _NOT_BUILT)),
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
