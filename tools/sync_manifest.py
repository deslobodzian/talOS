"""Writes the Bazel view of a robot config: `<stem>_manifest.bzl` beside it.

    bazel run //tools:sync_manifest -- <robot.toml>...
    bazel run //tools:sync_manifest -- --check <robot.toml>...

A robot config's `[[subsystems]]` list is the only list of a robot's nodes.
`talos_bundle` and `talos_graph_test` need the same nodes, and every file the
config reads, as Bazel labels. The C++ configuration resolver supplies the
effective configuration and dependency closure; this script only turns its
JSON output into two constants:

    NODES         every node label the config launches, sorted
    CONFIG_FILES  every file the config reads besides itself, as labels, sorted

`--check` writes nothing; it exits 1 with a diff and the regenerate command
when a manifest is stale. `manifest_sync_test` (tools/sync_manifest.bzl) runs
it under `bazel test`, where only the files the manifest lists are present, so
a config that reads an unlisted file is reported stale too.
"""

import argparse
import ast
import difflib
import json
import os
import posixpath
import sys
import subprocess
import tempfile

from python.runfiles import runfiles

COMMAND = "bazel run //tools:sync_manifest --"


class ManifestError(Exception):
    """The config cannot be turned into a manifest; the message says why."""


class Stale(Exception):
    """--check: the config reads a file the manifest does not list."""


def manifest_path(config):
    """Workspace-relative path of the manifest generated for `config`."""
    stem = posixpath.splitext(posixpath.basename(config))[0]
    return posixpath.join(posixpath.dirname(config), f"{stem}_manifest.bzl")


def _resolved(root, config, overrides=None, available=None):
    """Call the runtime resolver, supplying pending files without editing disk."""
    resolver = runfiles.Create()
    tool = resolver.Rlocation(
        (resolver.CurrentRepository() or "_main") + "/talOS/configuration/resolve")
    if not tool or not os.path.isfile(tool):
        raise ManifestError("configuration resolver is missing from Bazel runfiles")
    with tempfile.TemporaryDirectory() as overlay:
        for path, content in (overrides or {}).items():
            normalized = posixpath.normpath(path)
            if posixpath.isabs(normalized) or normalized == ".." or normalized.startswith("../"):
                raise ManifestError(f"{path}: pending file leaves the workspace")
            full = os.path.join(overlay, normalized)
            os.makedirs(os.path.dirname(full), exist_ok=True)
            with open(full, "w", encoding="utf-8") as output:
                output.write(content)
        command = [tool, config, overlay]
        if available is not None:
            allowed = os.path.join(overlay, ".available")
            with open(allowed, "w", encoding="utf-8") as output:
                output.write("".join(path + "\n" for path in sorted(available)))
            command.append(allowed)
        result = subprocess.run(command, cwd=root, text=True, capture_output=True)
    if result.returncode:
        message = result.stderr.strip()
        if "UNLISTED:" in message:
            raise Stale(message.split("UNLISTED:", 1)[1])
        raise ManifestError(message)
    return json.loads(result.stdout)


def effective(root, config, overrides=None, available=None):
    """Effective subsystem tables and dependencies from the C++ resolver."""
    result = _resolved(root, config, overrides, available)
    return result["subsystems"], result["files"]


def resolve(root, config, overrides=None, available=None):
    """Sorted node labels and dependencies from the C++ resolver."""
    result = _resolved(root, config, overrides, available)
    return result["nodes"], result["files"]


def package_of(root, path):
    """The Bazel package owning workspace-relative `path`."""
    directory = posixpath.dirname(path)
    while True:
        for build in ("BUILD", "BUILD.bazel"):
            if os.path.isfile(os.path.join(root, directory, build)):
                return directory
        if not directory:
            raise ManifestError(f"{path} is in no Bazel package")
        directory = posixpath.dirname(directory)


def label(package, path):
    return f"//{package}:{path[len(package) + 1:] if package else path}"


def label_path(lbl):
    """Workspace-relative path of a `//pkg:file` label."""
    package, _, name = lbl[2:].partition(":")
    return posixpath.join(package, name)


def render(config, nodes, file_labels):
    def constant(name, values):
        if not values:
            return f"{name} = []\n"
        return (f"{name} = [\n" +
                "".join(f'    "{v}",\n' for v in values) + "]\n")

    name = posixpath.basename(config)
    return (
        f'"""Generated from {name} by //tools:sync_manifest. Do not edit.\n'
        "\n"
        "Regenerate after changing the config:\n"
        "\n"
        f"    {COMMAND} {config}\n"
        '"""\n'
        "\n"
        f"# Every node label {name} launches.\n" +
        constant("NODES", nodes) +
        "\n"
        f"# Every file {name} reads besides itself.\n" +
        constant("CONFIG_FILES", sorted(file_labels)))


def generate(root, config, overrides=None):
    """The manifest text for `config`, labels found from the BUILD files."""
    nodes, files = resolve(root, config, overrides)
    return render(config, nodes,
                  [label(package_of(root, path), path) for path in files])


def listed_labels(text):
    """CONFIG_FILES of an existing manifest, or [] if it has none."""
    try:
        module = ast.parse(text)
    except SyntaxError:
        return []
    for statement in module.body:
        if (isinstance(statement, ast.Assign) and len(statement.targets) == 1
                and getattr(statement.targets[0], "id", None) == "CONFIG_FILES"):
            try:
                return [v for v in ast.literal_eval(statement.value)
                        if isinstance(v, str)]
            except ValueError:
                return []
    return []


def check(root, config):
    """Problems with `config`'s manifest, as text; empty if it is current.

    Labels come from the manifest itself, so this runs without BUILD files
    (in a test's runfiles); a file the manifest does not list is stale.
    """
    out = manifest_path(config)
    fix = f"Regenerate it with:\n    {COMMAND} {config}\n"
    try:
        with open(os.path.join(root, out), encoding="utf-8") as f:
            current = f.read()
    except OSError:
        return f"{out} is missing. {fix}"
    known = {label_path(lbl): lbl for lbl in listed_labels(current)}
    try:
        nodes, files = resolve(root, config, available=set(known))
    except Stale as stale:
        return (f"{out} is stale: {config} reads {stale}, which CONFIG_FILES "
                f"does not list. {fix}")
    expected = render(config, nodes, [known[path] for path in files])
    if expected == current:
        return ""
    diff = "".join(difflib.unified_diff(
        current.splitlines(keepends=True), expected.splitlines(keepends=True),
        f"{out} (checked in)", f"{out} (from {config})"))
    return f"{out} is stale. {fix}\n{diff}"


def write(root, config):
    """Regenerates `config`'s manifest; True if the file changed."""
    text = generate(root, config)
    path = os.path.join(root, manifest_path(config))
    try:
        with open(path, encoding="utf-8") as f:
            if f.read() == text:
                return False
    except OSError:
        pass
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return True


def main(argv):
    parser = argparse.ArgumentParser(
        prog=COMMAND,
        description="Write <stem>_manifest.bzl (NODES, CONFIG_FILES) beside "
        "each robot config, or check that it is current.")
    parser.add_argument("configs", nargs="+", metavar="robot.toml",
                        help="robot config, relative to the workspace root")
    parser.add_argument("--check", action="store_true",
                        help="write nothing; exit 1 if a manifest is stale")
    args = parser.parse_args(argv)

    # `bazel run` sets BUILD_WORKSPACE_DIRECTORY; `bazel test` runs in the
    # runfiles root, which mirrors the workspace layout.
    root = os.environ.get("BUILD_WORKSPACE_DIRECTORY") or os.getcwd()
    status = 0
    for config in args.configs:
        config = posixpath.normpath(config.replace(os.sep, "/"))
        if os.path.isabs(config):
            config = posixpath.relpath(config, root.replace(os.sep, "/"))
        try:
            if args.check:
                if problem := check(root, config):
                    print(problem, file=sys.stderr)
                    status = 1
            elif write(root, config):
                print(f"wrote {manifest_path(config)}")
            else:
                print(f"{manifest_path(config)} is up to date")
        except (ManifestError, Stale) as err:
            print(f"sync_manifest: {config}: {err}", file=sys.stderr)
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
