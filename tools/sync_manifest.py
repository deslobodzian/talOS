"""Writes the Bazel view of a robot config: `<stem>_manifest.bzl` beside it.

    bazel run //tools:sync_manifest -- <robot.toml>...
    bazel run //tools:sync_manifest -- --check <robot.toml>...

A robot config's `[[subsystems]]` list is the only list of a robot's nodes.
`talos_bundle` and `talos_graph_test` need the same nodes, and every file the
config reads, as Bazel labels. Bazel cannot read TOML, so this script reads the
config the way talOS/configuration/config_parser.h does (`include`, entry
`path`/`node`/`defaults`, `[subsystem] node`/`defaults`, single-file
`[subsystems.<name>]` tables) and writes the result as two constants:

    NODES         every node label the config launches, sorted
    CONFIG_FILES  every file the config reads besides itself, as labels, sorted

`--check` writes nothing; it exits 1 with a diff and the regenerate command
when a manifest is stale. `manifest_sync_test` (tools/sync_manifest.bzl) runs
it under `bazel test`, where only the files the manifest lists are present, so
a config that reads an unlisted file is reported stale too.
"""

import argparse
import ast
import copy
import difflib
import os
import posixpath
import sys
import tomllib

COMMAND = "bazel run //tools:sync_manifest --"


class ManifestError(Exception):
    """The config cannot be turned into a manifest; the message says why."""


class Stale(Exception):
    """--check: the config reads a file the manifest does not list."""


def manifest_path(config):
    """Workspace-relative path of the manifest generated for `config`."""
    stem = posixpath.splitext(posixpath.basename(config))[0]
    return posixpath.join(posixpath.dirname(config), f"{stem}_manifest.bzl")


def _join(base_dir, rel, where):
    if posixpath.isabs(rel):
        raise ManifestError(f"{where}: absolute path '{rel}' has no Bazel "
                            "label; use a path relative to the file")
    path = posixpath.normpath(posixpath.join(base_dir, rel))
    if path == ".." or path.startswith("../"):
        raise ManifestError(f"{where}: '{rel}' leaves the workspace")
    return path


_TOP_LEVEL_KEYS = ("include", "robot", "studio", "subsystems")
_ENTRY_KEYS = ("name", "path", "node", "defaults")
_STUDIO_KEYS = ("bridge", "webroot", "http_port", "topic", "pose_topic")


def _check_studio(studio, where):
    if not isinstance(studio, dict):
        raise ManifestError(f"{where}: [studio] must be a table")
    for key in studio:
        if key not in _STUDIO_KEYS:
            raise ManifestError(f"{where}: [studio] unknown key '{key}'")
    if "topic" in studio:
        _non_empty(studio, "topic", "[studio]")
    if "bridge" in studio:
        _non_empty(studio, "bridge", "[studio]")
    if "webroot" in studio:
        _non_empty(studio, "webroot", "[studio]")
    if "pose_topic" in studio:
        _non_empty(studio, "pose_topic", "[studio]")
    if "http_port" in studio:
        port = studio["http_port"]
        if (not isinstance(port, int) or isinstance(port, bool) or
                port < 1 or port > 65535):
            raise ManifestError(
                "[studio] field 'http_port' must be a port in [1, 65535]")


def _non_empty(table, key, where):
    """config_parser.h ReadNonEmpty: a present `key` must be a non-empty
    string."""
    value = table[key]
    if not isinstance(value, str) or not value:
        raise ManifestError(f"{where} field '{key}' must be a non-empty string")
    return value


def _check_top_level_keys(doc, path):
    for key in doc:
        if key not in _TOP_LEVEL_KEYS:
            raise ManifestError(f"{path}: unknown top-level key '{key}'")


def _merge_tables(base, over):
    """config_parser.h MergeTables: `over` wins; tables merge recursively."""
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge_tables(base[key], value)
        else:
            base[key] = copy.deepcopy(value)


def _entry_name(entry):
    name = entry.get("name") if isinstance(entry, dict) else None
    return name if isinstance(name, str) and name else None


class _Reader:
    """Parses the config's files, recording each one besides the config."""

    def __init__(self, root, overrides, available):
        self.root = root
        self.overrides = overrides
        self.available = available
        self.files = []

    def load(self, path, record=True):
        if record and path not in self.files:
            self.files.append(path)
        if path in self.overrides:
            text = self.overrides[path]
        else:
            if (record and self.available is not None
                    and path not in self.available):
                raise Stale(path)
            try:
                with open(os.path.join(self.root, path), encoding="utf-8") as f:
                    text = f.read()
            except OSError as err:
                raise ManifestError(f"cannot read {path}: {err.strerror}")
        try:
            return tomllib.loads(text)
        except tomllib.TOMLDecodeError as err:
            raise ManifestError(f"TOML parse error in {path}: {err}")

    def expand(self, entries, where):
        """[[subsystems]] entries (paths already workspace-relative) -> the
        [subsystems.<name>] view, as config_parser.h's ExpandManifest."""
        views = {}
        for entry in entries:
            name = _entry_name(entry)
            path = entry.get("path") if isinstance(entry, dict) else None
            if name is None or not isinstance(path, str) or not path:
                raise ManifestError(f"invalid [[subsystems]] entry in {where}: "
                                    "needs 'name' and 'path'")
            for key in entry:
                if key not in _ENTRY_KEYS:
                    raise ManifestError(
                        f"unknown key '{key}' in [[subsystems]] entry "
                        f"'{name}' in {where} (expected name, path, node, "
                        "defaults)")
            node = (_non_empty(entry, "node", "[[subsystems]] entry")
                    if "node" in entry else None)
            entry_defaults = (
                _non_empty(entry, "defaults", "[[subsystems]] entry")
                if "defaults" in entry else None)
            sub = self.load(path)
            header = sub.get("subsystem")
            if not isinstance(header, dict) or header.get("name") != name:
                raise ManifestError(f"{path}: [subsystem].name must match "
                                    f"manifest entry '{name}'")
            if name in views:
                raise ManifestError(f"duplicate [[subsystems]] entry in "
                                    f"{where}: '{name}'")
            header_defaults = (_non_empty(header, "defaults", "[subsystem]")
                               if "defaults" in header else None)
            defaults = entry_defaults
            if defaults is None and header_defaults is not None:
                defaults = _join(posixpath.dirname(path), header_defaults, path)
            if defaults is not None:
                merged = self.load(defaults)
                _merge_tables(merged, sub)
                sub = merged
                header = sub["subsystem"]
            view = {k: v for k, v in header.items()
                    if k not in ("name", "defaults")}
            view.update((k, v) for k, v in sub.items() if k != "subsystem")
            if node is not None:
                view["node"] = node
            views[name] = view
        return views


def _normalized(entries, base_dir, where):
    """config_parser.h NormalizeManifestPaths: string `path`/`defaults`
    relative to the workspace root; other values are left for expand."""
    out = []
    for entry in entries:
        if isinstance(entry, dict):
            entry = dict(entry)
            for key in ("path", "defaults"):
                if isinstance(entry.get(key), str) and entry[key]:
                    entry[key] = _join(base_dir, entry[key], where)
        out.append(entry)
    return out


def _merge_entries(base, child, where):
    """config_parser.h MergeManifests: child entries override base by name."""
    merged = [dict(e) if isinstance(e, dict) else e for e in base]
    seen = set()
    for entry in child:
        name = _entry_name(entry)
        if name is None:
            raise ManifestError(f"invalid [[subsystems]] entry in {where}: "
                                "needs 'name'")
        if name in seen:
            raise ManifestError(f"duplicate [[subsystems]] entry in {where}: "
                                f"'{name}'")
        seen.add(name)
        match = next((m for m in merged
                      if isinstance(m, dict) and m.get("name") == name), None)
        if match is None:
            merged.append(dict(entry))
        else:
            match.update(entry)
    return merged


def effective(root, config, overrides=None, available=None):
    """(`config`'s canonical {name: [subsystems.<name>] table} view, sorted
    workspace-relative paths of every file it reads besides itself).

    Mirrors config_parser.h ParseRobotConfig step for step; the shared cases
    in talOS/configuration/testdata hold both to the same decisions. Device
    tables are not checked here: the parser does that when the config loads.

    `overrides` maps a path to content that replaces the file on disk (a
    pending edit). `available`, when given, is the set of paths that may be
    read; any other path raises Stale.
    """
    reader = _Reader(root, overrides or {}, available)
    doc = reader.load(config, record=False)
    _check_top_level_keys(doc, config)
    config_dir = posixpath.dirname(config)
    if isinstance(doc.get("subsystems"), list):
        doc["subsystems"] = _normalized(doc["subsystems"], config_dir, config)
    if "include" in doc:
        include = _join(config_dir, _non_empty(doc, "include", "top-level"),
                        config)
        base = reader.load(include)
        _check_top_level_keys(base, include)
        if "include" in base:
            raise ManifestError(f"nested 'include' is not supported in {include}")
        del doc["include"]
        base_subs = base.get("subsystems")
        if isinstance(base_subs, list):
            base_subs = _normalized(base_subs, posixpath.dirname(include),
                                    include)
            base["subsystems"] = base_subs
            # A list child merges into a list base by name; any other child
            # [subsystems] merges over the base's expanded view.
            if isinstance(doc.get("subsystems"), list):
                base["subsystems"] = _merge_entries(
                    base_subs, doc.pop("subsystems"), config)
            elif "subsystems" in doc:
                base["subsystems"] = reader.expand(base_subs, include)
        _merge_tables(base, doc)
        doc = base
    if "studio" in doc:
        _check_studio(doc["studio"], config)
    if isinstance(doc.get("subsystems"), list):
        doc["subsystems"] = reader.expand(doc["subsystems"], config)
    subs = doc.get("subsystems")
    if not isinstance(subs, dict):
        raise ManifestError(f"{config}: missing [subsystems] table")
    for name, table in subs.items():
        if isinstance(table, dict) and "node" in table:
            _non_empty(table, "node", f"[subsystems.{name}]")
    return subs, sorted(reader.files)


def resolve(root, config, overrides=None, available=None):
    """(sorted node labels, sorted files read), from effective()."""
    subs, files = effective(root, config, overrides, available)
    nodes = {t["node"] for t in subs.values()
             if isinstance(t, dict) and "node" in t}
    return sorted(nodes), files


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
