#!/usr/bin/env python3
"""Assembles a relocatable Talos bundle directory."""

import argparse
import filecmp
import hashlib
import json
import os
import shlex
import shutil
import stat
import sys


def make_executable(path):
    mode = os.stat(path).st_mode
    os.chmod(path, mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def make_writable(path):
    if os.path.isdir(path) and not os.path.islink(path):
        os.chmod(path, os.stat(path).st_mode | stat.S_IWUSR)
        for root, dirs, files in os.walk(path):
            for d in dirs:
                p = os.path.join(root, d)
                os.chmod(p, os.stat(p).st_mode | stat.S_IWUSR)
            for f in files:
                p = os.path.join(root, f)
                if not os.path.islink(p):
                    os.chmod(p, os.stat(p).st_mode | stat.S_IWUSR)
    elif os.path.exists(path) and not os.path.islink(path):
        os.chmod(path, os.stat(path).st_mode | stat.S_IWUSR)


def safe_copy(src, dest):
    parent = os.path.dirname(dest)
    if parent and os.path.exists(parent):
        make_writable(parent)
    if os.path.islink(dest) or os.path.exists(dest):
        make_writable(dest)
        if os.path.isdir(dest) and not os.path.islink(dest):
            shutil.rmtree(dest)
        else:
            os.remove(dest)
    if parent:
        os.makedirs(parent, exist_ok=True)
    if os.path.isdir(src):  # a tree artifact, e.g. talos_messages' Python
        shutil.copytree(src, dest, symlinks=False)
    else:
        shutil.copy2(src, dest, follow_symlinks=True)
    make_writable(dest)


# Where a bundle holds the Python runtimes its nodes share, by repository.
RUNTIME_DIR = "python"

# Test-only PyPI distributions pulled in transitively by gtsam's wheel metadata
# (talOS/python/requirements.txt, REQ-OPS-48). Robot bundles never run pytest;
# omitting their external site-packages directories saves ~7 MB per Python node
# that depends on gtsam without touching user code (_main/) or runtime stdlib.
TEST_ONLY_WHEELS = frozenset({
    "pytest",
    "pluggy",
    "iniconfig",
    "pygments",
    "packaging",
})
TEST_ONLY_SITE_ENTRIES = TEST_ONLY_WHEELS | frozenset({"_pytest"})


def is_test_only_wheel_file(rf_rel, runtimes):
    """True when `rf_rel` belongs to one of TEST_ONLY_WHEELS inside an external
    wheel repository's site-packages/ directory (never _main user code and
    never a Python runtime's own repository)."""
    parts = rf_rel.split(os.sep)
    if len(parts) < 3:
        return False
    repo = parts[0]
    if repo == "_main" or repo in runtimes or parts[1] != "site-packages":
        return False
    entry = parts[2]
    if entry in TEST_ONLY_SITE_ENTRIES:
        return True
    if entry.endswith(".dist-info"):
        dist = entry.split("-", 1)[0].lower().replace("_", "-")
        if dist in TEST_ONLY_WHEELS:
            return True
    # pytest's wheel also ships top-level py.py beside pytest/ and _pytest/.
    if entry == "py.py" and "+pip+" in repo and "_pytest_" in repo:
        return True
    return False


def write_exec_script(dest, target, note):
    """Writes `dest` as a script that execs `target` (a path in the bundle)
    by its path relative to `dest`, so the bundle runs wherever it is
    installed. The exec keeps the pid, and the target runs as its own file:
    an interpreter finds its library from where it is."""
    rel = os.path.relpath(target, os.path.dirname(dest))
    if os.path.lexists(dest):
        make_writable(dest)
        os.remove(dest)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with open(dest, "w", encoding="utf-8") as f:
        f.write("#!/bin/sh\n"
                f"# {note}\n"
                f'exec "$(dirname "$0")"/{shlex.quote(rel)} "$@"\n')
    make_executable(dest)


def link_target(rf_rel, link):
    """The runfile a link at `rf_rel` to `link` (relative to the link's
    directory in the runfiles tree) names, as a runfiles-relative path."""
    return os.path.normpath(os.path.join(os.path.dirname(rf_rel), link))


def stage_link(name, dest, rf_rel, link, target, bundle_root):
    """Stages the runfile `rf_rel` at `dest`, a link Bazel declared to `link`
    whose target the bundle holds at `target`, without a symlink.

    Bazel may restore a tree artifact from its cache with every symlink in
    it replaced by a copy of its target, so the bundle holds none and runs
    the same either way. An executable is staged as a script that execs its
    target where the bundle holds it: an interpreter finds its library from
    its own location, so a copy of it elsewhere would not run (a py_binary
    venv's bin/python3; the node's stage-2 bootstrap then adds the venv's
    site-packages itself). Any other file is staged as a copy."""
    if not (target.startswith(bundle_root) and os.path.isfile(target)):
        sys.exit(f"bundle: node '{name}': runfile {rf_rel} links to {link}, "
                 "which the bundle does not hold")
    if not os.access(target, os.X_OK):
        safe_copy(target, dest)
        return
    write_exec_script(dest, target,
                      f"{rf_rel} -> {link}, kept out of the bundle as a link.")


def is_executable_file(path):
    """A regular file (a directory's X bit means search) that may run."""
    return os.path.isfile(path) and os.access(path, os.X_OK)


def runtime_repositories(files, links):
    """The repositories of a Python node's runfiles that hold its
    interpreter: those a declared link (a venv's bin/python3) names an
    executable file in. `files` maps each staged runfile to its source;
    `links` lists (runfile, link) pairs."""
    repos = set()
    for rf_rel, link in links:
        target = link_target(rf_rel, link)
        repo = target.split(os.sep, 1)[0]
        if (repo not in ("_main", os.pardir) and target in files
                and is_executable_file(files[target])):
            repos.add(repo)
    return repos


def unselected_runtimes(files, runtimes):
    """The repositories of `files` that hold an interpreter, an executable
    bin/python3, which no venv link selected (`runtimes`). Such a runtime
    would be staged whole in the node's runfiles, beside the bundle's one
    copy of it."""
    return sorted({rf_rel.split(os.sep, 1)[0] for rf_rel, src in files.items()
                   if rf_rel.split(os.sep, 1)[1:] == [os.path.join("bin", "python3")]
                   and rf_rel.split(os.sep, 1)[0] not in runtimes
                   and is_executable_file(src)})


def stage_runtime_file(name, src, rf_rel, dest, staged_from):
    """Stages a runtime's file at `dest` in the bundle's one runtime
    directory, once for every node that carries it. `staged_from` maps each
    staged destination to its (source, node). A later node's file from the
    same source is the same file; one from another source must have the
    same bytes, or the build exits: one directory cannot serve both."""
    seen = staged_from.get(dest)
    if seen is None:
        safe_copy(src, dest)
        staged_from[dest] = (src, name)
        return
    first_src, first = seen
    if src == first_src:
        return
    if not (os.path.isfile(src) and os.path.isfile(dest)):
        return  # a tree artifact; filecmp compares only files
    if not filecmp.cmp(src, dest, shallow=False):
        repo = rf_rel.split(os.sep, 1)[0]
        sys.exit(f"bundle: nodes '{first}' and '{name}' carry different "
                 f"Python runtimes named {repo} ({rf_rel} differs)")


def alias_interpreter_copies(interpreter):
    """Restages every other executable file of `interpreter`'s directory
    with its exact bytes (rules_python's runfiles copy it as python,
    python3 and python3.11) as a script that execs it, so the bundle holds
    the interpreter once."""
    size = os.path.getsize(interpreter)
    bin_dir = os.path.dirname(interpreter)
    for entry in sorted(os.listdir(bin_dir)):
        path = os.path.join(bin_dir, entry)
        if (path != interpreter and os.path.isfile(path)
                and os.access(path, os.X_OK)
                and os.path.getsize(path) == size
                and filecmp.cmp(path, interpreter, shallow=False)):
            write_exec_script(path, interpreter,
                              f"{entry}: the interpreter, held once as "
                              f"{os.path.basename(interpreter)}.")


def parse_repo_mappings(paths):
    """Reads Bazel 3-column CSV `_repo_mapping` files into
    `{target_canonical: [target_apparent, ...]}` for mappings visible from the
    main repository (`source_canonical == ""`), preserving order."""
    by_canonical = {}
    for path in paths:
        if not path or not os.path.isfile(path):
            continue
        with open(path, encoding="utf-8") as f:
            for line in f:
                parts = line.rstrip("\r\n").split(",")
                if len(parts) != 3:
                    continue
                source, apparent, canonical = parts
                if source == "" and apparent and canonical:
                    names = by_canonical.setdefault(canonical, [])
                    if apparent not in names:
                        names.append(apparent)
    return by_canonical


def parse_apparent_repos(entries, canonical_to_apparent=None):
    """Merges `apparent=canonical` pairs (from `--apparent-repo`) into
    `{canonical: [apparent, ...]}`."""
    by_canonical = {} if canonical_to_apparent is None else canonical_to_apparent
    for entry in entries:
        apparent, sep, canonical = entry.partition("=")
        if not sep or not apparent or not canonical:
            sys.exit(f"bundle: invalid --apparent-repo value {entry!r}; "
                     "expected apparent=canonical")
        names = by_canonical.setdefault(canonical, [])
        if apparent not in names:
            names.append(apparent)
    return by_canonical


def node_targets(node, canonical_to_apparent=None):
    """Every label the robot config may name `node` by."""
    targets = []
    for label in (node["target"], node.get("label")):
        if not label:
            continue
        targets.append(label)
        if label.startswith("@"):
            repo, package = label.lstrip("@").split("//", 1)
            # The config uses an apparent repository name; node metadata uses
            # Bazel's canonical repository name. Preserve both spellings.
            if repo == "" or repo == "_main":
                targets.append(f"//{package}")
                if canonical_to_apparent and "_main" in canonical_to_apparent:
                    for apparent in canonical_to_apparent["_main"]:
                        targets.append(f"@{apparent}//{package}")
            elif canonical_to_apparent and repo in canonical_to_apparent:
                for apparent in canonical_to_apparent[repo]:
                    targets.append(f"@{apparent}//{package}")
            else:
                targets.append("@" + repo.split("+", 1)[0].split("~", 1)[0]
                               + "//" + package)
        elif label.startswith("//") and canonical_to_apparent and "_main" in canonical_to_apparent:
            package = label[2:]
            for apparent in canonical_to_apparent["_main"]:
                targets.append(f"@{apparent}//{package}")
    return list(dict.fromkeys(targets))


def check_nodes(spec, resolved, canonical_to_apparent=None):
    """Exits unless the bundle's nodes are exactly the ones its config
    launches. `resolved` is the configuration resolver's JSON for the config:
    `nodes` holds every subsystem's node (the generated manifest's NODES), and
    the effective [robot] table's `hardware_node` names the bridge, the
    launcher's default `spec["default_hardware_node"]` when it is unset. The
    launcher finds a node's binary by label and, failing that, by the node's
    name, so a node missing here either stops the robot at boot or silently
    runs another binary of that name: the default bridge in place of a
    [robot] hardware_node, say."""
    hardware = resolved["effective"].get("robot", {}).get("hardware_node")
    if not isinstance(hardware, str):  # as the launcher reads it
        hardware = spec["default_hardware_node"]
    launched = set(resolved["nodes"]) | {hardware}
    bundled = [(node.get("label") or node["target"],
                node_targets(node, canonical_to_apparent))
               for node in spec["nodes"]]
    missing = sorted(label for label in launched
                     if not any(label in targets for _, targets in bundled))
    extra = sorted(label for label, targets in bundled
                   if launched.isdisjoint(targets))
    if missing or extra:
        sys.exit("\n".join(
            [f"bundle: nodes must be what {spec['config']['short_path']} "
             f"launches: its generated manifest's NODES and its hardware node "
             f"{hardware}"] +
            [f"  launched but not in nodes: {label}" for label in missing] +
            [f"  in nodes but not launched: {label}" for label in extra]))


def check_studio(spec, resolved, canonical_to_apparent=None):
    """Exits when the bundle carries a Studio bridge other than the one its
    config's `[studio] bridge` names, or a webroot without Studio's pages
    (index.html, and drive.html for the driver dashboard). Unset,
    the launcher's default label is used and falls back to the bundled bridge
    by name."""
    studio = spec.get("studio")
    if not studio:
        return
    configured = resolved["effective"].get("studio", {}).get("bridge")
    targets = node_targets({"target": studio["bridge_label"]},
                           canonical_to_apparent)
    if isinstance(configured, str) and configured not in targets:
        sys.exit(f"bundle: studio_bridge {studio['bridge_label']} is not the "
                 f"[studio] bridge {configured} that "
                 f"{spec['config']['short_path']} names")
    for page in ("index.html", "drive.html"):
        if not os.path.isfile(os.path.join(studio["webroot_path"], page)):
            sys.exit(f"bundle: studio_webroot {studio['webroot_path']} holds "
                     f"no {page}; build Studio's pages with `bazel build "
                     "//studio:web_dist` in talOS's checkout (from a robot "
                     "repository: the checkout its MODULE.bazel `talos` "
                     "override names)")


def source_revision(path):
    """The bundle_manifest.toml lines naming the source revision (and, when a
    separate robot repository builds against a talOS git checkout, talOS's
    revision), from a --stamp build's stable status file
    (tools/workspace_status.sh); none when neither commit key is present."""
    status = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            key, _, value = line.rstrip("\n").partition(" ")
            status[key] = value
    lines = []
    revision = status.get("STABLE_GIT_COMMIT", "")
    if revision:
        dirty = status.get("STABLE_GIT_DIRTY", "0") != "0"
        lines += [f"git_revision = {json.dumps(revision)}",
                  f"git_dirty = {'true' if dirty else 'false'}"]
    talos_revision = status.get("STABLE_TALOS_GIT_COMMIT", "")
    if talos_revision:
        talos_dirty = status.get("STABLE_TALOS_GIT_DIRTY", "0") != "0"
        lines += [f"talos_git_revision = {json.dumps(talos_revision)}",
                  f"talos_git_dirty = {'true' if talos_dirty else 'false'}"]
    return lines


def main():
    parser = argparse.ArgumentParser(description="Assemble a Talos robot bundle.")
    parser.add_argument("--spec", required=True, help="Path to JSON specification")
    parser.add_argument("--resolved", required=True,
                        help="The configuration resolver's JSON for the config")
    parser.add_argument("--out", required=True, help="Output bundle directory")
    parser.add_argument("--stable-status",
                        help="Bazel's stable status file, in a --stamp build")
    parser.add_argument("--repo-mapping", action="append", default=[],
                        help="Bazel `_repo_mapping` manifest (may repeat)")
    parser.add_argument("--apparent-repo", action="append", default=[],
                        help="Caller apparent=canonical repo mapping (may repeat)")
    args = parser.parse_args()

    with open(args.spec, "r") as f:
        spec = json.load(f)
    with open(args.resolved, "r") as f:
        resolved = json.load(f)
    canonical_to_apparent = parse_repo_mappings(args.repo_mapping)
    parse_apparent_repos(args.apparent_repo, canonical_to_apparent)
    check_nodes(spec, resolved, canonical_to_apparent)
    check_studio(spec, resolved, canonical_to_apparent)

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    bin_dir = os.path.join(out_dir, "bin")
    config_dir = os.path.join(out_dir, "config")
    os.makedirs(bin_dir, exist_ok=True)
    os.makedirs(config_dir, exist_ok=True)

    # 1. Launcher binary
    launcher_src = spec["launcher_path"]
    launcher_dest = os.path.join(out_dir, "launcher")
    safe_copy(launcher_src, launcher_dest)
    make_executable(launcher_dest)

    # 2. Config. Every file keeps its path relative to the others under
    # config/, rooted at their deepest common directory, so the config's
    # relative `include`, `path` and `defaults` resolve as in the source tree
    # (a `../shared/x.toml` puts the config one level down).
    config = spec["config"]
    config_files = spec.get("config_files", [])
    common = os.path.commonpath(
        [os.path.dirname(f["short_path"]) or "." for f in [config] + config_files]) or "."

    def staged(entry):
        return os.path.relpath(entry["short_path"], common).replace(os.sep, "/")

    # Config identity (config_sha256): sha256 over every staged file, the
    # robot config included, each as its config/-relative path, its length and
    # its bytes, sorted by path. Absolute and workspace locations never
    # participate, so relocating the tree (or the bundle) keeps the identity;
    # renaming or moving any file relative to the others, the root config
    # too, changes it. Stored in the manifest so a session can say exactly
    # which configuration it ran.
    staged_bytes = {}
    for entry in [config] + config_files:
        rel = staged(entry)
        dest = os.path.join(config_dir, rel)
        safe_copy(entry["src_path"], dest)
        with open(dest, "rb") as f:
            staged_bytes[rel] = f.read()
    config_rel = "config/" + staged(config)
    config_hash = hashlib.sha256()
    for rel in sorted(staged_bytes):
        content = staged_bytes[rel]
        config_hash.update(f"{rel}\0{len(content)}\0".encode("utf-8"))
        config_hash.update(content)
    config_sha256 = config_hash.hexdigest()

    # 4. Nodes. A Python node's runfiles carry libtalos_node.so (a data
    # dependency of //talOS/python:talos), which talos._abi finds beside itself.
    manifest_nodes = []
    # lib/: the shared libraries C++ nodes link (talos_cc_node's rpath is
    # ../lib from bin/). One file per name; two nodes may carry the same
    # library, but never two different libraries under one name.
    lib_dir = os.path.join(out_dir, "lib")
    staged_libs = {}
    # python/: each Python runtime file, with its source and the first node
    # that carried it, and the interpreters venv links exec there.
    staged_runtimes = {}
    interpreters = set()
    bundle_root = out_dir + os.sep
    for node in spec.get("nodes", []):
        name = node["name"]
        lang = node["language"]
        src_path = node["src_path"]
        bin_rel = f"bin/{name}"
        bin_dest = os.path.join(out_dir, bin_rel)
        safe_copy(src_path, bin_dest)
        make_executable(bin_dest)

        # Runfiles. Each is an input of this action, so one that does not
        # exist is a symlink Bazel declared unresolved, such as a py_binary
        # venv's bin/python3. Its target is relative to the runfiles tree, so
        # it is staged after the files (stage_link), and must name one of
        # them. The repository holding the interpreter such a link names is
        # staged once for every node, under python/ (the interpreter and its
        # library are most of a bundle's bytes); the link's script execs it
        # there, and the interpreter finds its library beside it as before.
        rf_entries = node.get("runfiles", [])
        if rf_entries:
            rf_dir = os.path.join(bin_dir, f"{name}.runfiles")
            os.makedirs(rf_dir, exist_ok=True)
            repo_mapping_src = None
            files = {}
            links = []
            for rf in rf_entries:
                rf_src = rf["src_path"]
                rf_rel = rf["rf_rel"]
                if os.path.islink(rf_src) and not os.path.exists(rf_src):
                    links.append((rf_rel, os.readlink(rf_src)))
                    continue
                if not os.path.exists(rf_src):
                    sys.exit(f"bundle: node '{name}': runfile {rf_rel} is "
                             f"missing: {rf_src}")
                files[os.path.normpath(rf_rel)] = rf_src
            runtimes = runtime_repositories(files, links)
            for repo in unselected_runtimes(files, runtimes):
                sys.exit(f"bundle: node '{name}': its runfiles carry the "
                         f"Python runtime {repo}, but no venv link selects "
                         "it, so the bundle would hold it again in the "
                         "node's runfiles")

            def staged_at(rf_rel):
                if rf_rel.split(os.sep, 1)[0] in runtimes:
                    return os.path.join(out_dir, RUNTIME_DIR, rf_rel)
                return os.path.join(rf_dir, rf_rel)

            for rf_rel, rf_src in files.items():
                if rf_rel.split(os.sep, 1)[0] in runtimes:
                    stage_runtime_file(name, rf_src, rf_rel, staged_at(rf_rel),
                                       staged_runtimes)
                    continue
                if is_test_only_wheel_file(rf_rel, runtimes):
                    continue
                safe_copy(rf_src, os.path.join(rf_dir, rf_rel))
                if rf_rel.endswith(".repo_mapping") or rf_rel == "_repo_mapping":
                    repo_mapping_src = rf_src
            for rf_rel, link in links:
                target = link_target(rf_rel, link)
                if (is_test_only_wheel_file(rf_rel, runtimes)
                        or is_test_only_wheel_file(target, runtimes)):
                    continue
                stage_link(name, os.path.join(rf_dir, rf_rel), rf_rel, link,
                           staged_at(target), bundle_root)
                if target.split(os.sep, 1)[0] in runtimes:
                    interpreters.add(staged_at(target))

            if repo_mapping_src:
                safe_copy(repo_mapping_src, os.path.join(rf_dir, "_repo_mapping"))
                safe_copy(repo_mapping_src, os.path.join(bin_dir, f"{name}.repo_mapping"))
        for lib in node.get("shared_libs", []):
            with open(lib["src_path"], "rb") as f:
                digest = hashlib.sha256(f.read()).hexdigest()
            seen = staged_libs.get(lib["name"])
            if seen is None:
                os.makedirs(lib_dir, exist_ok=True)
                safe_copy(lib["src_path"], os.path.join(lib_dir, lib["name"]))
                staged_libs[lib["name"]] = (digest, name)
            elif seen[0] != digest:
                sys.exit(f"bundle: nodes '{seen[1]}' and '{name}' link different "
                         f"shared libraries named {lib['name']}")
        with open(bin_dest, "rb") as f:
            binary_sha256 = hashlib.sha256(f.read()).hexdigest()
        manifest_nodes.append({
            "name": name,
            "targets": node_targets(node, canonical_to_apparent),
            "binary": bin_rel,
            "binary_sha256": binary_sha256,
            "language": lang,
        })

    for interpreter in sorted(interpreters):
        alias_interpreter_copies(interpreter)

    # 5. Studio: the bridge binary and the built site it serves, which the
    # launcher's LoadBundle registers for --start-studio-bridge.
    studio_lines = []
    studio = spec.get("studio")
    if studio:
        bridge_rel = "studio/studio_bridge"
        bridge_dest = os.path.join(out_dir, bridge_rel)
        safe_copy(studio["bridge_path"], bridge_dest)
        make_executable(bridge_dest)
        with open(bridge_dest, "rb") as f:
            bridge_sha256 = hashlib.sha256(f.read()).hexdigest()
        webroot_rel = "studio/webroot"
        safe_copy(studio["webroot_path"], os.path.join(out_dir, webroot_rel))
        studio_lines = [
            "",
            "[studio]",
            f"bridge = {json.dumps(bridge_rel)}",
            f"bridge_sha256 = {json.dumps(bridge_sha256)}",
            "targets = [" + ", ".join(
                json.dumps(t) for t in
                node_targets({"target": studio["bridge_label"]},
                             canonical_to_apparent)) + "]",
            f"webroot = {json.dumps(webroot_rel)}",
        ]

    # 6. bundle_manifest.toml, read by the launcher's LoadBundle. A JSON string
    # literal (ASCII, \uXXXX escapes) is also a valid TOML basic string.
    lines = [
        "bundle_version = 1",
        f"config_path = {json.dumps(config_rel)}",
        f"config_sha256 = {json.dumps(config_sha256)}",
    ]
    # A testonly talos_bundle says so, and tools/deploy.sh install refuses
    # it outside a Bazel test. Before any table: the installer reads only
    # the top table.
    if spec.get("testonly"):
        lines.append("testonly = true")
    if args.stable_status:
        lines += source_revision(args.stable_status)
    lines += studio_lines
    for entry in manifest_nodes:
        lines += [
            "",
            "[[nodes]]",
            f"name = {json.dumps(entry['name'])}",
            f"targets = [{', '.join(json.dumps(t) for t in entry['targets'])}]",
            f"binary = {json.dumps(entry['binary'])}",
            f"binary_sha256 = {json.dumps(entry['binary_sha256'])}",
            f"language = {json.dumps(entry['language'])}",
        ]
    with open(os.path.join(out_dir, "bundle_manifest.toml"), "w") as f:
        f.write("\n".join(lines) + "\n")

    # 7. run.sh. A bundle that carries Studio starts its bridge by default,
    # so a booted robot (deploy.sh enable-boot runs run.sh --serve-refusal)
    # and a run by hand serve the same Studio; --no-studio, which run.sh
    # consumes, declines it. A bridge that cannot run is one warning (REQ-OPS-30).
    start_studio = "true" if studio else "false"
    run_sh_path = os.path.join(out_dir, "run.sh")
    with open(run_sh_path, "w") as f:
        f.write(f"""#!/usr/bin/env bash
set -e
# Physical (pwd -P): started as <prefix>/current/run.sh, the launcher and
# every node it restarts stay in this bundle when deploy.sh moves `current`.
SCRIPT_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd -P)"
export PATH="${{SCRIPT_DIR}}/bin:${{PATH}}"

unset BUILD_WORKSPACE_DIRECTORY
unset RUNFILES_DIR RUNFILES_MANIFEST_FILE JAVA_RUNFILES PYTHON_RUNFILES PYTHONPATH VIRTUAL_ENV PYTHONHOME
cd "${{SCRIPT_DIR}}"

has_config=false
has_output_dir=false
start_studio={start_studio}
args=()
for arg in "$@"; do
  case "$arg" in
    --no-studio) start_studio=false; continue ;;
    --config|--config=*) has_config=true ;;
    --output-dir|--output-dir=*) has_output_dir=true ;;
  esac
  args+=("$arg")
done

extra_args=()
if [[ "$has_config" == "false" ]]; then
  extra_args+=(--config "{config_rel}")
fi
if [[ "$has_output_dir" == "false" && -n "${{TALOS_LOG_DIR:-}}" ]]; then
  extra_args+=(--output-dir "${{TALOS_LOG_DIR}}")
fi
if [[ "$start_studio" == "true" ]]; then
  extra_args+=(--start-studio-bridge)
fi

exec "./launcher" --bundle "${{SCRIPT_DIR}}" ${{extra_args[@]+"${{extra_args[@]}}"}} ${{args[@]+"${{args[@]}}"}}
""")
    make_executable(run_sh_path)


if __name__ == "__main__":
    main()
