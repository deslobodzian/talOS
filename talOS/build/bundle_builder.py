#!/usr/bin/env python3
"""Assembles a relocatable Talos bundle directory."""

import argparse
import hashlib
import json
import os
import shutil
import stat
import sys


def make_executable(path):
    try:
        mode = os.stat(path).st_mode
        os.chmod(path, mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    except Exception:
        pass


def make_writable(path):
    try:
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
    except Exception:
        pass


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


def safe_symlink(target, link_path):
    parent = os.path.dirname(link_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    if os.path.islink(link_path) or os.path.exists(link_path):
        if os.path.isdir(link_path) and not os.path.islink(link_path):
            shutil.rmtree(link_path)
        else:
            os.remove(link_path)
    os.symlink(target, link_path)


def main():
    parser = argparse.ArgumentParser(description="Assemble a Talos robot bundle.")
    parser.add_argument("--spec", required=True, help="Path to JSON specification")
    parser.add_argument("--out", required=True, help="Output bundle directory")
    args = parser.parse_args()

    with open(args.spec, "r") as f:
        spec = json.load(f)

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
    bundle_name = spec.get("bundle_name", "")
    if bundle_name:
        named_launcher_dir = os.path.join(out_dir, bundle_name)
        os.makedirs(named_launcher_dir, exist_ok=True)
        safe_symlink("../launcher", os.path.join(named_launcher_dir, "launcher"))

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
    for node in spec.get("nodes", []):
        name = node["name"]
        target = node["target"]
        lang = node["language"]
        src_path = node["src_path"]
        bin_rel = f"bin/{name}"
        bin_dest = os.path.join(out_dir, bin_rel)
        safe_copy(src_path, bin_dest)
        make_executable(bin_dest)

        # Runfiles
        rf_entries = node.get("runfiles", [])
        if rf_entries:
            rf_dir = os.path.join(bin_dir, f"{name}.runfiles")
            os.makedirs(rf_dir, exist_ok=True)
            repo_mapping_src = None
            python_bin_path = None

            for rf in rf_entries:
                rf_src = rf["src_path"]
                rf_rel = rf["rf_rel"]
                dest = os.path.join(rf_dir, rf_rel)
                parent = os.path.dirname(dest)
                if parent:
                    os.makedirs(parent, exist_ok=True)

                if rf_rel.endswith("bin/python3") and ".venv" in rf_rel:
                    # Defer venv python3 until target hermetic python binary is copied
                    continue

                if os.path.exists(rf_src):
                    safe_copy(rf_src, dest)

                if rf_rel.endswith("bin/python3") and "rules_python" in rf_rel and ".venv" not in rf_rel:
                    python_bin_path = dest
                if rf_rel.endswith(".repo_mapping") or rf_rel == "_repo_mapping":
                    repo_mapping_src = rf_src

            # Link any venv python3 to the bundled hermetic python3
            for rf in rf_entries:
                rf_rel = rf["rf_rel"]
                if rf_rel.endswith("bin/python3") and ".venv" in rf_rel:
                    dest = os.path.join(rf_dir, rf_rel)
                    parent = os.path.dirname(dest)
                    if parent:
                        os.makedirs(parent, exist_ok=True)
                    if python_bin_path and os.path.exists(python_bin_path):
                        rel_py = os.path.relpath(python_bin_path, os.path.dirname(dest))
                        if os.path.islink(dest) or os.path.exists(dest):
                            os.remove(dest)
                        os.symlink(rel_py, dest)
                        make_executable(dest)

            if repo_mapping_src and os.path.exists(repo_mapping_src):
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
        # Every label the robot config may name this node by.
        targets = []
        for label in (target, node.get("label")):
            if not label:
                continue
            targets.append(label)
            if label.startswith("@"):
                repo, package = label.lstrip("@").split("//", 1)
                # The config uses an apparent module name; node metadata uses
                # Bazel's canonical repository name. Preserve both spellings.
                targets.append("@" + repo.split("+", 1)[0] + "//" + package)
        manifest_nodes.append({
            "name": name,
            "targets": list(dict.fromkeys(targets)),
            "binary": bin_rel,
            "language": lang,
        })

    # 5. bundle_manifest.toml, read by the launcher's LoadBundle. A JSON string
    # literal (ASCII, \uXXXX escapes) is also a valid TOML basic string.
    lines = [
        "bundle_version = 1",
        f"config_path = {json.dumps(config_rel)}",
        f"config_sha256 = {json.dumps(config_sha256)}",
    ]
    for entry in manifest_nodes:
        lines += [
            "",
            "[[nodes]]",
            f"name = {json.dumps(entry['name'])}",
            f"targets = [{', '.join(json.dumps(t) for t in entry['targets'])}]",
            f"binary = {json.dumps(entry['binary'])}",
            f"language = {json.dumps(entry['language'])}",
        ]
    with open(os.path.join(out_dir, "bundle_manifest.toml"), "w") as f:
        f.write("\n".join(lines) + "\n")

    # 6. run.sh
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
for arg in "$@"; do
  case "$arg" in
    --config|--config=*) has_config=true ;;
    --output-dir|--output-dir=*) has_output_dir=true ;;
  esac
done

extra_args=()
if [[ "$has_config" == "false" ]]; then
  extra_args+=(--config "{config_rel}")
fi
if [[ "$has_output_dir" == "false" && -n "${{TALOS_LOG_DIR:-}}" ]]; then
  extra_args+=(--output-dir "${{TALOS_LOG_DIR}}")
fi

exec "./launcher" --bundle "${{SCRIPT_DIR}}" ${{extra_args[@]+"${{extra_args[@]}}"}} "$@"
""")
    make_executable(run_sh_path)


if __name__ == "__main__":
    main()
