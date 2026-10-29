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


def safe_copy(src, dest):
    if os.path.islink(dest) or os.path.exists(dest):
        if os.path.isdir(dest) and not os.path.islink(dest):
            shutil.rmtree(dest)
        else:
            os.remove(dest)
    parent = os.path.dirname(dest)
    if parent:
        os.makedirs(parent, exist_ok=True)
    if os.path.isdir(src):  # a tree artifact, e.g. talos_messages' Python
        shutil.copytree(src, dest)
    else:
        shutil.copy2(src, dest, follow_symlinks=True)


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

    # 2. Config
    config_src = spec["config_path"]
    config_basename = os.path.basename(config_src)
    config_dest = os.path.join(config_dir, config_basename)
    safe_copy(config_src, config_dest)

    # Config identity: sha256 over the robot config plus every subsystem file,
    # names mixed in so a rename is an identity change. Stored in the manifest
    # so a session can say exactly which configuration it ran.
    config_hash = hashlib.sha256()
    with open(config_dest, "rb") as f:
        config_hash.update(f.read())
    subsystem_bytes = {}

    # 3. Subsystems
    for sub in spec.get("subsystems", []):
        sub_src = sub["src_path"]
        sub_name = sub["name"]
        safe_copy(sub_src, os.path.join(config_dir, f"{sub_name}.toml"))
        safe_copy(sub_src, os.path.join(config_dir, "subsystems", f"{sub_name}.toml"))
        safe_copy(sub_src, os.path.join(out_dir, sub_name, "subsystem.toml"))
        safe_copy(sub_src, os.path.join(config_dir, sub_name, "subsystem.toml"))
        with open(os.path.join(config_dir, f"{sub_name}.toml"), "rb") as f:
            subsystem_bytes[sub_name] = f.read()
    for sub_name in sorted(subsystem_bytes):
        config_hash.update(sub_name.encode("utf-8"))
        config_hash.update(b"\x00")
        config_hash.update(subsystem_bytes[sub_name])
    config_sha256 = config_hash.hexdigest()

    # 4. Nodes. A Python node's runfiles carry libtalos_node.so (a data
    # dependency of //talOS/python:talos), which talos._abi finds beside itself.
    manifest_nodes = []
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
            manifest_lines = []
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
                    pass
                else:
                    if os.path.exists(rf_src):
                        try:
                            safe_copy(rf_src, dest)
                        except Exception:
                            if os.path.islink(rf_src):
                                link_target = os.readlink(rf_src)
                                if os.path.islink(dest) or os.path.exists(dest):
                                    os.remove(dest)
                                os.symlink(link_target, dest)
                    elif os.path.islink(rf_src):
                        link_target = os.readlink(rf_src)
                        if os.path.islink(dest) or os.path.exists(dest):
                            os.remove(dest)
                        os.symlink(link_target, dest)

                if rf_rel.endswith("bin/python3") and "rules_python" in rf_rel and not ".venv" in rf_rel:
                    python_bin_path = dest
                manifest_lines.append(f"{rf_rel} {dest}\n")
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

            with open(os.path.join(rf_dir, "MANIFEST"), "w") as mf:
                mf.writelines(manifest_lines)

            if repo_mapping_src and os.path.exists(repo_mapping_src):
                safe_copy(repo_mapping_src, os.path.join(rf_dir, "_repo_mapping"))
                safe_copy(repo_mapping_src, os.path.join(bin_dir, f"{name}.repo_mapping"))

        # Every label the robot config may name this node by.
        targets = [t for t in (target, node.get("label")) if t]
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
        f"config_path = {json.dumps(f'config/{config_basename}')}",
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
SCRIPT_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
export PATH="${{SCRIPT_DIR}}/bin:${{PATH}}"

unset BUILD_WORKSPACE_DIRECTORY
cd "${{SCRIPT_DIR}}"

has_config=false
for arg in "$@"; do
  if [[ "$arg" == "--config" ]]; then
    has_config=true
    break
  fi
done

if [[ "$has_config" == "true" ]]; then
  exec "./launcher" --bundle "${{SCRIPT_DIR}}" "$@"
else
  exec "./launcher" --bundle "${{SCRIPT_DIR}}" --config "config/{config_basename}" "$@"
fi
""")
    make_executable(run_sh_path)


if __name__ == "__main__":
    main()
