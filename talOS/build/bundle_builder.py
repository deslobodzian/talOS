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
    lib_dir = os.path.join(out_dir, "lib")
    os.makedirs(bin_dir, exist_ok=True)
    os.makedirs(config_dir, exist_ok=True)
    os.makedirs(lib_dir, exist_ok=True)

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

    # 2. Shared libraries
    for lib_path in spec.get("shared_lib_paths", []):
        dest = os.path.join(lib_dir, os.path.basename(lib_path))
        safe_copy(lib_path, dest)
        make_executable(dest)

    # 3. Config
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

    # 4. Subsystems
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

    # 5. Nodes
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

        # Create target subpath symlink for launcher ResolveBinary
        target_sub = target
        while target_sub.startswith("@") or target_sub.startswith("/"):
            target_sub = target_sub[1:]
        target_sub = target_sub.replace(":", "/").lstrip("/")
        if target_sub and target_sub != "node":
            target_link = os.path.join(out_dir, target_sub)
            rel_to_bin = os.path.relpath(bin_dest, os.path.dirname(target_link))
            safe_symlink(rel_to_bin, target_link)

            if target_sub.endswith("hardware_node"):
                alt_link = target_link[:-13] + "node"
                safe_symlink(rel_to_bin, alt_link)
            if target_sub.endswith("simulation/sim_gateway"):
                alt_sub = target_sub[: -len("simulation/sim_gateway")] + "drivetrain/sim_gateway"
                alt_link = os.path.join(out_dir, alt_sub)
                rel_alt = os.path.relpath(bin_dest, os.path.dirname(alt_link))
                safe_symlink(rel_alt, alt_link)
            if target_sub.endswith("drivetrain/sim_gateway"):
                alt_sub = target_sub[: -len("drivetrain/sim_gateway")] + "simulation/sim_gateway"
                alt_link = os.path.join(out_dir, alt_sub)
                rel_alt = os.path.relpath(bin_dest, os.path.dirname(alt_link))
                safe_symlink(rel_alt, alt_link)

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

            for lib_path in spec.get("shared_lib_paths", []):
                lib_name = os.path.basename(lib_path)
                for candidate in [
                    f"talos/talOS/ipc/python/talos_runtime/{lib_name}",
                    f"_main/talOS/ipc/python/talos_runtime/{lib_name}",
                    f"talos/talOS/node_api/{lib_name}",
                    f"_main/talOS/node_api/{lib_name}",
                ]:
                    lib_dest = os.path.join(rf_dir, candidate)
                    try:
                        safe_copy(lib_path, lib_dest)
                    except Exception:
                        pass
                    manifest_lines.append(f"{candidate} {lib_dest}\n")

            with open(os.path.join(rf_dir, "MANIFEST"), "w") as mf:
                mf.writelines(manifest_lines)

            if repo_mapping_src and os.path.exists(repo_mapping_src):
                safe_copy(repo_mapping_src, os.path.join(rf_dir, "_repo_mapping"))
                safe_copy(repo_mapping_src, os.path.join(bin_dir, f"{name}.repo_mapping"))

        node_contract = node.get("contract", "")
        if target and target != "//:node":
            manifest_nodes.append({
                "name": name,
                "target": target,
                "binary_path": bin_rel,
                "language": lang,
                "contract": node_contract,
            })
        label = node.get("label")
        if label:
            targets_to_add = {label}
            if label.startswith("@@"):
                targets_to_add.add(label[1:])
                targets_to_add.add(label[2:])
            elif label.startswith("@"):
                targets_to_add.add("@" + label)
                targets_to_add.add(label[1:])
            for t in list(targets_to_add):
                if "+" in t:
                    targets_to_add.add(t.replace("+", ""))
            for t in targets_to_add:
                manifest_nodes.append({
                    "name": name,
                    "target": t,
                    "binary_path": bin_rel,
                    "language": lang,
                    "contract": node_contract,
                })

    # 6. bundle_manifest.json
    manifest = {
        "bundle_version": 1,
        "config_path": f"config/{config_basename}",
        "config_sha256": config_sha256,
        "nodes": manifest_nodes,
    }
    manifest_path = os.path.join(out_dir, "bundle_manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")

    # 7. run.sh
    run_sh_path = os.path.join(out_dir, "run.sh")
    with open(run_sh_path, "w") as f:
        f.write(f"""#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
export LD_LIBRARY_PATH="${{SCRIPT_DIR}}/lib:${{LD_LIBRARY_PATH:-}}"
export DYLD_LIBRARY_PATH="${{SCRIPT_DIR}}/lib:${{DYLD_LIBRARY_PATH:-}}"
export TALOS_NODE_LIB="${{SCRIPT_DIR}}/lib/libtalos_node.so"
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
