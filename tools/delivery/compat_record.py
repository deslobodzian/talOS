#!/usr/bin/env python3
"""Deployment compatibility record (item 57).

Builds or verifies the record that travels with a deployment: which bundle
revision, which configuration hash, which per-node contract labels, and
which vendor/firmware versions the field crew must match. Provenance is
traceability, not a compatibility proof: a matching contract label does not
establish wire compatibility (whole-bundle rollback, never mixed versions).

    python3 tools/delivery/compat_record.py --bundle PATH --out record.json
    python3 tools/delivery/compat_record.py --bundle PATH --check
    python3 tools/delivery/compat_record.py --bundle PATH --out record.json \\
        --vendor vendor_expectations.json --workspace REV

--vendor JSON shape: {"devices": {"TalonFX": {"firmware": "25.3.2.1",
"vendordep": "CTRE-Phoenix6 v25.x"}}, "wpilib": "2026.2.1", "notes": "..."}.
Without it the record marks vendor expectations missing, which inhibits
readiness (see readiness.py).
"""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def recompute_config_sha256(config_dir):
    """Same identity hash as talOS/build/bundle_builder.py: the robot config
    plus every top-level <name>.toml subsystem file, names mixed in."""
    config_dir = Path(config_dir)
    robot_cfgs = sorted(config_dir.glob("*.toml"))
    robot_main = None
    for cand in ("robot.toml",):
        if (config_dir / cand).exists():
            robot_main = config_dir / cand
    if robot_main is None:
        raise FileNotFoundError(f"no robot.toml in {config_dir}")
    h = hashlib.sha256()
    h.update(robot_main.read_bytes())
    subs = {}
    for path in robot_cfgs:
        if path.name == robot_main.name:
            continue
        subs[path.stem] = path.read_bytes()
    for name in sorted(subs):
        h.update(name.encode("utf-8"))
        h.update(b"\x00")
        h.update(subs[name])
    return h.hexdigest()


def workspace_revision(path):
    try:
        out = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=30)
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return "unknown"


def build_record(bundle_dir, vendor=None, workspace=None):
    bundle_dir = Path(bundle_dir)
    manifest_path = bundle_dir / "bundle_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    config_dir = bundle_dir / "config"
    actual_sha = recompute_config_sha256(config_dir)
    record = {
        "bundle_version": manifest.get("bundle_version", 0),
        "config_path": manifest.get("config_path", ""),
        "config_sha256_manifest": manifest.get("config_sha256", ""),
        "config_sha256_actual": actual_sha,
        "config_match": actual_sha == manifest.get("config_sha256", ""),
        "workspace_revision": workspace or workspace_revision("."),
        "nodes": [
            {"name": n.get("name", ""), "target": n.get("target", ""),
             "language": n.get("language", ""),
             "contract": n.get("contract", "")}
            for n in manifest.get("nodes", [])
        ],
        "vendor_expectations": vendor or {},
        "vendor_expectations_recorded": bool(vendor),
    }
    return record


def check_bundle(bundle_dir, vendor=None):
    """Returns (ok, [problems]). Nonzero-impact items are errors; a missing
    contract label on a node is a warning (schema-less nodes own no schema)."""
    bundle_dir = Path(bundle_dir)
    problems = []
    manifest_path = bundle_dir / "bundle_manifest.json"
    if not manifest_path.exists():
        return False, ["no bundle_manifest.json: not a talos bundle"]
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("bundle_version") != 1:
        problems.append(
            f"unsupported bundle_version {manifest.get('bundle_version')!r}")
    try:
        actual = recompute_config_sha256(bundle_dir / "config")
    except FileNotFoundError as e:
        return False, [str(e)]
    if actual != manifest.get("config_sha256", ""):
        problems.append(
            "config_sha256 mismatch: the bundle's configuration was edited "
            "after assembly; rebuild the bundle, never ship a hand edit")
    seen_targets = set()
    for n in manifest.get("nodes", []):
        target = n.get("target", "")
        binary = bundle_dir / n.get("binary_path", "")
        if target in seen_targets:
            continue
        seen_targets.add(target)
        if not binary.exists():
            problems.append(f"node {n.get('name')!r}: missing binary "
                            f"{n.get('binary_path')!r}")
        if not n.get("contract"):
            problems.append(f"warning: node {n.get('name')!r} carries no "
                            f"contract label (ok only for schema-less nodes)")
    if not vendor:
        problems.append("warning: no vendor/firmware expectations recorded; "
                        "pass --vendor vendor_expectations.json")
    errors = [p for p in problems if not p.startswith("warning")]
    return (not errors), problems


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--out", default=None)
    parser.add_argument("--vendor", default=None)
    parser.add_argument("--workspace", default=None)
    parser.add_argument("--check", action="store_true",
                        help="verify the bundle, print problems, exit nonzero "
                             "on errors (warnings pass)")
    args = parser.parse_args(argv)
    vendor = None
    if args.vendor:
        vendor = json.loads(Path(args.vendor).read_text())
    if args.check:
        ok, problems = check_bundle(args.bundle, vendor)
        for p in problems:
            print(("error: " if not p.startswith("warning") else "") + p)
        return 0 if ok else 1
    record = build_record(args.bundle, vendor, args.workspace)
    text = json.dumps(record, indent=2) + "\n"
    if args.out:
        Path(args.out).write_text(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
