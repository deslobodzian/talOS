#!/usr/bin/env python3
"""Robot readiness report (item 60).

One visible report over four readiness dimensions, each with specific
inhibit reasons instead of a bare red/green:

  processes  every roster subsystem's node is present in the bundle
  hardware   the configuration is commissioned (uncommissioned gates
             commands on hardware; simulation is the only exception)
  config     the installed configuration hash matches the bundle manifest
  feedback   every motor's feedback source exists (remote CANcoder ids must
             name a real sensor on the motor's bus; rotor is always present)

Vendor/firmware expectations (--vendor, see compat_record.py) are required:
without them the report inhibits with the reason named.

    python3 tools/delivery/readiness.py --config robot.toml --bundle DIR \\
        [--sim] [--vendor vendor.json]

Exit 0 when ready, 1 when inhibited. docs/READINESS.md is captured output
of this tool plus a regeneration header; regenerate, never hand-edit.
"""

import argparse
import json
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib


def load_toml(path):
    with open(path, "rb") as f:
        return tomllib.load(f)


def roster_subsystems(robot_cfg):
    subs = robot_cfg.get("subsystems", [])
    if isinstance(subs, dict):
        return [(name, val) for name, val in subs.items()]
    return [(s.get("name", ""), s) for s in subs]


def subsystem_file(robot_path, entry):
    if isinstance(entry, dict) and entry.get("path"):
        return str(Path(robot_path).parent / entry["path"])
    return None


def check_processes(robot_cfg, manifest):
    """Every roster node must be staged in the bundle manifest (by target,
    with a name fallback, mirroring the launcher's own resolution)."""
    findings = []
    nodes = manifest.get("nodes", []) if manifest else []
    by_target = {n.get("target", ""): n for n in nodes}
    by_name = {n.get("name", ""): n for n in nodes}
    for name, entry in roster_subsystems(robot_cfg):
        target = entry.get("node", "") if isinstance(entry, dict) else ""
        hit = by_target.get(target) or by_name.get(name)
        if hit is None:
            findings.append((name, False,
                             f"node target {target!r} not staged in bundle"))
        else:
            findings.append((name, True,
                             f"{hit.get('binary_path', '')} "
                             f"[{hit.get('language', '')}]"))
    return findings


def check_hardware(robot_cfg, subsystem_cfgs, simulation):
    commissioned = robot_cfg.get("robot", {}).get("commissioned", False)
    findings = []
    for name, _ in roster_subsystems(robot_cfg):
        if commissioned:
            findings.append((name, True, "commissioned"))
        elif simulation:
            findings.append((name, True, "uncommissioned: sim only"))
        else:
            findings.append((name, False,
                             "uncommissioned: commands gated on hardware"))
    _ = subsystem_cfgs
    return findings


def check_config(manifest, actual_sha):
    if manifest is None:
        return [("bundle", False, "no bundle manifest supplied")]
    expected = manifest.get("config_sha256", "")
    if actual_sha == expected:
        return [("config_sha256", True, actual_sha[:16] + "...")]
    return [("config_sha256", False,
             f"installed {actual_sha[:16]}... != manifest {expected[:16]}...: "
             f"rebuild the bundle, never ship a hand edit")]


def check_feedback(subsystem_cfgs):
    # Subsystem TOML shape (see configuration/subsystems/drivetrain.toml):
    # [motors.<name>] feedback = "Rotor" | "RemoteCANcoder",
    # feedback_sensor = "<sensor name>"; [sensors.<name>] type/bus.
    findings = []
    for name, cfg in subsystem_cfgs.items():
        sensors = cfg.get("sensors", {})
        if isinstance(sensors, list):
            sensors = {s.get("name", ""): s for s in sensors
                       if isinstance(s, dict)}
        motors = cfg.get("motors", {})
        if isinstance(motors, list):
            motors = {m.get("name", ""): m for m in motors
                      if isinstance(m, dict)}
        if not motors:
            findings.append((name, True, "no motors"))
            continue
        for mname, m in motors.items():
            if m.get("feedback", "Rotor") != "RemoteCANcoder":
                findings.append((f"{name}.{mname}", True, "rotor feedback"))
                continue
            fsensor = m.get("feedback_sensor", "")
            s = sensors.get(fsensor)
            if s is None:
                findings.append((f"{name}.{mname}", False,
                                 f"feedback_sensor {fsensor!r} names no "
                                 f"sensor in this subsystem"))
            elif s.get("type", "CANcoder") != "CANcoder":
                findings.append((f"{name}.{mname}", False,
                                 f"feedback sensor {fsensor!r} is type "
                                 f"{s.get('type')!r}, not a CANcoder"))
            elif s.get("bus", "rio") != m.get("bus", "rio"):
                findings.append((f"{name}.{mname}", False,
                                 f"feedback sensor {fsensor!r} on bus "
                                 f"{s.get('bus')!r}, motor on "
                                 f"{m.get('bus')!r}"))
            else:
                findings.append((f"{name}.{mname}", True,
                                 f"remote CANcoder {fsensor!r}"))
    return findings


def render_report(sections, inhibited):
    lines = ["# talOS readiness report", ""]
    for title, findings in sections:
        lines.append(f"## {title}")
        for name, ok, reason in findings:
            lines.append(f"- [{'READY' if ok else 'INHIBIT'}] {name}: {reason}")
        lines.append("")
    lines.append(f"verdict: {'INHIBITED' if inhibited else 'READY'}")
    return "\n".join(lines) + "\n"


def assess(config_path, manifest=None, vendor=None, simulation=False):
    from compat_record import recompute_config_sha256  # local import: one package
    robot_cfg = load_toml(config_path)
    subsystem_cfgs = {}
    for name, entry in roster_subsystems(robot_cfg):
        spath = subsystem_file(config_path, entry)
        if spath and Path(spath).exists():
            subsystem_cfgs[name] = load_toml(spath)
    sections = [
        ("processes", check_processes(robot_cfg, manifest)),
        ("hardware", check_hardware(robot_cfg, subsystem_cfgs, simulation)),
    ]
    bundle_dir = None
    if manifest is not None:
        bundle_dir = manifest.get("_bundle_dir")
    if bundle_dir is not None:
        actual_sha = recompute_config_sha256(str(Path(bundle_dir) / "config"))
        sections.append(("config", check_config(manifest, actual_sha)))
    else:
        sections.append(("config", [("config_sha256", False,
                                     "no bundle supplied: install the bundle "
                                     "first (see DEPLOYMENT.md)")]))
    sections.append(("feedback", check_feedback(subsystem_cfgs)))
    if vendor:
        sections.append(("vendor/firmware",
                         [("expectations", True,
                           f"{len(vendor.get('devices', {}))} device types pinned")]))
    else:
        sections.append(("vendor/firmware",
                         [("expectations", False,
                           "no vendor/firmware expectations recorded; pass "
                           "--vendor vendor_expectations.json")]))
    inhibited = any(not ok for _, findings in sections for _, ok, _ in findings)
    return sections, inhibited


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True)
    parser.add_argument("--bundle", default=None)
    parser.add_argument("--vendor", default=None)
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args(argv)
    manifest = None
    if args.bundle:
        manifest = json.loads(
            (Path(args.bundle) / "bundle_manifest.json").read_text())
        manifest["_bundle_dir"] = args.bundle
    vendor = json.loads(Path(args.vendor).read_text()) if args.vendor else None
    sections, inhibited = assess(args.config, manifest, vendor,
                                 simulation=args.sim)
    sys.stdout.write(render_report(sections, inhibited))
    return 1 if inhibited else 0


if __name__ == "__main__":
    sys.exit(main())
