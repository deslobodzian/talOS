"""Parse a split-format subsystem.toml.

NEW split format (PLAN.md section 7, one file per subsystem)::

    [subsystem]
    name = "intake"
    node = "//2026-robot/main_processor/intake:node"
    period_us = 5000

    [subsystem.motors.roller]
    type = "TalonFX"
    bus = "rio"
    can_id = 9
    ...

    [subsystem.sensors.beam]
    type = "CANcoder"
    bus = "rio"
    can_id = 20
    ...

Any other [subsystem.<kind>.<device>] table is preserved verbatim. Motor and
sensor tables additionally require type/bus/can_id (legacy aliases
canbus->bus, id->can_id accepted) and share the robot-wide exclusivity rule:
no two devices on one (bus, can_id).
"""
import re
import tomllib
from pathlib import Path

from model import Device, Subsystem

VALID_SUBSYSTEM_KEYS = {"name", "node", "period_us"}

# Minimal keys both ends of the wire rely on. Everything else passes through.
REQUIRED_DEVICE_KEYS = {"type", "bus", "can_id"}
BUS_ALIASES = {"canbus": "bus", "can_bus": "bus"}
ID_ALIASES = {"id": "can_id", "canId": "can_id"}

NODE_NAME_RE = re.compile(r"[a-z][a-z0-9_]*")


def check_node_target(name: str, target: str) -> str | None:
    """Python mirror of naming::CheckNodeTarget, tightened per ARCHITECTURE.md
    step 7: the rule must be named `node` and the package leaf must be <name>.
    Returns an error string, or None when the target is acceptable."""
    if not target.startswith("//"):
        return f"target '{target}' for node '{name}' must be an absolute Bazel label"
    if ":" not in target:
        return f"target '{target}' for node '{name}' must name a rule, as //package:rule"
    package, rule = target[2:].split(":", 1)
    leaf = package.rsplit("/", 1)[-1]
    if rule != "node":
        return (
            f"node '{name}' is built by '{target}'; "
            f"the rule must be named 'node', not '{rule}'"
        )
    if leaf != name:
        return (
            f"node '{name}' is built by '{target}'; "
            f"the package leaf must be '{name}', not '{leaf}'"
        )
    return None


def _normalise_device_attrs(raw: dict, device_label: str) -> dict:
    attrs = dict(raw)
    for alias, canonical in BUS_ALIASES.items():
        if alias in attrs and canonical not in attrs:
            attrs[canonical] = attrs.pop(alias)
        elif alias in attrs:
            raise ValueError(f"{device_label}: '{alias}' duplicates '{canonical}'")
    for alias, canonical in ID_ALIASES.items():
        if alias in attrs and canonical not in attrs:
            attrs[canonical] = attrs.pop(alias)
        elif alias in attrs:
            raise ValueError(f"{device_label}: '{alias}' duplicates '{canonical}'")
    return attrs


def parse_subsystem(toml_path: Path | str) -> Subsystem:
    with open(toml_path, "rb") as f:
        data = tomllib.load(f)

    if "subsystem" not in data or not isinstance(data["subsystem"], dict):
        raise ValueError(f"{toml_path}: missing [subsystem] table")
    header = data["subsystem"]
    scalar = {k: v for k, v in header.items() if not isinstance(v, dict)}
    unknown = set(scalar) - VALID_SUBSYSTEM_KEYS
    if unknown:
        raise ValueError(f"{toml_path}: [subsystem] has unknown keys: {sorted(unknown)}")
    for key in ("name", "node", "period_us"):
        if key not in scalar:
            raise ValueError(f"{toml_path}: [subsystem] is missing '{key}'")

    name = scalar["name"]
    if not NODE_NAME_RE.fullmatch(name) or "__" in name or name.endswith("_"):
        raise ValueError(
            f"{toml_path}: subsystem name '{name}' violates NAMING.md "
            "(lowercase, digits, single underscores, no leading digit)"
        )
    problem = check_node_target(name, scalar["node"])
    if problem is not None:
        raise ValueError(f"{toml_path}: {problem}")
    period_us = scalar["period_us"]
    if not isinstance(period_us, int) or period_us <= 0:
        raise ValueError(f"{toml_path}: period_us must be a positive integer")

    devices: list[Device] = []
    seen_device_names: set[str] = set()
    for kind, table in header.items():
        if not isinstance(table, dict):
            continue
        for dev_name, attrs in table.items():
            if not isinstance(attrs, dict):
                raise ValueError(
                    f"{toml_path}: [subsystem.{kind}.{dev_name}] must be a table"
                )
            label = f"[subsystem.{kind}.{dev_name}]"
            if dev_name in seen_device_names:
                raise ValueError(f"{toml_path}: duplicate device name '{dev_name}'")
            seen_device_names.add(dev_name)
            norm = _normalise_device_attrs(attrs, label)
            if kind in ("motors", "sensors"):
                missing = REQUIRED_DEVICE_KEYS - set(norm)
                if missing:
                    raise ValueError(f"{toml_path}: {label} is missing {sorted(missing)}")
            devices.append(Device(name=dev_name, kind=kind, attrs=norm))

    # Ownership is exclusive: duplicate (bus, can_id) rejected at parse time.
    seen_bus_ids: dict[tuple, str] = {}
    for dev in devices:
        bus = dev.attrs.get("bus")
        can_id = dev.attrs.get("can_id")
        if bus is None or can_id is None:
            continue
        key = (bus, can_id)
        if key in seen_bus_ids:
            raise ValueError(
                f"{toml_path}: duplicate CAN device: bus={bus}, can_id={can_id} "
                f"claimed by '{seen_bus_ids[key]}' and '{dev.name}'"
            )
        seen_bus_ids[key] = dev.name

    # Every owned topic must fit the 31-char transport limit (NAMING.md).
    for topic in (f"/{name}/state", f"/{name}/target", f"/hw/request/{name}"):
        if len(topic) > 31:
            raise ValueError(
                f"{toml_path}: topic '{topic}' is {len(topic)} chars; "
                "the transport limit is 31"
            )

    return Subsystem(name=name, node_target=scalar["node"],
                     period_us=period_us, devices=devices)
