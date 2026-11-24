#!/usr/bin/env python3
"""Writes a robot's MuJoCo model (MJCF) from its parameter file.

    generate_robot.py --parameters <robot_params.toml> --out <robot.xml>
    generate_robot.py --check --parameters <...> --out <...>

Stdlib only (Python 3.11, `tomllib`). README.md gives every parameter, the
conventions the model follows and what it does not model. REQ-DEV-41.

The parameter file describes a robot by parts (frame, bumpers, swerve
modules, an intake ramp with rollers, a hopper whose floor falls to a
feeder roller that gates a pocket, a flywheel with its hood); every number
in the model comes from it or from the constants below. A parameter the file misspells, omits or sets
to an impossible value is refused, naming the key. `--check` writes nothing
and exits 1 with the regenerate command and a diff when `--out` is not what
the parameters give.

Frame: x forward (the intake side), y left, z up, metres, origin on the
carpet under the frame centre. Positive rotation of every motor's joint is
the mechanism's forward action (drive: +x on the carpet at steer 0; intake
roller: ingest; feeder: its underside toward the rear; flywheel: shoot).

Output is a function of the file's contents and name alone: attributes are
written in a fixed order and every number as `%.6g`, so a last-bit
difference between two hosts' libm shows only at a rounding boundary.
"""

import argparse
import difflib
import math
import os
import sys
import tomllib
import xml.etree.ElementTree as ET

# Contact settings, fixed for every generated model (README.md "Contact").
# Default robot geom: the field's collision bits (fields/rebuilt_2026).
_ROBOT_CONTYPE = "4"
_ROBOT_CONAFFINITY = "21"
_DEFAULT_FRICTION = "0.6 0.005 0.0001"
_BUMPER_FRICTION = "0.8 0.005 0.0001"
_WHEEL_CONTACT = {
    "friction": "1.1 0.005 0.0001",
    "condim": "3",
    "solref": "0.02 1",
    "solimp": "0.9 0.95 0.001",
    "priority": "1",
}
_ROLLER_CONTACT = {
    "friction": "1.2 0.005 0.0001",
    "condim": "3",
    "solref": "0.01 1",
    "solimp": "0.9 0.95 0.002",
    "priority": "2",
}
_BELT_SOLREF = "0.01 1"

# Fixed shapes: plate thickness, the hood's segment count, how far the exit
# check looks.
_PLATE_M = 0.01
_FENCE_GAP_M = 0.03
_HOOD_SEGMENTS = 8
_EXIT_CHECK_M = 0.3
_EXIT_CHECK_STEP_M = 0.005
_MAX_PRELOADS = 8
_MAX_ROLLERS = 8
_MAX_NAME_BYTES = 63  # talOS/hardware/config.h kMaxConfigNameLength
_TOLERANCE_M = 1e-6


class ParameterError(Exception):
    """A parameter is missing, unknown or impossible; `key` names it."""

    def __init__(self, key, why):
        super().__init__(f"{key}: {why}")
        self.key = key


# -- reading the parameter file ----------------------------------------------


class _Table:
    """One TOML table, read key by key; `close` refuses the keys not read."""

    def __init__(self, data, path):
        if not isinstance(data, dict):
            raise ParameterError(path, "is not a table")
        self.data = data
        self.path = path
        self.used = set()

    def key(self, name):
        return f"{self.path}.{name}" if self.path else name

    def has(self, name):
        return name in self.data

    def _get(self, name, optional):
        self.used.add(name)
        if name not in self.data:
            if optional:
                return None
            raise ParameterError(self.key(name), "is required")
        return self.data[name]

    def number(self, name, *, above=None, minimum=None, below=None,
               maximum=None, optional=False):
        value = self._get(name, optional)
        if value is None:
            return None
        return _check_number(value, self.key(name), above=above,
                             minimum=minimum, below=below, maximum=maximum)

    def integer(self, name, *, minimum, maximum):
        value = self._get(name, False)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ParameterError(self.key(name), "is not an integer")
        if not minimum <= value <= maximum:
            raise ParameterError(self.key(name),
                                 f"must be from {minimum} to {maximum}")
        return value

    def name(self, name):
        value = self._get(name, False)
        return _check_name(value, self.key(name))

    def choice(self, name, options):
        value = self._get(name, False)
        if value not in options:
            raise ParameterError(self.key(name),
                                 "must be one of " + ", ".join(options))
        return value

    def vector(self, name):
        return _check_vector(self._get(name, False), self.key(name))

    def vectors(self, name):
        value = self._get(name, False)
        if not isinstance(value, list):
            raise ParameterError(self.key(name), "is not an array")
        return [_check_vector(v, f"{self.key(name)}[{i}]")
                for i, v in enumerate(value)]

    def table(self, name, optional=False):
        value = self._get(name, optional)
        return None if value is None else _Table(value, self.key(name))

    def tables(self, name, optional=False):
        value = self._get(name, optional)
        if value is None:
            return []
        if not isinstance(value, list) or not value:
            raise ParameterError(self.key(name),
                                 "is not a non-empty array of tables")
        return [_Table(v, f"{self.key(name)}[{i}]") for i, v in enumerate(value)]

    def close(self):
        for name in self.data:
            if name not in self.used:
                raise ParameterError(self.key(name), "is not a parameter")


def _check_number(value, key, *, above=None, minimum=None, below=None,
                  maximum=None):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value)):
        raise ParameterError(key, "is not a finite number")
    value = float(value)
    if above is not None and not value > above:
        raise ParameterError(key, f"must be above {above:g}")
    if minimum is not None and not value >= minimum:
        raise ParameterError(key, f"must be at least {minimum:g}")
    if below is not None and not value < below:
        raise ParameterError(key, f"must be below {below:g}")
    if maximum is not None and not value <= maximum:
        raise ParameterError(key, f"must be at most {maximum:g}")
    return value


def _check_name(value, key):
    """A device name as a hardware config accepts it: a non-empty string of
    at most 63 bytes. A control character is refused too: XML cannot carry
    most of them, and a parser turns a tab or newline in an attribute into
    a space, so the model would name a different device."""
    if not isinstance(value, str) or not value:
        raise ParameterError(key, "is not a non-empty string")
    if len(value.encode("utf-8")) > _MAX_NAME_BYTES:
        raise ParameterError(key, f"is longer than {_MAX_NAME_BYTES} bytes, "
                             "a hardware config's limit on a device name")
    if any(ord(c) < 0x20 or ord(c) == 0x7F for c in value):
        raise ParameterError(key, "holds a control character, which the "
                             "model's XML cannot carry unchanged")
    return value


def _check_vector(value, key):
    if not isinstance(value, list) or len(value) != 3:
        raise ParameterError(key, "is not an array of three numbers")
    return tuple(_check_number(v, f"{key}[{i}]") for i, v in enumerate(value))


def read_parameters(data):
    """Validates a parsed parameter file; returns plain dicts by part."""
    top = _Table(data, "")
    p = {}

    t = top.table("robot")
    p["robot"] = {
        "name": t.name("name"),
        "root": t.choice("root", ("free", "welded")),
        "mass_kg": t.number("mass_kg", above=0),
    }
    t.close()

    t = top.table("game_piece")
    radius = t.number("radius_m", above=0)
    p["game_piece"] = {"radius_m": radius}
    t.close()

    t = top.table("frame")
    length = t.number("length_m", above=0)
    width = t.number("width_m", above=0)
    p["frame"] = {
        "length_m": length,
        "width_m": width,
        "belly_clearance_m": t.number("belly_clearance_m", minimum=0),
        "rail_height_m": t.number("rail_height_m", above=0),
        "rail_width_m": t.number("rail_width_m", above=0,
                                 below=min(length, width) / 2),
    }
    t.close()

    t = top.table("bumpers")
    p["bumpers"] = {
        # Below the game piece's equator, so a piece is pushed, not ridden.
        "bottom_m": t.number("bottom_m", minimum=0, below=radius),
        "height_m": t.number("height_m", above=0),
        "thickness_m": t.number("thickness_m", above=0),
    }
    t.close()

    motors = {}
    t = top.table("motors", optional=True)
    if t is not None:
        for name in t.data:
            _check_name(name, t.key(name))
            m = t.table(name)
            motors[name] = {
                "gear": m.number("gear", above=0),
                "rotor_inertia_kg_m2": m.number("rotor_inertia_kg_m2",
                                                above=0),
            }
            m.close()
        t.close()
    p["motors"] = motors

    t = top.table("swerve", optional=True)
    if t is not None:
        modules = []
        for m in t.tables("modules"):
            x = m.number("x_m", above=-length / 2, below=length / 2)
            y = m.number("y_m", above=-width / 2, below=width / 2)
            modules.append({
                "name": m.name("name"), "key": m.path, "x_m": x, "y_m": y,
                "drive_motor": m.name("drive_motor"),
                "steer_motor": m.name("steer_motor"),
            })
            m.close()
        if len(modules) < 3:
            raise ParameterError(t.key("modules"), "holds fewer than three "
                                 "modules; a swerve drivetrain needs three "
                                 "off one line")
        def area(a, b, c):
            return abs((b["x_m"] - a["x_m"]) * (c["y_m"] - a["y_m"]) -
                       (b["y_m"] - a["y_m"]) * (c["x_m"] - a["x_m"]))
        if all(area(a, b, c) < 1e-6 for i, a in enumerate(modules)
               for j, b in enumerate(modules[:i]) for c in modules[:j]):
            raise ParameterError(t.key("modules"), "are all on one line")
        p["swerve"] = {
            "wheel_radius_m": t.number("wheel_radius_m", above=0),
            "wheel_mass_kg": t.number("wheel_mass_kg", above=0),
            "module_mass_kg": t.number("module_mass_kg", above=0),
            "modules": modules,
        }
        t.close()

    diameter = 2 * radius
    t = top.table("intake", optional=True)
    if t is not None:
        first = t.number("roller_first_fraction", minimum=0, maximum=1)
        p["intake"] = {
            "motor": t.name("motor"),
            "roller_count": t.integer("roller_count", minimum=1,
                                      maximum=_MAX_ROLLERS),
            "roller_radius_m": t.number("roller_radius_m", above=0),
            "roller_mass_kg": t.number("roller_mass_kg", above=0),
            "roller_first_fraction": first,
            "roller_last_fraction": t.number("roller_last_fraction",
                                             minimum=first, maximum=1),
            "ramp_angle_deg": t.number("ramp_angle_deg", above=0, below=90),
            "lip_height_m": t.number("lip_height_m", minimum=0),
            "top_above_bumper_m": t.number("top_above_bumper_m", minimum=0),
            "width_m": t.number("width_m", above=diameter),
            "compression_m": t.number("compression_m", minimum=0,
                                      below=diameter),
        }
        t.close()

    t = top.table("hopper", optional=True)
    if t is not None:
        p["hopper"] = {
            "width_m": t.number("width_m", above=diameter),
            "floor_front_m": t.number("floor_front_m", above=0, optional=True),
            "floor_front_key": t.key("floor_front_m"),
            # A step down from the intake's bridge: a piece rolling forward
            # (braking, a nose-down tilt) meets a wall it cannot climb.
            "step_m": t.number("step_m", above=0, below=radius, optional=True),
            "step_key": t.key("step_m"),
            "floor_slope_deg": t.number("floor_slope_deg", above=0, below=45),
            "wall_top_m": t.number("wall_top_m", above=0),
        }
        t.close()

    t = top.table("feeder", optional=True)
    if t is not None:
        p["feeder"] = {
            "motor": t.name("motor"),
            "ratio": t.number("ratio", optional=True),
            "ratio_key": t.key("ratio"),
            "x_m": t.number("x_m"),
            "roller_radius_m": t.number("roller_radius_m", above=0),
            "roller_mass_kg": t.number("roller_mass_kg", above=0),
            "compression_m": t.number("compression_m", minimum=0,
                                      below=diameter),
        }
        t.close()

    t = top.table("flywheel", optional=True)
    if t is not None:
        p["flywheel"] = {
            "motor": t.name("motor"),
            "radius_m": t.number("radius_m", above=0),
            "width_m": t.number("width_m", above=0),
            "mass_kg": t.number("mass_kg", above=0),
            "inertia_kg_m2": t.number("inertia_kg_m2", above=0),
            "axle_x_m": t.number("axle_x_m"),
            "axle_z_m": t.number("axle_z_m", above=0),
            "compression_m": t.number("compression_m", minimum=0,
                                      below=diameter),
            "hood_start_deg": t.number("hood_start_deg", above=-90, below=90),
            "exit_elevation_deg": t.number("exit_elevation_deg", above=0,
                                           below=90),
            "exit_offset_m": t.number("exit_offset_m", minimum=0),
            "efficiency": t.number("efficiency", above=0, maximum=1),
        }
        t.close()

    beams = []
    for b in top.tables("beam_breaks", optional=True):
        direction = b.vector("direction")
        if math.hypot(*direction) == 0:
            raise ParameterError(b.key("direction"), "is the zero vector")
        beams.append({
            "name": b.name("name"), "key": b.path,
            "position_m": b.vector("position_m"),
            "direction": direction,
            "cutoff_m": b.number("cutoff_m", above=0),
        })
        b.close()
    p["beam_breaks"] = beams

    t = top.table("preload", optional=True)
    p["preload"] = []
    if t is not None:
        positions = t.vectors("positions_m")
        if len(positions) > _MAX_PRELOADS:
            raise ParameterError(t.key("positions_m"),
                                 f"holds {len(positions)} positions; a robot "
                                 f"starts with at most {_MAX_PRELOADS}")
        p["preload"] = positions
        t.close()

    top.close()
    _check_parts(p)
    return p


def _check_parts(p):
    """Cross-part rules: which parts need which, and who owns each motor."""
    root = p["robot"]["root"]
    if root == "free" and "swerve" not in p:
        raise ParameterError("robot.root", "a free root needs [swerve]")
    if root == "welded" and "swerve" in p:
        raise ParameterError("swerve", "a welded root has no drivetrain")
    for part, needs in (("hopper", ("flywheel", "feeder")),
                        ("feeder", ("hopper",)), ("flywheel", ("hopper",))):
        for need in needs:
            if part in p and need not in p:
                raise ParameterError(part, f"needs [{need}]")
    if "hopper" in p:
        hopper = p["hopper"]
        with_intake = "intake" in p
        for key, value, wanted in (
                (hopper["floor_front_key"], hopper["floor_front_m"],
                 not with_intake),
                (hopper["step_key"], hopper["step_m"], with_intake)):
            if wanted and value is None:
                raise ParameterError(key, "is required " + (
                    "with an [intake]" if with_intake else
                    "without an [intake]"))
            if not wanted and value is not None:
                raise ParameterError(key, "is refused " + (
                    "with an [intake]" if with_intake else
                    "without an [intake]"))
        if with_intake and p["intake"]["width_m"] > hopper["width_m"]:
            raise ParameterError("intake.width_m", "is wider than the "
                                 "hopper it delivers into")

    # Each motor drives one part; the feeder alone may share the intake's.
    references = []
    for module in p.get("swerve", {}).get("modules", []):
        references.append((module["drive_motor"],
                           module["key"] + ".drive_motor"))
        references.append((module["steer_motor"],
                           module["key"] + ".steer_motor"))
    for part in ("intake", "feeder", "flywheel"):
        if part in p:
            references.append((p[part]["motor"], part + ".motor"))
    owner = {}
    for motor, key in references:
        if motor not in p["motors"]:
            raise ParameterError(key, f"names motor '{motor}', which "
                                 "[motors] does not declare")
        shared = (key == "feeder.motor" and owner.get(motor) == "intake.motor")
        if motor in owner and not shared:
            raise ParameterError(key, f"motor '{motor}' already drives "
                                 f"{owner[motor]}")
        owner.setdefault(motor, key)
    for motor in p["motors"]:
        if motor not in owner:
            raise ParameterError(f"motors.{motor}", "drives no part")
    if "feeder" in p:
        feeder = p["feeder"]
        belted = "intake" in p and feeder["motor"] == p["intake"]["motor"]
        if belted and feeder["ratio"] is None:
            raise ParameterError(feeder["ratio_key"], "is required for a "
                                 "feeder belted to the intake's motor")
        if not belted and feeder["ratio"] is not None:
            raise ParameterError(feeder["ratio_key"], "applies only to a "
                                 "feeder belted to the intake's motor")
        if belted and feeder["ratio"] == 0:
            raise ParameterError(feeder["ratio_key"], "must not be zero")

    # talOS device names are unique across a config's motors and inputs.
    devices = {m: f"motors.{m}" for m in p["motors"]}
    for beam in p["beam_breaks"]:
        if beam["name"] in devices:
            raise ParameterError(beam["key"] + ".name",
                                 f"'{beam['name']}' is also "
                                 f"{devices[beam['name']]}")
        devices[beam["name"]] = beam["key"] + ".name"


# -- geometry -----------------------------------------------------------------

_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


class _Solid:
    """A collision geom in the root body's frame at the reference pose."""

    def __init__(self, name, kind, centre, rotation, size):
        self.name = name
        self.kind = kind
        self.centre = centre
        self.rotation = rotation
        self.size = size

    def distance(self, point):
        """Signed distance from `point` to the surface; negative inside."""
        d = [point[i] - self.centre[i] for i in range(3)]
        r = self.rotation
        q = [sum(r[k][i] * d[k] for k in range(3)) for i in range(3)]
        if self.kind == "sphere":
            return math.hypot(*q) - self.size[0]
        if self.kind == "box":
            gaps = [abs(q[i]) - self.size[i] for i in range(3)]
        else:  # cylinder along its local z
            gaps = [math.hypot(q[0], q[1]) - self.size[0],
                    abs(q[2]) - self.size[1]]
        outside = math.sqrt(sum(max(g, 0.0) ** 2 for g in gaps))
        return outside + min(max(gaps), 0.0)


def _num(x):
    if abs(x) < 1e-12:
        x = 0.0
    return "%.6g" % x


def _nums(*values):
    return " ".join(_num(v) for v in values)


def _axis_sign(centre, contact, travel):
    """+1 or -1: the sign of a rotation about +y that moves a roller's
    surface at `contact` along `travel` (x-z pairs)."""
    rx, rz = contact[0] - centre[0], contact[1] - centre[1]
    # omega = (0, s, 0); v = omega x r = (s*rz, 0, -s*rx).
    return 1 if rz * travel[0] - rx * travel[1] > 0 else -1


# -- the model ----------------------------------------------------------------


class _Builder:

    def __init__(self, p, source):
        self.p = p
        self.source = source
        self.radius = p["game_piece"]["radius_m"]
        self.diameter = 2 * self.radius
        self.names = {}
        self.solids = []
        self.moving = []    # (solid, the part's key, body) of moving parts
        self.hopper_volume = None
        self.moving_mass = 0.0
        self.actuated = {}  # motor -> actuated joint name
        self.inlets = []    # (centre, half size) of each inlet box
        self.intake_top = None

        self.root = ET.Element("mujoco", model=p["robot"]["name"])
        ET.SubElement(self.root, "compiler", angle="radian",
                      autolimits="true")
        default = ET.SubElement(self.root, "default")
        ET.SubElement(default, "geom", contype=_ROBOT_CONTYPE,
                      conaffinity=_ROBOT_CONAFFINITY, condim="3",
                      friction=_DEFAULT_FRICTION)
        self.world = ET.SubElement(self.root, "worldbody")
        self.contact = ET.SubElement(self.root, "contact")
        self.equality = ET.SubElement(self.root, "equality")
        self.actuators = ET.SubElement(self.root, "actuator")
        self.sensors = ET.SubElement(self.root, "sensor")
        self.custom = ET.SubElement(self.root, "custom")

    def claim(self, kind, name, key):
        """Every element name once per MuJoCo type; a clash names the key
        that asked for the second."""
        if (kind, name) in self.names:
            raise ParameterError(key, f"{kind} '{name}' is also made by "
                                 f"{self.names[(kind, name)]}")
        self.names[(kind, name)] = key
        return name

    # -- elements -------------------------------------------------------------

    def body(self, parent, name, key, pos):
        """A body; a moving part never collides with the chassis it is
        mounted in. MuJoCo's parent filter does not cover a welded root
        (its parent is then the world), nor a swerve wheel (a grandchild)."""
        self.claim("body", name, key)
        if parent is not self.world:
            ET.SubElement(self.contact, "exclude", body1="chassis", body2=name)
        return ET.SubElement(parent, "body", name=name, pos=_nums(*pos))

    def hinge(self, body, name, key, axis, armature=0.0):
        self.claim("joint", name, key)
        attributes = {"name": name, "type": "hinge", "axis": _nums(*axis)}
        if armature:
            attributes["armature"] = _num(armature)
        ET.SubElement(body, "joint", attributes)

    def geom(self, body, name, key, kind, centre, size, rotation=_IDENTITY,
             body_pos=(0.0, 0.0, 0.0), part=None, **extra):
        """A collision geom at `centre` in `body`'s frame (`body_pos` in the
        root's, bodies being unrotated at the reference pose). `part` names
        the parameter of a moving part, for the overlap check."""
        self.claim("geom", name, key)
        attributes = {"name": name, "type": kind}
        if any(centre):
            attributes["pos"] = _nums(*centre)
        attributes["size"] = _nums(*size)
        if rotation != _IDENTITY:
            attributes["xyaxes"] = _nums(
                rotation[0][0], rotation[1][0], rotation[2][0],
                rotation[0][1], rotation[1][1], rotation[2][1])
        attributes.update(extra)
        ET.SubElement(body, "geom", attributes)
        root_centre = tuple(body_pos[i] + centre[i] for i in range(3))
        solid = _Solid(name, kind, root_centre, rotation, size)
        self.solids.append(solid)
        if part is not None:
            self.moving.append((solid, part, body))

    def box(self, name, key, centre, half, rotation=_IDENTITY, **extra):
        self.geom(self.chassis, name, key, "box", centre, half, rotation,
                  **extra)

    def plate(self, name, key, p0, p1, width, **extra):
        """A plate in the x-z plane from p0 to p1, `width` along y. Its
        surface is the segment; its thickness lies on the right of p0->p1
        (x right, z up), so the surface faces left."""
        dx, dz = p1[0] - p0[0], p1[1] - p0[1]
        length = math.hypot(dx, dz)
        nx, nz = -dz / length, dx / length
        centre = ((p0[0] + p1[0]) / 2 - nx * _PLATE_M / 2, 0.0,
                  (p0[1] + p1[1]) / 2 - nz * _PLATE_M / 2)
        self.box(name, key, centre, (length / 2, width / 2, _PLATE_M / 2),
                 _ry(-math.atan2(dz, dx)), **extra)

    def roller(self, name, key, centre, radius, length, mass, sign, motor,
               driver=None, ratio=1.0):
        """A cylinder along y on a hinge. With `driver` (an actuated joint)
        it is belted to it at `ratio`; otherwise its joint is `motor`'s."""
        pos = (centre[0], 0.0, centre[1])
        body = self.body(self.chassis, name, key, pos)
        if driver is None:
            joint = motor + "_joint"
            motor_p = self.p["motors"][motor]
            self.hinge(body, joint, key, (0, sign, 0),
                       motor_p["gear"] ** 2 * motor_p["rotor_inertia_kg_m2"])
            self.actuated[motor] = joint
        else:
            joint = name + "_joint"
            self.hinge(body, joint, key, (0, sign, 0))
            ET.SubElement(self.equality, "joint", joint1=joint, joint2=driver,
                          polycoef=_nums(0, ratio, 0, 0, 0),
                          solref=_BELT_SOLREF)
        self.geom(body, name + "_geom", key, "cylinder", (0.0, 0.0, 0.0),
                  (radius, length / 2), _rx(math.pi / 2), body_pos=pos,
                  part=key, mass=_num(mass), **_ROLLER_CONTACT)
        self.moving_mass += mass
        return joint

    def motor(self, name, joint):
        ET.SubElement(self.actuators, "motor", name=name, joint=joint)

    # -- parts ----------------------------------------------------------------

    def build(self):
        p = self.p
        a = p["frame"]["length_m"] / 2
        b = p["frame"]["width_m"] / 2
        free = p["robot"]["root"] == "free"
        self.chassis = self.body(self.world, "chassis", "robot",
                                 (0.0, 0.0, 0.002 if free else 0.0))
        if free:
            self.claim("joint", "chassis_joint", "robot.root")
            ET.SubElement(self.chassis, "freejoint", name="chassis_joint",
                          align="false")
        self.frame(a, b)
        self.bumpers(a, b)
        for module in p.get("swerve", {}).get("modules", []):
            self.module(module)
        if "intake" in p:
            self.intake(a)
        if "flywheel" in p:
            self.flywheel()
        if "hopper" in p:
            self.hopper(a)
        for beam in p["beam_breaks"]:
            self.beam(beam)
        self.check_overlaps()
        self.chassis_inertia(a, b)
        self.check_exit()
        self.preloads()
        for section in (self.contact, self.equality, self.actuators,
                        self.sensors, self.custom):
            if len(section) == 0:
                self.root.remove(section)
        ET.indent(self.root, space="  ")
        return (f"<!-- Generated by generate_robot.py from {self.source}; do "
                "not edit (talOS/simulation/mujoco/robots/README.md). -->\n" +
                ET.tostring(self.root, encoding="unicode") + "\n")

    def frame(self, a, b):
        f = self.p["frame"]
        z0, rh, rw = (f["belly_clearance_m"], f["rail_height_m"],
                      f["rail_width_m"])
        zc = z0 + rh / 2
        self.box("rail_front", "frame", (a - rw / 2, 0.0, zc),
                 (rw / 2, b, rh / 2))
        self.box("rail_back", "frame", (-a + rw / 2, 0.0, zc),
                 (rw / 2, b, rh / 2))
        self.box("rail_left", "frame", (0.0, b - rw / 2, zc),
                 (a, rw / 2, rh / 2))
        self.box("rail_right", "frame", (0.0, -b + rw / 2, zc),
                 (a, rw / 2, rh / 2))
        self.box("belly", "frame", (0.0, 0.0, z0 + 0.003), (a, b, 0.003))

    def bumpers(self, a, b):
        bp = self.p["bumpers"]
        t, z = bp["thickness_m"], bp["bottom_m"] + bp["height_m"] / 2
        h = bp["height_m"] / 2
        sides = (("front", (a + t / 2, 0.0), (t / 2, b + t)),
                 ("back", (-a - t / 2, 0.0), (t / 2, b + t)),
                 ("left", (0.0, b + t / 2), (a, t / 2)),
                 ("right", (0.0, -b - t / 2), (a, t / 2)))
        for side, (cx, cy), (hx, hy) in sides:
            self.box("bumper_" + side, "bumpers", (cx, cy, z), (hx, hy, h),
                     friction=_BUMPER_FRICTION)

    def module(self, m):
        s = self.p["swerve"]
        r = s["wheel_radius_m"]
        key = m["key"]
        steer_p = self.p["motors"][m["steer_motor"]]
        drive_p = self.p["motors"][m["drive_motor"]]
        pos = (m["x_m"], m["y_m"], r)
        steer = self.body(self.chassis, m["name"] + "_module", key + ".name",
                          pos)
        self.hinge(steer, m["steer_motor"] + "_joint", key + ".steer_motor",
                   (0, 0, 1),
                   steer_p["gear"] ** 2 * steer_p["rotor_inertia_kg_m2"])
        ET.SubElement(steer, "inertial", pos="0 0 0.05",
                      mass=_num(s["module_mass_kg"]),
                      diaginertia="0.004 0.004 0.004")
        wheel = self.body(steer, m["name"] + "_wheel", key + ".name",
                          (0.0, 0.0, 0.0))
        self.hinge(wheel, m["drive_motor"] + "_joint", key + ".drive_motor",
                   (0, 1, 0),
                   drive_p["gear"] ** 2 * drive_p["rotor_inertia_kg_m2"])
        self.geom(wheel, m["name"] + "_wheel_geom", key + ".name", "sphere",
                  (0.0, 0.0, 0.0), (r,), body_pos=pos, part=key,
                  mass=_num(s["wheel_mass_kg"]), **_WHEEL_CONTACT)
        self.moving_mass += s["module_mass_kg"] + s["wheel_mass_kg"]
        self.motor(m["drive_motor"], m["drive_motor"] + "_joint")
        self.motor(m["steer_motor"], m["steer_motor"] + "_joint")

    def intake(self, a):
        it, bp = self.p["intake"], self.p["bumpers"]
        d = self.diameter
        # The ramp tops out just in front of the bumper's top edge; a bridge
        # carries the piece over the bumper into the hopper.
        top = (a + bp["thickness_m"] + 0.005,
               bp["bottom_m"] + bp["height_m"] + it["top_above_bumper_m"])
        rise = top[1] - it["lip_height_m"]
        if rise <= 0:
            raise ParameterError("intake.lip_height_m",
                                 "is not below the ramp's top")
        lip = (top[0] + rise / math.tan(math.radians(it["ramp_angle_deg"])),
               it["lip_height_m"])
        self.intake_top = top
        width = it["width_m"]
        self.plate("intake_ramp", "intake", top, lip, width)
        self.plate("intake_bridge", "intake", (a - 0.02, top[1]), top, width)
        length = math.hypot(lip[0] - top[0], lip[1] - top[1])
        up = ((top[0] - lip[0]) / length, (top[1] - lip[1]) / length)
        normal = (-up[1], up[0])  # away from the ramp's surface
        if normal[1] < 0:
            normal = (-normal[0], -normal[1])
        rr = it["roller_radius_m"]
        offset = d - it["compression_m"] + rr
        first, last = it["roller_first_fraction"], it["roller_last_fraction"]
        count = it["roller_count"]
        driver = None
        for i in range(count):
            s = length * (first + (last - first) * i / max(count - 1, 1))
            base = (lip[0] + up[0] * s, lip[1] + up[1] * s)
            centre = (base[0] + normal[0] * offset,
                      base[1] + normal[1] * offset)
            contact = (centre[0] - normal[0] * rr, centre[1] - normal[1] * rr)
            joint = self.roller(
                f"intake_roller_{i}", "intake", centre, rr, width,
                it["roller_mass_kg"], _axis_sign(centre, contact, up),
                it["motor"], driver)
            if driver is None:
                driver = joint
                self.motor(it["motor"], joint)
        for sy, side in ((1, "left"), (-1, "right")):
            self.box(f"intake_side_{side}", "intake",
                     ((lip[0] + top[0]) / 2, sy * (width / 2 + 0.01), 0.20),
                     ((lip[0] - top[0]) / 2 + 0.05, 0.005, 0.12))

    def flywheel(self):
        fw = self.p["flywheel"]
        d = self.diameter
        motor = fw["motor"]
        ax, az = fw["axle_x_m"], fw["axle_z_m"]
        motor_p = self.p["motors"][motor]
        pos = (ax, 0.0, az)
        body = self.body(self.chassis, "shooter_flywheel", "flywheel", pos)
        # Positive rotation moves the front (+x) surface up: about -y.
        joint = motor + "_joint"
        self.hinge(body, joint, "flywheel.motor", (0, -1, 0),
                   motor_p["gear"] ** 2 * motor_p["rotor_inertia_kg_m2"])
        self.actuated[motor] = joint
        self.geom(body, "shooter_flywheel_geom", "flywheel", "cylinder",
                  (0.0, 0.0, 0.0), (fw["radius_m"], fw["width_m"] / 2),
                  _rx(math.pi / 2), body_pos=pos, part="flywheel",
                  **_ROLLER_CONTACT)
        inertia = fw["inertia_kg_m2"]
        ET.SubElement(body, "inertial", pos="0 0 0", mass=_num(fw["mass_kg"]),
                      diaginertia=_nums(inertia / 2 + 0.001, inertia,
                                        inertia / 2 + 0.001))
        self.moving_mass += fw["mass_kg"]
        self.motor(motor, joint)
        # Hood: an arc about the axle, on the inside of radius `hood`, from
        # hood_start_deg to where the exit tangent has the exit elevation.
        hood = fw["radius_m"] + d - fw["compression_m"]
        a0 = math.radians(fw["hood_start_deg"])
        a1 = math.radians(90.0 - fw["exit_elevation_deg"])
        points = [(ax + hood * math.cos(a0 + (a1 - a0) * i / _HOOD_SEGMENTS),
                   az + hood * math.sin(a0 + (a1 - a0) * i / _HOOD_SEGMENTS))
                  for i in range(_HOOD_SEGMENTS + 1)]
        for i in range(_HOOD_SEGMENTS):
            self.plate(f"hood_{i}", "flywheel", points[i], points[i + 1],
                       0.20, **_ROLLER_CONTACT)
        # The scripted-shooter contract (README.md); the hopper places the
        # inlet.
        elevation = math.radians(fw["exit_elevation_deg"])
        tx, tz = -math.cos(elevation), math.sin(elevation)
        centre_radius = hood - d / 2
        self.exit = (ax + centre_radius * math.cos(a1) + tx * fw["exit_offset_m"],
                     0.0,
                     az + centre_radius * math.sin(a1) + tz * fw["exit_offset_m"])
        self.exit_axis = (tx, 0.0, tz)
        self.claim("site", motor + "_exit", "flywheel.motor")
        ET.SubElement(self.chassis, "site", name=motor + "_exit",
                      pos=_nums(*self.exit), xyaxes=_nums(tx, 0, tz, 0, 1, 0),
                      size="0.02", rgba="0 1 0 1", group="4")
        self.claim("numeric", motor + "_shooter", "flywheel.motor")
        ET.SubElement(self.custom, "numeric", name=motor + "_shooter",
                      data=_nums(fw["efficiency"], fw["radius_m"]))

    def hopper(self, a):
        """The hopper's floor falls toward the rear across its full width to
        the feeder's roller, a gate whose underside is a piece's diameter less
        `compression_m` above the floor: a piece against it is held while it
        is still and is pushed under, rolling along the floor, while it turns.
        Behind it a flat pocket, closed at the back and the sides, is the
        scripted shooter's inlet."""
        p = self.p
        hp, fw, fd = p["hopper"], p["flywheel"], p["feeder"]
        d, r = self.diameter, self.radius
        width = hp["width_m"]
        b = p["frame"]["width_m"] / 2
        if width / 2 + 0.01 > b:
            raise ParameterError("hopper.width_m", "is wider than the frame "
                                 "leaves room for between its side walls")
        front_x = a - 0.02
        front_z = hp["floor_front_m"]
        if front_z is None:  # a step below the intake's bridge
            front_z = self.intake_top[1] - hp["step_m"]
        gate_x, rr = fd["x_m"], fd["roller_radius_m"]
        back_x = gate_x - rr - d - 0.02  # the pocket's back wall
        if not back_x - 0.01 > -a:
            raise ParameterError("feeder.x_m", "leaves no room inside the "
                                 "frame for the pocket behind the roller")
        if not gate_x + rr + d < front_x:
            raise ParameterError("feeder.x_m", "leaves no hopper floor in "
                                 "front of the roller")
        slope = math.radians(hp["floor_slope_deg"])
        gate_z = front_z - (front_x - gate_x) * math.tan(slope)
        belly_top = p["frame"]["belly_clearance_m"] + 0.006
        if gate_z - _PLATE_M < belly_top:
            raise ParameterError("hopper.floor_slope_deg", "puts the floor at "
                                 "the feeder below the belly pan")
        roller_z = gate_z + d - fd["compression_m"] + rr
        if not hp["wall_top_m"] > roller_z + rr + 0.02:
            raise ParameterError("hopper.wall_top_m", "is not above the "
                                 "feeder's roller")
        if not fw["axle_z_m"] - fw["radius_m"] > gate_z + d + 0.02:
            raise ParameterError("flywheel.axle_z_m", "puts the flywheel "
                                 "within reach of a piece in the pocket")
        wall_top = hp["wall_top_m"]
        bottom = gate_z - _PLATE_M
        self.plate("hopper_floor", "hopper", (gate_x, gate_z),
                   (front_x, front_z), width)
        self.plate("pocket_floor", "hopper", (back_x, gate_z),
                   (gate_x, gate_z), width)
        for sy, side in ((1, "left"), (-1, "right")):
            self.box(f"hopper_side_{side}", "hopper",
                     (0.0, sy * (width / 2 + 0.005), (bottom + wall_top) / 2),
                     (a - 0.01, 0.005, (wall_top - bottom) / 2))
        pocket_top = gate_z + d + 0.05
        self.box("pocket_back", "hopper",
                 (back_x - 0.005, 0.0, (bottom + pocket_top) / 2),
                 (0.005, width / 2, (pocket_top - bottom) / 2))
        # The hopper's front: a wall without an intake; with one, the step's
        # face from the floor up to the bridge, and above the bridge a fence
        # leaving the intake one piece's height (plus _FENCE_GAP_M): pieces
        # thrown forward meet the face, or the fence if they ride higher.
        front_top = wall_top if "intake" not in p else self.intake_top[1]
        self.box("hopper_front", "hopper",
                 (front_x + 0.005, 0.0, (front_z - _PLATE_M + front_top) / 2),
                 (0.005, width / 2, (front_top - front_z + _PLATE_M) / 2))
        if "intake" in p:
            # One piece's depth behind the step's face, so a piece the intake
            # drops in rolls back under it on the floor.
            fence_x = front_x - d - 0.01
            fence_bottom = (front_z - (front_x - fence_x) * math.tan(slope) +
                            d + _FENCE_GAP_M)
            # The fence reaches at least a piece's radius down from the top,
            # so a piece riding on others meets it.
            if not wall_top > fence_bottom + self.radius:
                raise ParameterError("hopper.wall_top_m", "leaves the hopper's "
                                     "fence shorter than a piece's radius")
            self.box("hopper_fence", "hopper",
                     (fence_x, 0.0, (fence_bottom + wall_top) / 2),
                     (0.005, width / 2, (wall_top - fence_bottom) / 2))
        # The gate: one roller across the hopper, its underside moving
        # rearward for a positive rotation; a wall above it, so pieces pass
        # only under it.
        centre = (gate_x, roller_z)
        belted = "intake" in p and fd["motor"] == p["intake"]["motor"]
        driver = self.actuated[fd["motor"]] if belted else None
        joint = self.roller("feeder_roller", "feeder", centre, rr, width - 0.01,
                            fd["roller_mass_kg"],
                            _axis_sign(centre, (gate_x, roller_z - rr),
                                       (-1, 0)),
                            fd["motor"], driver, fd["ratio"] if belted else 1.0)
        if driver is None:
            self.motor(fd["motor"], joint)
        gate_top = roller_z + rr + 0.005
        self.box("gate_wall", "hopper",
                 (gate_x, 0.0, (gate_top + wall_top) / 2),
                 (0.005, width / 2, (wall_top - gate_top) / 2))
        # The inlet: the pocket, where the centre of every piece that has
        # cleared the roller is, a row of them abreast.
        motor = fw["motor"]
        inlet = ((back_x + gate_x) / 2, 0.0, gate_z + r)
        half = ((gate_x - back_x) / 2, width / 2, r)
        self.claim("site", motor + "_inlet", "hopper")
        ET.SubElement(self.chassis, "site", name=motor + "_inlet", type="box",
                      pos=_nums(*inlet), size=_nums(*half),
                      rgba="0 0 1 0.3", group="4")
        self.inlets.append((inlet, half))
        # Where a preload may sit: in front of the roller, between the side
        # walls, below their top.
        self.hopper_volume = ((gate_x + rr, front_x), (-width / 2, width / 2),
                              (0.0, wall_top))

    def beam(self, beam):
        key = beam["key"]
        self.claim("site", beam["name"], key + ".name")
        self.claim("sensor", beam["name"], key + ".name")
        norm = math.hypot(*beam["direction"])
        ET.SubElement(self.chassis, "site", name=beam["name"],
                      pos=_nums(*beam["position_m"]),
                      zaxis=_nums(*(v / norm for v in beam["direction"])),
                      size="0.01", rgba="1 0 0 1")
        ET.SubElement(self.sensors, "rangefinder", name=beam["name"],
                      site=beam["name"], cutoff=_num(beam["cutoff_m"]))

    def chassis_inertia(self, a, b):
        p = self.p
        mass = p["robot"]["mass_kg"] - self.moving_mass
        if mass <= 0:
            raise ParameterError(
                "robot.mass_kg", f"is not above the parts' "
                f"{self.moving_mass:g} kg; the chassis takes what they leave")
        # A uniform box over the bumpers' outline, from the belly to 0.3 m.
        t = p["bumpers"]["thickness_m"]
        h0, h1 = p["frame"]["belly_clearance_m"], 0.30
        lx, ly, lz = 2 * a + 2 * t, 2 * b + 2 * t, h1 - h0
        inertial = ET.Element("inertial", pos=_nums(0, 0, (h0 + h1) / 2),
                              mass=_num(mass),
                              diaginertia=_nums(mass * (ly**2 + lz**2) / 12,
                                                mass * (lx**2 + lz**2) / 12,
                                                mass * (lx**2 + ly**2) / 12))
        # MuJoCo wants a body's inertial before its children.
        position = 1 if self.p["robot"]["root"] == "free" else 0
        self.chassis.insert(position, inertial)

    def nearest(self, point):
        return min(((s.distance(point), s.name) for s in self.solids),
                   default=(math.inf, ""))

    def check_overlaps(self):
        """Two moving parts on different bodies may not overlap: MuJoCo
        would push them apart (or, excluded from each other, let them pass
        through) from the first step."""
        for i, (b, part_b, body_b) in enumerate(self.moving):
            for a, part_a, body_a in self.moving[:i]:
                if body_a is body_b:
                    continue
                if b.kind == "sphere":
                    gap = a.distance(b.centre) - b.size[0]
                elif a.kind == "sphere":
                    gap = b.distance(a.centre) - a.size[0]
                else:  # two cylinders along y
                    apart_y = (abs(a.centre[1] - b.centre[1]) -
                               a.size[1] - b.size[1])
                    gap = max(math.hypot(a.centre[0] - b.centre[0],
                                         a.centre[2] - b.centre[2]) -
                              a.size[0] - b.size[0], apart_y)
                if gap < -_TOLERANCE_M:
                    raise ParameterError(part_b, f"geom '{b.name}' overlaps "
                                         f"'{a.name}' of {part_a}")

    def check_exit(self):
        if "flywheel" not in self.p:
            return
        steps = round(_EXIT_CHECK_M / _EXIT_CHECK_STEP_M)
        for i in range(steps + 1):
            s = i * _EXIT_CHECK_STEP_M
            point = tuple(self.exit[k] + s * self.exit_axis[k]
                          for k in range(3))
            gap, name = self.nearest(point)
            if gap < self.radius - _TOLERANCE_M:
                raise ParameterError(
                    "flywheel.exit_offset_m",
                    f"a piece launched from the exit passes {gap:.3f} m from "
                    f"geom '{name}' {s:.3f} m along the launch axis; it must "
                    f"clear every robot geom by the game piece's radius")

    def preloads(self):
        positions = self.p["preload"]
        for i, point in enumerate(positions):
            key = f"preload.positions_m[{i}]"
            gap, name = self.nearest(point)
            if gap < self.radius - _TOLERANCE_M:
                raise ParameterError(key, f"is {gap:.3f} m from geom '{name}';"
                                     " a preload clears every robot geom by "
                                     "the game piece's radius")
            for j in range(i):
                if math.dist(point, positions[j]) < self.diameter - _TOLERANCE_M:
                    raise ParameterError(key, f"overlaps preload {j}")
            if self.hopper_volume is None:
                raise ParameterError(key, "has no [hopper] to sit in")
            (x0, x1), (y0, y1), (z0, z1) = self.hopper_volume
            if not (x0 + self.radius <= point[0] <= x1 and
                    y0 + self.radius <= point[1] <= y1 - self.radius and
                    point[2] <= z1):
                raise ParameterError(key, "is outside the hopper")
            for centre, half in self.inlets:
                if all(abs(point[k] - centre[k]) <= half[k] for k in range(3)):
                    raise ParameterError(key, "is inside the flywheel's inlet")
            name = self.claim("site", f"preload_{i}", key)
            ET.SubElement(self.chassis, "site", name=name, type="sphere",
                          pos=_nums(*point), size=_num(self.radius),
                          rgba="1 1 0 0.3", group="4")


def generate(data, source):
    """The MJCF text for a parsed parameter file named `source`."""
    return _Builder(read_parameters(data), source).build()


def generate_file(path):
    with open(path, "rb") as f:
        data = tomllib.load(f)
    return generate(data, os.path.basename(path))


_TARGET = "//talOS/simulation/mujoco/robots:generate_robot"


def _repo_mapping():
    """The lines of Bazel's runfiles repository mapping, `source,apparent,
    canonical`, or none outside Bazel's runfiles."""
    path = None
    if os.environ.get("RUNFILES_DIR"):
        path = os.path.join(os.environ["RUNFILES_DIR"], "_repo_mapping")
    elif os.environ.get("RUNFILES_MANIFEST_FILE"):
        try:
            with open(os.environ["RUNFILES_MANIFEST_FILE"],
                      encoding="utf-8") as f:
                for line in f:
                    if line.startswith("_repo_mapping "):
                        path = line.rstrip("\n").split(" ", 1)[1]
        except OSError:
            return []
    try:
        with open(path, encoding="utf-8") as f:
            return [line.rstrip("\n").split(",") for line in f]
    except (OSError, TypeError):
        return []


def command(repository):
    """The regenerate command as the main repository names this program:
    `//...` in talOS itself, `@<apparent>//...` where talOS is the
    dependency the canonical `repository` names, and `@@<canonical>//...`
    when the runfiles carry no mapping for it."""
    if not repository:
        return f"bazel run {_TARGET} --"
    for entry in _repo_mapping():
        if len(entry) == 3 and entry[0] == "" and entry[2] == repository:
            return f"bazel run @{entry[1]}{_TARGET} --"
    return f"bazel run @@{repository}{_TARGET} --"


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="generate_robot",
        description="Write a robot's MuJoCo model from its parameter file, "
        "or check that the model is current.")
    parser.add_argument("--parameters", required=True,
                        help="the robot's parameter file (TOML)")
    parser.add_argument("--out", required=True, help="the model to write")
    parser.add_argument("--check", action="store_true",
                        help="write nothing; exit 1 if --out is stale")
    parser.add_argument("--talos-repository", default="",
                        help="talOS's canonical repository name, for the "
                        "regenerate command --check prints (empty: talOS is "
                        "the main repository)")
    args = parser.parse_args(argv)

    # `bazel run` sets BUILD_WORKSPACE_DIRECTORY; `bazel test` runs in the
    # runfiles root, which mirrors the workspace layout.
    root = os.environ.get("BUILD_WORKSPACE_DIRECTORY") or os.getcwd()
    parameters = os.path.join(root, args.parameters)
    out = os.path.join(root, args.out)
    try:
        text = generate_file(parameters)
    except ParameterError as err:
        print(f"generate_robot: {args.parameters}: {err}", file=sys.stderr)
        return 1
    except (OSError, tomllib.TOMLDecodeError) as err:
        print(f"generate_robot: {args.parameters}: {err}", file=sys.stderr)
        return 1
    fix = (f"Regenerate it with:\n    {command(args.talos_repository)} "
           f"--parameters {args.parameters} --out {args.out}\n")
    try:
        with open(out, encoding="utf-8", newline="") as f:
            current = f.read()
    except OSError:
        current = None
    if args.check:
        if current is None:
            print(f"{args.out} is missing. {fix}", file=sys.stderr)
            return 1
        if current != text:
            diff = "".join(difflib.unified_diff(
                current.splitlines(keepends=True),
                text.splitlines(keepends=True), f"{args.out} (checked in)",
                f"{args.out} (from {args.parameters})"))
            print(f"{args.out} is stale. {fix}\n{diff}", file=sys.stderr)
            return 1
        return 0
    if current == text:
        print(f"{args.out} is up to date")
        return 0
    try:
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    except OSError as err:
        print(f"generate_robot: --out {args.out}: {err.strerror}",
              file=sys.stderr)
        return 1
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
