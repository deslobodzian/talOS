"""Discoverable control config for students (stdlib only).

Answers "what loop am I closing?" without opening three files: one
``MotorControlConfig`` per ``[motors.<name>]`` table in
``subsystem.toml`` carrying the PID slot's gains with their units, the
gearing point, the limits, and whether the loop is tuned at all.

Gain units (TalonFX onboard PID, as documented alongside
``2026-robot/main_processor/intake/subsystem.toml``)::

    slot  -- which onboard gain slot the demand selects (SLOT in
             node.py, slot0 here); demands carry their slot on the wire
    p     -- proportional: volts per unit of velocity error
             (volts per mechanism rev/s) / position error (volts per
             mechanism rotation), per the demand mode
    i     -- integral: volts per unit-error-second (same error units
             as p); leave zero until v+p+s cannot hold speed under load
    d     -- derivative: volts per unit-error-per-second; leave zero
             until v+p+s cannot hold speed under load
    s     -- static-friction feedforward: volts to break the wheel
             loose (sign follows the demand)
    v     -- velocity feedforward (kV): volts per mechanism rev/s;
             raise until steady-state measured ~= requested
    a     -- acceleration feedforward: volts per mechanism rev/s^2
             (zero on constant-speed rollers)
    g     -- gravity feedforward: volts of holding effort (zero on
             flat rollers; nonzero only on lifts/arms that sag)

Gearing point: ``sensor_to_mechanism_ratio`` (rotor rotations per one
mechanism rotation; 1.0 = direct drive) is where rotor and mechanism
frames meet -- see ``units.py``. The onboard loop closes on the rotor
side of that point: demands go out (and feedback comes back) in the
mechanism frame, and ``units`` crosses frames only at those
boundaries. Untuned behavior: every gain zero means UNTUNED -- the
demand still sends, but with no feedforward the wheel lags it and
requested-vs-measured splits apart in ``/intake/state``. That split
is the tune signal, never an error.
"""

from dataclasses import dataclass, field

try:
    import tomllib as _tomllib
except ImportError:  # System pythons older than 3.11 (bazel uses 3.11).
    _tomllib = None

GAIN_KEYS = ("p", "i", "d", "s", "v", "a", "g")


@dataclass
class MotorControlConfig:
    """One motor's control story, read from its ``[motors.<name>]`` table."""
    name: str
    slot: int = 0
    gains: dict = field(default_factory=dict)
    sensor_to_mechanism_ratio: float = 1.0
    max_velocity_rps: float = 100.0
    max_voltage: float = 12.0
    brake: bool = True
    inverted: bool = False

    @property
    def is_tuned(self):
        """True when any of p/i/d/s/v is nonzero (a/g don't count)."""
        return any(float(self.gains.get(k, 0.0)) != 0.0
                   for k in ("p", "i", "d", "s", "v"))

    def describe(self):
        """One human-readable paragraph: slot, gains, gearing, tune state."""
        gains = ", ".join("%s=%s" % (k, self.gains.get(k, 0.0))
                          for k in GAIN_KEYS)
        tune = ("TUNED" if self.is_tuned else
                "UNTUNED (all gains zero: demands send but the wheel "
                "lags; raise v until measured ~= requested)")
        return ("motor %r: slot %d gains {%s}; gearing "
                "sensor_to_mechanism_ratio=%s (rotor revs per mechanism "
                "rev); limits max_velocity=%s rps, max_voltage=%s V; "
                "brake=%s inverted=%s; %s"
                % (self.name, self.slot, gains,
                   self.sensor_to_mechanism_ratio, self.max_velocity_rps,
                   self.max_voltage, self.brake, self.inverted, tune))


def _coerce_motor_table(name, table):
    gains = {}
    slot_table = table.get("slot0", {})
    if isinstance(slot_table, dict):
        for key in GAIN_KEYS:
            if key in slot_table:
                gains[key] = float(slot_table[key])
    for key, value in gains.items():
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError("gain %r must be finite, got %r"
                             % (key, slot_table[key]))
    ratio = float(table.get("sensor_to_mechanism_ratio", 1.0))
    if not (ratio == ratio) or ratio <= 0.0 or ratio == float("inf"):
        raise ValueError("sensor_to_mechanism_ratio must be finite and > 0, "
                         "got %r" % (table.get("sensor_to_mechanism_ratio"),))
    max_velocity = float(table.get("max_velocity_rps", 100.0))
    if not (max_velocity == max_velocity) or max_velocity < 0.0:
        raise ValueError("max_velocity_rps must be finite and >= 0, got %r"
                         % (table.get("max_velocity_rps"),))
    max_voltage = float(table.get("max_voltage", 12.0))
    if not (max_voltage == max_voltage) or max_voltage <= 0.0:
        raise ValueError("max_voltage must be finite and > 0, got %r"
                         % (table.get("max_voltage"),))
    return MotorControlConfig(
        name=name,
        slot=int(table.get("slot", 0)),
        gains=gains,
        sensor_to_mechanism_ratio=ratio,
        max_velocity_rps=max_velocity,
        max_voltage=max_voltage,
        brake=bool(table.get("brake", True)),
        inverted=bool(table.get("inverted", False)),
    )


def _fallback_parse_motor(text, motor_name):
    """Minimal ``[motors.<name>]`` reader for pre-3.11 pythons.

    Handles only what subsystem files use: ``key = value`` lines with
    float/bool/quoted-string values and one inline ``{ k = v, ... }``
    table (slot0). Raises ``KeyError`` when the motor table is missing.
    """
    table = {}
    inside = False
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("["):
            inside = line == "[motors.%s]" % motor_name
            continue
        if not inside or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key, value = key.strip(), value.strip()
        table[key] = _fallback_value(value)
    if not table:
        raise KeyError("no [motors.%s] table found" % motor_name)
    return table


def _fallback_value(value):
    if value.startswith("{"):
        out = {}
        for item in value.strip("{}").split(","):
            if "=" not in item:
                continue
            key, _, val = item.partition("=")
            out[key.strip()] = float(val.strip())
        return out
    if value.startswith('"') and value.endswith('"'):
        return value[1:-1]
    if value in ("true", "false"):
        return value == "true"
    return float(value)


def load_motor_config(toml_path, motor_name, slot=0):
    """Read ``[motors.<motor_name>]`` from a subsystem TOML file.

    Returns a :class:`MotorControlConfig`. Raises ``KeyError`` for a
    missing motor table and ``ValueError`` for unreadable TOML.
    """
    with open(toml_path, "rb") as fh:
        raw = fh.read()
    if _tomllib is not None:
        try:
            doc = _tomllib.loads(raw.decode("utf-8"))
        except Exception as exc:
            raise ValueError("cannot parse %s: %s" % (toml_path, exc))
        try:
            table = doc["motors"][motor_name]
        except KeyError:
            raise KeyError("no [motors.%s] table in %s"
                           % (motor_name, toml_path))
    else:
        table = _fallback_parse_motor(raw.decode("utf-8"), motor_name)
        if not isinstance(table, dict):
            raise ValueError("cannot parse %s" % (toml_path,))
    config = _coerce_motor_table(motor_name, table)
    config.slot = slot
    return config
