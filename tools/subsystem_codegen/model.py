from dataclasses import dataclass, field


# Device-table kinds recognised under [subsystem.<kind>.<device>]. Unknown
# kinds are preserved verbatim so a new hardware taxonomy does not break the
# generator; motors/sensors get the extra validation both ends rely on.
KNOWN_DEVICE_KINDS = ("motors", "sensors")


def class_name(name: str) -> str:
    """intake_roller -> IntakeRoller."""
    return "".join(part[:1].upper() + part[1:] for part in name.split("_"))


@dataclass(frozen=True)
class Device:
    name: str
    kind: str
    # Remaining TOML keys verbatim (type, bus, can_id, limits, ...), kept in
    # file order so subsystem.toml round-trips.
    attrs: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Subsystem:
    name: str
    node_target: str
    period_us: int
    devices: list = field(default_factory=list)

    def of_kind(self, kind: str) -> list:
        return [d for d in self.devices if d.kind == kind]

    @property
    def motors(self) -> list:
        return self.of_kind("motors")

    @property
    def sensors(self) -> list:
        return self.of_kind("sensors")
