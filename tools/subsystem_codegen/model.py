from dataclasses import dataclass, field


# Device-table kinds recognised as top-level [<kind>.<device>] tables. A table
# whose values are not all tables (e.g. [geometry]) is node-private config,
# preserved verbatim so a new section does not break the generator;
# motors/sensors get the extra validation both ends rely on.
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
    # Node-private sections ([geometry], ...) kept verbatim, in file order,
    # so subsystem.toml round-trips through parse/generate.
    extra: dict = field(default_factory=dict)
    # Node-private scalars inside [subsystem] itself (tuning knobs like
    # max_linear_mps that no other node reads), kept verbatim.
    header_extra: dict = field(default_factory=dict)

    def of_kind(self, kind: str) -> list:
        return [d for d in self.devices if d.kind == kind]

    @property
    def motors(self) -> list:
        return self.of_kind("motors")

    @property
    def sensors(self) -> list:
        return self.of_kind("sensors")
