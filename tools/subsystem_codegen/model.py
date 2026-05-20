from dataclasses import dataclass
from enum import Enum


class MotorControllerType(Enum):
    TalonFX = "TalonFX"
    TalonsFXS = "TalonFXS"
    SparkMax = "SparkMax"


@dataclass(frozen=True)
class Motor:
    name: str
    type: MotorControllerType
    canbus: str
    can_id: int
    inverted: bool
    slot: int


# Only using most recent sensors
class SensorType(Enum):
    CANCoder = "CANCoder"
    CANRange = "CANRange"


@dataclass(frozen=True)
class Sensor:
    name: str
    type: SensorType
    canbus: str
    can_id: int


@dataclass(frozen=True)
class Subsystem:
    name: str
    subsystem_id: int
    motors: list[Motor]
    sensors: list[Sensor] | None
