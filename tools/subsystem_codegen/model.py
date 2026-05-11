from dataclasses import dataclass
from enum import Enum


class MotorControllerType(Enum):
    TalonFX = "TalonFX"
    TalonsFXS = "TalonFXS"
    SparkMax = "SparkMax"


@dataclass(frozen=True)
class Motor:
    type: MotorControllerType
    name: str
    can_id: int
    inverted: bool


@dataclass(frozen=True)
class Subsystem:
    name: str
    subsystem_id: int
