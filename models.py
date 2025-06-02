from dataclasses import dataclass
from typing import List, Tuple
from datetime import datetime
from enum import Enum
import numpy as np

class ConflictSeverity(Enum):
    HIGH = "HIGH"

@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    timestamp: datetime
    speed: float  # in m/s

@dataclass
class DroneState:
    position: Tuple[float, float, float]  # (x, y, z)
    velocity: Tuple[float, float, float]  # (vx, vy, vz)
    timestamp: datetime
    drone_id: str

@dataclass
class Mission:
    drone_id: str
    waypoints: List[Waypoint]
    start_time: datetime
    end_time: datetime

@dataclass
class ConflictReport:
    drone1_id: str
    drone2_id: str
    conflict_time: datetime
    conflict_position: Tuple[float, float, float]
    minimum_distance: float
    severity: ConflictSeverity
    description: str

@dataclass
class Conflict:
    drone1_id: str
    drone2_id: str
    time: datetime
    location: Tuple[float, float, float]
    distance: float  # in meters
    time_diff: float  # in seconds
