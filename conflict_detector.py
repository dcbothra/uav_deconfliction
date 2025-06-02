from typing import List, Dict, Tuple
from datetime import datetime, timedelta
import numpy as np
from models import Mission, Conflict

class ConflictDetector:
    def __init__(self, safety_buffer: float = 2.0, time_buffer: float = 2.0, time_step: float = 0.05):
        self.safety_buffer = safety_buffer  # meters
        self.time_buffer = time_buffer  # seconds
        self.time_step = time_step  # seconds

    def calculate_distance(self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]) -> float:
        """Calculate Euclidean distance between two positions."""
        return np.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))

    def detect_conflicts(self, missions: List[Mission], flight_paths: Dict[str, Dict[datetime, Tuple[float, float, float]]]) -> List[Conflict]:
        """Detect conflicts between all missions and group them into unique conflict intervals."""
        raw_conflicts = []
        # Compare each pair of missions
        for i, mission1 in enumerate(missions):
            for mission2 in missions[i+1:]:
                path1 = flight_paths[mission1.drone_id]
                path2 = flight_paths[mission2.drone_id]
                for t1, pos1 in path1.items():
                    for t2, pos2 in path2.items():
                        time_diff = (t1 - t2).total_seconds()
                        if abs(time_diff) <= self.time_buffer:
                            distance = self.calculate_distance(pos1, pos2)
                            if distance <= self.safety_buffer:
                                raw_conflicts.append(Conflict(
                                    drone1_id=mission1.drone_id,
                                    drone2_id=mission2.drone_id,
                                    time=t1,
                                    location=pos1,
                                    distance=distance,
                                    time_diff=time_diff
                                ))
        # Group raw conflicts into intervals
        grouped_conflicts = self.group_conflict_intervals(raw_conflicts)
        return grouped_conflicts

    def group_conflict_intervals(self, conflicts: List[Conflict]) -> List[Conflict]:
        """Group consecutive/conflicting points into single conflict events per drone pair."""
        if not conflicts:
            return []
        # Sort by drone pair and time
        conflicts.sort(key=lambda c: (tuple(sorted([c.drone1_id, c.drone2_id])), c.time))
        grouped = []
        current_group = []
        last_time = None
        last_pair = None
        time_gap = timedelta(seconds=self.time_step * 1.5)  # Allow a small gap
        for c in conflicts:
            pair = tuple(sorted([c.drone1_id, c.drone2_id]))
            if (not current_group or pair != last_pair or (c.time - last_time) > time_gap):
                # Save previous group
                if current_group:
                    grouped.append(self.summarize_conflict_group(current_group))
                current_group = [c]
            else:
                current_group.append(c)
            last_time = c.time
            last_pair = pair
        # Save last group
        if current_group:
            grouped.append(self.summarize_conflict_group(current_group))
        return grouped

    def summarize_conflict_group(self, group: List[Conflict]) -> Conflict:
        """Summarize a group of conflicts into a single event."""
        # Use the start time, end time, and minimum distance
        start = min(group, key=lambda c: c.time)
        end = max(group, key=lambda c: c.time)
        min_dist_conflict = min(group, key=lambda c: c.distance)
        # We'll use the start time and location of closest approach
        return Conflict(
            drone1_id=start.drone1_id,
            drone2_id=start.drone2_id,
            time=start.time,  # Start of conflict interval
            location=min_dist_conflict.location,  # Closest approach
            distance=min_dist_conflict.distance,
            time_diff=(end.time - start.time).total_seconds()  # Duration of conflict
        ) 