from typing import List, Dict, Tuple
import numpy as np
from datetime import datetime, timedelta
from models import Waypoint, DroneState, Mission
import math
import json

class FlightPathSimulator:
    def __init__(self, time_step: float = 0.1):
        """
        Initialize the flight path simulator.
        
        Args:
            time_step: Time step for simulation in seconds
        """
        self.time_step = time_step

    def _compute_distances(self, positions: List[Tuple[float, float, float]]) -> List[float]:
        """Calculate distances between consecutive positions."""
        return [math.sqrt((positions[i+1][0] - positions[i][0])**2 +
                         (positions[i+1][1] - positions[i][1])**2 +
                         (positions[i+1][2] - positions[i][2])**2)
                for i in range(len(positions)-1)]

    def _interpolate_timestamps(self, positions: List[Tuple[float, float, float]], 
                              speed: float, start_time: datetime, 
                              end_offset: float, start_offset: float) -> Tuple[List[datetime], float]:
        """Interpolate timestamps based on positions, speed, and time constraints."""
        distances = self._compute_distances(positions)
        total_distance = sum(distances)
        if total_distance == 0:
            return [start_time for _ in positions], speed
            
        segment_times = [d / speed for d in distances]
        total_time = sum(segment_times)
        
        # If end_time is specified and total_time would exceed it, reduce speed
        if end_offset > start_offset and total_time > (end_offset - start_offset):
            mission_duration = end_offset - start_offset
            speed = total_distance / mission_duration
            segment_times = [d / speed for d in distances]
            total_time = sum(segment_times)
            
        time_stamps = [start_time]
        for t in segment_times:
            time_stamps.append(time_stamps[-1] + timedelta(seconds=t))
        return time_stamps, speed

    def create_mission_from_waypoints(self, drone_id: str, waypoints: List[Tuple[float, float, float]], 
                                    start_offset: float, end_offset: float, speed: float, 
                                    global_start_time: datetime) -> Mission:
        """Create a mission from waypoints with interpolated timestamps."""
        start_time = global_start_time + timedelta(seconds=start_offset)
        time_stamps, used_speed = self._interpolate_timestamps(waypoints, speed, start_time, end_offset, start_offset)
        
        return Mission(
            drone_id=drone_id,
            waypoints=[Waypoint(x=wp[0], y=wp[1], z=wp[2], speed=used_speed, timestamp=time_stamps[i]) 
                      for i, wp in enumerate(waypoints)],
            start_time=time_stamps[0],
            end_time=time_stamps[-1]
        )

    def load_missions_from_file(self, filename: str, global_start_time: datetime) -> List[Mission]:
        """Load missions from a JSON file and create Mission objects."""
        with open(filename, 'r') as f:
            data = json.load(f)
            
        missions = []
        for drone in data['drones']:
            drone_start_offset = drone.get('start_time', 0)
            drone_end_offset = drone.get('end_time', 0)
            speed = drone.get('speed', 5)
            positions = [(wp['x'], wp['y'], wp['z']) for wp in drone['waypoints']]
            
            mission = self.create_mission_from_waypoints(
                drone_id=drone['drone_id'],
                waypoints=positions,
                start_offset=drone_start_offset,
                end_offset=drone_end_offset,
                speed=speed,
                global_start_time=global_start_time
            )
            missions.append(mission)
        return missions

    def interpolate_position(self, wp1: Waypoint, wp2: Waypoint, t: datetime) -> Tuple[float, float, float]:
        """Interpolate position between two waypoints at a given time."""
        if t < wp1.timestamp or t > wp2.timestamp:
            raise ValueError("Time must be between waypoint timestamps")

        # Calculate time ratio
        total_time = (wp2.timestamp - wp1.timestamp).total_seconds()
        elapsed_time = (t - wp1.timestamp).total_seconds()
        ratio = elapsed_time / total_time

        # Linear interpolation
        x = wp1.x + (wp2.x - wp1.x) * ratio
        y = wp1.y + (wp2.y - wp1.y) * ratio
        z = wp1.z + (wp2.z - wp1.z) * ratio

        return x, y, z

    def calculate_velocity(self, start_pos: Tuple[float, float, float], end_pos: Tuple[float, float, float], time_diff: float) -> Tuple[float, float, float]:
        """
        Calculate velocity vector between two positions.
        
        Args:
            start_pos: Start position (x, y, z)
            end_pos: End position (x, y, z)
            time_diff: Time difference in seconds
            
        Returns:
            Velocity vector (vx, vy, vz) in m/s
        """
        if time_diff <= 0:
            return (0.0, 0.0, 0.0)
        return tuple((end - start) / time_diff for start, end in zip(start_pos, end_pos))

    def simulate_flight_path(self, mission: Mission) -> Dict[datetime, Tuple[float, float, float]]:
        """Simulate the flight path and return positions at each time step."""
        positions = {}
        current_time = mission.start_time

        # Add initial position
        positions[current_time] = (
            mission.waypoints[0].x,
            mission.waypoints[0].y,
            mission.waypoints[0].z
        )

        # Simulate between waypoints
        for i in range(len(mission.waypoints) - 1):
            wp1 = mission.waypoints[i]
            wp2 = mission.waypoints[i + 1]
            
            # Calculate time between waypoints
            time_diff = (wp2.timestamp - wp1.timestamp).total_seconds()
            steps = int(time_diff / self.time_step)
            
            # Interpolate positions between waypoints
            for step in range(1, steps + 1):
                t = wp1.timestamp + timedelta(seconds=step * self.time_step)
                x, y, z = self.interpolate_position(wp1, wp2, t)
                positions[t] = (x, y, z)
        
        # Add final position if not already added
        final_time = mission.waypoints[-1].timestamp
        if final_time not in positions:
            positions[final_time] = (
                mission.waypoints[-1].x,
                mission.waypoints[-1].y,
                mission.waypoints[-1].z
            )

        return positions

    def get_drone_state_at_time(self, flight_path: Dict[datetime, Tuple[float, float, float]], 
                              target_time: datetime, drone_id: str) -> DroneState:
        """
        Get drone state at a specific time by interpolating between timestamps.
        
        Args:
            flight_path: Dictionary mapping timestamps to positions
            target_time: Target time
            drone_id: Identifier for the drone
            
        Returns:
            Interpolated DroneState
            
        Raises:
            ValueError: If target time is outside flight path range
        """
        timestamps = sorted(flight_path.keys())
        if target_time < timestamps[0] or target_time > timestamps[-1]:
            raise ValueError(f"Target time {target_time} outside flight path range")
        
        # Find timestamps that bracket the target time
        for i in range(len(timestamps) - 1):
            if timestamps[i] <= target_time <= timestamps[i + 1]:
                # Calculate interpolation ratio
                time_diff = (timestamps[i + 1] - timestamps[i]).total_seconds()
                if time_diff == 0:
                    ratio = 0
                else:
                    ratio = (target_time - timestamps[i]).total_seconds() / time_diff
                
                # Get positions
                pos1 = flight_path[timestamps[i]]
                pos2 = flight_path[timestamps[i + 1]]
                
                # Interpolate position
                position = tuple(p1 + (p2 - p1) * ratio for p1, p2 in zip(pos1, pos2))
                
                # Calculate velocity
                velocity = self.calculate_velocity(pos1, pos2, time_diff)
                
                return DroneState(
                    drone_id=drone_id,
                    timestamp=target_time,
                    position=position,
                    velocity=velocity
                )
        
        raise ValueError(f"Could not find appropriate timestamps for interpolation at {target_time}") 