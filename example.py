import json
from datetime import datetime, timedelta
from models import Mission, Waypoint
from flight_path_simulator import FlightPathSimulator
from conflict_detector import ConflictDetector
from visualization_4d import visualize_mission_4d, visualize_paths_3d
import random

# def get_internal_missions(start_time: datetime) -> list:
#     """Create test missions with random waypoints."""
#     drone1_path = [
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 0),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 20),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 40)
#     ]
#     drone2_path = [
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 10),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 30),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 50)
#     ]
#     drone3_path = [
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 20),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 40),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 60)
#     ]
#     drone4_path = [
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 15),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 35),
#         (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100), 5, 55)
#     ]
#     def create_test_mission(drone_id, start_time, waypoints_data):
#         waypoints = []
#         for x, y, z, speed, time_offset in waypoints_data:
#             waypoints.append(Waypoint(
#                 x=x, y=y, z=z, speed=speed, timestamp=start_time + timedelta(seconds=time_offset)
#             ))
#         return Mission(
#             drone_id=drone_id,
#             waypoints=waypoints,
#             start_time=start_time,
#             end_time=waypoints[-1].timestamp
#         )
#     return [
#         create_test_mission("drone1", start_time, drone1_path),
#         create_test_mission("drone2", start_time, drone2_path),
#         create_test_mission("drone3", start_time, drone3_path),
#         create_test_mission("drone4", start_time, drone4_path)
#     ]

def main():
    # Initialize components
    flight_simulator = FlightPathSimulator(time_step=0.05)
    conflict_detector = ConflictDetector(safety_buffer=1.0, time_buffer=15.1)
    
    # Manually specify drone1's waypoints, start_time, end_time, and speed here:
    drone1_start_offset = 0  # seconds
    drone1_end_offset = 40   # seconds
    drone1_speed = 5         # m/s
    drone1_waypoints = [
        # No Conflict
        # (37, 1, 2),
        # (5, 5, 34),
        # (11, 11, 6),
        # (89, 77, 40),
        # (94, 99, 120),
        # (73, 100, 2)

        # Conflict
        (37, 1, 2),
        (5, 5, 34),
        (11, 50, 6),
        (89, 50, 40),
        (94, 5, 120),
        (73, 100, 2)
    ]
    start_time = datetime.now()
    
    # Create drone1 mission using FlightPathSimulator
    drone1_mission = flight_simulator.create_mission_from_waypoints(
        drone_id="drone1",
        waypoints=drone1_waypoints,
        start_offset=drone1_start_offset,
        end_offset=drone1_end_offset,
        speed=drone1_speed,
        global_start_time=start_time
    )
    
    # Load other missions using FlightPathSimulator
    other_missions = flight_simulator.load_missions_from_file('waypoints.json', start_time)
    missions = [drone1_mission] + other_missions
    
    # Simulate flight paths
    flight_paths = {}
    for mission in missions:
        flight_paths[mission.drone_id] = flight_simulator.simulate_flight_path(mission)
    
    # Detect conflicts
    conflicts = conflict_detector.detect_conflicts(missions, flight_paths)
    
    # Print results
    print(f"\nTotal Conflicts Detected: {len(conflicts)}")
    
    if conflicts:
        print("\nUnique Conflict Events:")
        for conflict in conflicts:
            print(f"\nDrones: {conflict.drone1_id} & {conflict.drone2_id}")
            print(f"Start Time: {conflict.time}")
            print(f"Closest Approach: {conflict.location}")
            print(f"Minimum Distance: {conflict.distance:.2f}m")
            print(f"Conflict Duration: {conflict.time_diff:.2f}s")
    
    # Create visualizations
    print("\nGenerating visualizations...")
    visualize_paths_3d(missions, conflicts)  # 3D visualization
    visualize_mission_4d(flight_paths, conflicts)  # 4D visualization


if __name__ == "__main__":
    main() 