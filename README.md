# UAV Strategic Deconfliction System

A comprehensive system for ensuring safe drone operations in shared airspace by validating waypoint missions against other drones' trajectories in both space and time.

---

## Overview
This project simulates multiple UAV (drone) missions, detects spatio-temporal conflicts, and visualizes the results in 4D (3D space + time). It supports both external (JSON) and manual (Python) waypoint input.

---

## Workflow & Development Process

- **Modular Design:**
  - The codebase is structured into clear modules: models, flight path simulation, conflict detection, and visualization.
  - Utility functions are used to avoid redundancy (e.g., distance and timestamp interpolation).

- **Flexible Mission Input:**
  - Drone 1's waypoints, speed, and timing are set manually in `example.py` for easy experimentation.
  - All other drones are loaded from `waypoints.json`, allowing scalable and editable mission input.

- **Conflict Detection:**
  - The system detects conflicts only when drones are within both a spatial and temporal buffer.
  - Continuous/conflicting intervals are grouped as single conflict events for clarity.

- **Visualization:**
  - The system provides a 4D visualization (3D space + time) with clear color coding, start points, and conflict markers.
  - The visualization is modular and can be extended or customized.

---

## Key Components

### 1. Models (`models.py`)
- Simplified data structures to focus on essential information
- Key classes:
  - `Waypoint`: Represents a point in 3D space with timestamp and speed
  - `Mission`: Contains waypoints and timing information for a single drone
  - `Conflict`: Represents a detected conflict between two drones

### 2. Flight Path Simulator (`flight_path_simulator.py`)
- Interpolates drone positions between waypoints
- Key features:
  - Time step of 0.05 seconds for precise position sampling
  - Linear interpolation between waypoints
  - Returns a dictionary of timestamps to positions
  - Properly handles waypoint timestamps

### 3. Conflict Detector (`conflict_detector.py`)
- Enhanced conflict detection logic
- Key parameters:
  - Safety buffer: 1.0 meters (minimum distance between drones)
  - Time buffer: 15.1 seconds (slightly larger than the 15-second delay between drones)
- Improved time difference calculation and grouping of conflicts into unique events

### 4. Example Implementation (`example.py`)
- Demonstrates conflict detection between multiple drones
- Test scenario:
  - Drone 1: Path and timing set manually in code
  - Other drones: Loaded from `waypoints.json`
  - Flexible for any number of drones and waypoints

---

## Setup Instructions

### 1. Python Version
- Python 3.8 or higher, and using a seperate conda environment is recommended.

### 2. Install Dependencies
Install all required packages using pip:
```bash
pip install -r requirements.txt
```

### 3. Waypoints Configuration
- **External Drones:**
  - Edit `waypoints.json` to define all drones except the primary drone - drone 1.
  - Each drone entry should look like:
    ```json
    {
      "drone_id": "drone2",
      "start_time": 10,
      "end_time": 50,
      "speed": 5,
      "waypoints": [
        {"x": 0, "y": 100, "z": 20},
        {"x": 50, "y": 50, "z": 20},
        {"x": 100, "y": 0, "z": 20}
      ]
    }
    ```
  - Do **not** include drone 1 in the JSON file.

- **Drone 1 (Manual Input):**
  - In `example.py`, inside the `main()` function, set:
    ```python
    drone1_start_offset = 0  # seconds
    drone1_end_offset = 40   # seconds
    drone1_speed = 5         # m/s
    drone1_waypoints = [
        (0, 0, 10),
        (50, 50, 10),
        (100, 100, 10)
    ]
    ```
  - You can change these values to test different scenarios for drone 1.

---

## Execution Instructions

### 1. Run the Simulation
```bash
python3 example.py
```

### 2. Output
- The script will print the total number of unique conflict events detected.
- For each conflict, it will show:
  - Drones involved
  - Start time of the conflict
  - Closest approach (location)
  - Minimum distance
  - Duration of the conflict

### 3. Visualization
- A 4D visualization will be displayed:
  - Drone 1's path is highlighted and colored by time.
  - Other drones' paths are shown in distinct colors.
  - Conflict points are marked with red stars.
  - Start points are indicated with subtle black circles.

---

## Notes
- You can add or remove drones by editing `waypoints.json`.
- You can test different paths for drone 1 by editing its waypoints in `example.py`.
- The system is modular and can be extended for more advanced conflict detection or visualization features.

---

## Troubleshooting
- If you encounter a `JSONDecodeError`, ensure your `waypoints.json` is valid and contains no comments or stray characters.
- If you have issues with visualization, ensure all dependencies are installed and your Python environment supports GUI windows.

---

## License
MIT