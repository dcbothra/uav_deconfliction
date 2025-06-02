import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from datetime import datetime
from typing import Dict, List, Tuple
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D
from models import Mission, Conflict

class Visualization4D:
    def __init__(self):
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
    def visualize_4d(self, flight_paths: Dict[str, Dict[datetime, Tuple[float, float, float]]],
                    conflicts: List):
        """
        Clean 4D visualization:
        - Drone1: scatter points colored by time (hot colormap), line in red
        - Other drones: lines in distinct colors, points in same color
        - One colorbar for drone1
        - Legend inside plot
        - Large fonts
        - Start points indicated subtly
        """
        self.ax.clear()
        
        # Get time range for color normalization
        all_times = []
        for path in flight_paths.values():
            all_times.extend(list(path.keys()))
        time_min = min(all_times)
        time_max = max(all_times)
        time_norm = Normalize(vmin=time_min.timestamp(), vmax=time_max.timestamp())
        
        # Assign colors for fixed drones
        drone_colors = {
            'drone2': 'deepskyblue',
            'drone3': 'limegreen',
            'drone4': 'orange',
            'drone5': 'purple',
            'drone6': 'brown'
        }
        
        # Plot each drone's path
        for drone_id, path in flight_paths.items():
            times = list(path.keys())
            positions = list(path.values())
            x = np.array([p[0] for p in positions])
            y = np.array([p[1] for p in positions])
            z = np.array([p[2] for p in positions])
            time_values = np.array([t.timestamp() for t in times])
            
            if drone_id == "drone1":
                # Scatter points colored by time
                scatter = self.ax.scatter(x, y, z, c=time_values, cmap='hot', norm=time_norm, s=30, label='Drone 1')
                # Line in red
                self.ax.plot(x, y, z, color='red', alpha=0.7)
                # Subtle start point
                self.ax.scatter([x[0]], [y[0]], [z[0]], c='black', marker='o', s=60, alpha=0.4, label=None)
            else:
                color = drone_colors.get(drone_id, 'gray')
                self.ax.plot(x, y, z, color=color, alpha=0.7, label=f'{drone_id}')
                self.ax.scatter(x, y, z, color=color, s=15)
                # Subtle start point
                self.ax.scatter([x[0]], [y[0]], [z[0]], c='black', marker='o', s=60, alpha=0.4, label=None)
        
        # Plot conflicts
        if conflicts:
            conflict_x = [c.location[0] for c in conflicts]
            conflict_y = [c.location[1] for c in conflicts]
            conflict_z = [c.location[2] for c in conflicts]
            self.ax.scatter(conflict_x, conflict_y, conflict_z,
                            c='red', marker='*', s=120, edgecolor='black', linewidth=1.2, label='Conflict Point(s)')
        
        # Add colorbar for drone1
        mappable = plt.cm.ScalarMappable(norm=time_norm, cmap='hot')
        cbar = plt.colorbar(mappable, ax=self.ax, pad=0.1)
        cbar.set_label('Time (Drone 1)', fontsize=12)
        
        # Labels and title
        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        self.ax.set_zlabel('Z (m)', fontsize=12)
        self.ax.set_title('4D Mission Visualization\n(Color = Time for Drone 1)', fontsize=15, pad=20)
        
        # Custom legend
        legend_elements = [
            Line2D([0], [0], color='red', lw=2, label='Drone 1'),
            Line2D([0], [0], color='deepskyblue', lw=2, label='Drone 2'),
            Line2D([0], [0], color='limegreen', lw=2, label='Drone 3'),
            Line2D([0], [0], color='orange', lw=2, label='Drone 4'),
            Line2D([0], [0], color='purple', lw=2, label='Drone 5'),
            Line2D([0], [0], color='brown', lw=2, label='Drone 6'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='red', markeredgecolor='black', markersize=15, lw=0, label=f'Conflict Point(s) ({len(conflicts)})'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='black', alpha=0.4, markersize=10, lw=0, label='Start Point')
        ]
        self.ax.legend(handles=legend_elements, loc='upper left', fontsize=11)
        
        plt.tight_layout()
        plt.show()

def visualize_mission_4d(flight_paths: Dict[str, Dict[datetime, Tuple[float, float, float]]],
                        conflicts: List):
    viz = Visualization4D()
    viz.visualize_4d(flight_paths, conflicts)

def visualize_paths_3d(missions: List[Mission], conflicts: List[Conflict]):
    """Create a 3D visualization of the flight paths and conflicts."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Create a color cycle that can handle any number of missions
    colors = plt.cm.tab10(np.linspace(0, 1, len(missions)))
    
    # Plot each mission's path
    for i, mission in enumerate(missions):
        x = [wp.x for wp in mission.waypoints]
        y = [wp.y for wp in mission.waypoints]
        z = [wp.z for wp in mission.waypoints]
        ax.plot(x, y, z, color=colors[i], label=f'Drone {mission.drone_id}')
    
    # Plot conflicts
    for conflict in conflicts:
        x, y, z = conflict.location
        ax.scatter(x, y, z, c='red', marker='*', s=100)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    plt.show() 