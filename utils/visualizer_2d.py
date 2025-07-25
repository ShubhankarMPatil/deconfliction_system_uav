import matplotlib.pyplot as plt
from utils.input_loader import DroneMission
from typing import List


def plot_topdown_2d(missions: List[DroneMission], primary_id: str, safety_radius: float = 10.0):
    plt.figure(figsize=(10, 10))
    ax = plt.gca()

    colors = ['red', 'blue', 'green', 'orange', 'purple']
    for idx, mission in enumerate(missions):
        color = 'black' if mission.drone_id == primary_id else colors[idx % len(colors)]
        path = [(wp.x, wp.y) for wp in mission.waypoints]
        xs, ys = zip(*path)
        ax.plot(xs, ys, marker='o', linestyle='-', label=mission.drone_id, color=color)

        # Add safety buffer as circles
        for wp in mission.waypoints:
            circle_color = 'r' if mission.drone_id == primary_id else 'y'
            circ = plt.Circle((wp.x, wp.y), safety_radius, color=circle_color, alpha=0.1)
            ax.add_patch(circ)

    ax.set_title("Top-Down 2D Airspace View")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()
