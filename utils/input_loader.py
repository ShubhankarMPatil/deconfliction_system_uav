import json
from typing import List


class Waypoint:
    def __init__(self, x: float, y: float, z: float = 0.0, t: float = 0.0):
        self.x = x
        self.y = y
        self.z = z
        self.t = t

    def to_tuple(self):
        return (self.x, self.y, self.z, self.t)

    def __repr__(self):
        return f"Waypoint(x={self.x}, y={self.y}, z={self.z}, t={self.t})"


class DroneMission:
    def __init__(self, drone_id: str, waypoints: List[Waypoint], start_time: float, end_time: float, priority: str = "normal"):
        self.drone_id = drone_id
        self.waypoints = waypoints
        self.start_time = start_time
        self.end_time = end_time
        self.priority = priority

    def get_path(self) -> List[tuple]:
        return [wp.to_tuple() for wp in self.waypoints]

    def __repr__(self):
        return f"DroneMission(drone_id={self.drone_id}, waypoints={self.waypoints}, start_time={self.start_time}, end_time={self.end_time}, priority={self.priority})"


def load_missions(file_path: str) -> List[DroneMission]:
    with open(file_path, 'r') as f:
        data = json.load(f)

    missions = []
    for entry in data['missions']:
        waypoints = [
            Waypoint(
                x=wp['x'],
                y=wp['y'],
                z=wp.get('z', 0.0),
                t=wp['t']
            ) for wp in entry['waypoints']
        ]

        mission = DroneMission(
            drone_id=entry['drone_id'],
            waypoints=waypoints,
            start_time=entry['start_time'],
            end_time=entry['end_time'],
            priority=entry.get('priority', 'normal')
        )

        missions.append(mission)

    return missions
