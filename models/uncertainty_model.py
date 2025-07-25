import numpy as np
from typing import List, Tuple
from utils.input_loader import Waypoint, DroneMission

class UncertaintyParameters:
    def __init__(self, spatial_sigma: float = 3.0, time_sigma: float = 2.0):
        self.spatial_sigma = spatial_sigma
        self.time_sigma = time_sigma

class UncertainPosition:
    def __init__(self, mean: Tuple[float, float, float, float], cov: np.ndarray):
        self.mean = mean  # (x, y, z, t)
        self.cov = cov    # 4x4 covariance matrix

    def __repr__(self):
        return f"UncertainPosition(mean={self.mean}, cov={np.round(self.cov, 2)})"

def generate_uncertainty_tube(mission: DroneMission, params: UncertaintyParameters) -> List[UncertainPosition]:
    tube = []

    for wp in mission.waypoints:
        mean = (wp.x, wp.y, wp.z, wp.t)
        cov = np.diag([
            params.spatial_sigma ** 2,  # x
            params.spatial_sigma ** 2,  # y
            params.spatial_sigma ** 2,  # z
            params.time_sigma ** 2      # t
        ])
        tube.append(UncertainPosition(mean, cov))

    return tube
