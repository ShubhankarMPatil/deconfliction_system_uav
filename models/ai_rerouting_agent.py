import numpy as np
import networkx as nx
from typing import List, Tuple, Set
from utils.input_loader import DroneMission, Waypoint
from utils.conflict_checker import check_conflicts
from models.uncertainty_model import UncertaintyParameters
import gc

# Optimized grid parameters for better memory usage
GRID_RES = 15  # meters - increased for fewer nodes
TIME_STEP = 15  # seconds - increased for fewer temporal nodes
MAX_TIME = 200
MAX_NODES = 5000  # Limit total nodes to prevent memory overflow

def generate_optimized_waypoints(primary: DroneMission, others: List[DroneMission], 
                                  space_bounds: Tuple[int, int, int],
                                  uncertainty: UncertaintyParameters) -> List[Waypoint]:
    """Memory-efficient waypoint generation using conflict avoidance vectors"""
    optimized_waypoints = []
    safety_margin = 2 * uncertainty.spatial_sigma + 10  # 10m base safety
    
    for i, wp in enumerate(primary.waypoints):
        # Start with original waypoint
        new_x, new_y, new_z = wp.x, wp.y, wp.z
        
        # Check for conflicts with other drones at this time
        conflict_vectors = []
        
        for other in others:
            if other.drone_id == primary.drone_id:
                continue
                
            # Find other drone's position at this time
            other_pos = interpolate_position_fast(other.waypoints, wp.t)
            if other_pos is None:
                continue
                
            # Calculate distance
            dx = wp.x - other_pos['x']
            dy = wp.y - other_pos['y']
            dz = wp.z - other_pos['z']
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            
            # If too close, calculate avoidance vector
            if distance < safety_margin and distance > 0:
                # Normalize and scale avoidance vector
                scale = (safety_margin - distance) / distance
                conflict_vectors.append((dx * scale, dy * scale, dz * scale))
        
        # Apply avoidance vectors
        if conflict_vectors:
            avg_avoid_x = np.mean([v[0] for v in conflict_vectors])
            avg_avoid_y = np.mean([v[1] for v in conflict_vectors])
            avg_avoid_z = np.mean([v[2] for v in conflict_vectors])
            
            # Apply with dampening to avoid overcorrection
            dampening = 0.7
            new_x = max(0, min(space_bounds[0], wp.x + avg_avoid_x * dampening))
            new_y = max(0, min(space_bounds[1], wp.y + avg_avoid_y * dampening))
            new_z = max(0, min(space_bounds[2], wp.z + avg_avoid_z * dampening))
        
        optimized_waypoints.append(Waypoint(new_x, new_y, new_z, wp.t))
    
    return optimized_waypoints

def interpolate_position_fast(waypoints: List[Waypoint], t: float) -> dict:
    """Fast position interpolation without external dependencies"""
    if not waypoints:
        return None
    
    times = [wp.t for wp in waypoints]
    
    if t <= times[0]:
        return {"x": waypoints[0].x, "y": waypoints[0].y, "z": waypoints[0].z}
    if t >= times[-1]:
        return {"x": waypoints[-1].x, "y": waypoints[-1].y, "z": waypoints[-1].z}
    
    # Linear interpolation
    for i in range(len(times) - 1):
        if times[i] <= t <= times[i + 1]:
            t1, t2 = times[i], times[i + 1]
            wp1, wp2 = waypoints[i], waypoints[i + 1]
            
            if t2 == t1:
                return {"x": wp1.x, "y": wp1.y, "z": wp1.z}
                
            alpha = (t - t1) / (t2 - t1)
            
            return {
                "x": wp1.x + alpha * (wp2.x - wp1.x),
                "y": wp1.y + alpha * (wp2.y - wp1.y),
                "z": wp1.z + alpha * (wp2.z - wp1.z)
            }
    
    return {"x": waypoints[0].x, "y": waypoints[0].y, "z": waypoints[0].z}

def heuristic(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
    return np.linalg.norm(np.array(a[:3]) - np.array(b[:3])) + abs(a[3] - b[3]) / TIME_STEP

def find_nearest_node(nodes: List[Tuple[int, int, int, int]], target: Tuple[float, float, float, float]) -> Tuple[int, int, int, int]:
    if not nodes:
        return (0, 0, 0, 0)
    return min(nodes, key=lambda n: np.linalg.norm(np.array(n[:3]) - np.array(target[:3])) + abs(n[3] - target[3]))

def simple_reroute(primary: DroneMission) -> List[Waypoint]:
    """Simple rerouting that adds systematic offset to original path"""
    rerouted = []
    
    # Calculate a consistent offset based on drone ID to avoid random behavior
    drone_hash = hash(primary.drone_id) % 100
    offset_x = (drone_hash % 20) - 10  # -10 to +10
    offset_y = ((drone_hash // 20) % 20) - 10  # -10 to +10
    
    for i, wp in enumerate(primary.waypoints):
        # Gradually apply offset that decreases toward the end
        progress = i / max(1, len(primary.waypoints) - 1)
        current_offset_x = offset_x * (1 - progress * 0.5)
        current_offset_y = offset_y * (1 - progress * 0.5)
        
        new_wp = Waypoint(
            x=max(0, min(800, wp.x + current_offset_x)),  # Keep within bounds
            y=max(0, min(600, wp.y + current_offset_y)),
            z=wp.z + 2,  # Slight altitude adjustment
            t=wp.t + 5   # Small time delay
        )
        rerouted.append(new_wp)
    
    return rerouted

def reroute_path(
    primary: DroneMission,
    others: List[DroneMission],
    space_bounds: Tuple[int, int, int],
    uncertainty: UncertaintyParameters,
    time_range: Tuple[int, int] = None
) -> List[Waypoint]:
    """
    Memory-efficient rerouting using conflict avoidance vectors.
    This approach avoids creating large graphs and uses direct conflict resolution.
    """
    try:
        # First try the optimized approach
        optimized_waypoints = generate_optimized_waypoints(primary, others, space_bounds, uncertainty)
        
        # Quick conflict check on optimized path
        temp_mission = DroneMission(
            drone_id=primary.drone_id + "_temp",
            waypoints=optimized_waypoints,
            start_time=optimized_waypoints[0].t,
            end_time=optimized_waypoints[-1].t,
            priority=primary.priority
        )
        
        # Use a lighter conflict check (spatial only)
        has_conflicts = False
        safety_distance = 2 * uncertainty.spatial_sigma + 5
        
        for wp in optimized_waypoints:
            for other in others:
                other_pos = interpolate_position_fast(other.waypoints, wp.t)
                if other_pos is None:
                    continue
                    
                distance = np.sqrt(
                    (wp.x - other_pos['x'])**2 + 
                    (wp.y - other_pos['y'])**2 + 
                    (wp.z - other_pos['z'])**2
                )
                
                if distance < safety_distance:
                    has_conflicts = True
                    break
            if has_conflicts:
                break
        
        if not has_conflicts:
            return optimized_waypoints
        else:
            # Fallback to enhanced simple reroute with multiple attempts
            return enhanced_simple_reroute(primary, others, space_bounds, uncertainty)
            
    except Exception as e:
        # If anything fails, use the basic simple reroute
        print(f"Rerouting error: {e}. Using simple fallback.")
        return simple_reroute(primary)
    finally:
        # Force garbage collection to free memory
        gc.collect()

def enhanced_simple_reroute(primary: DroneMission, others: List[DroneMission], 
                           space_bounds: Tuple[int, int, int],
                           uncertainty: UncertaintyParameters) -> List[Waypoint]:
    """Enhanced simple reroute with conflict awareness"""
    rerouted = []
    safety_margin = 2 * uncertainty.spatial_sigma + 8
    
    # Calculate multiple offset strategies
    drone_hash = hash(primary.drone_id) % 1000
    strategies = [
        ((drone_hash % 30) - 15, ((drone_hash // 30) % 30) - 15, 5),  # XY offset + Z up
        (-(drone_hash % 25) + 12, -(((drone_hash // 25) % 25) - 12), -3),  # Opposite direction
        (0, (drone_hash % 40) - 20, 8),  # Y-only movement + higher altitude
        ((drone_hash % 40) - 20, 0, -5),  # X-only movement + lower altitude
    ]
    
    # Try each strategy and pick the one with fewer conflicts
    best_waypoints = None
    min_conflicts = float('inf')
    
    for offset_x, offset_y, offset_z in strategies:
        candidate_waypoints = []
        conflict_count = 0
        
        for i, wp in enumerate(primary.waypoints):
            # Apply progressive offset (stronger at start, weaker at end)
            progress = i / max(1, len(primary.waypoints) - 1)
            current_offset_x = offset_x * (1 - progress * 0.3)
            current_offset_y = offset_y * (1 - progress * 0.3)
            current_offset_z = offset_z * (1 - progress * 0.5)
            
            new_x = max(0, min(space_bounds[0], wp.x + current_offset_x))
            new_y = max(0, min(space_bounds[1], wp.y + current_offset_y))
            new_z = max(0, min(space_bounds[2], wp.z + current_offset_z))
            
            new_wp = Waypoint(new_x, new_y, new_z, wp.t + 2)  # Small time offset
            candidate_waypoints.append(new_wp)
            
            # Count conflicts for this waypoint
            for other in others:
                other_pos = interpolate_position_fast(other.waypoints, new_wp.t)
                if other_pos is None:
                    continue
                    
                distance = np.sqrt(
                    (new_wp.x - other_pos['x'])**2 + 
                    (new_wp.y - other_pos['y'])**2 + 
                    (new_wp.z - other_pos['z'])**2
                )
                
                if distance < safety_margin:
                    conflict_count += 1
        
        if conflict_count < min_conflicts:
            min_conflicts = conflict_count
            best_waypoints = candidate_waypoints
            
            # If we found a conflict-free path, use it immediately
            if conflict_count == 0:
                break
    
    return best_waypoints if best_waypoints else simple_reroute(primary)
