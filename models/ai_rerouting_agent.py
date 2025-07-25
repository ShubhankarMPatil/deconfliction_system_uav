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
    Enhanced rerouting system that finds truly conflict-free paths.
    Uses multiple strategies to find the best alternative route.
    """
    # Try multiple rerouting strategies in order of sophistication
    strategies = [
        lambda: advanced_spatio_temporal_reroute(primary, others, space_bounds, uncertainty),
        lambda: multi_path_astar_reroute(primary, others, space_bounds, uncertainty),
        lambda: layered_altitude_reroute(primary, others, space_bounds, uncertainty),
        lambda: temporal_shift_reroute(primary, others, space_bounds, uncertainty),
        lambda: enhanced_simple_reroute(primary, others, space_bounds, uncertainty)
    ]
    
    best_path = None
    best_conflict_score = float('inf')
    
    for strategy in strategies:
        try:
            candidate_path = strategy()
            if candidate_path and len(candidate_path) > 0:
                # Evaluate the quality of this path
                conflict_score = evaluate_path_conflicts(candidate_path, primary, others, uncertainty)
                
                if conflict_score == 0:  # Perfect path found
                    return candidate_path
                elif conflict_score < best_conflict_score:
                    best_conflict_score = conflict_score
                    best_path = candidate_path
        except Exception as e:
            print(f"Strategy failed: {e}")
            continue
    
    # Return the best path found, or original path if nothing better
    return best_path if best_path else primary.waypoints

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

def evaluate_path_conflicts(waypoints: List[Waypoint], primary: DroneMission, 
                           others: List[DroneMission], uncertainty: UncertaintyParameters) -> float:
    """Evaluate the conflict score of a path (lower is better, 0 is perfect)"""
    if not waypoints:
        return float('inf')
    
    # Create temporary mission for conflict checking
    temp_mission = DroneMission(
        drone_id=primary.drone_id + "_temp",
        waypoints=waypoints,
        start_time=waypoints[0].t,
        end_time=waypoints[-1].t,
        priority=primary.priority
    )
    
    # Check conflicts using the comprehensive checker
    from utils.conflict_checker import check_comprehensive_conflicts
    report = check_comprehensive_conflicts(temp_mission, others, uncertainty)
    
    # Calculate weighted conflict score
    conflict_score = 0
    conflict_score += len(report["conflicts"]) * 10  # Critical conflicts
    conflict_score += len(report["spatial_conflicts"]) * 5  # Spatial conflicts
    conflict_score += len(report["temporal_conflicts"]) * 1  # Temporal conflicts
    
    # Add penalty for path length deviation
    original_length = calculate_path_length(primary.waypoints)
    new_length = calculate_path_length(waypoints)
    length_penalty = max(0, (new_length - original_length) / original_length * 2)
    
    return conflict_score + length_penalty

def calculate_path_length(waypoints: List[Waypoint]) -> float:
    """Calculate total 3D path length"""
    if len(waypoints) < 2:
        return 0
    
    total_length = 0
    for i in range(len(waypoints) - 1):
        wp1, wp2 = waypoints[i], waypoints[i + 1]
        distance = np.sqrt(
            (wp2.x - wp1.x)**2 + 
            (wp2.y - wp1.y)**2 + 
            (wp2.z - wp1.z)**2
        )
        total_length += distance
    return total_length

def advanced_spatio_temporal_reroute(primary: DroneMission, others: List[DroneMission], 
                                    space_bounds: Tuple[int, int, int],
                                    uncertainty: UncertaintyParameters) -> List[Waypoint]:
    """Advanced rerouting using spatio-temporal conflict prediction"""
    safety_margin = 2 * uncertainty.spatial_sigma + 15
    time_buffer = 2 * uncertainty.time_sigma + 10
    
    # Build conflict map for the entire mission timeline
    conflict_zones = build_conflict_map(others, uncertainty, space_bounds)
    
    rerouted_waypoints = []
    
    for i, wp in enumerate(primary.waypoints):
        # Find the safest position around the original waypoint
        best_position = find_safest_position(
            wp, conflict_zones, safety_margin, time_buffer, space_bounds
        )
        
        # Adjust timing if needed to avoid temporal conflicts
        adjusted_time = find_safest_time(
            best_position, wp.t, others, uncertainty, time_buffer
        )
        
        new_waypoint = Waypoint(
            x=best_position[0],
            y=best_position[1], 
            z=best_position[2],
            t=adjusted_time
        )
        
        rerouted_waypoints.append(new_waypoint)
    
    return smooth_path(rerouted_waypoints, space_bounds)

def build_conflict_map(others: List[DroneMission], uncertainty: UncertaintyParameters, 
                      space_bounds: Tuple[int, int, int]) -> dict:
    """Build a map of areas with high conflict probability"""
    GRID_SIZE = 20  # meters
    conflict_map = {}
    
    # Discretize space
    x_cells = int(space_bounds[0] / GRID_SIZE) + 1
    y_cells = int(space_bounds[1] / GRID_SIZE) + 1
    z_cells = int(space_bounds[2] / GRID_SIZE) + 1
    
    # Sample time points
    all_times = []
    for mission in others:
        all_times.extend([wp.t for wp in mission.waypoints])
    
    if not all_times:
        return conflict_map
    
    time_points = np.arange(min(all_times), max(all_times) + 5, 5)
    
    for t in time_points:
        for x_idx in range(x_cells):
            for y_idx in range(y_cells):
                for z_idx in range(z_cells):
                    x = x_idx * GRID_SIZE
                    y = y_idx * GRID_SIZE  
                    z = z_idx * GRID_SIZE
                    
                    conflict_density = calculate_conflict_density(
                        (x, y, z, t), others, uncertainty
                    )
                    
                    if conflict_density > 0:
                        conflict_map[(x_idx, y_idx, z_idx, int(t))] = conflict_density
    
    return conflict_map

def calculate_conflict_density(position: Tuple[float, float, float, float], 
                             others: List[DroneMission], 
                             uncertainty: UncertaintyParameters) -> float:
    """Calculate conflict density at a given position and time"""
    x, y, z, t = position
    density = 0
    safety_radius = 2 * uncertainty.spatial_sigma + 10
    
    for other in others:
        other_pos = interpolate_position_fast(other.waypoints, t)
        if other_pos is None:
            continue
            
        distance = np.sqrt(
            (x - other_pos['x'])**2 + 
            (y - other_pos['y'])**2 + 
            (z - other_pos['z'])**2
        )
        
        if distance < safety_radius:
            # Higher density for closer conflicts
            density += max(0, safety_radius - distance) / safety_radius
    
    return density

def find_safest_position(original_wp: Waypoint, conflict_zones: dict, 
                        safety_margin: float, time_buffer: float,
                        space_bounds: Tuple[int, int, int]) -> Tuple[float, float, float]:
    """Find the safest position near the original waypoint"""
    SEARCH_RADIUS = 50  # meters
    GRID_SIZE = 20
    
    best_position = (original_wp.x, original_wp.y, original_wp.z)
    lowest_conflict = float('inf')
    
    # Search in a grid around the original position
    for dx in range(-SEARCH_RADIUS, SEARCH_RADIUS + 1, GRID_SIZE):
        for dy in range(-SEARCH_RADIUS, SEARCH_RADIUS + 1, GRID_SIZE):
            for dz in range(-20, 21, 10):  # Smaller altitude variations
                new_x = max(0, min(space_bounds[0], original_wp.x + dx))
                new_y = max(0, min(space_bounds[1], original_wp.y + dy))
                new_z = max(0, min(space_bounds[2], original_wp.z + dz))
                
                # Check conflict density at this position
                grid_x = int(new_x / GRID_SIZE)
                grid_y = int(new_y / GRID_SIZE)
                grid_z = int(new_z / GRID_SIZE)
                grid_t = int(original_wp.t)
                
                conflict_level = conflict_zones.get((grid_x, grid_y, grid_z, grid_t), 0)
                
                # Add penalty for distance from original position
                distance_penalty = np.sqrt(dx*dx + dy*dy + dz*dz) / 100
                total_cost = conflict_level + distance_penalty
                
                if total_cost < lowest_conflict:
                    lowest_conflict = total_cost
                    best_position = (new_x, new_y, new_z)
    
    return best_position

def find_safest_time(position: Tuple[float, float, float], original_time: float,
                    others: List[DroneMission], uncertainty: UncertaintyParameters,
                    time_buffer: float) -> float:
    """Find the safest time to pass through a position"""
    safety_radius = 2 * uncertainty.spatial_sigma + 10
    x, y, z = position
    
    # Try time adjustments within reasonable bounds
    time_options = [original_time]  # Start with original time
    
    # Add earlier and later options
    for offset in [5, 10, 15, -5, -10, -15]:
        time_options.append(original_time + offset)
    
    best_time = original_time
    lowest_conflict = float('inf')
    
    for candidate_time in time_options:
        if candidate_time < 0:  # Don't go to negative time
            continue
            
        conflict_count = 0
        
        for other in others:
            other_pos = interpolate_position_fast(other.waypoints, candidate_time)
            if other_pos is None:
                continue
                
            distance = np.sqrt(
                (x - other_pos['x'])**2 + 
                (y - other_pos['y'])**2 + 
                (z - other_pos['z'])**2
            )
            
            if distance < safety_radius:
                conflict_count += 1
        
        if conflict_count < lowest_conflict:
            lowest_conflict = conflict_count
            best_time = candidate_time
            
            if conflict_count == 0:  # Perfect time found
                break
    
    return best_time

def smooth_path(waypoints: List[Waypoint], space_bounds: Tuple[int, int, int]) -> List[Waypoint]:
    """Smooth the path to ensure reasonable drone movement"""
    if len(waypoints) < 3:
        return waypoints
    
    smoothed = [waypoints[0]]  # Keep first waypoint
    
    for i in range(1, len(waypoints) - 1):
        prev_wp = smoothed[-1]
        curr_wp = waypoints[i]
        next_wp = waypoints[i + 1]
        
        # Apply smoothing to reduce sharp turns
        smooth_x = (prev_wp.x + curr_wp.x + next_wp.x) / 3
        smooth_y = (prev_wp.y + curr_wp.y + next_wp.y) / 3
        smooth_z = (prev_wp.z + curr_wp.z + next_wp.z) / 3
        
        # Ensure bounds
        smooth_x = max(0, min(space_bounds[0], smooth_x))
        smooth_y = max(0, min(space_bounds[1], smooth_y))
        smooth_z = max(0, min(space_bounds[2], smooth_z))
        
        smoothed.append(Waypoint(smooth_x, smooth_y, smooth_z, curr_wp.t))
    
    smoothed.append(waypoints[-1])  # Keep last waypoint
    return smoothed

def multi_path_astar_reroute(primary: DroneMission, others: List[DroneMission], 
                            space_bounds: Tuple[int, int, int],
                            uncertainty: UncertaintyParameters) -> List[Waypoint]:
    """A* pathfinding with multiple path exploration"""
    from queue import PriorityQueue
    import math
    
    # Grid parameters - finer resolution for better paths
    GRID_RES = 15  # meters
    TIME_STEP = 10  # seconds
    MAX_EXPANSIONS = 15000
    
    start_wp = primary.waypoints[0]
    goal_wp = primary.waypoints[-1]
    
    def to_grid(wp):
        return (
            int(round(wp.x / GRID_RES)),
            int(round(wp.y / GRID_RES)),
            int(round(wp.z / GRID_RES)),
            int(round(wp.t / TIME_STEP))
        )
    
    def from_grid(g):
        return Waypoint(
            x=min(space_bounds[0], max(0, g[0] * GRID_RES)),
            y=min(space_bounds[1], max(0, g[1] * GRID_RES)),
            z=min(space_bounds[2], max(0, g[2] * GRID_RES)),
            t=g[3] * TIME_STEP
        )
    
    def heuristic_3d(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2) + abs(a[3]-b[3]) * 0.1
    
    def is_position_safe(grid_pos, others, uncertainty):
        """Check if a grid position is safe from conflicts"""
        wp = from_grid(grid_pos)
        safety_radius = 2 * uncertainty.spatial_sigma + 12
        
        for other in others:
            other_pos = interpolate_position_fast(other.waypoints, wp.t)
            if other_pos is None:
                continue
                
            distance = np.sqrt(
                (wp.x - other_pos['x'])**2 + 
                (wp.y - other_pos['y'])**2 + 
                (wp.z - other_pos['z'])**2
            )
            
            if distance < safety_radius:
                return False
        return True
    
    start = to_grid(start_wp)
    goal = to_grid(goal_wp)
    
    # Enhanced neighbor moves including diagonal and altitude changes
    neighbor_moves = [
        (1,0,0,1), (-1,0,0,1), (0,1,0,1), (0,-1,0,1),  # Basic moves
        (0,0,1,1), (0,0,-1,1), (0,0,0,1),  # Altitude and time
        (1,1,0,1), (1,-1,0,1), (-1,1,0,1), (-1,-1,0,1),  # Diagonal
        (1,0,1,1), (1,0,-1,1), (-1,0,1,1), (-1,0,-1,1),  # X+Z combinations
        (0,1,1,1), (0,1,-1,1), (0,-1,1,1), (0,-1,-1,1)   # Y+Z combinations
    ]
    
    visited = set()
    pq = PriorityQueue()
    pq.put((0, start, [start]))
    expansions = 0
    
    while not pq.empty() and expansions < MAX_EXPANSIONS:
        cost, current, path = pq.get()
        
        if current in visited:
            continue
        visited.add(current)
        expansions += 1
        
        # Check if goal reached
        if heuristic_3d(current, goal) < 2:
            candidate_wps = [from_grid(g) for g in path]
            return candidate_wps
        
        # Expand neighbors
        for move in neighbor_moves:
            neighbor = tuple(
                current[i] + move[i] for i in range(4)
            )
            
            # Check bounds
            if not (
                0 <= neighbor[0] * GRID_RES <= space_bounds[0] and
                0 <= neighbor[1] * GRID_RES <= space_bounds[1] and
                0 <= neighbor[2] * GRID_RES <= space_bounds[2] and
                neighbor[3] >= 0
            ):
                continue
            
            if neighbor in visited:
                continue
            
            # Check if this position is safe
            if not is_position_safe(neighbor, others, uncertainty):
                continue
            
            new_cost = cost + np.linalg.norm(move[:3]) + abs(move[3]) * 0.1
            priority = new_cost + heuristic_3d(neighbor, goal)
            
            pq.put((priority, neighbor, path + [neighbor]))
    
    return None  # No path found

def layered_altitude_reroute(primary: DroneMission, others: List[DroneMission], 
                            space_bounds: Tuple[int, int, int],
                            uncertainty: UncertaintyParameters) -> List[Waypoint]:
    """Rerouting using altitude layers to avoid conflicts"""
    safety_margin = 2 * uncertainty.spatial_sigma + 12
    
    # Define altitude layers
    min_alt = 5
    max_alt = min(space_bounds[2], 60)
    altitude_layers = [min_alt + i * 10 for i in range(int((max_alt - min_alt) / 10) + 1)]
    
    best_path = None
    best_conflict_count = float('inf')
    
    # Try each altitude layer
    for target_altitude in altitude_layers:
        if target_altitude > max_alt:
            continue
            
        candidate_waypoints = []
        conflict_count = 0
        
        for i, wp in enumerate(primary.waypoints):
            # Calculate altitude transition
            progress = i / max(1, len(primary.waypoints) - 1)
            
            # Gradual transition to target altitude
            if i == 0:
                new_z = wp.z  # Keep original start altitude
            elif i == len(primary.waypoints) - 1:
                new_z = wp.z  # Keep original end altitude
            else:
                # Transition through target altitude in middle sections
                transition_factor = 4 * progress * (1 - progress)  # Bell curve
                new_z = wp.z + (target_altitude - wp.z) * transition_factor
            
            new_z = max(0, min(space_bounds[2], new_z))
            
            # Small horizontal adjustments for better separation
            drone_hash = hash(primary.drone_id + str(target_altitude))
            offset_x = ((drone_hash % 20) - 10) * 0.5
            offset_y = (((drone_hash // 20) % 20) - 10) * 0.5
            
            new_x = max(0, min(space_bounds[0], wp.x + offset_x))
            new_y = max(0, min(space_bounds[1], wp.y + offset_y))
            
            new_wp = Waypoint(new_x, new_y, new_z, wp.t)
            candidate_waypoints.append(new_wp)
            
            # Count conflicts
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
        
        if conflict_count < best_conflict_count:
            best_conflict_count = conflict_count
            best_path = candidate_waypoints
            
            if conflict_count == 0:  # Perfect path found
                break
    
    return best_path

def temporal_shift_reroute(primary: DroneMission, others: List[DroneMission], 
                          space_bounds: Tuple[int, int, int],
                          uncertainty: UncertaintyParameters) -> List[Waypoint]:
    """Rerouting by shifting timing to avoid temporal conflicts"""
    safety_margin = 2 * uncertainty.spatial_sigma + 10
    
    # Try different time shift strategies
    time_shifts = [0, 10, 20, -10, -20, 5, 15, -5, -15]
    
    best_path = None
    best_conflict_count = float('inf')
    
    for time_shift in time_shifts:
        candidate_waypoints = []
        conflict_count = 0
        
        for i, wp in enumerate(primary.waypoints):
            new_time = wp.t + time_shift
            
            # Don't allow negative times
            if new_time < 0:
                new_time = wp.t + abs(time_shift)
            
            # Apply slight spatial adjustments to complement timing changes
            progress = i / max(1, len(primary.waypoints) - 1)
            spatial_adjustment = 5 * (1 - progress)  # Decreasing adjustment
            
            drone_hash = hash(primary.drone_id + str(time_shift))
            offset_x = ((drone_hash % 10) - 5) * spatial_adjustment / 5
            offset_y = (((drone_hash // 10) % 10) - 5) * spatial_adjustment / 5
            
            new_x = max(0, min(space_bounds[0], wp.x + offset_x))
            new_y = max(0, min(space_bounds[1], wp.y + offset_y))
            new_z = wp.z
            
            new_wp = Waypoint(new_x, new_y, new_z, new_time)
            candidate_waypoints.append(new_wp)
            
            # Count conflicts at the new time
            for other in others:
                other_pos = interpolate_position_fast(other.waypoints, new_time)
                if other_pos is None:
                    continue
                    
                distance = np.sqrt(
                    (new_wp.x - other_pos['x'])**2 + 
                    (new_wp.y - other_pos['y'])**2 + 
                    (new_wp.z - other_pos['z'])**2
                )
                
                if distance < safety_margin:
                    conflict_count += 1
        
        if conflict_count < best_conflict_count:
            best_conflict_count = conflict_count
            best_path = candidate_waypoints
            
            if conflict_count == 0:  # Perfect path found
                break
    
    return best_path
