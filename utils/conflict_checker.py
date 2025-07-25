import numpy as np
from typing import List, Dict
from models.uncertainty_model import UncertainPosition, generate_uncertainty_tube, UncertaintyParameters
from utils.input_loader import DroneMission
from utils.heatmap_generator import interpolate_position


def euclidean_distance(p1: tuple, p2: tuple) -> float:
    return np.linalg.norm(np.array(p1[:3]) - np.array(p2[:3]))


def time_distance(t1: float, t2: float) -> float:
    return abs(t1 - t2)


def check_conflicts(primary: DroneMission, others: List[DroneMission],
                    uncertainty: UncertaintyParameters, safety_distance: float = 5.0) -> Dict:
    """
    Returns a conflict report with both spatial and temporal conflict detection
    """
    primary_tube = generate_uncertainty_tube(primary, uncertainty)
    report = {
        "status": "clear",
        "conflicts": [],
        "spatial_conflicts": [],
        "temporal_conflicts": []
    }

    for other in others:
        if other.drone_id == primary.drone_id:
            continue

        other_tube = generate_uncertainty_tube(other, uncertainty)

        for i, p1 in enumerate(primary_tube):
            for j, p2 in enumerate(other_tube):
                spatial_dist = euclidean_distance(p1.mean, p2.mean)
                temporal_dist = time_distance(p1.mean[3], p2.mean[3])

                combined_spatial_threshold = 2 * uncertainty.spatial_sigma + safety_distance
                combined_time_threshold = 2 * uncertainty.time_sigma

                conflict_info = {
                    "time": round((p1.mean[3] + p2.mean[3]) / 2, 2),
                    "primary_position": tuple(round(x, 2) for x in p1.mean),
                    "conflicting_drone": other.drone_id,
                    "other_position": tuple(round(x, 2) for x in p2.mean),
                    "spatial_distance": round(spatial_dist, 2),
                    "time_difference": round(temporal_dist, 2)
                }

                # Check for spatial conflicts (close in space, regardless of time)
                if spatial_dist <= combined_spatial_threshold:
                    spatial_conflict = conflict_info.copy()
                    spatial_conflict["type"] = "spatial"
                    spatial_conflict["severity"] = "high" if spatial_dist <= safety_distance else "medium"
                    report["spatial_conflicts"].append(spatial_conflict)

                # Check for temporal conflicts (same time, regardless of space distance)
                if temporal_dist <= combined_time_threshold:
                    temporal_conflict = conflict_info.copy()
                    temporal_conflict["type"] = "temporal"
                    temporal_conflict["severity"] = "high" if temporal_dist <= 1.0 else "medium"
                    report["temporal_conflicts"].append(temporal_conflict)

                # Combined spatio-temporal conflict (original logic)
                if spatial_dist <= combined_spatial_threshold and temporal_dist <= combined_time_threshold:
                    report["status"] = "conflict_detected"
                    combined_conflict = conflict_info.copy()
                    combined_conflict["type"] = "spatio-temporal"
                    combined_conflict["severity"] = "critical"
                    report["conflicts"].append(combined_conflict)

    # Update overall status based on any conflicts found
    if report["conflicts"] or report["spatial_conflicts"] or report["temporal_conflicts"]:
        if report["conflicts"]:  # Critical spatio-temporal conflicts
            report["status"] = "critical_conflict"
        elif any(c["severity"] == "high" for c in report["spatial_conflicts"] + report["temporal_conflicts"]):
            report["status"] = "high_risk"
        else:
            report["status"] = "moderate_risk"
    
    return report

def check_comprehensive_conflicts(primary: DroneMission, others: List[DroneMission], 
                                 uncertainty: UncertaintyParameters, 
                                 safety_distance: float = 5.0, 
                                 time_step: float = 1.0) -> Dict:
    """
    Comprehensive conflict detection throughout the entire mission window
    Checks both spatial and temporal conflicts with detailed explanations
    """
    report = {
        "status": "clear",
        "conflicts": [],
        "spatial_conflicts": [],
        "temporal_conflicts": [],
        "conflict_explanations": []
    }
    
    # Determine time window for analysis
    all_missions = [primary] + others
    min_time = min(m.start_time for m in all_missions)
    max_time = max(m.end_time for m in all_missions)
    
    # Check conflicts at regular intervals throughout the mission window
    time_points = np.arange(min_time, max_time + time_step, time_step)
    
    for t in time_points:
        # Skip if primary drone is not active at this time
        if not (primary.start_time <= t <= primary.end_time):
            continue
            
        primary_pos = interpolate_position(primary.waypoints, t)
        
        for other in others:
            if other.drone_id == primary.drone_id:
                continue
                
            # Check if other drone is active at this time
            if not (other.start_time <= t <= other.end_time):
                continue
                
            other_pos = interpolate_position(other.waypoints, t)
            
            # Calculate spatial distance
            spatial_dist = np.sqrt(
                (primary_pos['x'] - other_pos['x'])**2 + 
                (primary_pos['y'] - other_pos['y'])**2 + 
                (primary_pos['z'] - other_pos['z'])**2
            )
            
            combined_spatial_threshold = 2 * uncertainty.spatial_sigma + safety_distance
            
            # Base conflict information
            conflict_base = {
                "time": round(t, 1),
                "primary_position": (round(primary_pos['x'], 1), round(primary_pos['y'], 1), 
                                    round(primary_pos['z'], 1)),
                "conflicting_drone": other.drone_id,
                "other_position": (round(other_pos['x'], 1), round(other_pos['y'], 1), 
                                 round(other_pos['z'], 1)),
                "spatial_distance": round(spatial_dist, 2),
                "time_difference": 0.0,
                "location": f"({primary_pos['x']:.1f}, {primary_pos['y']:.1f}, {primary_pos['z']:.1f})"
            }
            
            # Check for spatial conflicts
            if spatial_dist <= combined_spatial_threshold:
                spatial_conflict = conflict_base.copy()
                spatial_conflict["type"] = "spatial"
                spatial_conflict["severity"] = "critical" if spatial_dist <= safety_distance else "high"
                
                # Add detailed explanation
                explanation = {
                    "type": "Spatial Conflict",
                    "time": t,
                    "location": conflict_base["location"],
                    "primary_drone": primary.drone_id,
                    "conflicting_drone": other.drone_id,
                    "distance": spatial_dist,
                    "threshold": combined_spatial_threshold,
                    "explanation": f"At time {t:.1f}s, {primary.drone_id} at {conflict_base['location']} is {spatial_dist:.1f}m from {other.drone_id} (threshold: {combined_spatial_threshold:.1f}m)"
                }
                
                # Avoid duplicate spatial conflicts
                if not any(c['time'] == t and c['conflicting_drone'] == other.drone_id 
                          for c in report["spatial_conflicts"]):
                    report["spatial_conflicts"].append(spatial_conflict)
                    report["conflict_explanations"].append(explanation)
                
                # This is also a spatio-temporal conflict
                spatio_temporal_conflict = conflict_base.copy()
                spatio_temporal_conflict["type"] = "spatio-temporal"
                spatio_temporal_conflict["severity"] = "critical"
                
                if not any(c['time'] == t and c['conflicting_drone'] == other.drone_id 
                          for c in report["conflicts"]):
                    report["conflicts"].append(spatio_temporal_conflict)
            
            # Temporal conflicts - drones in same general area at same time
            elif spatial_dist <= 50.0:  # Within 50m considered temporal conflict
                temporal_conflict = conflict_base.copy()
                temporal_conflict["type"] = "temporal"
                temporal_conflict["severity"] = "medium"
                
                explanation = {
                    "type": "Temporal Conflict",
                    "time": t,
                    "location": conflict_base["location"],
                    "primary_drone": primary.drone_id,
                    "conflicting_drone": other.drone_id,
                    "distance": spatial_dist,
                    "explanation": f"At time {t:.1f}s, {primary.drone_id} and {other.drone_id} are both active in nearby airspace ({spatial_dist:.1f}m apart)"
                }
                
                # Avoid duplicate temporal conflicts
                if not any(c['time'] == t and c['conflicting_drone'] == other.drone_id 
                          for c in report["temporal_conflicts"]):
                    report["temporal_conflicts"].append(temporal_conflict)
                    report["conflict_explanations"].append(explanation)
    
    # Update overall status
    if report["conflicts"]:
        report["status"] = "critical_conflict"
    elif any(c["severity"] == "critical" for c in report["spatial_conflicts"]):
        report["status"] = "critical_conflict"
    elif any(c["severity"] == "high" for c in report["spatial_conflicts"]):
        report["status"] = "high_risk"
    elif report["spatial_conflicts"] or len(report["temporal_conflicts"]) > 5:
        report["status"] = "moderate_risk"
    
    return report

def check_realtime_conflicts(primary: DroneMission, others: List[DroneMission], 
                           current_time: float, uncertainty: UncertaintyParameters, 
                           safety_distance: float = 5.0) -> Dict:
    """
    Check conflicts at the current time using interpolated positions
    """
    report = {
        "status": "clear",
        "conflicts": [],
        "spatial_conflicts": [],
        "temporal_conflicts": []
    }
    
    # Check if primary drone is active at current time
    if not (primary.start_time <= current_time <= primary.end_time):
        return report
    
    primary_pos = interpolate_position(primary.waypoints, current_time)
    
    for other in others:
        if other.drone_id == primary.drone_id:
            continue
            
        # Check if other drone is active at current time
        if not (other.start_time <= current_time <= other.end_time):
            continue
            
        other_pos = interpolate_position(other.waypoints, current_time)
        
        # Calculate spatial distance
        spatial_dist = np.sqrt(
            (primary_pos['x'] - other_pos['x'])**2 + 
            (primary_pos['y'] - other_pos['y'])**2 + 
            (primary_pos['z'] - other_pos['z'])**2
        )
        
        combined_spatial_threshold = 2 * uncertainty.spatial_sigma + safety_distance
        
        conflict_info = {
            "time": current_time,
            "primary_position": (primary_pos['x'], primary_pos['y'], primary_pos['z'], current_time),
            "conflicting_drone": other.drone_id,
            "other_position": (other_pos['x'], other_pos['y'], other_pos['z'], current_time),
            "spatial_distance": round(spatial_dist, 2),
            "time_difference": 0.0
        }
        
        # Check for spatial conflict - this is the main concern
        if spatial_dist <= combined_spatial_threshold:
            spatial_conflict = conflict_info.copy()
            spatial_conflict["type"] = "spatial"
            spatial_conflict["severity"] = "critical" if spatial_dist <= safety_distance else "high"
            report["spatial_conflicts"].append(spatial_conflict)
            
            # This is also a spatio-temporal conflict - most critical
            spatio_temporal_conflict = conflict_info.copy()
            spatio_temporal_conflict["type"] = "spatio-temporal"
            spatio_temporal_conflict["severity"] = "critical"
            report["conflicts"].append(spatio_temporal_conflict)
        
        # Temporal conflicts - nearby active drones
        elif spatial_dist <= 50.0:
            temporal_conflict = conflict_info.copy()
            temporal_conflict["type"] = "temporal"
            temporal_conflict["severity"] = "medium"
            report["temporal_conflicts"].append(temporal_conflict)
    
    # Update overall status based on severity
    if report["conflicts"]:
        report["status"] = "critical_conflict"
    elif any(c["severity"] == "critical" for c in report["spatial_conflicts"]):
        report["status"] = "critical_conflict"
    elif any(c["severity"] == "high" for c in report["spatial_conflicts"]):
        report["status"] = "high_risk"
    elif report["spatial_conflicts"]:
        report["status"] = "moderate_risk"
    
    return report

# --- Conflict Detection Utilities ---

def get_current_spatial_conflicts(primary: DroneMission, others: List[DroneMission], current_time: float, uncertainty: UncertaintyParameters, safety_distance: float = 5.0) -> list:
    """
    Returns a list of spatial conflicts for the primary drone at the given current_time.
    This is useful for real-time visualization of spatial conflicts.
    """
    conflicts = []
    primary_pos = interpolate_position(primary.waypoints, current_time)
    for other in others:
        if other.drone_id == primary.drone_id:
            continue
        if not (other.start_time <= current_time <= other.end_time):
            continue
        other_pos = interpolate_position(other.waypoints, current_time)
        spatial_dist = np.sqrt(
            (primary_pos['x'] - other_pos['x'])**2 +
            (primary_pos['y'] - other_pos['y'])**2 +
            (primary_pos['z'] - other_pos['z'])**2
        )
        combined_spatial_threshold = 2 * uncertainty.spatial_sigma + safety_distance
        if spatial_dist <= combined_spatial_threshold:
            conflicts.append({
                'primary_position': (primary_pos['x'], primary_pos['y'], primary_pos['z']),
                'other_position': (other_pos['x'], other_pos['y'], other_pos['z']),
                'conflicting_drone': other.drone_id,
                'spatial_distance': spatial_dist,
                'threshold': combined_spatial_threshold
            })
    return conflicts
