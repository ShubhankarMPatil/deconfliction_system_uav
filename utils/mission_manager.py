from typing import List, Dict
from utils.input_loader import DroneMission, Waypoint
from models.ai_rerouting_agent import reroute_path
from models.uncertainty_model import UncertaintyParameters

class MissionManager:
    """
    Manages missions and applies rerouting changes
    """
    
    def __init__(self, missions: List[DroneMission]):
        self.original_missions = {m.drone_id: m for m in missions}
        self.current_missions = {m.drone_id: self._copy_mission(m) for m in missions}
        self.reroute_history = {}
    
    def _copy_mission(self, mission: DroneMission) -> DroneMission:
        """Create a deep copy of a mission"""
        new_waypoints = [Waypoint(wp.x, wp.y, wp.z, wp.t) for wp in mission.waypoints]
        return DroneMission(
            drone_id=mission.drone_id,
            waypoints=new_waypoints,
            start_time=mission.start_time,
            end_time=mission.end_time,
            priority=mission.priority
        )
    
    def get_current_missions(self) -> List[DroneMission]:
        """Get current list of missions (including any rerouted ones)"""
        return list(self.current_missions.values())
    
    def get_mission(self, drone_id: str) -> DroneMission:
        """Get current mission for a specific drone"""
        return self.current_missions.get(drone_id)
    
    def apply_reroute(self, drone_id: str, new_waypoints: List[Waypoint]) -> Dict:
        """Apply a rerouted path to a specific drone"""
        if drone_id not in self.current_missions:
            return {"status": "error", "message": f"Drone {drone_id} not found"}
        
        # Store original for rollback if needed
        if drone_id not in self.reroute_history:
            self.reroute_history[drone_id] = []
        
        # Backup current waypoints
        current_waypoints = self.current_missions[drone_id].waypoints.copy()
        self.reroute_history[drone_id].append(current_waypoints)
        
        # Apply new waypoints
        mission = self.current_missions[drone_id]
        mission.waypoints = new_waypoints
        
        # Update mission timing if needed
        if new_waypoints:
            mission.start_time = new_waypoints[0].t
            mission.end_time = new_waypoints[-1].t
        
        return {
            "status": "success",
            "message": f"Reroute applied to {drone_id}: {len(new_waypoints)} waypoints",
            "old_waypoint_count": len(current_waypoints),
            "new_waypoint_count": len(new_waypoints)
        }
    
    def rollback_reroute(self, drone_id: str) -> Dict:
        """Rollback the last reroute for a drone"""
        if drone_id not in self.reroute_history or not self.reroute_history[drone_id]:
            return {"status": "error", "message": f"No reroute history for {drone_id}"}
        
        # Restore previous waypoints
        previous_waypoints = self.reroute_history[drone_id].pop()
        self.current_missions[drone_id].waypoints = previous_waypoints
        
        return {"status": "success", "message": f"Reroute rolled back for {drone_id}"}
    
    def reset_mission(self, drone_id: str) -> Dict:
        """Reset a mission to its original state"""
        if drone_id not in self.original_missions:
            return {"status": "error", "message": f"Original mission for {drone_id} not found"}
        
        # Reset to original
        self.current_missions[drone_id] = self._copy_mission(self.original_missions[drone_id])
        self.reroute_history[drone_id] = []
        
        return {"status": "success", "message": f"Mission {drone_id} reset to original state"}
    
    def get_reroute_status(self, drone_id: str) -> Dict:
        """Get rerouting status for a drone"""
        if drone_id not in self.current_missions:
            return {"status": "error", "message": "Drone not found"}
        
        original = self.original_missions[drone_id]
        current = self.current_missions[drone_id]
        
        is_rerouted = len(self.reroute_history.get(drone_id, [])) > 0
        
        return {
            "drone_id": drone_id,
            "is_rerouted": is_rerouted,
            "reroute_count": len(self.reroute_history.get(drone_id, [])),
            "original_waypoints": len(original.waypoints),
            "current_waypoints": len(current.waypoints),
            "original_time_span": f"{original.start_time}s - {original.end_time}s",
            "current_time_span": f"{current.start_time}s - {current.end_time}s"
        }

    def reroute_and_apply(self, drone_id: str, others: List[DroneMission], space_bounds: tuple, uncertainty: UncertaintyParameters, time_range: tuple = None) -> dict:
        """
        Reroute the specified drone using the ai_rerouting_agent and update its mission.
        This method is 3D-compatible and will update the drone's path to a conflict-free one if possible.
        Usage:
            manager.reroute_and_apply(drone_id, others, (800, 600, 50), uncertainty)
        """
        from models.ai_rerouting_agent import reroute_path
        primary = self.get_mission(drone_id)
        if not primary:
            return {"status": "error", "message": f"Drone {drone_id} not found"}
        new_waypoints = reroute_path(primary, others, space_bounds, uncertainty, time_range)
        return self.apply_reroute(drone_id, new_waypoints)

# Global mission manager instance
global_mission_manager = None

def get_mission_manager(missions: List[DroneMission] = None) -> MissionManager:
    """Get or create the global mission manager"""
    global global_mission_manager
    if global_mission_manager is None and missions is not None:
        global_mission_manager = MissionManager(missions)
    return global_mission_manager
