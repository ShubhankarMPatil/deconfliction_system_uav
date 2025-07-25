def get_time_bounds(drone_data):
    all_times = []
    for waypoints in drone_data.values():
        all_times.extend([wp["t"] for wp in waypoints])
    return int(min(all_times)), int(max(all_times))
