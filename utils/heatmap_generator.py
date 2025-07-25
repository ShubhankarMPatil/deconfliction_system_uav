import numpy as np
import plotly.graph_objects as go
from typing import List, Tuple, Dict
from models.uncertainty_model import generate_uncertainty_tube, UncertaintyParameters
from utils.input_loader import DroneMission
from scipy.ndimage import gaussian_filter
from scipy.interpolate import interp1d

GRID_RES = 16  # meters - coarser resolution for performance
TIME_RES = 5  # seconds - coarser time resolution

def interpolate_position(waypoints, t):
    """Smoothly interpolate drone position at time t"""
    if not waypoints:
        return {"x": 0, "y": 0, "z": 0}
    
    times = [wp.t for wp in waypoints]
    
    if t <= times[0]:
        return {"x": waypoints[0].x, "y": waypoints[0].y, "z": waypoints[0].z}
    if t >= times[-1]:
        return {"x": waypoints[-1].x, "y": waypoints[-1].y, "z": waypoints[-1].z}
    
    # Linear interpolation between waypoints
    for i in range(len(times) - 1):
        if times[i] <= t <= times[i + 1]:
            t1, t2 = times[i], times[i + 1]
            wp1, wp2 = waypoints[i], waypoints[i + 1]
            
            alpha = (t - t1) / (t2 - t1) if t2 != t1 else 0
            
            x = wp1.x + alpha * (wp2.x - wp1.x)
            y = wp1.y + alpha * (wp2.y - wp1.y)
            z = wp1.z + alpha * (wp2.z - wp1.z)
            
            return {"x": x, "y": y, "z": z}
    
    return {"x": waypoints[0].x, "y": waypoints[0].y, "z": waypoints[0].z}

def generate_smooth_heatmap(missions: List[DroneMission], current_time: float, 
                           x_range: Tuple[float, float] = (0, 800),
                           y_range: Tuple[float, float] = (0, 600)) -> go.Figure:
    """Generate smooth 2D heatmap with yellow glow around drones"""
    
    # Create grid
    x_grid = np.arange(x_range[0], x_range[1] + GRID_RES, GRID_RES)
    y_grid = np.arange(y_range[0], y_range[1] + GRID_RES, GRID_RES)
    X, Y = np.meshgrid(x_grid, y_grid)
    
    # Initialize heat matrix
    heat_matrix = np.zeros_like(X, dtype=float)
    
    # Add heat contribution from each active drone
    for mission in missions:
        if mission.start_time <= current_time <= mission.end_time:
            pos = interpolate_position(mission.waypoints, current_time)
            drone_x, drone_y = pos["x"], pos["y"]
            
            # Priority-based heat intensity
            priority_multiplier = {
                'emergency': 3.0,
                'high': 2.0,
                'normal': 1.0
            }.get(mission.priority, 1.0)
            
            # Vectorized distance calculation for better performance
            distances = np.sqrt((X - drone_x)**2 + (Y - drone_y)**2)
            
            # Gaussian falloff for smooth glow (vectorized)
            sigma = 20  # Smaller glow radius for speed
            heat_contribution = priority_multiplier * np.exp(-(distances**2) / (2 * sigma**2))
            heat_matrix += heat_contribution
    
    # Apply Gaussian smoothing for ultra-smooth appearance
    heat_matrix = gaussian_filter(heat_matrix, sigma=0.8)  # Less smoothing for speed
    
    # Create heatmap figure
    fig = go.Figure(data=go.Heatmap(
        z=heat_matrix,
        x=x_grid,
        y=y_grid,
        colorscale=[
            [0.0, 'rgba(0,0,0,0)'],      # Transparent for zero values
            [0.1, 'rgba(255,255,0,0.1)'], # Very light yellow
            [0.3, 'rgba(255,255,0,0.3)'], # Light yellow
            [0.5, 'rgba(255,200,0,0.5)'], # Medium yellow
            [0.7, 'rgba(255,150,0,0.7)'], # Orange-yellow
            [0.9, 'rgba(255,100,0,0.8)'], # Orange
            [1.0, 'rgba(255,0,0,0.9)']    # Red for highest intensity
        ],
        showscale=True,
        colorbar=dict(
            title=dict(text="Risk Level", font=dict(color='white')),
            tickfont=dict(color='white'),
            x=1.02,
            len=0.8
        ),
        hovertemplate='X: %{x}<br>Y: %{y}<br>Risk: %{z:.2f}<extra></extra>'
    ))
    
    # Add drone positions as markers
    for mission in missions:
        if mission.start_time <= current_time <= mission.end_time:
            pos = interpolate_position(mission.waypoints, current_time)
            
            color = {
                'emergency': 'red',
                'high': 'orange', 
                'normal': 'blue'
            }.get(mission.priority, 'blue')
            
            fig.add_trace(go.Scatter(
                x=[pos["x"]],
                y=[pos["y"]],
                mode='markers+text',
                marker=dict(
                    size=15,
                    color=color,
                    symbol='diamond',
                    line=dict(width=2, color='white')
                ),
                text=[mission.drone_id],
                textposition="top center",
                textfont=dict(color='white', size=10),
                name=mission.drone_id,
                showlegend=True
            ))
    
    fig.update_layout(
        title=f"Risk Heatmap at Time {current_time:.1f}s",
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        xaxis=dict(range=x_range, scaleanchor="y", scaleratio=1),
        yaxis=dict(range=y_range),
        plot_bgcolor='black',
        paper_bgcolor='black',
        font=dict(color='white'),
        height=500
    )
    
    return fig

def get_grid_index(x, y, z, t, bounds):
    xi = int(x // GRID_RES)
    yi = int(y // GRID_RES)
    zi = int(z // GRID_RES)
    ti = int(t // TIME_RES)
    return (xi, yi, zi, ti)

def generate_heatmap(missions: List[DroneMission], bounds: Tuple[int, int, int], time_range: Tuple[int, int],
                     uncertainty: UncertaintyParameters) -> dict:
    """Legacy heatmap function for backwards compatibility"""
    heatmap = {}

    for mission in missions:
        tube = generate_uncertainty_tube(mission, uncertainty)

        for u in tube:
            x, y, z, t = u.mean
            idx = get_grid_index(x, y, z, t, bounds)

            # Add heat contribution to this cell
            if idx not in heatmap:
                heatmap[idx] = 0
            heatmap[idx] += 1  # Simple density count (can add priority weight later)

    return heatmap
