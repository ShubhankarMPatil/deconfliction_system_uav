import numpy as np
import plotly.graph_objects as go
from typing import List, Tuple
from models.uncertainty_model import generate_uncertainty_tube, UncertaintyParameters
from utils.input_loader import DroneMission
from utils.heatmap_generator import interpolate_position

def generate_3d_visualization(missions: List[DroneMission], current_time: float, 
                             uncertainty: UncertaintyParameters = None) -> go.Figure:
    """Generate interactive 3D visualization with animated drone paths"""
    
    fig = go.Figure()
    
    # Color mapping for priorities
    priority_colors = {
        'emergency': '#FF0000',  # Red
        'high': '#FF8000',       # Orange  
        'normal': '#0080FF'      # Blue
    }
    
    for mission in missions:
        color = priority_colors.get(mission.priority, '#0080FF')
        
        # Full path as line
        path_x = [wp.x for wp in mission.waypoints]
        path_y = [wp.y for wp in mission.waypoints]
        path_z = [wp.z for wp in mission.waypoints]
        
        # Add full trajectory
        fig.add_trace(go.Scatter3d(
            x=path_x,
            y=path_y,
            z=path_z,
            mode='lines',
            line=dict(color=color, width=3, dash='dot'),
            name=f'{mission.drone_id}_path',
            opacity=0.6,
            showlegend=False
        ))
        
        # Add waypoints as markers
        fig.add_trace(go.Scatter3d(
            x=path_x,
            y=path_y,
            z=path_z,
            mode='markers',
            marker=dict(
                size=4,
                color=color,
                opacity=0.7
            ),
            name=f'{mission.drone_id}_waypoints',
            showlegend=False
        ))
        
        # Current position if drone is active
        if mission.start_time <= current_time <= mission.end_time:
            pos = interpolate_position(mission.waypoints, current_time)
            
            # Current drone position with larger marker
            fig.add_trace(go.Scatter3d(
                x=[pos['x']],
                y=[pos['y']],
                z=[pos['z']],
                mode='markers+text',
                marker=dict(
                    size=12,
                    color=color,
                    symbol='diamond',
                    line=dict(width=2, color='white')
                ),
                text=[mission.drone_id],
                textposition='top center',
                name=mission.drone_id,
                showlegend=True
            ))
            
            # Add uncertainty sphere around current position
            if uncertainty:
                # Create sphere points
                phi, theta = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
                radius = uncertainty.spatial_sigma
                
                sphere_x = radius * np.cos(phi) * np.sin(theta) + pos['x']
                sphere_y = radius * np.sin(phi) * np.sin(theta) + pos['y']
                sphere_z = radius * np.cos(theta) + pos['z']
                
                fig.add_trace(go.Surface(
                    x=sphere_x,
                    y=sphere_y,
                    z=sphere_z,
                    colorscale=[[0, color], [1, color]],
                    opacity=0.2,
                    showscale=False,
                    name=f'{mission.drone_id}_uncertainty',
                    showlegend=False
                ))
    
    # Add ground plane for reference
    x_ground = np.linspace(0, 800, 10)
    y_ground = np.linspace(0, 600, 10)
    X_ground, Y_ground = np.meshgrid(x_ground, y_ground)
    Z_ground = np.zeros_like(X_ground)
    
    fig.add_trace(go.Surface(
        x=X_ground,
        y=Y_ground,
        z=Z_ground,
        colorscale=[[0, 'rgba(128,128,128,0.3)'], [1, 'rgba(128,128,128,0.3)']],
        showscale=False,
        name='Ground',
        showlegend=False
    ))
    
    fig.update_layout(
        scene=dict(
            xaxis_title='X Position (m)',
            yaxis_title='Y Position (m)',
            zaxis_title='Altitude (m)',
            bgcolor='black',
            xaxis=dict(range=[0, 800], gridcolor='gray'),
            yaxis=dict(range=[0, 600], gridcolor='gray'),
            zaxis=dict(range=[0, 50], gridcolor='gray'),
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.5)
            )
        ),
        title=f'4D Visualization (3D + Time) - t={current_time:.1f}s',
        paper_bgcolor='black',
        plot_bgcolor='black',
        font=dict(color='white'),
        height=600,
        margin=dict(l=0, r=0, t=50, b=0)
    )
    
    return fig

def plot_3d_uncertainty(missions: List[DroneMission], primary_id: str, uncertainty: UncertaintyParameters):
    """Legacy matplotlib function for backwards compatibility"""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    colors = ['red', 'blue', 'green', 'orange', 'purple']
    for idx, mission in enumerate(missions):
        tube = generate_uncertainty_tube(mission, uncertainty)
        color = 'black' if mission.drone_id == primary_id else colors[idx % len(colors)]

        xs = [p.mean[0] for p in tube]
        ys = [p.mean[1] for p in tube]
        zs = [p.mean[2] for p in tube]

        ax.plot(xs, ys, zs, label=f"{mission.drone_id}", color=color)
        for p in tube:
            # Add uncertainty spheres (as transparent dots)
            ax.scatter(p.mean[0], p.mean[1], p.mean[2], alpha=0.2, color=color)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Visualization of Uncertainty Tubes')
    ax.legend()
    plt.tight_layout()
    plt.show()
