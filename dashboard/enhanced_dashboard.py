import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import json
import dash
from dash import dcc, html, ctx
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
import plotly.figure_factory as ff
import numpy as np
import dash_bootstrap_components as dbc

from utils.input_loader import load_missions
from utils.heatmap_generator import interpolate_position, generate_smooth_heatmap
from utils.conflict_checker import check_conflicts, check_realtime_conflicts, check_comprehensive_conflicts
from utils.visualizer_3d import generate_3d_visualization
from utils.mission_manager import get_mission_manager
from models.ai_rerouting_agent import reroute_path
from models.uncertainty_model import UncertaintyParameters

# Load missions and setup
MISSIONS_FILE = "input/missions.json"
if not os.path.exists(MISSIONS_FILE):
    MISSIONS_FILE = "../input/missions.json"  # Fallback path
original_missions = load_missions(MISSIONS_FILE)
mission_manager = get_mission_manager(original_missions)
missions = mission_manager.get_current_missions()  # This will be updated as reroutes are applied
uncertainty = UncertaintyParameters(spatial_sigma=7.0, time_sigma=5.0)

# Calculate time bounds from mission data
time_bounds = [
    min(wp.t for m in missions for wp in m.waypoints),
    max(wp.t for m in missions for wp in m.waypoints)
]
MAX_TIME = int(time_bounds[1])
BOUNDS = (800, 600, 50)

# Initialize Dash app with Bootstrap theme
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])
app.title = "🛰️ Advanced UAV Deconfliction Dashboard"

# Enhanced layout with modern styling
app.layout = dbc.Container([
    # Header
    dbc.Row([
        dbc.Col([
            html.H1("🛰️ Advanced UAV Deconfliction Dashboard", 
                   className="text-center mb-4 text-light"),
            html.Hr(className="border-light")
        ])
    ], className="mb-4"),
    
    # Control Panel
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H5("🎮 Control Panel", className="card-title text-light text-white"),
                    dbc.Row([
                        dbc.Col([
                            html.Label("Select Drone:", className="text-light fw-bold text-white"),
                            dcc.Dropdown(
                                id='drone-dropdown',
                                options=[{'label': m.drone_id, 'value': m.drone_id} for m in missions],
                                value=missions[0].drone_id,
                                className="mb-2",
                                style={'color': 'black'}
                            )
                        ], width=6),
                        dbc.Col([
                            html.Label("View Mode:", className="text-light fw-bold text-white"),
                            dcc.Dropdown(
                                id='view-mode',
                                options=[
                                    {'label': '📍 2D Live Map', 'value': '2d'},
                                    {'label': '🌡️ Risk Heatmap', 'value': 'heatmap'},
                                    {'label': '🎯 4D Visualization', 'value': '3d'}
                                ],
                                value='2d',
                                className="mb-2",
                                style={'color': 'black'}
                            )
                        ], width=6)
                    ]),
                    dbc.Row([
                        dbc.Col([
                            dbc.ButtonGroup([
                                dbc.Button("▶️ Play", id="play-button", color="success", size="sm"),
                                dbc.Button("⏸️ Pause", id="pause-button", color="warning", size="sm"),
                                dbc.Button("🔄 Reset", id="reset-button", color="info", size="sm"),
                                dbc.Button("🔁 Reroute", id="reroute-button", color="danger", size="sm")
                            ], className="mb-2"),
                            html.Div(id='reroute-status', className="text-light mt-2")
                        ])
                    ])
                ])
            ], color="dark", outline=True)
        ])
    ], className="mb-4"),
    
    # Main Visualization
    dbc.Row([
        dbc.Col([
            dcc.Graph(id='main-visualization', style={'height': '600px'})
        ])
    ], className="mb-4"),
    
    # Time Control
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H6(f"⏱️ Time Control", className="card-title text-light text-white"),
                    html.Label(f"Time: 0 / {MAX_TIME} seconds", 
                             id='time-label', 
                             className='fw-bold mb-2 text-light text-white'),
                    dcc.Slider(
                        id='time-slider',
                        min=time_bounds[0], 
                        max=time_bounds[1], 
                        step=1, 
                        value=time_bounds[0],
                        marks={i: {'label': f'{i}s', 'style': {'color': 'white'}} 
                              for i in range(int(time_bounds[0]), int(time_bounds[1])+1, 60)},
                        tooltip={"placement": "bottom", "always_visible": True},
                        updatemode='drag'
                    )
                ])
            ], color="dark", outline=True)
        ])
    ], className="mb-4"),
    
    # Information Panels
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader(html.H5("📊 Drone Information", className="mb-0 text-light text-white")),
                dbc.CardBody([
                    html.Div(id='info-panel', className="text-light text-white")
                ])
            ], color="dark", outline=True)
        ], width=6),
        dbc.Col([
            dbc.Card([
                dbc.CardHeader(html.H5("⚠️ Conflict Analysis", className="mb-0 text-light text-white")),
                dbc.CardBody([
                    html.Div(id='conflict-panel', className="text-light")
                ])
            ], color="dark", outline=True)
        ], width=6)
    ], className="mb-4"),
    
    # Additional Analytics
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader(html.H5("📈 Distance Analysis", className="mb-0 text-light text-white    ")),
                dbc.CardBody([
                    dcc.Graph(id='distance-chart', style={'height': '300px'})
                ])
            ], color="dark", outline=True)
        ], width=12)
    ]),
    
    # Background components for state management
    dcc.Interval(id='autoplay-timer', interval=150, n_intervals=0, disabled=True),
    dcc.Store(id='playback-state', data={'playing': False, 'time': time_bounds[0]}),
    dcc.Store(id='reroute-output', data={})
    
], fluid=True, className="bg-dark min-vh-100 py-3")

# Callback for playback control
@app.callback(
    [Output('autoplay-timer', 'disabled'),
     Output('play-button', 'children'),
     Output('pause-button', 'children')],
    [Input('play-button', 'n_clicks'),
     Input('pause-button', 'n_clicks'),
     Input('reset-button', 'n_clicks')],
    [State('autoplay-timer', 'disabled')]
)
def toggle_playback(play_clicks, pause_clicks, reset_clicks, is_disabled):
    if ctx.triggered_id == 'play-button':
        return False, "⏸️ Playing...", "⏸️ Pause"
    elif ctx.triggered_id == 'pause-button':
        return True, "▶️ Play", "⏸️ Paused"
    elif ctx.triggered_id == 'reset-button':
        return True, "▶️ Play", "⏸️ Pause"
    return is_disabled, "▶️ Play", "⏸️ Pause"

# Callback for auto-advancing time slider
@app.callback(
    Output('time-slider', 'value'),
    [Input('autoplay-timer', 'n_intervals'),
     Input('reset-button', 'n_clicks')],
    [State('time-slider', 'value'),
     State('autoplay-timer', 'disabled')]
)
def update_time_slider(n_intervals, reset_clicks, current_time, is_paused):
    if ctx.triggered_id == 'reset-button':
        return time_bounds[0]
    
    if not is_paused:
        new_time = current_time + 2
        if new_time > time_bounds[1]:
            return time_bounds[0]  # Loop back to start
        return new_time
    return current_time

# Callback for time label update
@app.callback(
    Output('time-label', 'children'),
    Input('time-slider', 'value')
)
def update_time_label(current_time):
    return f"⏱️ Time: {current_time:.0f} / {MAX_TIME} seconds"

# Main visualization callback
@app.callback(
    Output('main-visualization', 'figure'),
    [Input('time-slider', 'value'),
     Input('view-mode', 'value'),
     Input('drone-dropdown', 'value')]
)
def update_main_visualization(current_time, view_mode, selected_drone):
    if view_mode == '2d':
        return generate_2d_live_map(current_time)
    elif view_mode == 'heatmap':
        return generate_smooth_heatmap(missions, current_time)
    elif view_mode == '3d':
        return generate_3d_visualization(missions, current_time, uncertainty)
    else:
        return generate_2d_live_map(current_time)

def generate_2d_live_map(current_time):
    """Generate 2D live map with smooth interpolation"""
    fig = go.Figure()
    
    # Add drone trajectories and current positions
    for mission in missions:
        # Full path
        path_x = [wp.x for wp in mission.waypoints]
        path_y = [wp.y for wp in mission.waypoints]
        
        color = {
            'emergency': 'red',
            'high': 'orange',
            'normal': 'blue'
        }.get(mission.priority, 'blue')
        
        # Add full trajectory as dotted line
        fig.add_trace(go.Scatter(
            x=path_x, y=path_y,
            mode='lines',
            line=dict(color=color, width=2, dash='dot'),
            name=f'{mission.drone_id}_path',
            opacity=0.5,
            showlegend=False
        ))
        
        # Add waypoints
        fig.add_trace(go.Scatter(
            x=path_x, y=path_y,
            mode='markers',
            marker=dict(size=6, color=color, opacity=0.7),
            name=f'{mission.drone_id}_waypoints',
            showlegend=False
        ))
        
        # Current position if active
        if mission.start_time <= current_time <= mission.end_time:
            pos = interpolate_position(mission.waypoints, current_time)
            
            # Current drone position
            fig.add_trace(go.Scatter(
                x=[pos["x"]], y=[pos["y"]],
                mode='markers+text',
                marker=dict(size=15, color=color, symbol='diamond',
                           line=dict(width=2, color='white')),
                text=[mission.drone_id],
                textposition="top center",
                textfont=dict(color='white', size=10),
                name=mission.drone_id,
                showlegend=True
            ))
            
            # Safety radius
            fig.add_trace(go.Scatter(
                x=[pos["x"]], y=[pos["y"]],
                mode='markers',
                marker=dict(size=60, color=color, opacity=0.2, symbol='circle'),
                showlegend=False,
                hoverinfo='skip'
            ))
    
    fig.update_layout(
        title=f"📍 Live Airspace Map - Time: {current_time:.1f}s",
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        xaxis=dict(range=[0, 800], scaleanchor="y", scaleratio=1, gridcolor='gray'),
        yaxis=dict(range=[0, 600], gridcolor='gray'),
        plot_bgcolor='black',
        paper_bgcolor='black',
        font=dict(color='white'),
        height=600,
        margin=dict(l=40, r=40, t=60, b=40)
    )
    
    return fig

# Information panel callback
@app.callback(
    [Output('info-panel', 'children'),
     Output('conflict-panel', 'children')],
    [Input('drone-dropdown', 'value'),
     Input('time-slider', 'value')]
)
def update_info_panels(selected_drone_id, current_time):
    selected = next(m for m in missions if m.drone_id == selected_drone_id)
    others = [m for m in missions if m.drone_id != selected_drone_id]
    
    # Drone information
    current_pos = interpolate_position(selected.waypoints, current_time)
    status = "🟢 ACTIVE" if selected.start_time <= current_time <= selected.end_time else "🔴 INACTIVE"
    
    info_content = [
        html.P(f"🆔 Drone ID: {selected.drone_id}"),
        html.P(f"📊 Priority: {selected.priority.upper()}"),
        html.P(f"📍 Current Position: ({current_pos['x']:.1f}, {current_pos['y']:.1f}, {current_pos['z']:.1f})"),
        html.P(f"🎯 Status: {status}"),
        html.P(f"📈 Waypoints: {len(selected.waypoints)}"),
        html.P(f"⏱️ Mission Time: {selected.start_time}s - {selected.end_time}s"),
    ]
    
    # Conflict analysis - use real-time checker for current time
    report = check_realtime_conflicts(selected, others, current_time, uncertainty, safety_distance=5.0)
    
    if report["status"] == "clear":
        conflict_content = [
            html.Div([
                html.H6("✅ NO CONFLICTS DETECTED", className="text-success"),
                html.P("All systems operating normally", className="text-muted")
            ])
        ]
    else:
        severity_colors = {
            "critical": "danger",
            "high": "warning", 
            "medium": "info"
        }
        
        conflict_items = []
        
        # Spatio-temporal conflicts (most critical)
        if report["conflicts"]:
            conflict_items.append(html.H6("🚨 CRITICAL CONFLICTS", className="text-danger"))
            for conflict in report["conflicts"][:3]:
                conflict_items.append(
                    dbc.Alert([
                        html.Strong(f"vs {conflict['conflicting_drone']}"),
                        html.Br(),
                        f"Time: {conflict['time']}s | Distance: {conflict['spatial_distance']:.1f}m"
                    ], color="danger", className="py-2")
                )
        
        # Spatial conflicts
        if report["spatial_conflicts"]:
            conflict_items.append(html.H6("📍 SPATIAL CONFLICTS", className="text-warning"))
            for conflict in report["spatial_conflicts"][:2]:
                conflict_items.append(
                    dbc.Alert([
                        html.Strong(f"vs {conflict['conflicting_drone']}"),
                        html.Br(),
                        f"Distance: {conflict['spatial_distance']:.1f}m"
                    ], color=severity_colors.get(conflict['severity'], 'info'), className="py-1")
                )
        
        # Temporal conflicts
        if report["temporal_conflicts"]:
            conflict_items.append(html.H6("⏰ TEMPORAL CONFLICTS", className="text-info"))
            for conflict in report["temporal_conflicts"][:2]:
                conflict_items.append(
                    dbc.Alert([
                        html.Strong(f"vs {conflict['conflicting_drone']}"),
                        html.Br(),
                        f"Time Overlap: {conflict['time_difference']:.1f}s"
                    ], color=severity_colors.get(conflict['severity'], 'info'), className="py-1")
                )
        
        conflict_content = conflict_items[:6]  # Limit display
    
    return info_content, conflict_content

# Distance analysis callback
@app.callback(
    Output('distance-chart', 'figure'),
    [Input('drone-dropdown', 'value')]
)
def update_distance_chart(selected_drone_id):
    selected = next(m for m in missions if m.drone_id == selected_drone_id)
    others = [m for m in missions if m.drone_id != selected_drone_id]
    
    fig = go.Figure()
    
    # Calculate distances over time
    time_points = np.arange(time_bounds[0], time_bounds[1], 5)
    
    for other in others:
        distances = []
        for t in time_points:
            pos1 = interpolate_position(selected.waypoints, t)
            pos2 = interpolate_position(other.waypoints, t)
            
            dist = np.sqrt((pos1['x'] - pos2['x'])**2 + 
                          (pos1['y'] - pos2['y'])**2 + 
                          (pos1['z'] - pos2['z'])**2)
            distances.append(dist)
        
        color = {
            'emergency': 'red',
            'high': 'orange',
            'normal': 'blue'
        }.get(other.priority, 'blue')
        
        fig.add_trace(go.Scatter(
            x=time_points,
            y=distances,
            mode='lines',
            name=other.drone_id,
            line=dict(color=color, width=2)
        ))
    
    # Add safety threshold line
    fig.add_hline(y=5.0, line_dash="dash", line_color="red", 
                  annotation_text="Safety Threshold")
    
    fig.update_layout(
        title="Distance to Other Drones Over Time",
        xaxis_title="Time (s)",
        yaxis_title="Distance (m)",
        plot_bgcolor='black',
        paper_bgcolor='black',
        font=dict(color='white'),
        height=300,
        margin=dict(l=40, r=40, t=40, b=40)
    )
    
    return fig

# Rerouting callback
@app.callback(
    Output('reroute-output', 'data'),
    Input('reroute-button', 'n_clicks'),
    State('drone-dropdown', 'value')
)
def handle_rerouting(n_clicks, drone_id):
    if n_clicks == 0:
        return {}
    
    # Get current missions from mission manager
    current_missions = mission_manager.get_current_missions()
    selected = next(m for m in current_missions if m.drone_id == drone_id)
    others = [m for m in current_missions if m.drone_id != drone_id]
    
    # Generate rerouted path
    rerouted_waypoints = reroute_path(selected, others, space_bounds=BOUNDS, uncertainty=uncertainty)
    
    if rerouted_waypoints:
        # Apply the reroute using mission manager
        apply_result = mission_manager.apply_reroute(drone_id, rerouted_waypoints)
        
        if apply_result["status"] == "success":
            # Update global missions reference
            global missions
            missions = mission_manager.get_current_missions()
            
            return {
                "status": "success",
                "waypoints": [(wp.x, wp.y, wp.z, wp.t) for wp in rerouted_waypoints],
                "message": f"✅ Reroute applied: {apply_result['message']}",
                "applied": True
            }
        else:
            return {
                "status": "error",
                "message": f"❌ Failed to apply reroute: {apply_result['message']}"
            }
    else:
        return {
            "status": "failed",
            "message": "❌ No conflict-free path found"
        }

# Reroute status display callback
@app.callback(
    Output('reroute-status', 'children'),
    Input('reroute-output', 'data')
)
def update_reroute_status(reroute_data):
    if not reroute_data:
        return ""
    
    message = reroute_data.get('message', '')
    status = reroute_data.get('status', '')
    
    if status == 'success':
        return dbc.Alert(message, color="success", className="py-2 mt-2")
    elif status == 'failed':
        return dbc.Alert(message, color="danger", className="py-2 mt-2")
    else:
        return ""

if __name__ == '__main__':
    app.run(debug=True, host='127.0.0.1', port=8050)
