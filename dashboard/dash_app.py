import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import dash
from dash import html, dcc, Output, Input, State
import dash_bootstrap_components as dbc
import plotly.graph_objs as go
import json
import time
from utils.input_loader import load_missions
from utils.conflict_checker import check_conflicts
from models.uncertainty_model import UncertaintyParameters
from models.ai_rerouting_agent import reroute_path
from dashboard.plotting import generate_2d_figure_at_time

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

missions = load_missions("../input/missions.json")
dr_map = {m.drone_id: m for m in missions}
uncertainty = UncertaintyParameters()

time_bounds = [min(wp.t for m in missions for wp in m.waypoints),
               max(wp.t for m in missions for wp in m.waypoints)]

app.layout = dbc.Container([
    dbc.Row([
        dbc.Col(html.H3("UAV Deconfliction Dashboard (Dash)"), width=12)
    ]),
    dbc.Row([
        dbc.Col([
            dcc.Graph(id='drone-map', style={"height": "600px"})
        ], width=8),
        dbc.Col([
            html.Label("Select Drone"),
            dcc.Dropdown([m.drone_id for m in missions], id='drone-select', value=missions[0].drone_id),
            html.Br(),
            html.Div(id='drone-info'),
            html.Br(),
            html.Button("Reroute Drone", id="reroute-btn", n_clicks=0),
            html.Div(id='reroute-status'),
            html.Br(),
            html.Div(id='conflict-report', style={"whiteSpace": "pre-wrap"})
        ], width=4)
    ]),
    dbc.Row([
        dbc.Col([
            dcc.Slider(id='time-slider', min=time_bounds[0], max=time_bounds[1], step=1, value=time_bounds[0],
                       marks=None, tooltip={"always_visible": False}),
            dcc.Interval(id='play-interval', interval=500, n_intervals=0, disabled=True),
            html.Button("▶ Play", id='play-btn', n_clicks=0),
        ])
    ])
])

@app.callback(
    Output('drone-map', 'figure'),
    Output('drone-info', 'children'),
    Output('conflict-report', 'children'),
    Input('time-slider', 'value'),
    Input('drone-select', 'value')
)
def update_map(t, selected_id):
    drone_fig = generate_2d_figure_at_time(missions, t)
    selected = next(m for m in missions if m.drone_id == selected_id)
    info = f"Drone ID: {selected.drone_id}\nPriority: {selected.priority}\nWaypoints: {len(selected.waypoints)}"
    others = [m for m in missions if m.drone_id != selected_id]
    report = check_conflicts(selected, others, uncertainty)
    summary = "No conflicts." if report['status'] == 'clear' else "Conflicts detected:\n" + \
              "\n".join([f"- {c['conflicting_drone']} at t={c['time']}" for c in report['conflicts'][:3]])
    return drone_fig, info, summary

@app.callback(
    Output('time-slider', 'value'),
    Input('play-interval', 'n_intervals'),
    State('time-slider', 'value')
)
def advance_slider(n, t):
    return t + 1 if t < time_bounds[1] else time_bounds[0]

@app.callback(
    Output('play-interval', 'disabled'),
    Output('play-btn', 'children'),
    Input('play-btn', 'n_clicks'),
    State('play-interval', 'disabled')
)
def toggle_play(n_clicks, is_disabled):
    return not is_disabled, ("⏸ Pause" if is_disabled else "▶ Play")

@app.callback(
    Output('reroute-status', 'children'),
    Input('reroute-btn', 'n_clicks'),
    State('drone-select', 'value')
)
def trigger_reroute(n, drone_id):
    if n == 0:
        return ""
    primary = dr_map[drone_id]
    others = [m for m in missions if m.drone_id != drone_id]
    new_path = reroute_path(primary, others, (200, 200, 30), uncertainty)
    if new_path:
        return f"Reroute successful: {len(new_path)} waypoints"
    else:
        return "No conflict-free path found."

if __name__ == '__main__':
    app.run(debug=True)
