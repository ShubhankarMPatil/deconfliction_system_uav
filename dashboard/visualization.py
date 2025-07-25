import plotly.graph_objects as go
import numpy as np

# Preprocess drone data into numpy arrays for efficiency

def preprocess_drone_data(drone_data):
    processed = {}
    for drone_id, waypoints in drone_data.items():
        arr = np.array([[wp['x'], wp['y'], wp['t']] for wp in waypoints])
        processed[drone_id] = arr
    return processed

# Store preprocessed data globally (assume drone_data is loaded once)
preprocessed_data = None

def generate_2d_figure_at_time(drone_data, current_time):
    global preprocessed_data
    if preprocessed_data is None:
        preprocessed_data = preprocess_drone_data(drone_data)
    fig = go.Figure()

    for drone_id, arr in preprocessed_data.items():
        mask = arr[:, 2] <= current_time
        if not np.any(mask):
            continue
        latest = arr[mask][-1]
        fig.add_trace(go.Scatter(
            x=[latest[0]], y=[latest[1]],
            mode="markers+text",
            marker=dict(size=12),
            name=drone_id,
            text=[drone_id],
            textposition="top center"
        ))
        fig.add_trace(go.Scatter(
            x=arr[mask][:, 0], y=arr[mask][:, 1],
            mode="lines",
            name=f"{drone_id}_path",
            line=dict(width=2, dash="dot")
        ))

    fig.update_layout(
        title=f"Drone Positions at t={current_time}",
        xaxis_title="X",
        yaxis_title="Y",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        height=600,
    )
    return fig
