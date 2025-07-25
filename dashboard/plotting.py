import plotly.graph_objects as go
import pandas as pd

def generate_2d_figure_at_time(missions, current_time):
    fig = go.Figure()

    for mission in missions:
        waypoints = mission.waypoints
        df = pd.DataFrame([{
            "x": wp.x,
            "y": wp.y,
            "z": wp.z,
            "t": wp.t
        } for wp in waypoints])

        df = df[df["t"] <= current_time]
        if df.empty:
            continue
        latest = df.iloc[-1]

        fig.add_trace(go.Scatter(
            x=[latest["x"]],
            y=[latest["y"]],
            mode="markers+text",
            name=mission.drone_id,
            text=[mission.drone_id],
            textposition="top center",
            marker=dict(size=12)
        ))

        fig.add_trace(go.Scatter(
            x=df["x"], y=df["y"],
            mode="lines",
            name=f"{mission.drone_id}_path",
            line=dict(width=2, dash="dot")
        ))

    fig.update_layout(
        title=f"Drone Paths at t={current_time}",
        xaxis_title="X",
        yaxis_title="Y",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        height=600,
    )

    return fig
