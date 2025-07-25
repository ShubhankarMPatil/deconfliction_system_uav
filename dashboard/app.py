import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import streamlit as st
import time as pytime
import json
from visualization import generate_2d_figure_at_time
from utils.time_utils import get_time_bounds

st.set_page_config(layout="wide")
st.title("Drone Simulation 2D")

# 🔽 Load simulated drone data from JSON
with open("input/simulated_drone_positions.json") as f:
    drone_data = json.load(f)

min_time, max_time = get_time_bounds(drone_data)

# ▶️ Playback controls
col1, col2 = st.columns([5, 1])
with col2:
    if "play" not in st.session_state:
        st.session_state.play = False
    if st.button("▶️ Play" if not st.session_state.play else "⏸ Pause"):
        st.session_state.play = not st.session_state.play

with col1:
    time_val = st.slider("Time", min_time, max_time, min_time, 1, key="time_slider")

# 🎞️ Animate if playing
if st.session_state.play:
    last_time = time_val
    for t in range(time_val, max_time + 1):
        if t != last_time:
            fig = generate_2d_figure_at_time(drone_data, t)
            st.plotly_chart(fig, use_container_width=True)
            st.session_state.time_slider = t
            last_time = t
        pytime.sleep(0.5)  # Throttle to 2 FPS for performance
        if not st.session_state.play:
            break
else:
    fig = generate_2d_figure_at_time(drone_data, time_val)
    st.plotly_chart(fig, use_container_width=True)
