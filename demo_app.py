"""
Interactive browser demo (Streamlit): sliders for flight condition, disturbance, and gains.

Run from the repository root:
  pip install streamlit
  streamlit run demo_app.py
"""
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

from flight_dyn.aircraft import Aircraft
from flight_dyn.controls import ControlGains, FlightControlSystem
from flight_dyn.experiments import run_closed_loop_demo

ROOT = Path(__file__).resolve().parent


def main() -> None:
    st.set_page_config(page_title="6-DOF Flight Dynamics Demo", layout="wide")
    st.title("6-DOF jet — closed-loop demo")
    st.caption(
        "Nonlinear simulation with cascaded PID. Adjust sliders and click **Run simulation**."
    )

    cfg_path = ROOT / "configs" / "f16_stevens_nominal.json"
    ac = Aircraft.from_json(cfg_path)

    with st.sidebar:
        st.header("Flight condition")
        V_m_s = st.slider("True airspeed (m/s)", 150.0, 280.0, 213.0, 1.0)
        h_m = st.slider("Altitude (m)", 1000.0, 11000.0, 5000.0, 100.0)
        pitch_deg = st.slider("Initial pitch disturbance (deg)", 0.0, 8.0, 2.0, 0.25)
        t_final = st.slider("Simulation length (s)", 20.0, 120.0, 60.0, 5.0)

        st.header("Controller — outer loop scale")
        alt_scale = st.slider("Altitude PID scale (×)", 0.25, 2.5, 1.0, 0.05)
        pitch_att_scale = st.slider("Pitch attitude PID scale (×)", 0.25, 2.5, 1.0, 0.05)

        run = st.button("Run simulation", type="primary")

    if not run:
        st.info("Use the sidebar and click **Run simulation**.")
        return

    g = ControlGains()
    g.alt.kp *= alt_scale
    g.alt.ki *= alt_scale
    g.alt.kd *= alt_scale
    g.pitch_att.kp *= pitch_att_scale
    g.pitch_att.ki *= pitch_att_scale
    g.pitch_att.kd *= pitch_att_scale

    fcs = FlightControlSystem(gains=g)
    with st.spinner("Integrating…"):
        res = run_closed_loop_demo(
            ac,
            V_m_s=V_m_s,
            h_m=h_m,
            pitch_disturb_deg=pitch_deg,
            t_final=t_final,
            dt=0.02,
            fcs=fcs,
        )

    t = res.t
    x = res.x
    h_plot = -x[:, 2]
    theta_deg = np.degrees(x[:, 7])
    phi_deg = np.degrees(x[:, 6])
    Va = np.linalg.norm(x[:, 3:6], axis=1)
    ctr = res.controls

    c1, c2 = st.columns(2)
    fig1, ax1 = plt.subplots(figsize=(6, 3))
    ax1.plot(t, h_plot, color="C0")
    ax1.axhline(h_m, color="k", ls="--", lw=0.8, label="Command")
    ax1.set_ylabel("Altitude (m)")
    ax1.set_xlabel("t (s)")
    ax1.legend()
    ax1.grid(True)
    c1.pyplot(fig1)
    plt.close(fig1)

    fig2, ax2 = plt.subplots(figsize=(6, 3))
    ax2.plot(t, theta_deg, label="pitch θ")
    ax2.plot(t, phi_deg, label="roll φ", alpha=0.7)
    ax2.set_ylabel("deg")
    ax2.set_xlabel("t (s)")
    ax2.legend()
    ax2.grid(True)
    c2.pyplot(fig2)
    plt.close(fig2)

    fig3, ax3 = plt.subplots(figsize=(10, 3))
    ax3.plot(t, Va, color="C2")
    ax3.set_ylabel("TAS (m/s)")
    ax3.set_xlabel("t (s)")
    ax3.grid(True)
    st.pyplot(fig3)
    plt.close(fig3)

    fig4, ax4 = plt.subplots(figsize=(10, 3))
    ax4.plot(t, np.degrees(ctr["delta_e"]), label="elev")
    ax4.plot(t, np.degrees(ctr["delta_a"]), label="ail")
    ax4.plot(t, np.degrees(ctr["delta_r"]), label="rud")
    ax4.set_ylabel("deg")
    ax4.set_xlabel("t (s)")
    ax4.legend(ncol=3)
    ax4.grid(True)
    st.pyplot(fig4)
    plt.close(fig4)


if __name__ == "__main__":
    main()
