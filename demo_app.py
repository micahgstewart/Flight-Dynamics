"""
Interactive browser demo (Streamlit): full dashboard, trim summary, CSV/PNG export per run.

Run from the repository root:
  pip install -r requirements-demo.txt
  streamlit run demo_app.py
"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

from flight_dyn.aircraft import Aircraft
from flight_dyn.controls import ControlGains, FlightControlSystem
from flight_dyn.experiments import run_closed_loop_demo
from flight_dyn.simulation import SimResult
from flight_dyn.validation import find_trim
from flight_dyn.visualize import export_simresult_csv, make_result_figure

ROOT = Path(__file__).resolve().parent


def end_of_run_summary(res: SimResult, altitude_cmd_m: float) -> dict[str, float]:
    """
    State at the last time step (after disturbance + transient).
    Not a second 'trim' — trim only exists for equilibrium; this is actual simulated state.
    """
    x = res.x[-1]
    u, v, w = float(x[3]), float(x[4]), float(x[5])
    Va = float(np.linalg.norm([u, v, w]))
    h = float(-x[2])
    theta_deg = float(np.degrees(x[7]))
    phi_deg = float(np.degrees(x[6]))
    alpha_deg = float(np.degrees(np.arctan2(w, u)))
    beta_deg = (
        float(np.degrees(np.arcsin(np.clip(v / Va, -1.0, 1.0)))) if Va > 1e-6 else 0.0
    )
    h_all = -res.x[:, 2]
    return {
        "t_s": float(res.t[-1]),
        "altitude_m": h,
        "altitude_error_m": h - altitude_cmd_m,
        "theta_deg": theta_deg,
        "phi_deg": phi_deg,
        "tas_m_s": Va,
        "alpha_deg": alpha_deg,
        "beta_deg": beta_deg,
        "altitude_min_m": float(np.min(h_all)),
        "altitude_max_m": float(np.max(h_all)),
    }


def _json_safe(obj: object):
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, float)):
        return float(obj)
    if isinstance(obj, (np.integer, int)):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    return obj


def main() -> None:
    st.set_page_config(page_title="6-DOF Flight Dynamics Demo", layout="wide")
    st.title("6-DOF jet — closed-loop demo")
    st.caption(
        "Same physics as `main.py`. Each run saves PNG + CSV + metadata under `outputs/ui_runs/<timestamp>/`."
    )

    if "ui_last" not in st.session_state:
        st.session_state["ui_last"] = None

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

    if run:
        out_dir = ROOT / "outputs" / "ui_runs" / datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir.mkdir(parents=True, exist_ok=True)

        trim_out = find_trim(ac, V_m_s, h_m)

        g = ControlGains()
        g.alt.kp *= alt_scale
        g.alt.ki *= alt_scale
        g.alt.kd *= alt_scale
        g.pitch_att.kp *= pitch_att_scale
        g.pitch_att.ki *= pitch_att_scale
        g.pitch_att.kd *= pitch_att_scale
        fcs = FlightControlSystem(gains=g)

        with st.spinner("Trim + integrating…"):
            res = run_closed_loop_demo(
                ac,
                V_m_s=V_m_s,
                h_m=h_m,
                pitch_disturb_deg=pitch_deg,
                t_final=t_final,
                dt=0.02,
                fcs=fcs,
            )

        title = f"Closed-loop demo (V={V_m_s:.0f} m/s, h={h_m:.0f} m)"
        fig = make_result_figure(res, title)
        png_path = out_dir / "dashboard.png"
        fig.savefig(png_path, dpi=150)
        plt.close(fig)

        csv_path = out_dir / "state_history.csv"
        export_simresult_csv(res, csv_path)

        end = end_of_run_summary(res, h_m)
        meta = {
            "config": str(cfg_path.name),
            "V_m_s": V_m_s,
            "h_m": h_m,
            "pitch_disturb_deg": pitch_deg,
            "t_final_s": t_final,
            "dt_s": 0.02,
            "alt_pid_scale": alt_scale,
            "pitch_att_pid_scale": pitch_att_scale,
            "trim_before_disturbance": {
                "alpha_deg": float(trim_out["alpha_deg"]),
                "delta_e_deg": float(trim_out["delta_e_deg"]),
                "throttle": float(trim_out["throttle"]),
                "residual_norm": float(trim_out["residual_norm"]),
            },
            "end_of_simulation": end,
            "output_dir": str(out_dir),
        }
        with (out_dir / "run_meta.json").open("w", encoding="utf-8") as f:
            json.dump(_json_safe(meta), f, indent=2)

        st.session_state["ui_last"] = {
            "res": res,
            "trim": trim_out,
            "out_dir": out_dir,
            "meta": meta,
            "title": title,
        }
        st.success(f"Saved: `{out_dir.relative_to(ROOT)}`")

    last = st.session_state["ui_last"]
    if last is None:
        st.info("Use the sidebar and click **Run simulation** to trim, integrate, and save results.")
        return

    res = last["res"]
    trim_out = last["trim"]
    out_dir: Path = last["out_dir"]
    meta = last["meta"]
    title = last["title"]

    tab_sum, tab_dash, tab_files = st.tabs(
        ["Summary & trim", "Full dashboard", "Saved files & downloads"]
    )

    end = meta.get("end_of_simulation") or meta.get("end_of_run")
    trim_key = "trim_before_disturbance" if "trim_before_disturbance" in meta else "trim"

    with tab_sum:
        st.subheader("Run parameters")
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("TAS (m/s)", f"{meta['V_m_s']:.1f}")
        c2.metric("Altitude (m)", f"{meta['h_m']:.0f}")
        c3.metric("Pitch disturb (deg)", f"{meta['pitch_disturb_deg']:.2f}")
        c4.metric("Sim length (s)", f"{meta['t_final_s']:.1f}")
        st.write(
            f"**PID scales:** altitude ×{meta['alt_pid_scale']:.2f}, "
            f"pitch attitude ×{meta['pitch_att_pid_scale']:.2f}"
        )

        st.subheader("Equilibrium trim at (V, h) — before pitch disturbance")
        st.caption(
            "**Trim** means forces/moments balanced for steady flight at that speed and height. "
            "It applies to the **undisturbed** initial condition, not after you add +pitch°."
        )
        tm = meta[trim_key]
        t1, t2, t3, t4 = st.columns(4)
        t1.metric("Trim α (deg)", f"{tm['alpha_deg']:.3f}")
        t2.metric("Trim δ_e (deg)", f"{tm['delta_e_deg']:.3f}")
        t3.metric("Throttle", f"{tm['throttle']:.4f}")
        t4.metric("‖trim residual‖", f"{tm['residual_norm']:.2e}")

        if end:
            st.subheader("End of simulation — last time step (after transient)")
            st.caption(
                "Not a second trim: this is the **actual** altitude, angles, and wind–body angles "
                "at **t = t_final**, after the controller has acted."
            )
            e1, e2, e3, e4 = st.columns(4)
            e1.metric("Altitude (m)", f"{end['altitude_m']:.1f}")
            e2.metric("Alt error vs cmd (m)", f"{end['altitude_error_m']:.2f}")
            e3.metric("θ (deg)", f"{end['theta_deg']:.2f}")
            e4.metric("φ (deg)", f"{end['phi_deg']:.2f}")
            e5, e6, e7, e8 = st.columns(4)
            e5.metric("TAS (m/s)", f"{end['tas_m_s']:.1f}")
            e6.metric("α (deg)", f"{end['alpha_deg']:.2f}")
            e7.metric("β (deg)", f"{end['beta_deg']:.2f}")
            e8.metric("h min / max (m)", f"{end['altitude_min_m']:.0f} / {end['altitude_max_m']:.0f}")

        st.subheader("Files written this run")
        st.code(str(out_dir), language="text")

    with tab_dash:
        st.caption("Same 6-panel layout as `main.py` / `visualize.plot_result`: altitude, θ/φ, TAS, surfaces, throttle, 3D path.")
        fig = make_result_figure(res, title)
        st.pyplot(fig)
        plt.close(fig)

    with tab_files:
        st.write("**Folder on disk** (under project root):")
        st.code(str(out_dir), language="text")

        png_path = out_dir / "dashboard.png"
        csv_path = out_dir / "state_history.csv"
        json_path = out_dir / "run_meta.json"

        col_a, col_b, col_c = st.columns(3)
        if png_path.is_file():
            col_a.download_button(
                "Download dashboard.png",
                data=png_path.read_bytes(),
                file_name="dashboard.png",
                mime="image/png",
            )
        if csv_path.is_file():
            col_b.download_button(
                "Download state_history.csv",
                data=csv_path.read_bytes(),
                file_name="state_history.csv",
                mime="text/csv",
            )
        if json_path.is_file():
            col_c.download_button(
                "Download run_meta.json",
                data=json_path.read_bytes(),
                file_name="run_meta.json",
                mime="application/json",
            )

        with st.expander("Preview `run_meta.json`"):
            st.json(meta)

        n = len(res.t)
        st.caption(
            f"**state_history.csv**: {n} rows, columns t + 12 states + 4 controls (see header row)."
        )


if __name__ == "__main__":
    main()
