"""Plotting: state history, controls, 3-D trajectory."""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from flight_dyn.simulation import SimResult


def export_simresult_csv(res: SimResult, path: Path) -> None:
    """Write time history: state (12) + controls to a CSV file."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "t_s",
        "pn_m",
        "pe_m",
        "pd_m",
        "u_m_s",
        "v_m_s",
        "w_m_s",
        "phi_rad",
        "theta_rad",
        "psi_rad",
        "p_rad_s",
        "q_rad_s",
        "r_rad_s",
        "delta_e_rad",
        "delta_a_rad",
        "delta_r_rad",
        "throttle",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for k in range(len(res.t)):
            row = [res.t[k]]
            row.extend(float(v) for v in res.x[k])
            row.append(float(res.controls["delta_e"][k]))
            row.append(float(res.controls["delta_a"][k]))
            row.append(float(res.controls["delta_r"][k]))
            row.append(float(res.controls["throttle"][k]))
            w.writerow(row)


def make_result_figure(res: SimResult, title: str) -> plt.Figure:
    """Build the standard 6-panel figure (altitude, angles, TAS, surfaces, throttle, 3D path)."""
    t = res.t
    x = res.x
    pn, pe, pd = x[:, 0], x[:, 1], x[:, 2]
    h = -pd
    phi, theta, psi = x[:, 6], x[:, 7], x[:, 8]
    uvw = x[:, 3:6]
    Va = np.linalg.norm(uvw, axis=1)
    ctr = res.controls

    fig = plt.figure(figsize=(12, 10))
    fig.suptitle(title, fontsize=12)

    ax1 = fig.add_subplot(3, 2, 1)
    ax1.plot(t, h)
    ax1.set_ylabel("Altitude h (m)")
    ax1.grid(True)

    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(t, np.degrees(theta), label="theta")
    ax2.plot(t, np.degrees(phi), label="phi")
    ax2.set_ylabel("deg")
    ax2.legend()
    ax2.grid(True)

    ax3 = fig.add_subplot(3, 2, 3)
    ax3.plot(t, Va)
    ax3.set_ylabel("TAS |V| (m/s)")
    ax3.grid(True)

    ax4 = fig.add_subplot(3, 2, 4)
    ax4.plot(t, np.degrees(ctr["delta_e"]), label="de")
    ax4.plot(t, np.degrees(ctr["delta_a"]), label="da")
    ax4.plot(t, np.degrees(ctr["delta_r"]), label="dr")
    ax4.set_ylabel("Surf (deg)")
    ax4.legend()
    ax4.grid(True)

    ax5 = fig.add_subplot(3, 2, 5)
    ax5.plot(t, ctr["throttle"])
    ax5.set_ylabel("Throttle")
    ax5.set_xlabel("t (s)")
    ax5.grid(True)

    ax6 = fig.add_subplot(3, 2, 6, projection="3d")
    ax6.plot(pn / 1000.0, pe / 1000.0, h / 1000.0, lw=1.2)
    ax6.set_xlabel("North (km)")
    ax6.set_ylabel("East (km)")
    ax6.set_zlabel("Alt (km)")
    ax6.set_title("Trajectory (NED origin)")

    plt.tight_layout()
    return fig


def plot_result(res: SimResult, title: str, out_path: Path | None = None) -> None:
    fig = make_result_figure(res, title)
    if out_path:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_compare_open_closed(
    open_res: SimResult, closed_res: SimResult, out_path: Path
) -> None:
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax[0].plot(open_res.t, -open_res.x[:, 2], label="Open loop", color="C3")
    ax[0].plot(closed_res.t, -closed_res.x[:, 2], label="Closed loop", color="C2")
    ax[0].set_ylabel("h (m)")
    ax[0].legend()
    ax[0].grid(True)
    ax[0].set_title("Altitude: instability vs control")

    ax[1].plot(open_res.t, np.degrees(open_res.x[:, 7]), label="Open theta", color="C3")
    ax[1].plot(
        closed_res.t, np.degrees(closed_res.x[:, 7]), label="Closed theta", color="C2"
    )
    ax[1].set_ylabel("Pitch (deg)")
    ax[1].set_xlabel("t (s)")
    ax[1].legend()
    ax[1].grid(True)
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
