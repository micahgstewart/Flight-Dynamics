"""Structured experiments: open-loop, closed-loop, step response, sensitivity."""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np

from flight_dyn.aircraft import Aircraft
from flight_dyn.controls import FlightControlSystem
from flight_dyn.simulation import SimResult, rk4_step, run_simulation
from flight_dyn.validation import find_trim


def _initial_trim_state(ac: Aircraft, V: float, h: float) -> np.ndarray:
    t = find_trim(ac, V, h)
    return t["x_trim"]


def _trim_throttle(ac: Aircraft, V: float, h: float) -> float:
    return float(find_trim(ac, V, h)["throttle"])


def experiment_open_loop_instability(
    ac: Aircraft, out_dir: Path, t_final: float = 40.0, dt: float = 0.02
):
    """
    Open loop: small pitch disturbance grows (longitudinal divergence / oscillation).
    """
    trim = find_trim(ac, 213.0, 5000.0)
    x0 = trim["x_trim"].copy()
    x0[7] += np.radians(2.0)  # theta +2 deg perturbation
    thr0 = float(trim["throttle"])
    res = run_simulation(
        ac, x0, t_final, dt, fcs=None, u_constant=(0.0, 0.0, 0.0, thr0)
    )
    res.meta = {
        "name": "open_loop_disturbance",
        "description": "Zero control; elevator/rudder/aileron fixed; throttle fixed. Pitch perturbed.",
    }
    return res


def experiment_closed_loop_recovery(
    ac: Aircraft, out_dir: Path, t_final: float = 60.0, dt: float = 0.02
):
    """Closed loop: same initial disturbance, autopilot regains altitude."""
    x0 = _initial_trim_state(ac, 213.0, 5000.0)
    x0[7] += np.radians(2.0)
    fcs = FlightControlSystem()
    fcs.control_on = True
    fcs.alt_cmd_m = 5000.0
    fcs.throttle = _trim_throttle(ac, 213.0, 5000.0)
    res = run_simulation(ac, x0, t_final, dt, fcs=fcs)
    res.meta = {"name": "closed_loop_recovery", "description": "Multi-loop FCS ON."}
    return res


def experiment_altitude_step(
    ac: Aircraft, out_dir: Path, t_final: float = 120.0, dt: float = 0.02
):
    """Outer loop: step altitude command at t=10 s."""
    x0 = _initial_trim_state(ac, 213.0, 5000.0)
    fcs = FlightControlSystem()
    fcs.alt_cmd_m = 5000.0
    fcs.throttle = _trim_throttle(ac, 213.0, 5000.0)

    n = int(np.ceil(t_final / dt)) + 1
    t = np.linspace(0.0, t_final, n)
    x_hist = np.zeros((n, 12))
    x_hist[0] = x0
    de_h = np.zeros(n)
    da_h = np.zeros(n)
    dr_h = np.zeros(n)
    thr_h = np.zeros(n)
    el, al, rd = ac.limits.as_rad()

    for k in range(n - 1):
        if t[k] >= 10.0:
            fcs.alt_cmd_m = 5200.0
        xk = x_hist[k]
        de, da, dr, thr = fcs.compute(xk, dt, el, al, rd)
        de_h[k], da_h[k], dr_h[k], thr_h[k] = de, da, dr, thr
        x_hist[k + 1] = rk4_step(xk, ac, dt, de, da, dr, thr)

    de_h[-1], da_h[-1], dr_h[-1], thr_h[-1] = de_h[-2], da_h[-2], dr_h[-2], thr_h[-2]
    res = SimResult(
        t=t,
        x=x_hist,
        controls={
            "delta_e": de_h,
            "delta_a": da_h,
            "delta_r": dr_h,
            "throttle": thr_h,
        },
        meta={
            "name": "altitude_step",
            "description": "Step +200 m altitude at 10 s",
        },
    )
    return res


def experiment_mass_sensitivity(
    ac: Aircraft, out_dir: Path, t_final: float = 50.0, dt: float = 0.02
):
    """Heavier aircraft: +15% mass, same gains — show degraded / different response."""
    ac2 = replace(ac, mass_kg=ac.mass_kg * 1.15)
    x0 = _initial_trim_state(ac2, 213.0, 5000.0)
    x0[7] += np.radians(1.0)
    fcs = FlightControlSystem()
    fcs.control_on = True
    fcs.alt_cmd_m = 5000.0
    fcs.throttle = _trim_throttle(ac2, 213.0, 5000.0)
    res = run_simulation(ac2, x0, t_final, dt, fcs=fcs)
    res.meta = {
        "name": "mass_plus_15pct",
        "description": "+15% mass with identical control gains",
    }
    return res


def run_all_experiments(
    ac: Aircraft, output_dir: str | Path = "outputs"
) -> dict[str, object]:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    results["open"] = experiment_open_loop_instability(ac, out)
    results["closed"] = experiment_closed_loop_recovery(ac, out)
    results["step"] = experiment_altitude_step(ac, out)
    results["mass"] = experiment_mass_sensitivity(ac, out)
    return results
