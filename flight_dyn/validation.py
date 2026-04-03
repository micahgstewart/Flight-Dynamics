"""
Validation helpers: steady-state trim vs handbook-style checks.

Uses the same equations as simulation (not an independent 'truth' model).
Reference: compare trim angle-of-attack order-of-magnitude to published F-16 cruise
discussion in Stevens & Lewis (2015) and verification literature.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

from flight_dyn.aircraft import Aircraft
from flight_dyn.dynamics import pack_state, state_derivative


def trim_residual(
    y: np.ndarray,
    ac: Aircraft,
    V: float,
    h_m: float,
) -> np.ndarray:
    """
    Wings-level symmetric trim with flight-path angle ≈ 0: theta = alpha, v=p=q=r=0.
    Unknowns y = [alpha, delta_e, throttle].
    """
    alpha, delta_e, throttle = float(y[0]), float(y[1]), float(y[2])
    theta = alpha
    u = V * np.cos(alpha)
    w = V * np.sin(alpha)
    x = pack_state(
        ned=np.array([0.0, 0.0, -h_m]),
        uvw=np.array([u, 0.0, w]),
        euler=np.array([0.0, theta, 0.0]),
        pqr=np.zeros(3),
    )
    xd = state_derivative(x, ac, delta_e, 0.0, 0.0, throttle)
    # Longitudinal symmetric trim: u_dot, w_dot, q_dot (indices 3,5,10 in xdot)
    return np.array([xd[3], xd[5], xd[10]], dtype=float)


def find_trim(
    ac: Aircraft,
    V_m_s: float = 213.0,
    h_m: float = 5000.0,
    x0: np.ndarray | None = None,
) -> dict:
    """
    Solve for trim alpha, delta_e, throttle at given TAS and altitude.
    """
    if x0 is None:
        x0 = np.array([0.05, -0.02, 0.55])  # rad, rad, --
    res = least_squares(
        trim_residual,
        x0,
        args=(ac, V_m_s, h_m),
        bounds=(
            np.array([-0.15, -0.45, 0.05]),
            np.array([0.45, 0.45, 1.0]),
        ),
        ftol=1e-9,
    )
    alpha, de, thr = res.x
    theta = alpha
    u = V_m_s * np.cos(alpha)
    w = V_m_s * np.sin(alpha)
    x_trim = pack_state(
        ned=np.array([0.0, 0.0, -h_m]),
        uvw=np.array([u, 0.0, w]),
        euler=np.array([0.0, theta, 0.0]),
        pqr=np.zeros(3),
    )
    xd = state_derivative(x_trim, ac, de, 0.0, 0.0, thr)
    return {
        "success": res.success,
        "alpha_rad": alpha,
        "alpha_deg": np.degrees(alpha),
        "theta_rad": theta,
        "delta_e_rad": de,
        "delta_e_deg": np.degrees(de),
        "throttle": thr,
        "residual_norm": float(np.linalg.norm(res.fun)),
        "xdot_u_w_q": xd[[3, 5, 10]],
        "x_trim": x_trim,
        "message": res.message,
    }


def print_trim_report(ac: Aircraft, V_m_s: float, h_m: float) -> dict:
    """Console report for README / interview notes."""
    out = find_trim(ac, V_m_s, h_m)
    print("=== Trim validation (same EOM as sim) ===")
    print(f"  TAS = {V_m_s:.1f} m/s, h = {h_m:.0f} m")
    print(f"  Trim alpha: {out['alpha_deg']:.2f} deg")
    print(f"  Trim elevator: {out['delta_e_deg']:.2f} deg")
    print(f"  Throttle: {out['throttle']:.3f}")
    print(f"  Residual ||[udot,wdot,qdot]||: {out['residual_norm']:.2e}")
    print(
        "  Compare alpha order-of-magnitude to handbook / textbook cruise "
        "figures for a fighter-class aircraft (typically a few degrees)."
    )
    return out
