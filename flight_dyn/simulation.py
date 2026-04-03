"""Time integration (RK4) and result container."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from flight_dyn.aircraft import Aircraft
from flight_dyn.controls import FlightControlSystem
from flight_dyn.dynamics import state_derivative


@dataclass
class SimResult:
    t: np.ndarray
    x: np.ndarray
    controls: dict[str, np.ndarray] = field(default_factory=dict)
    meta: dict = field(default_factory=dict)


def rk4_step(
    x: np.ndarray,
    ac: Aircraft,
    dt: float,
    delta_e: float,
    delta_a: float,
    delta_r: float,
    throttle: float,
) -> np.ndarray:
    """One RK4 step with fixed controls over the interval."""

    def f(xx: np.ndarray) -> np.ndarray:
        return state_derivative(xx, ac, delta_e, delta_a, delta_r, throttle)

    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def run_simulation(
    ac: Aircraft,
    x0: np.ndarray,
    t_final: float,
    dt: float,
    fcs: FlightControlSystem | None,
    u_constant: tuple[float, float, float, float] | None = None,
) -> SimResult:
    """
    If fcs is not None, controls come from multi-loop each step.
    Else u_constant = (de, da, dr, throttle) is used (open loop).
    """
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
        tk = t[k]
        xk = x_hist[k]
        if fcs is not None:
            de, da, dr, thr = fcs.compute(xk, dt, el, al, rd)
        else:
            assert u_constant is not None
            de, da, dr, thr = u_constant
        de_h[k] = de
        da_h[k] = da
        dr_h[k] = dr
        thr_h[k] = thr
        x_hist[k + 1] = rk4_step(xk, ac, dt, de, da, dr, thr)

    de_h[-1], da_h[-1], dr_h[-1], thr_h[-1] = de_h[-2], da_h[-2], dr_h[-2], thr_h[-2]
    return SimResult(
        t=t,
        x=x_hist,
        controls={
            "delta_e": de_h,
            "delta_a": da_h,
            "delta_r": dr_h,
            "throttle": thr_h,
        },
    )
