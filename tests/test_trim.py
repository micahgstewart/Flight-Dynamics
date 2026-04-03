"""Trim and EOM consistency checks."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from flight_dyn.aircraft import Aircraft
from flight_dyn.dynamics import state_derivative
from flight_dyn.validation import find_trim


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CFG = ROOT / "configs" / "f16_stevens_nominal.json"


@pytest.fixture(scope="module")
def aircraft() -> Aircraft:
    return Aircraft.from_json(DEFAULT_CFG)


def test_find_trim_longitudinal_residual_small(aircraft: Aircraft) -> None:
    """At trim, u_dot, w_dot, q_dot should be negligible (same definition as trim_residual)."""
    V, h = 213.0, 5000.0
    out = find_trim(aircraft, V, h)
    assert out["success"]
    x = out["x_trim"]
    de = float(out["delta_e_rad"])
    thr = float(out["throttle"])
    xd = state_derivative(x, aircraft, de, 0.0, 0.0, thr)
    udot, wdot, qdot = xd[3], xd[5], xd[10]
    assert abs(udot) < 1e-6
    assert abs(wdot) < 1e-6
    assert abs(qdot) < 1e-6


def test_trim_report_residual_matches_solver(aircraft: Aircraft) -> None:
    out = find_trim(aircraft, 213.0, 5000.0)
    assert out["residual_norm"] < 1e-8
