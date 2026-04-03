"""ISA troposphere density for dynamic pressure (validation-grade, simple)."""
from __future__ import annotations

import numpy as np

G0 = 9.80665  # m/s^2
R_AIR = 287.05  # J/(kg K) specific gas constant
T0 = 288.15  # K sea level
P0 = 101325.0  # Pa
RHO0 = 1.225  # kg/m^3
LAPSE = -0.0065  # K/m troposphere


def isa_troposphere(h_m: float) -> tuple[float, float, float]:
    """
    Returns (rho, pressure, temperature) for 0 <= h <= 11000 m (troposphere).
    """
    h = float(np.clip(h_m, 0.0, 11000.0))
    T = T0 + LAPSE * h
    # Hypsometric / power-law from lapse rate
    exponent = -G0 / (R_AIR * LAPSE) - 1.0
    P = P0 * (T / T0) ** ((-G0) / (R_AIR * LAPSE))
    rho = P / (R_AIR * T)
    return rho, P, T
