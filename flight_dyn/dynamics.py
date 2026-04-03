"""
Nonlinear 6-DOF rigid-body equations of motion.

Frames
------
* Inertial: flat-Earth NED (North, East, Down), origin fixed.
* Body: +X forward, +Y right, +Z down (standard for NED-aligned body at zero attitude).

State (12,)
----------
[ pn, pe, pd, u, v, w, phi, theta, psi, p, q, r ]
  Position in NED (m), body velocities (m/s), Euler angles (rad), body rates (rad/s).
"""
from __future__ import annotations

import numpy as np

from flight_dyn.aircraft import Aircraft, aero_forces_moments
from flight_dyn import atmosphere


def rotation_body_to_ned(phi: float, theta: float, psi: float) -> np.ndarray:
    """Rotation R_bn : v_ned = R_bn @ v_body."""
    cph, sph = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cps, sps = np.cos(psi), np.sin(psi)
    return np.array(
        [
            [cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
            [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
            [-sth, sph * cth, cph * cth],
        ]
    )


def euler_rates(phi: float, theta: float, p: float, q: float, r: float) -> np.ndarray:
    """[phi_dot, theta_dot, psi_dot] from body rates (singularity at cos(theta)=0)."""
    cph, sph = np.cos(phi), np.sin(phi)
    th = float(np.clip(theta, -1.55, 1.55))
    cth = np.cos(th)
    tth = np.sin(th) / cth if abs(cth) > 1e-6 else 0.0
    return np.array(
        [
            p + q * sph * tth + r * cph * tth,
            q * cph - r * sph,
            q * sph / cth + r * cph / cth if abs(cth) > 1e-6 else 0.0,
        ]
    )


def state_derivative(
    x: np.ndarray,
    ac: Aircraft,
    delta_e: float,
    delta_a: float,
    delta_r: float,
    throttle: float,
) -> np.ndarray:
    """
    dx/dt for full 6-DOF model. Flat Earth, rho(h) from ISA below 11 km (h = -pd).
    """
    pn, pe, pd = x[0], x[1], x[2]
    uvw = x[3:6].astype(float)
    phi, theta, psi = float(x[6]), float(x[7]), float(x[8])
    pqr = x[9:12].astype(float)

    h = -pd  # altitude (pd positive down)
    rho, _, _ = atmosphere.isa_troposphere(h)

    F_b, M_b, _, _, _ = aero_forces_moments(
        ac, rho, uvw, pqr, delta_e, delta_a, delta_r, throttle
    )

    # Gravity in body (force toward +Z_ned in NED = weight vector direction on mass)
    R_bn = rotation_body_to_ned(phi, theta, psi)
    g = atmosphere.G0
    F_grav_ned = np.array([0.0, 0.0, ac.mass_kg * g])
    F_grav_body = R_bn.T @ F_grav_ned

    F_total = F_b + F_grav_body
    uvw_dot = F_total / ac.mass_kg - np.cross(pqr, uvw)

    I = ac.inertia.matrix()
    Iinv = ac.inertia.inverse()
    omega_dot = Iinv @ (M_b - np.cross(pqr, I @ pqr))

    pos_dot = R_bn @ uvw
    euler_dot = euler_rates(phi, theta, pqr[0], pqr[1], pqr[2])

    return np.concatenate([pos_dot, uvw_dot, euler_dot, omega_dot])


def pack_state(
    ned: np.ndarray,
    uvw: np.ndarray,
    euler: np.ndarray,
    pqr: np.ndarray,
) -> np.ndarray:
    return np.concatenate([ned, uvw, euler, pqr])
