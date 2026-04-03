"""
Aircraft parameters and aerodynamic / propulsive force & moment model.

Loads from JSON so any jet-like vehicle can be swapped without changing dynamics code.
Coefficients follow stability-axis / wind-axis conventions common in Stevens & Lewis:
  alpha, beta in rad; damping terms use non-dimensional rates p_hat, q_hat, r_hat.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class Inertia:
    Ixx: float
    Iyy: float
    Izz: float
    Ixz: float = 0.0

    def matrix(self) -> np.ndarray:
        return np.array(
            [
                [self.Ixx, 0.0, -self.Ixz],
                [0.0, self.Iyy, 0.0],
                [-self.Ixz, 0.0, self.Izz],
            ],
            dtype=float,
        )

    def inverse(self) -> np.ndarray:
        return np.linalg.inv(self.matrix())


@dataclass
class AeroCoeffs:
    CL0: float
    CL_alpha: float
    CD0: float
    K_induced: float
    CY_beta: float

    Cl_beta: float
    Cl_p: float
    Cl_delta_a: float

    Cm0: float
    Cm_alpha: float
    Cm_q: float
    Cm_delta_e: float

    Cn_beta: float
    Cn_r: float
    Cn_delta_r: float


@dataclass
class ControlLimits:
    elev_deg: float
    ail_deg: float
    rud_deg: float

    def as_rad(self) -> tuple[float, float, float]:
        d = np.pi / 180.0
        return self.elev_deg * d, self.ail_deg * d, self.rud_deg * d


@dataclass
class Aircraft:
    name: str
    references: list[str]
    mass_kg: float
    inertia: Inertia
    S: float
    b: float
    c_bar: float
    T_max: float
    aero: AeroCoeffs
    limits: ControlLimits
    notes: str = ""

    @classmethod
    def from_json(cls, path: str | Path) -> Aircraft:
        p = Path(path)
        with open(p, encoding="utf-8") as f:
            d: dict[str, Any] = json.load(f)
        i = d["inertia_kgm2"]
        ins = Inertia(
            Ixx=i["Ixx"],
            Iyy=i["Iyy"],
            Izz=i["Izz"],
            Ixz=i.get("Ixz", 0.0),
        )
        g = d["geometry"]
        a = d["aero"]
        aero = AeroCoeffs(
            CL0=a["CL0"],
            CL_alpha=a["CL_alpha_per_rad"],
            CD0=a["CD0"],
            K_induced=a["K_induced"],
            CY_beta=a["CY_beta_per_rad"],
            Cl_beta=a["Cl_beta_per_rad"],
            Cl_p=a["Cl_p_per_rad"],
            Cl_delta_a=a["Cl_delta_a_per_rad"],
            Cm0=a["Cm0"],
            Cm_alpha=a["Cm_alpha_per_rad"],
            Cm_q=a["Cm_q_per_rad"],
            Cm_delta_e=a["Cm_delta_e_per_rad"],
            Cn_beta=a["Cn_beta_per_rad"],
            Cn_r=a["Cn_r_per_rad"],
            Cn_delta_r=a["Cn_delta_r_per_rad"],
        )
        lim = d["control_limits_deg"]
        limits = ControlLimits(
            elev_deg=lim["elevator"],
            ail_deg=lim["aileron"],
            rud_deg=lim["rudder"],
        )
        return cls(
            name=d["name"],
            references=list(d.get("references", [])),
            mass_kg=d["mass_kg"],
            inertia=ins,
            S=g["S_m2"],
            b=g["b_m"],
            c_bar=g["c_bar_m"],
            T_max=d["propulsion"]["T_max_N"],
            aero=aero,
            limits=limits,
            notes=d.get("notes", ""),
        )


def aero_forces_moments(
    ac: Aircraft,
    rho: float,
    uvw: np.ndarray,
    pqr: np.ndarray,
    delta_e: float,
    delta_a: float,
    delta_r: float,
    throttle: float,
) -> tuple[np.ndarray, np.ndarray, float, float, float]:
    """
    Returns (F_body, M_body, alpha, beta, Va) with F in N, M in N·m.
    """
    u, v, w = uvw
    Va = float(np.linalg.norm(uvw) + 1e-6)
    alpha = float(np.arctan2(w, u))
    beta = float(np.arcsin(np.clip(v / Va, -1.0, 1.0)))

    p, q, r = pqr
    ph = p * ac.b / (2.0 * Va)
    qh = q * ac.c_bar / (2.0 * Va)
    rh = r * ac.b / (2.0 * Va)

    a = ac.aero
    CL = a.CL0 + a.CL_alpha * alpha
    CD = a.CD0 + a.K_induced * CL ** 2
    CY = a.CY_beta * beta

    Cl = (
        a.Cl_beta * beta
        + a.Cl_p * ph
        + a.Cl_delta_a * delta_a
    )
    Cm = (
        a.Cm0
        + a.Cm_alpha * alpha
        + a.Cm_q * qh
        + a.Cm_delta_e * delta_e
    )
    Cn = (
        a.Cn_beta * beta
        + a.Cn_r * rh
        + a.Cn_delta_r * delta_r
    )

    Q = 0.5 * rho * Va ** 2
    S, b, c = ac.S, ac.b, ac.c_bar

    # Wind-axis drag/side/lift → body (R_w2b @ [-D, Y, -L])
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta), np.sin(beta)
    R_w2b = np.array(
        [
            [ca * cb, -ca * sb, -sa],
            [sb, cb, 0.0],
            [sa * cb, -sa * sb, ca],
        ]
    )
    D, Y_w, L = Q * S * CD, Q * S * CY, Q * S * CL
    F_aero = R_w2b @ np.array([-D, Y_w, -L])

    F_thrust = np.array([ac.T_max * float(np.clip(throttle, 0.0, 1.0)), 0.0, 0.0])
    F_body = F_aero + F_thrust

    M_body = np.array(
        [Q * S * b * Cl, Q * S * c * Cm, Q * S * b * Cn],
        dtype=float,
    )
    return F_body, M_body, alpha, beta, Va
