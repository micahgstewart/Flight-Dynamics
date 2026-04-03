"""
Multi-loop flight control: outer guidance + inner attitude / rate loops.

Architecture (cascaded)
-----------------------
* Altitude hold → θ_cmd (deg) via PID on h error
* Pitch attitude → q_cmd via PID on θ error; elevator via PID on q error
* Roll attitude → p_cmd via PID on φ error; aileron via PID on p error
* Yaw damper: rudder from r (inner loop)

All angles in radians internally; gains tuned in sensible SI / rad units.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    out_min: float
    out_max: float


@dataclass
class ControlGains:
    """Bundled gains for interview-friendly tuning."""
    alt: PIDGains = field(
        default_factory=lambda: PIDGains(2.5e-4, 5e-6, 8e-4, -0.35, 0.35)
    )  # outputs theta_cmd rad (~±20 deg)
    pitch_att: PIDGains = field(
        default_factory=lambda: PIDGains(4.0, 0.15, 0.85, -0.55, 0.55)
    )  # q_cmd rad/s
    pitch_rate: PIDGains = field(
        default_factory=lambda: PIDGains(-18.0, -3.0, -1.2, -0.45, 0.45)
    )  # delta_e rad (negative for pull-up convention after plant)
    roll_att: PIDGains = field(
        default_factory=lambda: PIDGains(6.0, 0.2, 0.35, -1.2, 1.2)
    )  # p_cmd rad/s
    roll_rate: PIDGains = field(
        default_factory=lambda: PIDGains(0.15, 0.02, 0.03, -0.35, 0.35)
    )  # delta_a rad
    yaw_damp: PIDGains = field(
        default_factory=lambda: PIDGains(-0.45, 0.0, -0.12, -0.35, 0.35)
    )  # delta_r from r
    throttle_trim: float = 0.62


class PID:
    def __init__(self, g: PIDGains):
        self.g = g
        self.i = 0.0
        self.prev_e = 0.0

    def reset(self) -> None:
        self.i = 0.0
        self.prev_e = 0.0

    def step(self, e: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        self.i += e * dt
        d = (e - self.prev_e) / dt
        self.prev_e = e
        u = self.g.kp * e + self.g.ki * self.i + self.g.kd * d
        return float(np.clip(u, self.g.out_min, self.g.out_max))


@dataclass
class ControlState:
    alt: PID
    pitch_att: PID
    pitch_rate: PID
    roll_att: PID
    roll_rate: PID
    yaw_damp: PID

    @classmethod
    def from_gains(cls, g: ControlGains) -> ControlState:
        return cls(
            alt=PID(g.alt),
            pitch_att=PID(g.pitch_att),
            pitch_rate=PID(g.pitch_rate),
            roll_att=PID(g.roll_att),
            roll_rate=PID(g.roll_rate),
            yaw_damp=PID(g.yaw_damp),
        )

    def reset(self) -> None:
        for p in (
            self.alt,
            self.pitch_att,
            self.pitch_rate,
            self.roll_att,
            self.roll_rate,
            self.yaw_damp,
        ):
            p.reset()


def clip_surface(
    de: float, da: float, dr: float, elev_lim: float, ail_lim: float, rud_lim: float
) -> tuple[float, float, float]:
    return (
        float(np.clip(de, -elev_lim, elev_lim)),
        float(np.clip(da, -ail_lim, ail_lim)),
        float(np.clip(dr, -rud_lim, rud_lim)),
    )


class FlightControlSystem:
    """
    control_on: bypass surfaces to zeros / throttle only when False.
    """

    def __init__(self, gains: ControlGains | None = None):
        self.g = gains or ControlGains()
        self.pid = ControlState.from_gains(self.g)
        self.control_on = True
        self.alt_cmd_m: float = 5000.0
        self.phi_cmd_rad: float = 0.0
        self.throttle: float = self.g.throttle_trim

    def reset(self) -> None:
        self.pid.reset()

    def compute(
        self,
        x: np.ndarray,
        dt: float,
        elev_lim: float,
        ail_lim: float,
        rud_lim: float,
    ) -> tuple[float, float, float, float]:
        pn, pe, pd = x[0], x[1], x[2]
        u, v, w = x[3], x[4], x[5]
        phi, theta = float(x[6]), float(x[7])
        p, q, r = float(x[9]), float(x[10]), float(x[11])

        h = -pd
        if not self.control_on:
            return 0.0, 0.0, 0.0, float(np.clip(self.throttle, 0.0, 1.0))

        theta_cmd = self.pid.alt.step(self.alt_cmd_m - h, dt)
        q_cmd = self.pid.pitch_att.step(theta_cmd - theta, dt)
        delta_e = self.pid.pitch_rate.step(q_cmd - q, dt)

        p_cmd = self.pid.roll_att.step(self.phi_cmd_rad - phi, dt)
        delta_a = self.pid.roll_rate.step(p_cmd - p, dt)

        delta_r = self.pid.yaw_damp.step(r, dt)

        delta_e, delta_a, delta_r = clip_surface(
            delta_e, delta_a, delta_r, elev_lim, ail_lim, rud_lim
        )
        return delta_e, delta_a, delta_r, float(np.clip(self.throttle, 0.0, 1.0))
