# Flight Dynamics — 6-DOF Nonlinear Simulation & Control

Nonlinear **six-degree-of-freedom** rigid-body simulation of a **jet-class** fixed-wing aircraft, with **multi-loop PID flight control**, trim validation, scripted experiments, and plotting.

## Overview

This repository integrates:

- **Equations of motion** — full rigid-body translation and rotation (flat Earth, NED), including \(I_{xz}\) and gravity in body axes.
- **Aerodynamics** — low-order stability-axis style coefficients (lift, drag, side force; rolling, pitching, yawing moments) with control deflections.
- **Propulsion** — thrust along body \(+X\), proportional to throttle \([0,1]\).
- **Atmosphere** — ISA troposphere density vs altitude for dynamic pressure.
- **Control** — cascaded PID: altitude → pitch attitude → pitch rate → elevator; roll attitude → roll rate → aileron; yaw damper on yaw rate.
- **Simulation** — fourth-order Runge–Kutta integration with configurable timestep.

Default trim and experiment conditions use **213 m/s true airspeed** and **5000 m altitude** (subsonic cruise–class point appropriate for this reduced aero model).

## What is modeled

| Component | Description |
|-----------|-------------|
| **State (12)** | NED position \((p_N,p_E,p_D)\), body velocities \((u,v,w)\), Euler angles \((\phi,\theta,\psi)\), body angular rates \((p,q,r)\). |
| **Frames** | Flat Earth NED inertial; body axes \(+X\) forward, \(+Y\) right, \(+Z\) down. |
| **Aero** | Angle of attack and sideslip from \((u,v,w)\); wind-axis \(L,D,Y\) mapped to body; moment coefficients with damping and surface terms. |
| **Assumptions** | Still air (no wind or turbulence in the default experiments); ISA atmosphere; no flexible structure or fuel slosh. |

**Scope:** The aircraft JSON uses **F-16–class** scales and coefficient *structure* aligned with textbook and benchmark literature (e.g. Stevens & Lewis–style tables). This is a **reduced-order academic / analysis model**, not a flight-test or program-of-record simulator.

## Requirements

- Python ≥ 3.10  
- Dependencies: `numpy`, `matplotlib`, `scipy` (see `requirements.txt`)

## Install and run

From the repository root:

```bash
python -m pip install -r requirements.txt
python main.py
```

- **Full run:** prints trim information, runs all experiments, writes figures under `outputs/`.
- **Trim only** (fast check, no plots):

  ```bash
  python main.py --trim-only
  ```

Generated plots include altitude, pitch/roll, airspeed, control surfaces, throttle, and a 3D trajectory. `outputs/` is listed in `.gitignore` by default so generated PNGs are not committed; remove that entry if you want figures in version control.

## Project layout

| Path | Role |
|------|------|
| `main.py` | Entry point: load config, trim report, experiments, visualization |
| `configs/*.json` | Aircraft parameters (mass, inertia, geometry, aero, limits) |
| `flight_dyn/aircraft.py` | `Aircraft` model, aerodynamic and thrust forces/moments |
| `flight_dyn/atmosphere.py` | ISA troposphere \(\rho(h)\) |
| `flight_dyn/dynamics.py` | `state_derivative` — full 6-DOF EOM |
| `flight_dyn/controls.py` | PID blocks and `FlightControlSystem` |
| `flight_dyn/simulation.py` | RK4 step and `run_simulation()` |
| `flight_dyn/validation.py` | Nonlinear trim (`scipy.optimize.least_squares`) |
| `flight_dyn/experiments.py` | Open/closed loop, altitude step, mass sensitivity |
| `flight_dyn/visualize.py` | Matplotlib figures |

## Default aircraft configuration

`configs/f16_stevens_nominal.json` holds the default vehicle. References cited in that file include *Aircraft Control and Simulation* (Stevens & Lewis, 3rd ed.) and F-16 verification-style literature. **This codebase is independent** of any government or vendor release.

The JSON **notes** field explains that **`CL0`** was adjusted slightly so that wings-level trim at the default \((V,h)\) is **self-consistent** with the same nonlinear equations used in time integration—common when porting a subset of coefficients into a reduced model.

## Validation and experiments

**Trim** solves for \((\alpha, \delta_e, \text{throttle})\) at fixed TAS and altitude so that the trim residual from the **same** `state_derivative` used in the simulator is negligible. Use `python main.py --trim-only` to print trim \(\alpha\), elevator, throttle, and residual norm.

**Experiments** (no atmospheric turbulence; deterministic initial conditions and commands):

1. **Open loop** — Trim-like initial state with an extra **+2° pitch**; control surfaces fixed at zero, throttle at trim. Shows **uncontrolled** longitudinal response after an initial attitude offset.
2. **Closed loop** — Same initial offset with the **flight control system** enabled and altitude commanded to the trim altitude. Shows **recovery** toward the commanded state.
3. **Altitude step** — Level trim start; at \(t = 10\,\text{s}\), altitude command increases by **200 m**. Shows **outer-loop** tracking and transient behavior.
4. **Mass sensitivity** — **+15%** mass with identical controller gains and a small pitch perturbation. Illustrates **sensitivity** to parameter changes without retuning.

Together, these cases contrast **plant behavior** vs **closed-loop behavior**, command tracking, and a simple robustness check.

## Adding or changing an aircraft

1. Copy `configs/f16_stevens_nominal.json` to a new file.  
2. Edit mass, geometry, inertias, `aero` coefficients, and `control_limits_deg`.  
3. Point `main.py` at your JSON (or load it in your own driver).  
4. Run `python main.py --trim-only` and confirm trim \(\alpha\), surfaces, and throttle are **within bounds** and **physically plausible**; adjust `CL0`, `Cm0`, or thrust-related inputs if trim sits on limits.

## Limitations (read before interpreting results)

- **Low-order aero** — Valid for small-to-moderate perturbations around a trim point; not a full transonic/supersonic database.  
- **Simple engine** — Throttle × \(T_{\max}\); no spool dynamics or altitude–Mach thrust tables.  
- **No wind, gusts, or sensor noise** unless you extend the code.  
- **Euler angles** — Kinematic singularity near \(\theta = \pm 90^\circ\); not intended for extreme attitudes without modification.

## License

Add a `LICENSE` file if you distribute or reuse this project publicly.
