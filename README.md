# Flight Dynamics — 6-DOF Nonlinear Simulation & Control

Nonlinear **six-degree-of-freedom** rigid-body simulation of a **jet-class** fixed-wing aircraft, with **multi-loop PID flight control**, trim validation, scripted experiments, and plotting.

## Overview

This repository integrates:

- **Equations of motion** — full rigid-body translation and rotation (flat Earth, NED), including the inertia cross-term **Ixz** (roll–yaw coupling) and gravity in body axes.
- **Aerodynamics** — low-order stability-axis style coefficients (lift, drag, side force; rolling, pitching, yawing moments) with control deflections.
- **Propulsion** — thrust along body **+X**, proportional to throttle from 0 to 1.
- **Atmosphere** — ISA troposphere: air density vs altitude, which sets dynamic pressure for a given speed.
- **Control** — cascaded PID: altitude → pitch attitude → pitch rate → elevator; roll attitude → roll rate → aileron; yaw damper on yaw rate.
- **Simulation** — fourth-order Runge–Kutta integration with configurable timestep.

Default trim and experiment conditions use **213 m/s true airspeed** and **5000 m altitude** (subsonic cruise–class point appropriate for this reduced aero model).

## What is modeled

| Component | Description |
|-----------|-------------|
| **State (12)** | NED position (north, east, down), body velocities (u, v, w), Euler angles (roll φ, pitch θ, yaw ψ), body angular rates (p, q, r). |
| **Frames** | Flat Earth NED inertial; body axes +X forward, +Y right, +Z down. |
| **Aero** | Angle of attack and sideslip from (u, v, w); lift/drag/side force in wind axes, then mapped to body; moment coefficients with damping and surface terms. |
| **Assumptions** | Still air (no wind or turbulence in the default experiments); ISA atmosphere; no flexible structure or fuel slosh. |

**Scope:** The aircraft JSON uses **F-16–class** scales and coefficient layout similar to common textbook and benchmark examples. This is a **reduced-order academic / analysis model**, not a flight-test or program-of-record simulator.

## Requirements

- Python ≥ 3.10  
- Core: `numpy`, `matplotlib`, `scipy`, `pytest` (see `requirements.txt`)

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

**CLI options** (all optional):

| Flag | Meaning |
|------|---------|
| `--config PATH` | Aircraft JSON (default: `configs/f16_stevens_nominal.json`) |
| `--output-dir PATH` | Where to write PNGs (default: `outputs`) |
| `--tas FLOAT` | True airspeed (m/s) for the trim printout |
| `--altitude FLOAT` | Altitude (m) for the trim printout |

Example: `python main.py --config configs/f16_stevens_nominal.json --output-dir ./out --tas 213 --altitude 5000`

Generated plots include altitude, pitch/roll, airspeed, control surfaces, throttle, and a 3D trajectory. `outputs/` is listed in `.gitignore` by default so generated PNGs are not committed; remove that entry if you want figures in version control.

## Tests

```bash
python -m pytest tests -q
```

Checks that trim produces near-zero longitudinal accelerations (same definition as the trim solver).

## Interactive demo (optional)

Install Streamlit and run the browser UI (sliders for speed, altitude, pitch disturbance, gain scaling):

```bash
python -m pip install -r requirements-demo.txt
streamlit run demo_app.py
```

The UI is organized in **tabs**: **Summary & trim** (run settings, **equilibrium trim** at (V, h) before the pitch disturbance, and **end-of-run** state at the last time step — altitude error, θ/φ, α/β, TAS, h min/max), **Full dashboard** (same **6-panel** figure as `visualize.plot_result`), and **Saved files & downloads**. Each **Run simulation** writes a timestamped folder:

`outputs/ui_runs/<YYYYMMDD_HHMMSS>/` containing `dashboard.png`, `state_history.csv`, and `run_meta.json`.

This is for **exploration and demos**; batch results from `main.py` still go to `outputs/` (default filenames like `01_open_loop.png`).

**How the UI differs from `main.py`:** the Streamlit app runs **one scenario** — **closed loop** only (autopilot on, recovery from an initial pitch disturbance at sliders for V, h, and gain scaling). It uses the default config file only. **`python main.py`** runs the **full suite**: trim printout, **open loop**, **closed loop**, **altitude step**, **mass sensitivity**, and writes **all** PNGs to `outputs/`. Same underlying simulation and control code; not the same set of cases.

## Project layout

| Path | Role |
|------|------|
| `main.py` | Entry point: load config, trim report, experiments, visualization |
| `configs/*.json` | Aircraft parameters (mass, inertia, geometry, aero, limits) |
| `flight_dyn/aircraft.py` | `Aircraft` model, aerodynamic and thrust forces/moments |
| `flight_dyn/atmosphere.py` | ISA troposphere density vs altitude |
| `flight_dyn/dynamics.py` | `state_derivative` — full 6-DOF equations of motion |
| `flight_dyn/controls.py` | PID blocks and `FlightControlSystem` |
| `flight_dyn/simulation.py` | RK4 step and `run_simulation()` |
| `flight_dyn/validation.py` | Nonlinear trim (`scipy.optimize.least_squares`) |
| `flight_dyn/experiments.py` | Open/closed loop, altitude step, mass sensitivity |
| `flight_dyn/visualize.py` | Matplotlib figures |
| `demo_app.py` | Optional Streamlit UI (tabs, full dashboard, `outputs/ui_runs/…`) |
| `tests/` | Pytest trim / EOM checks |

## Default aircraft configuration

`configs/f16_stevens_nominal.json` holds the default vehicle. **This codebase is independent** of any government or vendor release.

The JSON **notes** field explains that **CL0** (baseline lift coefficient) was adjusted slightly so that wings-level trim at the default speed and altitude is **self-consistent** with the same nonlinear equations used in time integration. That is common when you only use a **subset** of coefficients from a larger aero model.

## Validation and experiments

**Trim** finds angle of attack **alpha**, elevator **delta_e**, and **throttle** at fixed true airspeed and altitude so that the trim residual from the **same** `state_derivative` used in the simulator is negligible. Run `python main.py --trim-only` to print trim alpha, elevator, throttle, and residual norm.

**Experiments** (no atmospheric turbulence; deterministic initial conditions and commands):

1. **Open loop** — Trim-like initial state with an extra **+2° pitch**; control surfaces fixed at zero, throttle at trim. Shows **uncontrolled** longitudinal response after an initial attitude offset.
2. **Closed loop** — Same initial offset with the **flight control system** enabled and altitude commanded to the trim altitude. Shows **recovery** toward the commanded state.
3. **Altitude step** — Level trim start; at **t = 10 s**, altitude command increases by **200 m**. Shows **outer-loop** tracking and transient behavior.
4. **Mass sensitivity** — **+15%** mass with identical controller gains and a small pitch perturbation. Illustrates **sensitivity** to parameter changes without retuning.

Together, these cases contrast **plant behavior** (aircraft physics only) vs **closed-loop behavior** (with controller), command tracking, and a simple robustness check.

## Adding or changing an aircraft

1. Copy `configs/f16_stevens_nominal.json` to a new file.  
2. Edit mass, geometry, inertias, `aero` coefficients, and `control_limits_deg`.  
3. Point `main.py` at your JSON (or load it in your own driver).  
4. Run `python main.py --trim-only` and confirm trim angle of attack, surfaces, and throttle are **within bounds** and **physically plausible**; adjust **CL0**, **Cm0**, or thrust-related inputs if trim sits on limits.

## Limitations (read before interpreting results)

- **Low-order aero** — Valid for small-to-moderate perturbations around a trim point; not a full transonic/supersonic database.  
- **Simple engine** — Throttle times **maximum thrust**; no spool dynamics or altitude–Mach thrust tables.  
- **No wind, gusts, or sensor noise** unless you extend the code.  
- **Euler angles** — The kinematic equations become **singular** if pitch angle goes to **straight up or straight down** (about ±90° from level); this project is not aimed at sustained aerobatic attitudes without a different formulation.

## License

Add a `LICENSE` file if you distribute or reuse this project publicly.
