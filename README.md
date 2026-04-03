# 6-DOF Nonlinear Jet Flight Dynamics & Control

Professional-scope **nonlinear six-degree-of-freedom** simulation of a **jet-like fixed-wing** aircraft with **multi-loop PID control**, built for flight dynamics and controls interview discussions.

## What is modeled

- **State (12):** NED position \((p_N,p_E,p_D)\), body velocities \((u,v,w)\), Euler angles \((\phi,\theta,\psi)\), body angular rates \((p,q,r)\).
- **Dynamics:** Full rigid-body kinematics and kinetics; gravity in body axes; inertia tensor including \(I_{xz}\); aerodynamic forces via wind-axis \(L,D,Y\) mapped to body; thrust along body \(+X\).
- **Atmosphere:** ISA troposphere density for dynamic pressure vs altitude.
- **Controls:** Cascaded loops — altitude → pitch attitude → pitch rate → elevator; roll attitude → roll rate → aileron; yaw damper on yaw rate; throttle held trim-like (constant in demos unless you extend it).

## Reference aircraft (validation-oriented)

Default parameters live in `configs/f16_stevens_nominal.json`. They follow the **F-16-class modeling tradition** used in:

- Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). *Aircraft Control and Simulation*, 3rd ed. Wiley.
- Heidlauf, P., et al. (2018). *Verification Challenges in F-16 Ground Collision Avoidance and Other Automated Maneuvers*, ARCH@ADHS.

Open benchmark codebases (e.g. AeroBenchVVPython / CSAF) use the same **structural** idea: nonlinear rigid body + low-order aerodynamics for algorithm testing. **This repository is independent code**; it is **not** a government release and **not** claiming flight-test accuracy.

### How we validate

1. **Trim consistency:** `flight_dyn/validation.py` solves for \((\alpha,\delta_e,\text{throttle})\) so that, at fixed true airspeed and altitude, \(\dot u\), \(\dot w\), and \(\dot q\) from the **same** `state_derivative` used in simulation are near zero (see `python main.py --trim-only`).
2. **Order-of-magnitude check:** Trim angle of attack should sit in a **plausible** range for high-speed cruise; exact degrees will not match a handbook unless **all** coefficients and compressibility effects are carried over from a specific report.
3. **Controlled vs uncontrolled:** Experiments show **open-loop divergence** after a pitch upset vs **closed-loop recovery** with the autopilot — the clearest “model + control” interview figure.

The README in the config notes why **`CL0`** was nudged: published tables are often tied to a **full** aero model; a **reduced** 6-DOF checkpoint must have a **self-consistent** baseline lift at trim.

## Project layout

| File / folder        | Role |
|----------------------|------|
| `flight_dyn/aircraft.py` | Parameters, `Aircraft.from_json()`, aero & thrust force/moment |
| `flight_dyn/atmosphere.py` | ISA \(\rho(h)\) |
| `flight_dyn/dynamics.py` | Equations of motion |
| `flight_dyn/controls.py` | PID + multi-loop `FlightControlSystem` |
| `flight_dyn/simulation.py` | RK4 + `run_simulation()` |
| `flight_dyn/validation.py` | Trim solve (nonlinear least squares) |
| `flight_dyn/experiments.py` | Mandated scenarios |
| `flight_dyn/visualize.py` | Matplotlib + 3D trajectory |
| `configs/*.json`     | Swap vehicle without changing equations |
| `main.py`            | Entry point |
| `outputs/`           | Generated figures (after run) |

## Install & run

```bash
cd "Flight Dynamics"
python -m pip install -r requirements.txt
python main.py
```

- **Trim only (quick validation):** `python main.py --trim-only`

Figures are written to `outputs/`, including `00_open_vs_closed_alt_pitch.png`.

## Experiments (mandated set)

1. **Open-loop:** Trim state + **+2° pitch**; controls fixed at trim throttle, zero surfaces → **divergent / diverging-oscillatory** longitudinal motion.
2. **Closed-loop:** Same IC with FCS **ON** → recovery toward commanded altitude.
3. **Altitude step:** **+200 m** command at \(t = 10\) s → transient for rise/overshoot/settling discussion.
4. **Mass sensitivity:** **+15% mass**, same gains → different damping / phugoid-like behavior (may need retuning in a real program).

## Interview talking points

- **6-DOF vs 2-D:** Full translation + rotation; Euler kinematics (gimbal risk at \(\theta\rightarrow\pm90^\circ\) noted in code comments if you extend).
- **Where CFD fits:** High-fidelity solvers usually produce **tables or derivatives** that **populate** this model; coupling CFD inside the time loop is a different (heavy) workflow.
- **Any aircraft:** Change `configs/your_jet.json` — same code path.

## Authoring another aircraft

Copy `configs/f16_stevens_nominal.json`, edit mass, geometry, inertias, `aero` block, and limits. Run `main.py --trim-only` to see if trim is physically plausible; adjust `CL0` / `Cm0` / thrust if trim hits bounds.

## Requirements

- Python ≥ 3.10
- `numpy`, `matplotlib`, `scipy` (for `least_squares` trim)
