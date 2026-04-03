"""
6-DOF jet simulation entry point.

From this directory:
  python main.py              # run experiments + plots + trim report
  python main.py --trim-only  # only print trim validation
  python main.py --config configs/other.json --output-dir out2
"""
from __future__ import annotations

import argparse
from pathlib import Path

from flight_dyn.aircraft import Aircraft
from flight_dyn.experiments import run_all_experiments
from flight_dyn.validation import print_trim_report
from flight_dyn.visualize import plot_compare_open_closed, plot_result


def main() -> None:
    root = Path(__file__).resolve().parent

    ap = argparse.ArgumentParser(description="6-DOF nonlinear jet simulation + experiments.")
    ap.add_argument(
        "--trim-only",
        action="store_true",
        help="Print trim residuals only (no experiments or plots).",
    )
    ap.add_argument(
        "--config",
        type=Path,
        default=root / "configs" / "f16_stevens_nominal.json",
        help="Path to aircraft JSON (default: configs/f16_stevens_nominal.json).",
    )
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=root / "outputs",
        help="Directory for PNG figures (default: outputs/).",
    )
    ap.add_argument(
        "--tas",
        type=float,
        default=213.0,
        help="True airspeed (m/s) for trim printout.",
    )
    ap.add_argument(
        "--altitude",
        type=float,
        default=5000.0,
        help="Altitude (m) for trim printout.",
    )
    args = ap.parse_args()

    cfg = args.config.resolve()
    if not cfg.is_file():
        raise SystemExit(f"Config not found: {cfg}")

    ac = Aircraft.from_json(cfg)
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)

    print_trim_report(ac, V_m_s=args.tas, h_m=args.altitude)
    if args.trim_only:
        return

    results = run_all_experiments(ac, output_dir=out)
    plot_result(results["open"], "Open-loop (pitch disturbance)", out / "01_open_loop.png")
    plot_result(
        results["closed"], "Closed-loop recovery", out / "02_closed_loop.png"
    )
    plot_result(results["step"], "Altitude step +200 m @ 10 s", out / "03_alt_step.png")
    plot_result(
        results["mass"], "Sensitivity: +15% mass", out / "04_mass_sensitivity.png"
    )
    plot_compare_open_closed(
        results["open"], results["closed"], out / "00_open_vs_closed_alt_pitch.png"
    )
    print(f"Figures written to {out}")


if __name__ == "__main__":
    main()
