"""
6-DOF jet simulation entry point.

From this directory:
  python main.py              # run experiments + plots + trim report
  python main.py --trim-only  # only print trim validation
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
    cfg = root / "configs" / "f16_stevens_nominal.json"
    ac = Aircraft.from_json(cfg)
    out = root / "outputs"

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--trim-only",
        action="store_true",
        help="Print trim residuals only (quick validation vs literature)",
    )
    args = ap.parse_args()

    print_trim_report(ac, V_m_s=213.0, h_m=5000.0)
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
