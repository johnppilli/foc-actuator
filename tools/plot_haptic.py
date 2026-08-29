#!/usr/bin/env python3
"""Plot torque-vs-angle for every haptic preset.

    ./build/focsim haptic-curves > build/haptic-curves.csv
    python3 tools/plot_haptic.py build/haptic-curves.csv --out build/haptic-curves.png

The curves come from the C implementation (foc/impedance.c), so what you see
is exactly what the firmware will command. Tweak a preset there and re-run.
"""
import argparse
import csv
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default="haptic-curves.png")
    args = ap.parse_args()

    with open(args.csv) as f:
        f.readline()  # "# scenario=haptic-curves"
        rows = list(csv.DictReader(f))
    names = [k for k in rows[0].keys() if k != "theta"]
    theta_deg = [math.degrees(float(r["theta"])) for r in rows]

    n = len(names)
    cols = 3
    rws = (n + cols - 1) // cols
    fig, axes = plt.subplots(rws, cols, figsize=(4.2 * cols, 3.2 * rws), sharex=True)
    axes = axes.flatten()
    for ax, name in zip(axes, names):
        ax.plot(theta_deg, [float(r[name]) for r in rows])
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title(name)
        ax.set_ylabel("Nm")
        ax.grid(alpha=0.3)
    for ax in axes[-cols:]:
        ax.set_xlabel("angle (deg)")
    for ax in axes[n:]:
        ax.set_visible(False)
    fig.suptitle("Haptic presets: torque vs angle (omega = 0)")
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(args.out)


if __name__ == "__main__":
    main()
