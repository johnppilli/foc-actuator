#!/usr/bin/env python3
"""Plot simulator scenario CSVs produced by ./build/focsim.

    python3 tools/plot_sim.py build/step.csv [more.csv ...] --outdir build
    python3 tools/plot_sim.py build/step.csv --show

Each CSV starts with "# scenario=<name>"; the scenario picks the panel layout.
"""
import argparse
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path):
    with open(path) as f:
        first = f.readline().strip()
        scenario = first.split("=", 1)[1] if first.startswith("# scenario=") else "unknown"
        rows = list(csv.DictReader(f))
    cols = {k: [float(r[k]) for r in rows] for k in rows[0].keys()} if rows else {}
    return scenario, cols


def ms(t):
    return [x * 1e3 for x in t]


def plot_step(cols, title):
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    t = ms(cols["t"])
    ax[0].plot(t, cols["iq_ref"], "k--", label="iq ref")
    ax[0].plot(t, cols["iq"], label="iq")
    ax[0].plot(t, cols["id"], label="id")
    ax[0].set_ylabel("A")
    ax[0].legend(loc="lower right")
    ax[1].plot(t, cols["vq"], label="vq")
    ax[1].plot(t, cols["vd"], label="vd")
    ax[1].set_ylabel("V")
    ax[1].legend(loc="lower right")
    ax[2].plot(t, cols["omega"], label="omega (rad/s)")
    ax[2].set_ylabel("rad/s")
    ax[2].set_xlabel("ms")
    ax[2].legend(loc="lower right")
    fig.suptitle(title)
    return fig


def plot_load(cols, title):
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    t = ms(cols["t"])
    ax[0].plot(t, cols["iq_ref"], "k--", label="iq ref")
    ax[0].plot(t, cols["iq"], label="iq")
    ax[0].set_ylabel("A")
    ax[0].legend()
    ax[1].plot(t, cols["tau_e"], label="motor torque")
    ax[1].plot(t, cols["tau_load"], "r--", label="load torque")
    ax[1].set_ylabel("Nm")
    ax[1].legend()
    ax[2].plot(t, cols["omega"], label="omega")
    ax[2].set_ylabel("rad/s")
    ax[2].set_xlabel("ms")
    ax[2].legend()
    fig.suptitle(title)
    return fig


def plot_calib(cols, title):
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    t = cols["t"]
    ax[0].plot(t, cols["theta_e_cmd"], label="theta_e cmd")
    ax[0].plot(t, cols["theta_m_enc"], label="encoder theta_m")
    ax[0].set_ylabel("rad")
    ax[0].legend()
    ax[1].plot(t, cols["v_d"], label="v_d applied")
    ax[1].set_ylabel("V")
    ax[1].legend()
    ax[2].plot(t, cols["mech_accum"], label="accumulated mech angle")
    ax[2].plot(t, cols["state"], "k:", label="state")
    ax[2].set_xlabel("s")
    ax[2].legend()
    fig.suptitle(title)
    return fig


def plot_haptic(cols, title):
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    t = cols["t"]
    ax[0].plot(t, cols["theta_finger"], "k--", label="finger")
    ax[0].plot(t, cols["theta"], label="knob")
    ax[0].set_ylabel("rad")
    ax[0].legend()
    ax[1].plot(t, cols["tau_cmd"], label="commanded torque")
    ax[1].plot(t, cols["tau_ext"], label="finger torque")
    ax[1].set_ylabel("Nm")
    ax[1].legend()
    ax[2].plot(t, cols["iq_ref"], "k--", label="iq ref")
    ax[2].plot(t, cols["iq"], label="iq")
    ax[2].set_ylabel("A")
    ax[2].set_xlabel("s")
    ax[2].legend()
    fig.suptitle(title)
    return fig


def plot_openloop(cols, title):
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    t = cols["t"]
    ax[0].plot(t, cols["theta_e_cmd"], label="commanded theta_e")
    ax[0].plot(t, cols["theta_e_true"], label="true theta_e")
    ax[0].set_ylabel("rad")
    ax[0].legend()
    ax[1].plot(t, cols["id"], label="id (wasted)")
    ax[1].plot(t, cols["iq"], label="iq (torque)")
    ax[1].set_ylabel("A")
    ax[1].legend()
    ax[2].plot(t, cols["slip"], label="slip (cmd - true)")
    ax[2].set_ylabel("rad")
    ax[2].set_xlabel("s")
    ax[2].legend()
    fig.suptitle(title)
    return fig


def plot_generic(cols, title):
    fig, ax = plt.subplots(figsize=(8, 5))
    t = cols.get("t") or cols.get("theta")
    for k, v in cols.items():
        if k in ("t", "theta"):
            continue
        ax.plot(t, v, label=k)
    ax.legend()
    fig.suptitle(title)
    return fig


PLOTTERS = {
    "step": plot_step,
    "step-free": plot_step,
    "load": plot_load,
    "calib": plot_calib,
    "haptic-spring": plot_haptic,
    "haptic-detents": plot_haptic,
    "haptic-endstops": plot_haptic,
    "openloop": plot_openloop,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--outdir", default=None, help="write <name>.png here")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if args.show:
        matplotlib.use("MacOSX" if sys.platform == "darwin" else "TkAgg", force=True)
    for path in args.csv:
        scenario, cols = read_csv(path)
        if not cols:
            print(f"{path}: empty", file=sys.stderr)
            continue
        fig = PLOTTERS.get(scenario, plot_generic)(cols, scenario)
        if args.outdir:
            out = os.path.join(args.outdir, os.path.splitext(os.path.basename(path))[0] + ".png")
            fig.savefig(out, dpi=110)
            print(out)
        if args.show:
            plt.show()
        plt.close(fig)


if __name__ == "__main__":
    main()
