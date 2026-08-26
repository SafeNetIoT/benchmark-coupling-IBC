"""
Campaign summary figures: one figure per campaign (coupling x modulation).
x = electrode separation, y = cell SNR (mean of the five session means),
one curve per bit rate, error bars = between-session standard deviation.

Marker coding: filled = error-free cell (no slips, no bit errors),
open = cell with slips or bit errors (threshold region).

Reads summary_cells.csv (produced by analyze_all_cells.py) from this
script's folder and writes summary/fig_summary_<coupling>_<mod>.{png,pdf}
next to it.

Usage:
  python plot_campaign_summary.py
"""

import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

COLOURS = {20: "#0072B2", 40: "#D55E00", 100: "#CC79A7"}
STYLES = {20: "-", 40: "--", 100: ":"}
MARKERS = {20: "o", 40: "s", 100: "^"}
CAMPAIGNS = [("galvanic", "bfsk"), ("capacitive", "bfsk"),
             ("galvanic", "ook"), ("capacitive", "ook")]

cells = []
BASE = Path(__file__).parent
with open(BASE / "summary_cells.csv", newline="") as fh:
    for row in csv.DictReader(fh):
        cells.append(row)

out = BASE / "summary"
out.mkdir(exist_ok=True)

for coupling, mod in CAMPAIGNS:
    sel = [c for c in cells if c["coupling"] == coupling and c["mod"] == mod]
    if not sel:
        continue
    plt.rcParams.update({"font.size": 8, "axes.linewidth": 0.6})
    fig, ax = plt.subplots(figsize=(3.5, 2.4), dpi=300)
    top = 0.0
    for rate in (20, 40, 100):
        pts = sorted((c for c in sel if int(c["rate"]) == rate),
                     key=lambda c: int(c["dist"]))
        if not pts:
            continue
        d = [int(c["dist"]) for c in pts]
        y = [float(c["snr_mean"]) for c in pts]
        e = [float(c["snr_sd"]) for c in pts]
        top = max(top, max(yy + ee for yy, ee in zip(y, e)))
        ax.errorbar(d, y, yerr=e, lw=1.1, color=COLOURS[rate],
                    linestyle=STYLES[rate], marker=MARKERS[rate],
                    ms=4, capsize=2, elinewidth=0.7,
                    markerfacecolor=COLOURS[rate], label=f"{rate} bps")
        # re-draw threshold-region cells (slips or errors) as open markers
        for c in pts:
            if int(c["slips"]) > 0 or int(c["errors"]) > 0:
                ax.plot(int(c["dist"]), float(c["snr_mean"]),
                        marker=MARKERS[rate], ms=4.5, lw=0,
                        markerfacecolor="white", markeredgecolor=COLOURS[rate],
                        markeredgewidth=1.0, zorder=5)
    ax.set_xlabel("Electrode separation (cm)")
    ax.set_ylabel("SNR (dB)")
    ax.set_xticks([10, 20, 30])
    ax.set_xlim(7, 33)
    ax.set_ylim(0, math.ceil(top) + 3)
    ax.grid(True, lw=0.3, alpha=0.5)
    ax.legend(ncol=3, fontsize=7, frameon=False,
              loc="lower center", bbox_to_anchor=(0.5, 1.005),
              columnspacing=1.2, handletextpad=0.5)
    fig.tight_layout()
    stem = out / f"fig_summary_{'gc' if coupling == 'galvanic' else 'cc'}_{mod}"
    for ext in ("png", "pdf"):
        fig.savefig(f"{stem}.{ext}", bbox_inches="tight")
    plt.close(fig)
    print(f"saved {stem}.png/.pdf  ({len(sel)} cells)")
