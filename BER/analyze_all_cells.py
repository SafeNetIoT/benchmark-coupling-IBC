"""
BER campaign analysis: every cell (coupling x modulation x rate x distance),
exact statistics from the raw receiver logs.

For each cell, it reads the five session logs, takes the session SNR mean
from CSV column 13, and the decode totals from the FINAL lines. Prints a
table and writes summary_cells.csv next to this script.

Expected file naming (searched recursively below this script's folder):
    BER_<COUPLING>_<MOD>_<rate>bps_<dist>cm_session<n>.txt
    e.g. BER_GC_BFSK_20bps_10cm_session1.txt

Log line formats (RX firmware, BER TEST MODE):
    CSV,<t_ms>,<wins>,<syncs>,<lock>,<slip>,<bits>,<err>,<BER_cum>,<BER_cond>,
        <p10>,<p20>,<p_noise>,<SNR_real_dB>,<SNR_tone_dB>
    FINAL: syncs=.. lock=.. slip=.. slip_rate=..
    FINAL: bits_cum=.. err_cum=.. BER_cum=..
    FINAL: bits_lock=.. err_lock=.. BER_cond=..

Usage:
  python analyze_all_cells.py
"""

import csv
import re
import statistics
from pathlib import Path

BASE = Path(__file__).parent
OUT = BASE / "summary_cells.csv"

CAMPAIGNS = [("galvanic", "bfsk"), ("capacitive", "bfsk"),
             ("galvanic", "ook"), ("capacitive", "ook")]
COUP_TAG = {"galvanic": "GC", "capacitive": "CC"}
RATES = (20, 40, 100)
DISTS = (10, 20, 30)

NAME_RE = re.compile(
    r"BER_(?P<coup>GC|CC)_(?P<mod>BFSK|OOK)_(?P<rate>\d+)bps_(?P<dist>\d+)cm_session(?P<s>\d+)",
    re.IGNORECASE,
)


def scan_session(path):
    snr = []
    fin = {}
    for line in path.read_text(errors="ignore").splitlines():
        if line.startswith("CSV,"):
            c = line.split(",")
            if len(c) < 15:
                continue   # truncated line (serial glitch) - ignore
            try:
                snr.append(float(c[13]))
            except ValueError:
                continue
        elif line.startswith("FINAL:"):
            for k, v in re.findall(r"(\w+)=([0-9.eE+-]+)", line):
                fin[k] = float(v)
    return snr, fin


# group the session logs by cell from their file names
cells = {}
for f in sorted(BASE.rglob("BER_*.txt")):
    m = NAME_RE.search(f.name)
    if m is None:
        print(f"  ! {f.name}: name does not match convention, skipped")
        continue
    key = (m.group("coup").upper(), m.group("mod").upper(),
           int(m.group("rate")), int(m.group("dist")))
    cells.setdefault(key, []).append(f)

rows = []
for coupling, mod in CAMPAIGNS:
    for rate in RATES:
        for dist in DISTS:
            files = cells.get((COUP_TAG[coupling], mod.upper(), rate, dist), [])
            means, tot = [], dict(syncs=0, lock=0, slip=0, bits=0, err=0,
                                  bits_lock=0, err_lock=0)
            for f in files:
                snr, fin = scan_session(f)
                if not snr or "syncs" not in fin:
                    print(f"  WARNING: {f} incomplete (rows={len(snr)}, "
                          f"final={'yes' if fin else 'no'}) - skipped")
                    continue
                means.append(sum(snr) / len(snr))
                tot["syncs"] += int(fin["syncs"])
                tot["lock"] += int(fin["lock"])
                tot["slip"] += int(fin["slip"])
                tot["bits"] += int(fin["bits_cum"])
                tot["err"] += int(fin["err_cum"])
                tot["bits_lock"] += int(fin.get("bits_lock", 0))
                tot["err_lock"] += int(fin.get("err_lock", 0))
            if not means:
                continue
            n = len(means)
            mean = sum(means) / n
            sd = statistics.stdev(means) if n > 1 else 0.0
            slip_rate = tot["slip"] / tot["syncs"] if tot["syncs"] else 0.0
            ber_cond = (tot["err_lock"] / tot["bits_lock"]
                        if tot["bits_lock"] else float("nan"))
            ber_cum = tot["err"] / tot["bits"] if tot["bits"] else float("nan")
            goodput = rate * (511 / 588) * (1 - slip_rate) * \
                (0.0 if ber_cond != ber_cond else (1 - ber_cond))
            rows.append(dict(coupling=coupling, mod=mod, rate=rate, dist=dist,
                             n=n, snr_mean=round(mean, 2), snr_sd=round(sd, 2),
                             session_means=";".join(f"{v:.2f}" for v in means),
                             epochs=tot["syncs"], lock=tot["lock"],
                             slips=tot["slip"], bits=tot["bits"],
                             errors=tot["err"],
                             bits_lock=tot["bits_lock"],
                             err_lock=tot["err_lock"],
                             slip_rate=round(slip_rate, 4),
                             ber_cond=("" if ber_cond != ber_cond
                                       else f"{ber_cond:.3e}"),
                             ber_cum=f"{ber_cum:.3e}",
                             goodput=round(goodput, 1)))

with OUT.open("w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

print(f"{'campaign':<18}{'rate':>5}{'dist':>5}{'n':>3}{'SNR':>8}{'sd':>6}"
      f"{'epochs':>7}{'slips':>6}{'bits':>9}{'err':>7}{'BERcond':>11}{'Rgood':>7}")
for r in rows:
    print(f"{r['coupling'][:4]+'-'+r['mod'].upper():<18}{r['rate']:>5}"
          f"{r['dist']:>5}{r['n']:>3}{r['snr_mean']:>8.2f}{r['snr_sd']:>6.2f}"
          f"{r['epochs']:>7}{r['slips']:>6}{r['bits']:>9}{r['errors']:>7}"
          f"{r['ber_cond'] or 'undef':>11}{r['goodput']:>7.1f}")
print(f"\n{len(rows)} cells -> {OUT}")
