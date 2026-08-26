# BER measurements

Scripts and raw data of the BER experiment (Experiment 1 in the main
README).

## Contents

| File | Description |
|---|---|
| `analyze_all_cells.py` | Analysis script: reads the five session logs of every combination, computes the per-combination statistics (mean SNR and between-session standard deviation, epochs, slips, payload bits, in-lock and cumulative errors, conditional BER, goodput) and writes `summary_cells.csv` |
| `plot_campaign_summary.py` | Figure script: reads `summary_cells.csv` and draws the four SNR-versus-distance summary figures, one per coupling x modulation, with filled markers for error-free combinations and open markers for combinations with slips or bit errors |
| `data/*.txt` | The 180 receiver logs: five 25-minute sessions for each of the 36 combinations of coupling (GC/CC), modulation (BFSK/OOK), bit rate (20/40/100 bit/s), and distance (10/20/30 cm) |

File naming: `BER_<coupling>_<modulation>_<rate>bps_<distance>cm_session<n>.txt`,
e.g. `BER_GC_BFSK_20bps_10cm_session1.txt`.

Each log is the raw receiver UART capture of one session: a `CSV` status
line every 5 s, `BER: SYNC` / `BER: END_SEQ` lines per PRBS-9 sequence, and
the `FINAL:` block written at auto-stop (line formats in the main README,
Experiment 1).

## Usage

```
python analyze_all_cells.py
python plot_campaign_summary.py
```

The first command recomputes the per-combination table from all 180 logs
and writes `summary_cells.csv` next to the script: the SNR, slip, payload,
conditional-BER, and goodput values are the ones in the BER tables of the
paper. The second command reads that CSV and writes the four summary
figures to `summary/`. `analyze_all_cells.py` needs only the Python
standard library; `plot_campaign_summary.py` needs `matplotlib`.
