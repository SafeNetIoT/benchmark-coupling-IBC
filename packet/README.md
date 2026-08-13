# Packet-delivery measurements

Scripts and raw data of the packet-delivery experiment (Experiment 2 in the
main README).

## Contents

| File | Description |
|---|---|
| `analyze_packet.py` | Analysis script: reads the final cumulative counters of each log and builds the delivery table |
| `data/*.txt` | The 36 receiver logs, one 100-frame run per combination of coupling (GC/CC), modulation (BFSK/OOK), bit rate (20/40/100 bit/s), and distance (10/20/30 cm) |

File naming: `PKT_<coupling>_<modulation>_<rate>bps_<distance>cm.txt`,
e.g. `PKT_GC_BFSK_20bps_10cm.txt`.

Each log is the raw receiver UART capture of one run: one `PKT` line per
delivered frame, `CRC FAIL` + `PKTFAIL` lines for corrupted frames,
`SEQ GAP` lines for never-acquired frames, and an `RX:` status line every
5 s (line formats in the main README, Experiment 2).

## Usage

```
python analyze_packet.py
```

Recomputes the delivery table from all 36 logs and writes
`packet_delivery.csv` next to the script. The delivered counts are the
ones in the packet-delivery table of the paper. No dependencies beyond
the Python standard library.
