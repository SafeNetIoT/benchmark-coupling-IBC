# Power-consumption measurements

Scripts and raw data of the power experiment (Experiment 3 in the main
README).

## Contents

| File | Description |
|---|---|
| `rigol_log.py` | Acquisition script: drives a Rigol DM3058 over USB (SCPI via pyvisa), logs DC current samples to CSV |
| `final_table.py` | Analysis script: computes the results table (mean current, standard deviation, power, energy per bit) from the ten acquisitions |
| `data/*.csv` | The ten 100 s acquisitions (200 samples at 0.5 s), one per measured state |

Each CSV holds one state: idle plus the four modulation x coupling
combinations (BFSK/OOK x GC/CC) for the transmitter (`TX_*`) and the
receiver (`RX_*`). All active states were measured in packet mode at
40 bit/s with the complete link running on the phantom.

## Usage

Acquisition (200 samples at 0.5 s, as used for all the data here):

```
python rigol_log.py data/TX_BFSK_GC_40bps.csv -n 200 -i 0.5
```

Analysis (recomputes the full results table from `data/`):

```
python final_table.py
```

`rigol_log.py` requires `pyvisa` and a VISA library; `final_table.py` has
no dependencies beyond the Python standard library.
