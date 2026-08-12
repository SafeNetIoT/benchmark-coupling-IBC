import csv, os

V_TX, V_RX = 3.317, 3.315
R_B = 40.0  # bit rate of the active states, bit/s

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')

files = [
 ('TX idle',    'TX', 'TX_IDLE.csv'),
 ('TX BFSK GC', 'TX', 'TX_BFSK_GC_40bps.csv'),
 ('TX BFSK CC', 'TX', 'TX_BFSK_CC_40bps.csv'),
 ('TX OOK GC',  'TX', 'TX_OOK_GC_40bps.csv'),
 ('TX OOK CC',  'TX', 'TX_OOK_CC_40bps.csv'),
 ('RX idle',    'RX', 'RX_IDLE.csv'),
 ('RX BFSK GC', 'RX', 'RX_BFSK_GC_40bps.csv'),
 ('RX BFSK CC', 'RX', 'RX_BFSK_CC_40bps.csv'),
 ('RX OOK GC',  'RX', 'RX_OOK_GC_40bps.csv'),
 ('RX OOK CC',  'RX', 'RX_OOK_CC_40bps.csv'),
]

print(f"{'Cell':12s} {'Iavg':>8s} {'sigma':>7s} {'P_mW':>7s} {'Eb_mJ':>7s}")
for label, board, name in files:
    vdd = V_TX if board == 'TX' else V_RX
    vals = []
    with open(os.path.join(DATA, name), 'r', newline='') as f:
        for r in csv.DictReader(f):
            vals.append(float(r['I_mA']))
    n = len(vals)
    avg = sum(vals) / n
    std = (sum((x - avg) ** 2 for x in vals) / n) ** 0.5
    p = vdd * avg
    eb = p / R_B if 'idle' not in label else None
    print(f"{label:12s} {avg:8.3f} {std:7.3f} {p:7.2f} {eb if eb else 0:7.3f}  (N={n})")
