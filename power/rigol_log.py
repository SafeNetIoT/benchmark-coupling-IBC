#!/usr/bin/env python3
import sys
import time
import csv
import argparse

try:
    import pyvisa
except ImportError:
    print("ERROR: pyvisa is not installed.")
    print("Install it with:  pip install pyvisa")
    sys.exit(1)


V_DD_DEFAULT = 3.3   # assumed supply voltage, V


def main():
    ap = argparse.ArgumentParser(description="Log Rigol DM3058 DC current to CSV")
    ap.add_argument("outfile", help="Output CSV file name")
    ap.add_argument("-n", "--samples", type=int, default=100,
                    help="Number of samples (default: 100)")
    ap.add_argument("-i", "--interval", type=float, default=1.0,
                    help="Interval between samples in seconds (default: 1.0)")
    ap.add_argument("-r", "--range", type=float, default=0.2,
                    help="Current range in A (default: 0.2 = 200 mA)")
    ap.add_argument("-v", "--vdd", type=float, default=V_DD_DEFAULT,
                    help=f"V_DD for the power calculation (default: {V_DD_DEFAULT} V)")
    args = ap.parse_args()

    # --- Connection ---
    print("Searching for VISA instruments...")
    rm = pyvisa.ResourceManager()
    resources = rm.list_resources()
    print(f"  found: {resources}")

    if not resources:
        print("ERROR: no VISA instrument found.")
        print("  Check that the USB cable is connected and that Ultra Sigma sees the meter.")
        sys.exit(1)

    # Look for a Rigol (vendor ID 0x1AB1) among the USB resources
    rigol_devs = [r for r in resources if "USB" in r and "0x1AB1" in r]
    target = rigol_devs[0] if rigol_devs else resources[0]

    print(f"  opening: {target}")
    rigol = rm.open_resource(target)
    rigol.timeout = 5000

    idn = rigol.query("*IDN?").strip()
    print(f"  connected: {idn}")

    # --- Configure DC current mode ---
    rigol.write(":FUNC:CURR:DC")
    rigol.write(f":CURR:DC:RANG {args.range}")
    time.sleep(0.3)

    # --- Acquisition ---
    print()
    print(f"Acquisition: {args.samples} samples @ {args.interval}s interval")
    print(f"Output:      {args.outfile}")
    print(f"Est. length: {args.samples * args.interval:.0f} seconds")
    print()

    samples = []
    t0 = time.time()

    with open(args.outfile, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sample", "t_s", "I_mA"])

        for i in range(args.samples):
            try:
                val = float(rigol.query(":MEAS:CURR:DC?"))
            except Exception as e:
                print(f"  WARN [{i+1}]: {e}, retry...")
                time.sleep(0.5)
                try:
                    val = float(rigol.query(":MEAS:CURR:DC?"))
                except Exception as e2:
                    print(f"  ERR  [{i+1}]: skip ({e2})")
                    continue

            t = time.time() - t0
            i_mA = val * 1000.0
            samples.append(i_mA)
            w.writerow([i + 1, f"{t:.3f}", f"{i_mA:.6f}"])
            print(f"  {i+1:3d}/{args.samples}  t={t:6.2f}s  I={i_mA:9.4f} mA")

            # wait until the next tick
            next_t = (i + 1) * args.interval
            sleep_t = next_t - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    rigol.close()

    # --- Stats ---
    if not samples:
        print("\nERROR: no valid sample acquired.")
        sys.exit(1)

    n = len(samples)
    avg = sum(samples) / n
    mn = min(samples)
    mx = max(samples)
    var = sum((s - avg) ** 2 for s in samples) / n
    std = var ** 0.5
    power = args.vdd * avg

    print()
    print(f"=========== RESULT ============")
    print(f"File:   {args.outfile}")
    print(f"N      = {n}")
    print(f"Avg    = {avg:9.4f} mA")
    print(f"Std    = {std:9.4f} mA   ({std/avg*100:.2f}% of Avg)")
    print(f"Max    = {mx:9.4f} mA")
    print(f"Min    = {mn:9.4f} mA")
    print(f"Spread = {mx-mn:9.4f} mA")
    print(f"Power  = {power:9.3f} mW   (V_DD={args.vdd} V)")
    print(f"================================")


if __name__ == "__main__":
    main()
