import csv
import re
import sys
from pathlib import Path

BASE = Path(__file__).parent
OUT = Path(__file__).parent / "packet_delivery.csv"

NAME_RE = re.compile(
    r"PKT_(?P<coup>GC|CC)_(?P<mod>BFSK|OOK)_(?P<rate>\d+)bps_(?P<dist>\d+)cm",
    re.IGNORECASE,
)
PKT_RE = re.compile(
    r"^PKT,(?P<t>\d+),(?P<seq>\d+),(?P<mgdl>\d+),(?P<ok>\d+),(?P<fail>\d+),(?P<gaps>\d+)"
)

FRAMES_SENT = 100  # PKT_TEST_COUNT in the TX firmware


def parse_log(path: Path):
    """Return the final cumulative counters of one packet log."""
    last = None
    payload_values = set()
    for line in path.read_text(errors="ignore").splitlines():
        m = PKT_RE.match(line.strip())
        if m:
            last = m
            payload_values.add(int(m.group("mgdl")))
    if last is None:
        return None
    return {
        "ok": int(last.group("ok")),
        "fail": int(last.group("fail")),
        "gaps": int(last.group("gaps")),
        "last_seq": int(last.group("seq")),
        "payloads": sorted(payload_values),
    }


def main():
    if not BASE.is_dir():
        sys.exit(f"missing folder: {BASE}")

    rows = []
    seen = {}
    for f in sorted(BASE.rglob("PKT_*.txt")):
        name = NAME_RE.search(f.name)
        stats = parse_log(f)
        if name is None:
            print(f"  ! {f.name}: name does not match convention, skipped")
            continue
        if stats is None:
            print(f"  ! {f.name}: no PKT lines, skipped")
            continue
        key = (name.group("coup").upper(), name.group("mod").upper(),
               int(name.group("rate")), int(name.group("dist")))
        if key in seen:
            print(f"  ! {f.name}: duplicate of {seen[key]}, skipped")
            continue
        seen[key] = f.name
        # every delivered frame must carry the expected payload
        payload_note = (
            "" if stats["payloads"] in ([], [120]) else f"UNEXPECTED PAYLOADS {stats['payloads']}"
        )
        rows.append({
            "coupling": name.group("coup").upper(),
            "modulation": name.group("mod").upper(),
            "rate_bps": int(name.group("rate")),
            "distance_cm": int(name.group("dist")),
            "sent": FRAMES_SENT,
            "delivered": stats["ok"],
            "crc_fail": stats["fail"],
            "gaps": stats["gaps"],
            "pdr": stats["ok"] / FRAMES_SENT,
            "note": payload_note,
            "file": f.name,
        })

    rows.sort(key=lambda r: (r["rate_bps"], r["coupling"], r["modulation"], r["distance_cm"]))

    with OUT.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else
                           ["coupling", "modulation", "rate_bps", "distance_cm",
                            "sent", "delivered", "crc_fail", "gaps", "pdr", "note", "file"])
        w.writeheader()
        w.writerows(rows)

    print(f"{'config':<12}{'rate':>6}{'dist':>6}{'delivered':>11}{'crc_fail':>10}{'gaps':>7}{'PDR':>8}")
    for r in rows:
        cfg = f"{r['coupling']}-{r['modulation']}"
        print(f"{cfg:<12}{r['rate_bps']:>6}{r['distance_cm']:>6}"
              f"{r['delivered']:>8}/{r['sent']}{r['crc_fail']:>10}{r['gaps']:>7}"
              f"{r['pdr']:>8.2f}"
              + (f"   {r['note']}" if r['note'] else ""))
    print(f"\nwritten: {OUT}")


if __name__ == "__main__":
    main()
