# benchmark-coupling-IBC

A benchmark of **galvanic coupling (GC)** versus **capacitive coupling (CC)**
intra-body communication, with **BFSK** and **OOK** modulation, implemented on
two STM32 Nucleo boards. The same transmitter and receiver chain is used for
every configuration, so coupling modes and modulation schemes are compared
under identical conditions: same hardware, same demodulation chain, same
protocol, same metrics.

The benchmark covers three experiments, run for both couplings and both
modulations. The first two cover three bit rates (20, 40, 100 bit/s) and
three electrode distances (10, 20, 30 cm); the power experiment is measured
at 40 bit/s:

1. **BER characterization** — slip rate, conditional and cumulative bit error
   rate, and SNR over long PRBS transmissions.
2. **Packet delivery** — delivery ratio of CRC-protected frames carrying a
   sensor reading.
3. **Power consumption** — supply current, power, and energy per bit of both
   nodes, idle and active.

This repository contains the complete firmware, the raw logs of every
measurement in the published benchmark, and the scripts that turn those logs
into the published tables and figures.

## What is in this repository

| Folder | Contents |
|---|---|
| `Transmitter/` | STM32CubeIDE project for the NUCLEO-L476RG: tone generation with DAC + DMA |
| `Receiver/` | STM32CubeIDE project for the NUCLEO-F411RE: acquisition and Goertzel demodulation with ADC + DMA |
| `BER/` | Experiment 1: 180 raw session logs, analysis and figure scripts |
| `packet/` | Experiment 2: 36 raw run logs and the delivery-table script |
| `power/` | Experiment 3: 10 raw current acquisitions, acquisition and analysis scripts |

All application logic lives in `Core/Src/main.c` of each firmware project;
the rest is STM32CubeMX-generated code and ST HAL drivers. Each data folder
has its own README with a file-by-file description; running its analysis
script reproduces the corresponding published results.

## Requirements

- One NUCLEO-L476RG (transmitter) and one NUCLEO-F411RE (receiver)
- Self-adhesive gel electrodes (four for GC, two for CC), a propagation
  medium (a synthetic-skin phantom in the original benchmark), and one jumper
  wire for the CC ground connection
- A USB power bank for the transmitter and a battery-powered laptop for the
  receiver (see *Electrical isolation* below)
- For the power experiment: a bench multimeter with a logged DC-current mode
  (a Rigol DM3058 driven over USB in the original benchmark)
- STM32CubeIDE and a serial terminal that can log to a file

## Quick start

Import both projects in STM32CubeIDE (*File > Import > Existing Projects into
Workspace*), set the compile-time switches at the top of each `main.c`, build,
and flash. **Transmitter and Receiver must be built with identical switch
values**: the receiver cannot detect a mismatch, it simply fails to decode.

| Switch | Values | Meaning |
|---|---|---|
| `BER_TEST_MODE` | `1` / `0` | `1` = BER stream (Experiment 1), `0` = packet mode (Experiment 2) |
| `MOD_MODE` | `0` / `1` | `0` = BFSK (10 kHz = bit 0, 20 kHz = bit 1), `1` = OOK (20 kHz carrier on = bit 1, off = bit 0) |
| `BPS_MODE` | `20` / `40` / `100` | Bit rate in bit/s |
| `POWER_IDLE_TEST` | `0` / `1` | `1` = idle power bench: peripherals up, signal chain never started (Experiment 3) |
| `TEST_DURATION_MIN` (RX only) | `25` | Minutes after the first sync before the BER run auto-stops (`0` = run forever) |

Serial consoles are on the ST-LINK virtual COM port: transmitter at
**115200 baud**, receiver at **921600 baud**, both 8N1.

To run an experiment: flash both boards, start logging the receiver port to a
file, reset the receiver and check its boot banner (it states mode,
modulation, and bit rate), then reset the transmitter last: it starts
transmitting on boot. If the transmitter warns `LSE FAILED`, power-cycle it
and discard the run.

## Wiring

- Transmitter output: DAC on pin **PA4** plus a board **GND** pin.
- Receiver input: ADC on pin **PC0** plus a board **GND** pin.

**GC** uses four electrodes and no wire between the boards: PA4 and TX GND to
one electrode pair, PC0 and RX GND to a second pair, all four on the medium.
The distance `d` (10, 20, or 30 cm) is the separation between the two pairs,
and the return current closes through the medium.

**CC** uses two electrodes: PA4 and PC0 each to one electrode on the medium
at distance `d`, and a jumper wire from TX GND to RX GND as the return path.

**Electrical isolation.** Power the transmitter from a USB power bank and run
the receiver from a laptop on battery. If both boards share a mains-connected
supply, the mains earth creates an uncontrolled return path that invalidates
the galvanic measurements.

**Electrode contact.** The electrode interface dominates the galvanic link.
Use fresh gel electrodes and watch the logged SNR: a slow drift of a few dB
across a session is normal.

## Experiment 1: BER characterization

Build with `BER_TEST_MODE 1`. The transmitter repeats a fixed 588-bit cycle:
a 64-bit alternating preamble, a 13-bit Barker marker, and one 511-bit period
of a PRBS-9. The receiver detects the Barker marker (gated by 8 preamble
bits to suppress false synchronizations), reseeds a local PRBS-9 replica, and
compares the next 511 bits with it.

Each 511-bit sequence is classified by its error rate: below 10% it counts
as **LOCK**, otherwise as a **SLIP** (synchronization failure). The reported
metrics are the slip rate, the conditional BER (errors in locked sequences
only), the cumulative BER (all errors over all bits), and the SNR (strongest
tone bin over a silent 5 kHz reference bin, from identical Goertzel filters).
OOK and BFSK SNR values are not directly comparable; compare modulations
through decoding outcomes. For error-free combinations, the 95% upper bound
on the BER is `3/n` over the `n` transmitted bits (rule of three).

A session stops automatically `TEST_DURATION_MIN` minutes (default 25) after
the first synchronization and prints `FINAL:` summary lines with all
counters. Every 5 s the receiver prints one machine-readable line:

```
CSV,<t_ms>,<win_cnt>,<syncs>,<locks>,<slips>,<bits_cum>,<errs_cum>,<BER_cum>,<BER_cond>,<p10>,<p20>,<p_noise>,<SNR_real_dB>,<SNR_tone_dB>
```

The published benchmark aggregates five sessions per combination. Logs and
scripts are in `BER/`; `python BER/analyze_all_cells.py` reproduces the
published BER tables and `python BER/plot_campaign_summary.py` the summary
figures.

## Experiment 2: Packet delivery

Build with `BER_TEST_MODE 0`. The transmitter sends **100 frames** and then
idles. Each frame carries one two-byte sensor reading:

```
16 x 0x55 preamble | sync 0xD3 0x91 | LEN | SEQ | payload | CRC-16/CCITT-FALSE
```

The receiver accepts a frame only if the CRC verifies, and losses appear in
the log through two distinct mechanisms: a frame that is never acquired
leaves a jump in the sequence number (`SEQ GAP a->b`), while a corrupted
frame is rejected with a `CRC FAIL` + `PKTFAIL` line and never delivered
with wrong content. The log carries one line per delivered and per rejected
frame:

```
PKT,<t_ms>,<seq>,<mgdl>,<ok>,<crc_fail>,<gaps>
PKTFAIL,<t_ms>,<seq>,<ok>,<crc_fail>
```

plus `RX:` status lines every 5 s with the same SNR fields as in BER mode.

The metric is the packet delivery ratio `PDR = delivered / 100`. Take the
delivered count from the last `ok=` counter; frames lost at the very end of
a run leave no `SEQ GAP`, so also check that the last delivered `SEQ` is 99.
A run takes about 37 min at 20 bit/s, 18.5 min at 40 bit/s, and 7.5 min at
100 bit/s. Logs and script are in `packet/`;
`python packet/analyze_packet.py` reproduces the published delivery table.

## Experiment 3: Power consumption

For each node, five states are measured: idle (`POWER_IDLE_TEST 1`, signal
chain never started, core in WFI) and the four active combinations of
BFSK/OOK and GC/CC, in packet mode at 40 bit/s with the complete link
running on the medium. The boards are measured one at a time with the other
node running normally, and during the active acquisitions the receiver log
must show CRC-valid frames: the measured current is that of a link actually
delivering data.

The current is measured at the **JP6 (IDD)** jumper, in series between the
3.3 V regulator and the MCU supply rail, so the reading excludes the ST-LINK
debugger, LEDs, and regulator. Measure the actual rail voltage on a 3.3 V
header pin **before** removing the jumper (an open JP6 floats), then insert
the ammeter, power-cycle after every flash, wait 30 s after boot, and log
the current (200 samples at 0.5 s in the original benchmark). Power and
energy per bit follow as `P = V_DD * I_avg` and `E_b = P / R_b`.

Scripts and acquisitions are in `power/`; `python power/final_table.py`
reproduces the published power table.

## The full benchmark

The complete matrix is 4 configurations (GC/CC x BFSK/OOK) x 3 bit rates x 3
distances: five 25-minute BER sessions and one 100-frame packet run per
combination, plus the ten power acquisitions. Between a galvanic and a
capacitive run only the wiring changes; the firmware is identical.

## License

This repository (firmware, data, scripts, and documentation) is released
under the **CC BY-NC-ND 4.0** license: you may download and share it for
non-commercial purposes with attribution, but you may not use it
commercially or distribute modified versions. See the `LICENSE` file.
The ST HAL and CMSIS code in `Drivers/` is provided by STMicroelectronics
under its own license terms (see the license files in those folders) and is
not covered by this license.
