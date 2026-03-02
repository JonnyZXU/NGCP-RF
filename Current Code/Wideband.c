#!/usr/bin/env bash
# scan_log_run.sh — one-shot or continuous scan to find strongest FRS/GMRS channel

ONCE=0
if [[ "${1:-}" == "--once" ]]; then
  ONCE=1
fi
set -u

# Config via environment variables (systemd Environment=...)
GAIN="${GAIN:-15}"
SCAN_SEC="${SCAN_SEC:-2}"     # may be float (e.g., 0.5)
LOG="${LOG:-$HOME/scan_log.csv}"
ERRLOG="${ERRLOG:-$HOME/scan_log_err.log}"
TMP="${TMP:-/tmp/rtl_scan.csv}"

# Create CSV header if needed
if [ ! -f "$LOG" ]; then
  echo "timestamp,best_ch,best_freq_mhz,avg_db,peak_db,gain,scan_sec" > "$LOG"
fi

# Float-safe timeout for GNU timeout (seconds)
TIMEOUT_SEC="$(
  /usr/bin/python3 - <<PY
import os
scan = float(os.environ.get("SCAN_SEC","2"))
print(f"{scan+2.0:.3f}")
PY
)"

while true; do
  TS="$(date '+%Y-%m-%d %H:%M:%S')"

  # 1) Capture one snapshot across 462–468 MHz
  : > "$TMP"
  /usr/bin/timeout "${TIMEOUT_SEC}" /usr/bin/rtl_power \
    -f 462M:468M:2500 \
    -i "$SCAN_SEC" \
    -g "$GAIN" \
    "$TMP" 2>>"$ERRLOG" || true

  # If rtl_power didn't write anything, retry
  if [ ! -s "$TMP" ]; then
    echo "$TS rtl_power produced empty output (retry)" >> "$ERRLOG"
    sleep 0.2
    continue
  fi

  # 2) Parse snapshot and pick strongest channel
  BEST="$(
    /usr/bin/python3 - <<'PY'
import csv, math, sys

CH2MHZ = {
  1:462.5625,  2:462.5875,  3:462.6125,  4:462.6375,  5:462.6625,  6:462.6875,  7:462.7125,
  8:467.5625,  9:467.5875, 10:467.6125, 11:467.6375, 12:467.6625, 13:467.6875, 14:467.7125,
  15:462.5500, 16:462.5750, 17:462.6000, 18:462.6250, 19:462.6500, 20:462.6750, 21:462.7000, 22:462.7250
}
HALF_BW = 6250.0  # +/- 6.25 kHz (~12.5 kHz channel window)
PATH = "/tmp/rtl_scan.csv"

rows = []
with open(PATH, newline="") as f:
    for r in csv.reader(f):
        if not r or r[0].startswith("#"):
            continue
        rows.append(r)

if not rows:
    sys.exit(3)

def parse_row(r):
    # rtl_power row format: date,time,fstart,fstop,step,samples, v1,v2,...
    fstart = float(r[2])
    fstop  = float(r[3])
    step   = float(r[4])
    vals   = list(map(float, r[6:]))
    return fstart, fstop, step, vals

parsed = [parse_row(r) for r in rows]

def avg_peak_near(center_hz):
    for fstart, fstop, step, vals in parsed:
        if fstart <= center_hz <= fstop:
            i0 = int(max(0, math.floor((center_hz - HALF_BW - fstart)/step)))
            i1 = int(min(len(vals)-1, math.ceil((center_hz + HALF_BW - fstart)/step)))
            if i1 < i0:
                return (-1e9, -1e9)
            seg = vals[i0:i1+1]
            if not seg:
                return (-1e9, -1e9)
            return (sum(seg)/len(seg), max(seg))
    return (-1e9, -1e9)

best = None
for ch, mhz in CH2MHZ.items():
    avg, pk = avg_peak_near(mhz * 1e6)
    if best is None or avg > best[2]:
        best = (ch, mhz, avg, pk)

print(f"{best[0]} {best[1]:.4f} {best[2]:.2f} {best[3]:.2f}")
PY
  )" || {
    echo "$TS parse failed (retry)" >> "$ERRLOG"
    sleep 0.2
    continue
  }

  CH=$(echo "$BEST"  | awk '{print $1}')
  MHZ=$(echo "$BEST" | awk '{print $2}')
  AVG=$(echo "$BEST" | awk '{print $3}')
  PEAK=$(echo "$BEST"| awk '{print $4}')

  # 3) Log result
  echo "$TS,$CH,$MHZ,$AVG,$PEAK,$GAIN,$SCAN_SEC" >> "$LOG"
  echo "$TS  best_ch=$CH  freq=${MHZ}MHz  avg=${AVG}dB  peak=${PEAK}dB"

  if [[ "$ONCE" -eq 1 ]]; then
    break
  fi
done