#!/usr/bin/env bash
set -euo pipefail

GAIN="${GAIN:-20}"
SCAN_SEC="${SCAN_SEC:-1.5}"

TMP="${TMP:-/tmp/rtl_lock_scan.csv}"
LOCK_FILE="${LOCK_FILE:-/tmp/chnl_lock.txt}"
ERRLOG="${ERRLOG:-$HOME/rf_pcb/lock_scan_err.log}"

# Saved PPM (persistent)
PPM_FILE="${PPM_FILE:-$HOME/rf_pcb/ppm_val.txt}"
PPM="$(cat "$PPM_FILE" 2>/dev/null || echo 0)"

# SNR settings
HALF_BW_HZ="${HALF_BW_HZ:-6250}"        # +/-6.25k signal window
NOISE_GUARD_MULT="${NOISE_GUARD_MULT:-2.0}"  # exclude +/- (guard_mult*half_bw) from noise
LOCK_LOCAL_SPAN_HZ="${LOCK_LOCAL_SPAN_HZ:-80000}"  # local span used for noise around each channel
SNR_MIN="${SNR_MIN:-1.0}"              # warn if lock SNR is below this

: > "$ERRLOG"
: > "$TMP"

# Detect one-shot support (-1)
ONE_SHOT=0
if rtl_power -h 2>&1 | grep -qE '(^|[[:space:]])-1([[:space:]]|$)'; then
  ONE_SHOT=1
fi

if [[ "$ONE_SHOT" == "1" ]]; then
  rtl_power -1 \
    -f 462M:468M:2500 \
    -i "$SCAN_SEC" \
    -g "$GAIN" \
    -p "$PPM" \
    "$TMP" 2>>"$ERRLOG" || { echo "rtl_power failed. See $ERRLOG"; exit 1; }
else
  rc=0
  timeout 6 rtl_power \
    -f 462M:468M:2500 \
    -i "$SCAN_SEC" \
    -g "$GAIN" \
    -p "$PPM" \
    "$TMP" 2>>"$ERRLOG" || rc=$?
  if [[ $rc -ne 0 && $rc -ne 124 ]]; then
    echo "rtl_power failed (rc=$rc). See $ERRLOG"
    exit 1
  fi
fi

if ! grep -qv '^#' "$TMP" 2>/dev/null; then
  echo "No rtl_power data captured. See $ERRLOG"
  exit 1
fi

python3 - <<'PY'
import csv, os, time, math, statistics

tmp = os.environ.get("TMP", "/tmp/rtl_lock_scan.csv")
lock_file = os.environ.get("LOCK_FILE", "/tmp/chnl_lock.txt")

half_bw = float(os.environ.get("HALF_BW_HZ", "6250"))
guard_mult = float(os.environ.get("NOISE_GUARD_MULT", "2.0"))
local_span = float(os.environ.get("LOCK_LOCAL_SPAN_HZ", "80000"))
snr_min = float(os.environ.get("SNR_MIN", "1.0"))

CH2MHZ = {
  1: 462.5625, 2: 462.5875, 3: 462.6125, 4: 462.6375, 5: 462.6625, 6: 462.6875, 7: 462.7125,
  8: 467.5625, 9: 467.5875, 10: 467.6125, 11: 467.6375, 12: 467.6625, 13: 467.6875, 14: 467.7125,
  15: 462.5500, 16: 462.5750, 17: 462.6000, 18: 462.6250, 19: 462.6500, 20: 462.6750, 21: 462.7000, 22: 462.7250,
}

# Flatten bins: (freq_hz, db)
bins=[]
with open(tmp, newline="") as f:
  r = csv.reader(f)
  for row in r:
    if not row or row[0].startswith("#"):
      continue
    start_hz = float(row[2])
    step_hz  = float(row[4])
    vals = [float(x) for x in row[6:]]
    for i, db in enumerate(vals):
      bins.append((start_hz + i*step_hz, db))

guard = half_bw * guard_mult
half_local = local_span / 2.0

def chan_stats(center_mhz: float):
  c = center_mhz * 1e6
  sig_lo, sig_hi = c-half_bw, c+half_bw
  loc_lo, loc_hi = c-half_local, c+half_local

  sig = [db for (f,db) in bins if sig_lo <= f <= sig_hi]
  if not sig:
    return None

  peak = max(sig)
  avg  = sum(sig)/len(sig)

  noise = [db for (f,db) in bins if (loc_lo <= f <= loc_hi) and (abs(f-c) >= guard)]
  noise_db = statistics.median(noise) if noise else float("nan")
  snr = (peak - noise_db) if not math.isnan(noise_db) else float("nan")
  return avg, peak, noise_db, snr

best = None  # (snr, peak, ch, mhz, avg, noise)
for ch, mhz in CH2MHZ.items():
  st = chan_stats(mhz)
  if not st:
    continue
  avg, peak, noise_db, snr = st
  if math.isnan(snr):
    continue
  cand = (snr, peak, ch, mhz, avg, noise_db)
  if best is None or cand[0] > best[0] or (cand[0] == best[0] and cand[1] > best[1]):
    best = cand

if best is None:
  raise SystemExit("No valid channel SNR computed")

snr, peak, ch, mhz, avg, noise_db = best
epoch = time.time()

with open(lock_file, "w") as out:
  out.write(f"{epoch:.3f} {ch} {mhz:.6f} {avg:.2f} {peak:.2f} {noise_db:.2f} {snr:.2f}\n")

msg = f"LOCKED ch={ch} mhz={mhz:.6f} avg_db={avg:.2f} peak_db={peak:.2f} noise_db={noise_db:.2f} snr_db={snr:.2f}"
if snr < snr_min:
  msg += "  (WARNING: weak lock)"
print(msg)
PY
