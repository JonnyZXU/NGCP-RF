#!/usr/bin/env bash
set -euo pipefail

# --- User knobs ---
GAIN="${GAIN:-0}"
SCAN_SEC="${SCAN_SEC:-1}"            # seconds
SRATE="${SRATE:-1024000}"            # Hz
NFFT="${NFFT:-4096}"                 # FFT size

SPAN_HZ="${SPAN_HZ:-120000}"         # analyze +/- SPAN_HZ/2 around center
HALF_BW_HZ="${HALF_BW_HZ:-6250}"     # signal half-bandwidth
NOISE_GUARD_MULT="${NOISE_GUARD_MULT:-6.0}"
DC_NOTCH_HZ="${DC_NOTCH_HZ:-800}"

LOCK_FILE="${LOCK_FILE:-/tmp/chnl_lock.txt}"
OUT="${OUT:-/tmp/rf_latest.txt}"
DIAG_OUT="${DIAG_OUT:-/tmp/rf_diag.txt}"   # NEW: extra metrics here
IQ_TMP="${IQ_TMP:-/tmp/rtl_iq.bin}"

# Saved PPM (persistent)
PPM_FILE="${PPM_FILE:-$HOME/rf_pcb/ppm_val.txt}"
PPM="$(cat "$PPM_FILE" 2>/dev/null || echo 0)"

# --- Read locked center frequency ---
if [[ ! -s "$LOCK_FILE" ]]; then
  echo "Lock file missing: $LOCK_FILE (run lock_scan.sh first)" >&2
  exit 1
fi

read -r _epoch CH CENTER_MHZ _rest < "$LOCK_FILE"
CENTER_HZ=$(python3 - <<PY
print(int(float("$CENTER_MHZ")*1e6))
PY
)

# --- Capture IQ for SCAN_SEC seconds ---
NSAMP=$(python3 - <<PY
import math
print(int(max(1, round(float("$SRATE") * float("$SCAN_SEC")))))
PY
)

rtl_sdr -f "$CENTER_HZ" -s "$SRATE" -g "$GAIN" -p "$PPM" -n "$NSAMP" "$IQ_TMP" >/dev/null 2>&1

# --- Compute metrics ---
python3 - <<PY
import numpy as np, time, math, sys

iq_path = "$IQ_TMP"
out_path = "$OUT"
diag_path = "$DIAG_OUT"

center_hz = float("$CENTER_HZ")
srate = float("$SRATE")
nfft = int("$NFFT")

span_hz = float("$SPAN_HZ")
half_bw = float("$HALF_BW_HZ")
guard_mult = float("$NOISE_GUARD_MULT")
dc_notch = float("$DC_NOTCH_HZ")

guard = half_bw * guard_mult

raw = np.fromfile(iq_path, dtype=np.uint8)
ts = time.time()

if raw.size < 2*nfft:
  with open(out_path, "w") as w:
    w.write(f"{ts:.3f} nan nan nan nan\n")
  with open(diag_path, "w") as w:
    w.write(f"{ts:.3f} nan nan nan nan nan nan nan\n")
  print("TRACK (IQ) insufficient_samples")
  sys.exit(0)

raw = raw[: (raw.size // 2) * 2]
iq = raw.astype(np.float32).reshape(-1, 2)
Iu = iq[:,0]
Qu = iq[:,1]
I = Iu - 127.5
Q = Qu - 127.5
x = I + 1j*Q

# --- Clipping / headroom diagnostics ---
mag = np.sqrt(I*I + Q*Q)
mag_rms = float(np.sqrt(np.mean(mag**2)))
mag_max = float(np.max(mag))
pct_rails = float(np.mean((Iu < 5) | (Iu > 250) | (Qu < 5) | (Qu > 250)) * 100.0)

nblocks = len(x) // nfft
x = x[: nblocks*nfft].reshape(nblocks, nfft)

win = np.hanning(nfft).astype(np.float32)
win_pow = float(np.sum(win*win) + 1e-12)

Pacc = None
for b in range(nblocks):
  seg = x[b].astype(np.complex64)
  seg = seg - np.mean(seg)         # DC removal
  seg = seg * win
  X = np.fft.fftshift(np.fft.fft(seg))
  P = (np.abs(X)**2) / win_pow     # linear power
  Pacc = P if Pacc is None else (Pacc + P)

Pavg = Pacc / max(1, nblocks)      # linear power spectrum (FFT bins)
Pdb  = 10.0*np.log10(Pavg + 1e-12) # dBFS-ish

freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0/srate))  # Hz rel. to center

span_mask  = np.abs(freqs) <= (span_hz/2.0)
sig_mask   = span_mask & (np.abs(freqs) <= half_bw) & (np.abs(freqs) >= dc_notch)
noise_mask = span_mask & (np.abs(freqs) >= guard)

sig_db = Pdb[sig_mask]
noi_db = Pdb[noise_mask]

# Existing metric (peak-minus-median in dB)
avg_db = float(np.mean(sig_db)) if sig_db.size else float("nan")
peak_db = float(np.max(sig_db)) if sig_db.size else float("nan")
noise_db = float(np.median(noi_db)) if noi_db.size else float("nan")
snr_peak_db = (peak_db - noise_db) if (not math.isnan(peak_db) and not math.isnan(noise_db)) else float("nan")

# New metric (power ratio in linear domain)
sig_lin = Pavg[sig_mask]
noi_lin = Pavg[noise_mask]
Psig = float(np.mean(sig_lin)) if sig_lin.size else float("nan")
Pnoi = float(np.mean(noi_lin)) if noi_lin.size else float("nan")

snr_pow_db = (10.0*math.log10(Psig / Pnoi)) if (Psig > 0 and Pnoi > 0 and not math.isnan(Psig) and not math.isnan(Pnoi)) else float("nan")

Psig_db = (10.0*math.log10(Psig)) if (Psig > 0 and not math.isnan(Psig)) else float("nan")
Pnoi_db = (10.0*math.log10(Pnoi)) if (Pnoi > 0 and not math.isnan(Pnoi)) else float("nan")

# Write legacy output (DO NOT BREAK existing readers)
with open(out_path, "w") as w:
  w.write(f"{ts:.3f} {avg_db:.2f} {peak_db:.2f} {noise_db:.2f} {snr_peak_db:.2f}\n")

# Write diagnostics (new)
with open(diag_path, "w") as w:
  w.write(f"{ts:.3f} {snr_pow_db:.2f} {Psig_db:.2f} {Pnoi_db:.2f} {mag_rms:.2f} {mag_max:.2f} {pct_rails:.3f}\n")

print(
  f"TRACK CH={int('$CH')} center_mhz={center_hz/1e6:.6f} "
  f"avg_db={avg_db:.2f} peak_db={peak_db:.2f} noise_db={noise_db:.2f} "
  f"snr_peak_db={snr_peak_db:.2f} snr_pow_db={snr_pow_db:.2f} "
  f"pct_rails={pct_rails:.3f}%"
)
PY
ngcp2025@frs-pi:~ $ cd ~/rf_pcb
ngcp2025@frs-pi:~/rf_pcb $ ls -a
.                        lock_scan_err.log  ppm_test_center.sh  pwm_test.c          servo_control.c         sweep_log.csv
..                       lock_scan.sh       ppm_test_err.log    __pycache__         servomotor_testing      Theory_Tri.py
fast_track_err.log       mc_df_accuracy.py  ppm_test.sh         rf_track_stream.py  sim_sweep_log.csv       triangulation
fast_track_rtl_power.sh  mc_results.csv     ppm_val.txt         scan_log_run.sh     simulate_df_vectors.py  triangulation.c
fast_track.sh            mission_run.sh     pwm_test            servo_control       sim_vectors.csv
ngcp2025@frs-pi:~/rf_pcb $ cd ~
ngcp2025@frs-pi:~ $ cat ~/rf_pcb/ppm_test_center.sh
#!/usr/bin/env bash
set -euo pipefail

# Preset: Channel 9 (467.5875 MHz)
CENTER_MHZ="467.5875"

# Use lower gain by default to reduce overload risk
GAIN="${GAIN:-10}"
SCAN_SEC="${SCAN_SEC:-1.0}"

PPM_MIN="${PPM_MIN:--30}"
PPM_MAX="${PPM_MAX:-30}"
PPM_STEP="${PPM_STEP:-2}"

SPAN_HZ="${SPAN_HZ:-80000}"     # scan +/-40 kHz around center
STEP_HZ="${STEP_HZ:-2500}"

# When deciding alignment, only look for peak within +/-20 kHz of expected center
PEAK_WIN_HZ="${PEAK_WIN_HZ:-20000}"

TMP="${TMP:-/tmp/rtl_ppm_center.csv}"
PPM_FILE="${PPM_FILE:-$HOME/rf_pcb/ppm_val.txt}"

CENTER_HZ=$(python3 - <<PY
print(int(float("$CENTER_MHZ")*1e6))
PY
)

HALF_SPAN=$(( SPAN_HZ / 2 ))
START_HZ=$(( CENTER_HZ - HALF_SPAN ))
STOP_HZ=$(( CENTER_HZ + HALF_SPAN ))

best_ppm=""
best_abs_off="1e18"
best_peak="-1e9"
best_peak_hz="0"

for ((ppm=PPM_MIN; ppm<=PPM_MAX; ppm+=PPM_STEP)); do
  : > "$TMP"

  # rtl_power runs continuously; use timeout and treat rc=124 as normal
  rc=0
  timeout 4 rtl_power \
    -f "${START_HZ}:${STOP_HZ}:${STEP_HZ}" \
    -i "$SCAN_SEC" \
    -g "$GAIN" \
    -p "$ppm" \
    "$TMP" 2>/dev/null || rc=$?

  if [[ $rc -ne 0 && $rc -ne 124 ]]; then
    echo "rtl_power failed at ppm=$ppm (rc=$rc)"
    exit 1
  fi

  # Must have at least one non-comment data line
  if ! grep -qv '^#' "$TMP" 2>/dev/null; then
    echo "No data captured at ppm=$ppm (TMP empty)"
    exit 1
  fi

  line=$(python3 - <<PY
import csv, math
tmp="$TMP"
center=float("$CENTER_HZ")
win=float("$PEAK_WIN_HZ")
lo, hi = center-win, center+win

best_db=-1e9
best_f=None

with open(tmp,newline="") as f:
  r=csv.reader(f)
  for row in r:
    if not row or row[0].startswith("#"):
      continue
    start=float(row[2]); step=float(row[4])
    vals=[float(x) for x in row[6:]]
    for i,db in enumerate(vals):
      f_hz=start+i*step
      if lo <= f_hz <= hi and db > best_db:
        best_db=db
        best_f=f_hz

if best_f is None:
  print("0 -1e9 1e18")
else:
  print(f"{best_f:.0f} {best_db:.2f} {abs(best_f-center):.0f}")
PY
)

  read -r peak_hz peak_db abs_off <<< "$line"
  echo "ppm=$ppm peak_db=$peak_db peak_freq_hz=$peak_hz abs_offset_hz=$abs_off"

  better=$(python3 - <<PY
off=float("$abs_off"); best=float("$best_abs_off")
pk=float("$peak_db"); bpk=float("$best_peak")
print(1 if (off < best) or (off == best and pk > bpk) else 0)
PY
)
  if [[ "$better" == "1" ]]; then
    best_ppm="$ppm"
    best_abs_off="$abs_off"
    best_peak="$peak_db"
    best_peak_hz="$peak_hz"
  fi
done

mkdir -p "$(dirname "$PPM_FILE")"
echo "$best_ppm" > "$PPM_FILE"
echo "BEST_PPM=$best_ppm  abs_offset_hz=$best_abs_off  peak_db=$best_peak  peak_freq_hz=$best_peak_hz"
echo "Saved: $PPM_FILE"
echo "export PPM_VAL=$best_ppm"
