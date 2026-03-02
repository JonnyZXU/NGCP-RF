#!/usr/bin/env python3
import csv, math, os, sys, time, subprocess

# Usage:
#   python3 rf_track_stream.py 467.5875
#
# Writes latest reading to /tmp/rf_latest.txt as:
#   <epoch_seconds> <avg_db> <peak_db>

CENTER_MHZ = float(sys.argv[1])

GAIN = int(os.environ.get("GAIN", "25"))
SRATE = int(os.environ.get("SRATE", "1024000"))
SCAN_SEC = float(os.environ.get("SCAN_SEC", "0.5"))      # 0.5 = 500ms updates
STEP_HZ = int(os.environ.get("STEP_HZ", "6250"))
SPAN_HZ = float(os.environ.get("SPAN_HZ", "50000"))      # 25k–50k good
HALF_BW = float(os.environ.get("HALF_BW_HZ", "6250"))    # +/-6.25k (12.5k channel)
OUT_PATH = os.environ.get("LATEST_PATH", "/tmp/rf_latest.txt")

center_hz = CENTER_MHZ * 1e6
fstart_m = (center_hz - SPAN_HZ/2) / 1e6
fstop_m  = (center_hz + SPAN_HZ/2) / 1e6

cmd = [
    "stdbuf", "-oL", "rtl_power",
    "-f", f"{fstart_m:.6f}M:{fstop_m:.6f}M:{STEP_HZ}",
    "-i", str(SCAN_SEC),
    "-g", str(GAIN),
    "-s", str(SRATE),
    "-"
]

proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

def write_latest(avg_db: float, peak_db: float):
    tmp = OUT_PATH + ".tmp"
    with open(tmp, "w") as f:
        f.write(f"{time.time():.3f} {avg_db:.2f} {peak_db:.2f}\n")
    os.replace(tmp, OUT_PATH)  # atomic swap

try:
    assert proc.stdout is not None
    reader = csv.reader(proc.stdout)
    for row in reader:
        if not row or row[0].startswith("#"):
            continue

        # rtl_power row: date,time,fstart,fstop,step,samples,v1,v2,...
        try:
            fstart = float(row[2]); fstop = float(row[3]); step = float(row[4])
            vals = list(map(float, row[6:]))
        except Exception:
            continue

        if step <= 0 or not vals:
            continue
        if not (fstart <= center_hz <= fstop):
            continue

        i0 = int(max(0, math.floor((center_hz - HALF_BW - fstart) / step)))
        i1 = int(min(len(vals) - 1, math.ceil((center_hz + HALF_BW - fstart) / step)))
        if i1 < i0:
            continue

        seg = vals[i0:i1+1]
        avg = sum(seg) / len(seg)
        peak = max(seg)

        write_latest(avg, peak)

except KeyboardInterrupt:
    pass
finally:
    proc.terminate()