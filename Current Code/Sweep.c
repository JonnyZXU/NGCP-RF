#!/usr/bin/env bash
set -euo pipefail

DIR="$HOME/rf_pcb"
ACQ="$DIR/scan_log_run.sh"
TRACK="$DIR/rf_track_stream.py"
SERVO_BIN="$DIR/servo_control"

# Mission knobs
TRACK_SECONDS="${TRACK_SECONDS:-240}"
SWEEP_START="${SWEEP_START:--60}"
SWEEP_STOP="${SWEEP_STOP:-60}"
SWEEP_STEP="${SWEEP_STEP:-2}"

# Servo motion timing (seconds). Keep small; RF sample wait will dominate.
MOVE_SEC="${MOVE_SEC:-0.15}"

LATEST_PATH="${LATEST_PATH:-/tmp/rf_latest.txt}"
SWEEP_LOG="${SWEEP_LOG:-$HOME/sweep_log.csv}"
SERVO_LOG="${SERVO_LOG:-/tmp/servo_control.log}"

# -------- Replace GPS hook when ready ----------
read_gps() {
  # TODO: print "lat lon"
  echo "34.000000 -117.000000"
}
# ----------------------------------------------

ensure_sweep_log() {
  if [ ! -f "$SWEEP_LOG" ]; then
    echo "epoch,lat,lon,angle_deg,avg_db,peak_db,channel_mhz" > "$SWEEP_LOG"
  fi
}

# ---- Servo: start once, keep alive ----
start_servo() {
  if [ ! -x "$SERVO_BIN" ]; then
    echo "[mission] ERROR: $SERVO_BIN not found/executable. Build it with:"
    echo "  gcc $DIR/servo_control.c -o $DIR/servo_control -lm"
    exit 1
  fi

  # Start servo_control once; redirect stdout/stderr so it cannot block.
  # NOTE: sudo may prompt for a password. If so, run mission with sudo OR set NOPASSWD for this binary.
  coproc SERVO_PROC { sudo -n "$SERVO_BIN" >"$SERVO_LOG" 2>&1; }

  # If sudo -n fails, the process exits immediately; catch that early.
  sleep 0.1
  if ! kill -0 "$SERVO_PROC_PID" 2>/dev/null; then
    echo "[mission] ERROR: servo_control didn't start. Likely sudo needs a password."
    echo "Try:"
    echo "  sudo -E GAIN=... SCAN_SEC=... $DIR/mission_run.sh"
    echo "or add a sudoers NOPASSWD rule for $SERVO_BIN."
    exit 1
  fi

  SERVO_IN_FD="${SERVO_PROC[1]}"
}

servo_send() {
  # Send one command line to servo_control
  printf "%s\n" "$*" >&$SERVO_IN_FD
}

rotate_to() {
  local ang="$1"
  servo_send "goto $ang $MOVE_SEC 0"
  # give the servo a moment; RF sample wait below is the real sync
  sleep "$MOVE_SEC"
}

stop_servo() {
  # Graceful quit
  if [[ -n "${SERVO_IN_FD:-}" ]]; then
    { printf "quit\n" >&$SERVO_IN_FD; } 2>/dev/null || true
  fi
  if [[ -n "${SERVO_PROC_PID:-}" ]]; then
    kill "$SERVO_PROC_PID" 2>/dev/null || true
  fi
}
# ---------------------------------------

# Acquire channel (use your existing script once)
echo "[mission] acquiring channel..."
ACQ_LINE="$("$ACQ" --once | tail -n 1)"

CH=""
MHZ=""
for tok in $ACQ_LINE; do
  case "$tok" in
    best_ch=*) CH="${tok#best_ch=}" ;;
    freq=*)    f="${tok#freq=}" ; MHZ="${f%MHz}" ;;
  esac
done

if [[ -z "$MHZ" ]]; then
  echo "[mission] ERROR: couldn't parse MHZ from: $ACQ_LINE" >&2
  exit 1
fi
echo "[mission] locked channel ch=$CH mhz=$MHZ"

# Start servo + RF tracker
start_servo
trap 'stop_servo' EXIT

export LATEST_PATH
export GAIN="${GAIN:-15}"
export SRATE="${SRATE:-1024000}"
export SCAN_SEC="${SCAN_SEC:-0.5}"
export STEP_HZ="${STEP_HZ:-12500}"
export SPAN_HZ="${SPAN_HZ:-25000}"
export HALF_BW_HZ="${HALF_BW_HZ:-6250}"

echo "[mission] starting RF tracker (SCAN_SEC=$SCAN_SEC s)..."
python3 "$TRACK" "$MHZ" &
TRACK_PID=$!
trap 'kill $TRACK_PID 2>/dev/null || true; stop_servo' EXIT

ensure_sweep_log

# Wait for first RF sample
echo "[mission] waiting for first RF sample..."
for _ in {1..50}; do
  [[ -f "$LATEST_PATH" ]] && break
  sleep 0.05
done

last_epoch="0"
if [[ -f "$LATEST_PATH" ]]; then
  read -r last_epoch _ _ < "$LATEST_PATH" || last_epoch="0"
fi

echo "[mission] sweeping angles ${SWEEP_START}..${SWEEP_STOP} step=${SWEEP_STEP}"
start_epoch="$(date +%s)"

for ((ang=SWEEP_START; ang<=SWEEP_STOP; ang+=SWEEP_STEP)); do
  now="$(date +%s)"
  if (( now - start_epoch >= TRACK_SECONDS )); then
    echo "[mission] time budget reached"
    break
  fi

  rotate_to "$ang"

  # Wait for a NEW RF sample (epoch increases)
  avg=""; peak=""; epoch="$last_epoch"
  for _ in {1..60}; do   # up to ~3 seconds (60 * 0.05)
    if [[ -f "$LATEST_PATH" ]]; then
      read -r epoch avg peak < "$LATEST_PATH" || true
      if awk "BEGIN{exit !($epoch > $last_epoch)}"; then
        break
      fi
    fi
    sleep 0.05
  done
  last_epoch="$epoch"

  read -r lat lon < <(read_gps)

  # We don't have a real angle sensor here; log the commanded target.
  cur_ang="$ang"

  echo "$epoch,$lat,$lon,$cur_ang,$avg,$peak,$MHZ" >> "$SWEEP_LOG"
  echo "[sweep] ang=$cur_ang avg=$avg peak=$peak"
done

echo "[mission] sweep done. log: $SWEEP_LOG"