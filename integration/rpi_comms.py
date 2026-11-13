"""
rpi_comms.py
------------
Runs on the Raspberry Pi.

Two responsibilities running in parallel:
  1. GPS SERVER (port 5100):
     Accept GPS from Jetson -> write atomically to jetson_gps.txt
     so pi_node.py always has the current rover position.

  2. RESULT WATCHER:
     Watch pi_gps.txt for changes written by pi_node.py.
     Forward every change to the Jetson via TCP (port 5101).
     - status=0 : intermediate waypoint (rover needs to move here)
     - status=1 : final target confirmed (mission complete)
"""

from __future__ import annotations

import logging
import os
import socket
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Optional

from comms_protocol import (
    GPS_PORT,
    JETSON_HOST,
    RECV_TIMEOUT,
    RESULT_PORT,
    MSG_ACK,
    MSG_GPS,
    MSG_LOCATION,
    MSG_REPOSITION,
    recv_message,
    send_message,
)

log = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [RPI-COMMS] %(levelname)s %(message)s",
)

JETSON_GPS_FILE  = "/home/ngcp2025/rf_integration/jetson_gps.txt"
PI_GPS_FILE      = "/home/ngcp2025/rf_integration/pi_gps.txt"
GPS_TIMEOUT_S    = 60.0
WATCH_INTERVAL_S = 0.25
SEND_RETRIES     = 3
SEND_RETRY_DELAY = 2.0


def _write_gps_atomic(gps_msg: Mapping[str, Any], filepath: str) -> None:
    path = Path(filepath)
    path.parent.mkdir(parents=True, exist_ok=True)
    content = (
        f"latitude: {float(gps_msg['latitude']):.7f}\n"
        f"longitude: {float(gps_msg['longitude']):.7f}\n"
        f"heading: {float(gps_msg['heading']):.2f}\n"
        f"status: {int(gps_msg.get('status', 0))}\n"
    )
    tmp = filepath + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(content)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, filepath)
    log.debug("Wrote GPS to %s", filepath)


def _validate_gps(msg: Mapping[str, Any]) -> bool:
    try:
        lat     = float(msg["latitude"])
        lon     = float(msg["longitude"])
        heading = float(msg["heading"])
    except (KeyError, TypeError, ValueError):
        return False
    if not (-90.0  <= lat     <= 90.0):  return False
    if not (-180.0 <= lon     <= 180.0): return False
    if not (-360.0 <= heading <  360.0): return False
    return True


def _read_pi_gps(filepath: str = PI_GPS_FILE) -> Optional[dict]:
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            raw = f.read()
    except (FileNotFoundError, OSError):
        return None
    fields = {}
    for line in raw.splitlines():
        line = line.strip()
        if not line or ":" not in line:
            continue
        key, _, value = line.partition(":")
        fields[key.strip().lower()] = value.strip()
    if not all(r in fields for r in ("latitude", "longitude", "heading", "status")):
        return None
    try:
        return {
            "latitude":  float(fields["latitude"]),
            "longitude": float(fields["longitude"]),
            "heading":   float(fields["heading"]),
            "status":    int(fields["status"]),
            "raw":       raw,
        }
    except (ValueError, KeyError):
        return None


def _send_result_to_jetson(result: dict) -> bool:
    status = result["status"]
    if status == 1:
        message = {
            "type":      MSG_LOCATION,
            "latitude":  result["latitude"],
            "longitude": result["longitude"],
            "heading":   result["heading"],
            "status":    1,
        }
        label = "LOCATION (target)"
    else:
        message = {
            "type":      MSG_REPOSITION,
            "latitude":  result["latitude"],
            "longitude": result["longitude"],
            "heading":   result["heading"],
            "status":    0,
            "reason":    "waypoint",
        }
        label = "REPOSITION (waypoint)"

    for attempt in range(1, SEND_RETRIES + 1):
        try:
            with socket.create_connection(
                (JETSON_HOST, RESULT_PORT), timeout=10.0
            ) as sock:
                sock.settimeout(RECV_TIMEOUT)
                send_message(sock, message)
                ack = recv_message(sock)
                if ack.get("type") == MSG_ACK:
                    log.info(
                        "Sent %s to Jetson: lat=%.7f lon=%.7f heading=%.2f",
                        label, result["latitude"],
                        result["longitude"], result["heading"],
                    )
                    return True
                log.warning("Unexpected response from Jetson: %s", ack)
        except Exception as exc:
            log.warning("Send attempt %d/%d failed: %s", attempt, SEND_RETRIES, exc)
            if attempt < SEND_RETRIES:
                time.sleep(SEND_RETRY_DELAY)
    log.error("Failed to send result to Jetson after %d attempts", SEND_RETRIES)
    return False


def gps_server_loop():
    log.info("GPS server thread started on port %d", GPS_PORT)
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind(("0.0.0.0", GPS_PORT))
                srv.listen(1)
                srv.settimeout(GPS_TIMEOUT_S)
                try:
                    conn, addr = srv.accept()
                except TimeoutError:
                    log.warning("GPS server: timed out waiting for Jetson")
                    continue
                with conn:
                    conn.settimeout(GPS_TIMEOUT_S)
                    try:
                        msg = recv_message(conn)
                    except Exception as exc:
                        log.error("GPS server: read error: %s", exc)
                        continue
                    if msg.get("type") != MSG_GPS:
                        log.warning("GPS server: unexpected type: %s", msg.get("type"))
                        continue
                    if not _validate_gps(msg):
                        log.error("GPS server: validation failed: %s", msg)
                        continue
                    send_message(conn, {"type": MSG_ACK})
                    log.info(
                        "GPS received: lat=%.7f lon=%.7f heading=%.2f status=%d",
                        msg["latitude"], msg["longitude"],
                        msg["heading"], msg.get("status", 0),
                    )
                    _write_gps_atomic(msg, JETSON_GPS_FILE)
        except Exception as exc:
            log.error("GPS server loop error: %s — restarting in 2s", exc)
            time.sleep(2)


def result_watcher_loop():
    log.info("Result watcher thread started — watching %s", PI_GPS_FILE)
    last_content  = None
    last_mtime_ns = None
    while True:
        try:
            try:
                st = os.stat(PI_GPS_FILE)
            except FileNotFoundError:
                time.sleep(WATCH_INTERVAL_S)
                continue
            if last_mtime_ns is not None and st.st_mtime_ns == last_mtime_ns:
                time.sleep(WATCH_INTERVAL_S)
                continue
            result = _read_pi_gps(PI_GPS_FILE)
            if result is None:
                time.sleep(WATCH_INTERVAL_S)
                continue
            if result["raw"] == last_content:
                last_mtime_ns = st.st_mtime_ns
                time.sleep(WATCH_INTERVAL_S)
                continue
            last_content  = result["raw"]
            last_mtime_ns = st.st_mtime_ns
            log.info(
                "pi_gps.txt changed: lat=%.7f lon=%.7f heading=%.2f status=%d",
                result["latitude"], result["longitude"],
                result["heading"], result["status"],
            )
            _send_result_to_jetson(result)
        except Exception as exc:
            log.error("Result watcher error: %s", exc)
            time.sleep(WATCH_INTERVAL_S)


if __name__ == "__main__":
    log.info("RPI comms starting")
    log.info("  GPS server    : port %d -> writes %s", GPS_PORT, JETSON_GPS_FILE)
    log.info("  Result watcher: watching %s -> Jetson port %d", PI_GPS_FILE, RESULT_PORT)

    gps_thread = threading.Thread(target=gps_server_loop, daemon=True, name="gps-server")
    result_thread = threading.Thread(target=result_watcher_loop, daemon=True, name="result-watcher")

    gps_thread.start()
    result_thread.start()

    log.info("Both threads running")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down")
