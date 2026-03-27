"""
rpi_comms.py
------------
Runs on the **Raspberry Pi**.

This module handles only Raspberry Pi communication responsibilities:

  GpsServer
      Receives GPS_DATA messages from the Jetson.

  ResultSender
      Sends the already-computed triangulation result back to the Jetson as
      either a LOCATION message or a REPOSITION message.

This file does **not** perform triangulation. It assumes another Pi-side
subsystem already computes the result and hands it to this module.

Typical integration:
    from rpi_comms import GpsServer, ResultSender

    gps_server = GpsServer()
    result_sender = ResultSender()

    gps_data = gps_server.receive_gps()  # if your existing system needs it

    triangulation_result = get_existing_triangulation_result()
    result_sender.send_triangulation_result(triangulation_result)
"""

from __future__ import annotations

import logging
import socket
import time
from typing import Any, Mapping, Optional

from comms_protocol import (
    CONNECT_TIMEOUT,
    GPS_PORT,
    JETSON_HOST,
    RECV_TIMEOUT,
    RESULT_PORT,
    MSG_ACK,
    MSG_GPS,
    make_location_message,
    make_reposition_message,
    recv_message,
    send_message,
)

log = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [RPI-COMMS] %(levelname)s %(message)s",
)


class GpsServer:
    """Receive one GPS_DATA message from the Jetson per scan attempt."""

    def __init__(self, port: int = GPS_PORT, timeout: float = RECV_TIMEOUT):
        self.port = port
        self.timeout = timeout

    def receive_gps(self) -> Optional[dict[str, Any]]:
        """
        Wait for one valid GPS_DATA message.

        Returns the decoded message dictionary, or None on timeout/error.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", self.port))
            srv.listen(1)
            srv.settimeout(self.timeout)
            log.info("GPS server listening on port %d", self.port)

            try:
                conn, addr = srv.accept()
                log.info("Jetson connected from %s", addr)
            except TimeoutError:
                log.error("Timed out waiting for GPS connection from Jetson")
                return None

            with conn:
                conn.settimeout(self.timeout)
                try:
                    msg = recv_message(conn)
                except Exception as exc:
                    log.error("Error reading GPS message: %s", exc)
                    return None

                if msg.get("type") != MSG_GPS:
                    log.warning("Unexpected message type on GPS port: %s", msg.get("type"))
                    return None

                if not _validate_gps(msg):
                    log.error("GPS message failed validation: %s", msg)
                    return None

                send_message(conn, {"type": MSG_ACK})
                log.info(
                    "GPS received and ACKed: lat=%.6f lon=%.6f heading=%.1f",
                    msg["latitude"],
                    msg["longitude"],
                    msg["heading"],
                )
                return msg


class ResultSender:
    """
    Send the Pi's already-available triangulation outcome to the Jetson.

    Public options:
      - send_location(...)
      - send_reposition(...)
      - send_triangulation_result(result)
    """

    def __init__(
        self,
        jetson_host: str = JETSON_HOST,
        result_port: int = RESULT_PORT,
        retries: int = 3,
        retry_delay: float = 2.0,
    ):
        self.jetson_host = jetson_host
        self.result_port = result_port
        self.retries = retries
        self.retry_delay = retry_delay

    def send_location(self, lat: float, lon: float, confidence: float = 1.0) -> bool:
        """Send a successful survivor location result to the Jetson."""
        msg = make_location_message(lat, lon, confidence)
        log.info("Sending LOCATION to Jetson: lat=%.6f lon=%.6f conf=%.2f", lat, lon, confidence)
        return self._send(msg)

    def send_reposition(self, reason: str = "Insufficient RF data") -> bool:
        """Send a retry/reposition signal to the Jetson."""
        msg = make_reposition_message(reason)
        log.info("Sending REPOSITION to Jetson: %s", reason)
        return self._send(msg)

    def send_triangulation_result(self, result: Optional[Mapping[str, Any]]) -> bool:
        """
        Send an already-computed triangulation result to the Jetson.

        Accepted success shapes include:
            {"success": True, "latitude": ..., "longitude": ..., "confidence": ...}
            {"latitude": ..., "longitude": ..., "confidence": ...}
            {"lat": ..., "lon": ..., "confidence": ...}

        Accepted failure shapes include:
            None
            {"success": False, "reason": "..."}
            {"reason": "..."}

        Returns True when the Jetson ACKs the delivered message.
        """
        normalized = normalize_triangulation_result(result)
        if normalized["type"] == "success":
            return self.send_location(
                normalized["latitude"],
                normalized["longitude"],
                normalized["confidence"],
            )
        return self.send_reposition(normalized["reason"])

    def _send(self, message: dict[str, Any]) -> bool:
        for attempt in range(1, self.retries + 1):
            try:
                log.info(
                    "Connecting to Jetson at %s:%d (attempt %d/%d)",
                    self.jetson_host,
                    self.result_port,
                    attempt,
                    self.retries,
                )
                with socket.create_connection(
                    (self.jetson_host, self.result_port),
                    timeout=CONNECT_TIMEOUT,
                ) as sock:
                    sock.settimeout(RECV_TIMEOUT)
                    send_message(sock, message)
                    ack = recv_message(sock)
                    if ack.get("type") == MSG_ACK:
                        log.info("Jetson ACKed the result message")
                        return True
                    log.warning("Unexpected response after result send: %s", ack)
            except (ConnectionRefusedError, TimeoutError, OSError) as exc:
                log.warning("Result send attempt %d failed: %s", attempt, exc)
                if attempt < self.retries:
                    time.sleep(self.retry_delay)

        log.error("Failed to deliver result to Jetson after %d attempts", self.retries)
        return False


def normalize_triangulation_result(result: Optional[Mapping[str, Any]]) -> dict[str, Any]:
    """
    Convert your existing Pi triangulation output into a communication decision.

    Output shape:
      success -> {"type": "success", "latitude": float, "longitude": float, "confidence": float}
      failure -> {"type": "failure", "reason": str}
    """
    if result is None:
        return {"type": "failure", "reason": "Insufficient RF data"}

    success_flag = result.get("success") if isinstance(result, Mapping) else None

    latitude = _first_present(result, "latitude", "lat")
    longitude = _first_present(result, "longitude", "lon")
    confidence = result.get("confidence", 1.0) if isinstance(result, Mapping) else 1.0
    reason = result.get("reason", "Insufficient RF data") if isinstance(result, Mapping) else "Insufficient RF data"

    has_coords = latitude is not None and longitude is not None

    if success_flag is False:
        return {"type": "failure", "reason": str(reason)}

    if success_flag is True or has_coords:
        try:
            return {
                "type": "success",
                "latitude": float(latitude),
                "longitude": float(longitude),
                "confidence": float(confidence),
            }
        except (TypeError, ValueError):
            return {"type": "failure", "reason": "Triangulation output contained invalid coordinates"}

    return {"type": "failure", "reason": str(reason)}


def _first_present(result: Mapping[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in result and result[key] is not None:
            return result[key]
    return None


def _validate_gps(msg: Mapping[str, Any]) -> bool:
    try:
        lat = float(msg["latitude"])
        lon = float(msg["longitude"])
        heading = float(msg["heading"])
        fix_quality = int(msg["fix_quality"])
    except (KeyError, TypeError, ValueError):
        return False

    if not (-90.0 <= lat <= 90.0):
        return False
    if not (-180.0 <= lon <= 180.0):
        return False
    if not (0.0 <= heading < 360.0):
        return False
    if fix_quality < 1:
        return False
    return True


if __name__ == "__main__":
    # Simple local behavior example. Replace fake_result with your existing
    # triangulation output object when integrating into the Pi system.
    sender = ResultSender()
    fake_result = {
        "success": False,
        "reason": "Insufficient RF data",
    }
    ok = sender.send_triangulation_result(fake_result)
    raise SystemExit(0 if ok else 1)
