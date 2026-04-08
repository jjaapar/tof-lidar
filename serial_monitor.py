#!/usr/bin/env python3
"""serial_monitor.py — Reads door state events from Arduino Nano ESP32
over USB serial and writes JSON lines to stdout and/or a log file.

Requirements:
    pip install pyserial

Usage:
    python3 serial_monitor.py [--port /dev/door-sensor] [--output /var/log/door-monitor/events.jsonl]
"""

import argparse
import json
import logging
import os
import signal
import sys
import time

import serial
import serial.tools.list_ports

# ── Configuration defaults ─────────────────────────────────────────────────
DEFAULT_SERIAL_PORT = "/dev/door-sensor"
DEFAULT_BAUD_RATE   = 115200

# ── Logging setup ──────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
    stream=sys.stderr,  # Logs to stderr so stdout stays clean for JSON events
)
logger = logging.getLogger("serial_monitor")

# ── Graceful shutdown ──────────────────────────────────────────────────────
running = True

def handle_signal(signum, frame):
    global running
    logger.info("Received signal %d, shutting down...", signum)
    running = False

signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


# ── Serial helpers ─────────────────────────────────────────────────────────
def find_serial_port(preferred: str) -> str:
    """Return the preferred port if it exists, otherwise auto-detect."""
    if os.path.exists(preferred):
        return preferred

    for port_info in serial.tools.list_ports.comports():
        vid = port_info.vid
        if vid in (0x303A, 0x2341):  # Espressif or Arduino vendor IDs
            logger.info("Auto-detected serial port: %s (%s)", port_info.device, port_info.description)
            return port_info.device

    logger.warning("Preferred port %s not found; falling back to /dev/ttyACM0", preferred)
    return "/dev/ttyACM0"


def open_serial(port: str, baud: int) -> serial.Serial:
    """Open serial port with retry on failure."""
    while running:
        try:
            ser = serial.Serial(port, baud, timeout=2)
            logger.info("Opened serial port %s at %d baud", port, baud)
            time.sleep(2)  # Wait for Arduino reset after USB connection
            ser.reset_input_buffer()
            return ser
        except serial.SerialException as exc:
            logger.error("Cannot open %s: %s — retrying in 5 s", port, exc)
            time.sleep(5)
    sys.exit(0)


# ── Event writer ───────────────────────────────────────────────────────────
class EventWriter:
    """Writes JSON events to stdout and optionally to a JSONL file."""

    def __init__(self, output_path: str = None):
        self.output_path = output_path
        self.file_handle = None
        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            self.file_handle = open(output_path, "a", buffering=1)  # line-buffered
            logger.info("Writing events to %s", output_path)

    def write(self, event: dict):
        """Write a JSON event to stdout and optional file."""
        line = json.dumps(event)
        # Always write to stdout
        print(line, flush=True)
        # Optionally write to file
        if self.file_handle:
            self.file_handle.write(line + "\n")

    def close(self):
        if self.file_handle:
            self.file_handle.close()


# ── Main read loop ─────────────────────────────────────────────────────────
def read_loop(ser: serial.Serial, writer: EventWriter):
    """Read JSON lines from serial and forward valid events."""
    while running:
        try:
            raw_line = ser.readline()
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line or not line.startswith("{"):
                continue

            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                logger.debug("Malformed JSON: %s", line)
                continue

            event = data.get("event", "")

            if event == "state_change":
                logger.info(
                    "Door %s — distance=%d cm strength=%d",
                    data.get("state", "?").upper(),
                    data.get("distance_cm", -1),
                    data.get("signal_strength", 0),
                )
                writer.write(data)

            elif event == "boot":
                logger.info("Arduino booted: %s", data.get("message", ""))
                writer.write(data)

            elif event == "debug":
                # Pass through debug/calibration events
                writer.write(data)

        except serial.SerialException as exc:
            logger.error("Serial error: %s — reconnecting", exc)
            ser.close()
            time.sleep(2)
            ser = open_serial(ser.port, ser.baudrate)
        except Exception as exc:
            logger.error("Unexpected error: %s", exc)
            time.sleep(1)


# ── Entry point ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Door sensor serial monitor")
    parser.add_argument(
        "--port", default=DEFAULT_SERIAL_PORT,
        help=f"Serial port (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD_RATE,
        help=f"Baud rate (default: {DEFAULT_BAUD_RATE})",
    )
    parser.add_argument(
        "--output", default=None,
        help="Optional JSONL file path for event logging",
    )
    args = parser.parse_args()

    port = find_serial_port(args.port)
    writer = EventWriter(args.output)
    ser = open_serial(port, args.baud)

    logger.info("Serial monitor running — press Ctrl+C to stop")
    read_loop(ser, writer)

    writer.close()
    ser.close()
    logger.info("Serial monitor stopped")


if __name__ == "__main__":
    main()
