#!/usr/bin/env python3
"""
start_mqtt_broker.py  —  Auto-detect LAN IP and launch Mosquitto MQTT broker
================================================================================

What it does:
  1. Detects your PC's LAN IP address automatically
  2. Generates a minimal mosquitto.conf bound to that IP + localhost
  3. Starts the Mosquitto broker as a subprocess
  4. Prints the connection string to paste into your firmware #defines

Prerequisites:
  - Mosquitto installed:
      Windows:  choco install mosquitto   OR  download from mosquitto.org
      macOS:    brew install mosquitto
      Linux:    sudo apt install mosquitto mosquitto-clients

Usage:
  python start_mqtt_broker.py              # auto-detect IP, port 1883
  python start_mqtt_broker.py --port 1884  # custom port
  python start_mqtt_broker.py --ip 192.168.1.50  # force specific IP

Press Ctrl+C to stop the broker cleanly.
"""

import argparse
import os
import platform
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time


def get_lan_ip() -> str:
    """
    Detect the LAN-facing IP by opening a UDP socket toward an external host.
    No data is actually sent — this just lets the OS pick the right interface.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2)
        s.connect(("8.8.8.8", 80))      # Google DNS — never actually contacted
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        pass

    # Fallback: iterate interfaces
    hostname = socket.gethostname()
    try:
        addrs = socket.getaddrinfo(hostname, None, socket.AF_INET)
        for addr in addrs:
            ip = addr[4][0]
            if ip.startswith("192.168.") or ip.startswith("10.") or ip.startswith("172."):
                return ip
    except Exception:
        pass

    return "127.0.0.1"


def find_mosquitto() -> str:
    """Locate the mosquitto binary on the system PATH."""
    binary = shutil.which("mosquitto")
    if binary:
        return binary

    # Common install locations
    candidates = []
    if platform.system() == "Windows":
        candidates = [
            r"C:\Program Files\mosquitto\mosquitto.exe",
            r"C:\mosquitto\mosquitto.exe",
        ]
    elif platform.system() == "Darwin":
        candidates = [
            "/usr/local/sbin/mosquitto",
            "/opt/homebrew/sbin/mosquitto",
        ]
    else:
        candidates = [
            "/usr/sbin/mosquitto",
            "/usr/local/sbin/mosquitto",
        ]

    for path in candidates:
        if os.path.isfile(path):
            return path

    return ""


def generate_config(lan_ip: str, port: int, conf_path: str) -> None:
    """Write a minimal mosquitto.conf that listens on LAN IP + localhost."""
    config = f"""\
# ─────────────────────────────────────────────────────────────────────────────
# Auto-generated Mosquitto config for ToF Indoor Localization
# Broker binds to LAN IP: {lan_ip}  and  localhost  on port {port}
# ─────────────────────────────────────────────────────────────────────────────

# Listener on all interfaces (LAN + localhost)
listener {port} 0.0.0.0

# Allow anonymous connections (fine for a local lab network)
allow_anonymous true

# Protocol
protocol mqtt

# Logging
log_type error
log_type warning
log_type notice
log_type information
log_dest stdout

# Persistence (keep retained messages across restarts)
persistence true
persistence_location {tempfile.gettempdir()}/

# Connection limits
max_connections -1
"""
    with open(conf_path, "w") as f:
        f.write(config)


def main():
    parser = argparse.ArgumentParser(
        description="Start Mosquitto MQTT broker on your LAN IP for ToF localization"
    )
    parser.add_argument("--ip",   type=str, default=None,
                        help="Force a specific LAN IP instead of auto-detecting")
    parser.add_argument("--port", type=int, default=1883,
                        help="MQTT broker port (default: 1883)")
    args = parser.parse_args()

    # ── Detect LAN IP ────────────────────────────────────────────────────────
    lan_ip = args.ip or get_lan_ip()
    port   = args.port

    print("=" * 66)
    print("  ToF Indoor Localization — MQTT Broker Launcher")
    print("=" * 66)
    print()
    print(f"  Detected LAN IP : {lan_ip}")
    print(f"  Broker port     : {port}")
    print(f"  Broker URI      : mqtt://{lan_ip}:{port}")
    print()

    # ── Find Mosquitto ───────────────────────────────────────────────────────
    mosquitto_bin = find_mosquitto()
    if not mosquitto_bin:
        print("ERROR: Mosquitto not found on your system.")
        print()
        print("Install it with:")
        if platform.system() == "Windows":
            print("  choco install mosquitto")
            print("  — OR download from https://mosquitto.org/download/")
        elif platform.system() == "Darwin":
            print("  brew install mosquitto")
        else:
            print("  sudo apt install mosquitto mosquitto-clients")
        print()
        sys.exit(1)

    print(f"  Mosquitto binary : {mosquitto_bin}")

    # ── Generate config ──────────────────────────────────────────────────────
    conf_dir  = tempfile.mkdtemp(prefix="tof_mqtt_")
    conf_path = os.path.join(conf_dir, "mosquitto.conf")
    generate_config(lan_ip, port, conf_path)
    print(f"  Config file      : {conf_path}")
    print()

    # ── Print firmware defines ───────────────────────────────────────────────
    print("─" * 66)
    print("  Copy this into your anchor_main.c  AND  tag_main.c :")
    print()
    print(f'    #define MQTT_BROKER_URI  "mqtt://{lan_ip}:{port}"')
    print()
    print("  And into dashboard/app.py :")
    print()
    print(f'    MQTT_BROKER_HOST = "{lan_ip}"')
    print(f'    MQTT_BROKER_PORT = {port}')
    print("─" * 66)
    print()

    # ── Launch Mosquitto ─────────────────────────────────────────────────────
    print(f"Starting Mosquitto on {lan_ip}:{port} ...  (Ctrl+C to stop)")
    print()

    proc = subprocess.Popen(
        [mosquitto_bin, "-c", conf_path, "-v"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    # ── Handle Ctrl+C gracefully ─────────────────────────────────────────────
    def shutdown(sig, frame):
        print("\n\nShutting down Mosquitto broker...")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        # Clean up temp config
        try:
            os.remove(conf_path)
            os.rmdir(conf_dir)
        except OSError:
            pass
        print("Broker stopped.")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ── Stream broker output ─────────────────────────────────────────────────
    try:
        for line in proc.stdout:
            print(f"  [mosquitto] {line}", end="")
    except KeyboardInterrupt:
        shutdown(None, None)

    # If process exits on its own
    rc = proc.wait()
    if rc != 0:
        print(f"\nMosquitto exited with code {rc}")
        print("Check that port {port} is not already in use:")
        if platform.system() == "Windows":
            print(f"  netstat -ano | findstr :{port}")
        else:
            print(f"  lsof -i :{port}")
    else:
        print("\nMosquitto stopped normally.")


if __name__ == "__main__":
    main()
