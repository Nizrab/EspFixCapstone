"""
app.py  —  ToF Indoor Localization Dashboard
=============================================

Architecture
------------
  MQTT broker  ──>  this server  ──(WebSocket)──>  browser map

Topics consumed
  tof/anchor/+/info   : anchor position + BSSID (retained, from anchor firmware)
  tof/tag/range       : live ranging payload from the tag

Topics produced
  tof/position        : computed (x, y) position published back to MQTT

Usage
-----
  pip install -r requirements.txt
  # Edit CONFIG section below, then:
  python app.py
  # Open  http://localhost:5000  in a browser
"""

import json
import time
import threading
import logging
from typing import Dict, Optional, Tuple, List

import numpy as np
import paho.mqtt.client as mqtt_client
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION  ← Edit to match your setup
# ─────────────────────────────────────────────────────────────────────────────
MQTT_BROKER_HOST    = "localhost"   # IP of your MQTT broker
MQTT_BROKER_PORT    = 1883
MQTT_CLIENT_ID      = "tof-dashboard"

# Room dimensions (cm).  These are also the DEFAULT anchor corner positions.
# If your anchors publish their own coordinates via MQTT these are overridden.
ROOM_WIDTH_CM       = 500   # horizontal extent (cm)
ROOM_HEIGHT_CM      = 400   # vertical extent (cm)

# Default anchor positions (corners, looking down at the floor):
#   ID 0 → Bottom-Left   ID 1 → Bottom-Right
#   ID 2 → Top-Right     ID 3 → Top-Left
DEFAULT_ANCHOR_POSITIONS: Dict[int, Tuple[float, float]] = {
    0: (0.0,            0.0),
    1: (ROOM_WIDTH_CM,  0.0),
    2: (ROOM_WIDTH_CM,  ROOM_HEIGHT_CM),
    3: (0.0,            ROOM_HEIGHT_CM),
}

FLASK_HOST          = "0.0.0.0"
FLASK_PORT          = 5000
LOG_LEVEL           = logging.INFO
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
)
log = logging.getLogger("dashboard")

app     = Flask(__name__)
app.config["SECRET_KEY"] = "tof-indoor-localization"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ── Shared state (protected by a lock) ───────────────────────────────────────
_lock = threading.Lock()

# anchor_id → {"x_cm", "y_cm", "bssid", "channel", "ssid", "uptime_s", "last_seen"}
anchor_info: Dict[int, dict] = {}

# anchor_id → distance in cm (latest valid reading)
latest_distances: Dict[int, float] = {}

# Computed position history (circular buffer of last 20 fixes)
MAX_HISTORY = 20
position_history: List[dict] = []
current_position: Optional[dict] = None

# Anchor online status
anchor_status: Dict[int, str] = {}  # "online" / "offline"


# ─────────────────────────────────────────────────────────────────────────────
#  Trilateration Engine
# ─────────────────────────────────────────────────────────────────────────────
def trilaterate(anchor_positions: Dict[int, Tuple[float, float]],
                distances: Dict[int, float]) -> Optional[Tuple[float, float]]:
    """
    Weighted Least-Squares 2-D trilateration.

    Given N anchors at known (xi, yi) with measured distances di, solve for
    tag position (x, y) that minimises the sum of squared residuals.

    Method: subtract anchor-0 equation from the rest to linearise.
      (x-xi)²+(y-yi)² = di²   for all i
    Subtracting i=0 from i>0:
      2(xi-x0)·x + 2(yi-y0)·y = d0²-di² + xi²+yi² - x0²-y0²

    With ≥3 equations for 2 unknowns → over-determined → numpy lstsq.
    Weights = 1/di² (closer anchors are more reliable).

    Returns (x, y) in cm, or None if insufficient data.
    """
    # Only use anchors for which we have BOTH position and distance
    common = [aid for aid in anchor_positions if aid in distances and distances[aid] > 0]
    if len(common) < 3:
        log.warning("Trilateration needs ≥3 valid anchors, have %d", len(common))
        return None

    x0, y0 = anchor_positions[common[0]]
    d0      = distances[common[0]]

    A_rows, b_vals, weights = [], [], []

    for aid in common[1:]:
        xi, yi = anchor_positions[aid]
        di     = distances[aid]

        A_rows.append([2.0 * (xi - x0), 2.0 * (yi - y0)])
        b_vals.append(d0**2 - di**2 + xi**2 + yi**2 - x0**2 - y0**2)
        # Weight inversely proportional to distance squared
        weights.append(1.0 / max(di**2, 1.0))

    A = np.array(A_rows, dtype=float)
    b = np.array(b_vals, dtype=float)
    W = np.diag(weights)

    try:
        # Weighted least squares: (A'WA)⁻¹ A'Wb
        AtW  = A.T @ W
        AtWA = AtW @ A
        AtWb = AtW @ b
        result = np.linalg.solve(AtWA, AtWb)
        x, y = float(result[0]), float(result[1])

        # Sanity clamp to ±50 cm outside room bounds
        MARGIN = 50.0
        x = max(-MARGIN, min(ROOM_WIDTH_CM  + MARGIN, x))
        y = max(-MARGIN, min(ROOM_HEIGHT_CM + MARGIN, y))

        return x, y
    except np.linalg.LinAlgError as e:
        log.error("Trilateration linear algebra error: %s", e)
        return None


def get_anchor_positions() -> Dict[int, Tuple[float, float]]:
    """Return anchor positions: prefer MQTT-reported values, fall back to defaults."""
    positions = dict(DEFAULT_ANCHOR_POSITIONS)
    with _lock:
        for aid, info in anchor_info.items():
            positions[aid] = (info["x_cm"], info["y_cm"])
    return positions


# ─────────────────────────────────────────────────────────────────────────────
#  MQTT Callbacks
# ─────────────────────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log.info("MQTT connected")
        client.subscribe("tof/anchor/+/info",   qos=1)
        client.subscribe("tof/anchor/+/status", qos=1)
        client.subscribe("tof/tag/range",        qos=0)
        client.subscribe("tof/tag/status",       qos=1)
    else:
        log.error("MQTT connect failed, rc=%d", rc)


def on_disconnect(client, userdata, rc):
    log.warning("MQTT disconnected (rc=%d) — will auto-reconnect", rc)


def on_message(client, userdata, msg):
    topic   = msg.topic
    payload = msg.payload.decode("utf-8", errors="replace")

    # ── Anchor info ──────────────────────────────────────────────────────────
    if "/anchor/" in topic and topic.endswith("/info"):
        try:
            data = json.loads(payload)
            aid  = int(data.get("anchor_id", -1))
            if 0 <= aid < 4:
                with _lock:
                    anchor_info[aid] = {
                        "anchor_id": aid,
                        "x_cm":      float(data.get("x_cm", DEFAULT_ANCHOR_POSITIONS.get(aid, (0,0))[0])),
                        "y_cm":      float(data.get("y_cm", DEFAULT_ANCHOR_POSITIONS.get(aid, (0,0))[1])),
                        "ssid":      data.get("ssid",  f"TOF_ANCHOR_{aid}"),
                        "bssid":     data.get("bssid", ""),
                        "channel":   data.get("channel", 6),
                        "uptime_s":  data.get("uptime_s", 0),
                        "last_seen": time.time(),
                    }
                log.debug("Anchor %d info updated", aid)
        except (json.JSONDecodeError, ValueError) as e:
            log.warning("Bad anchor info payload: %s — %s", payload[:80], e)

    # ── Anchor online/offline status ─────────────────────────────────────────
    elif "/anchor/" in topic and topic.endswith("/status"):
        try:
            # Topic: tof/anchor/<id>/status
            parts = topic.split("/")
            aid = int(parts[2])
            with _lock:
                anchor_status[aid] = payload.strip()
            socketio.emit("anchor_status", {"anchor_id": aid, "status": payload.strip()})
        except (ValueError, IndexError):
            pass

    # ── Tag ranging data ─────────────────────────────────────────────────────
    elif topic == "tof/tag/range":
        try:
            data   = json.loads(payload)
            ranges = data.get("ranges", [])
            ts_us  = data.get("timestamp_us", time.time() * 1e6)

            new_distances = {}
            for r in ranges:
                aid  = int(r["anchor_id"])
                dist = float(r["distance_cm"])
                if dist > 0:
                    new_distances[aid] = dist

            with _lock:
                latest_distances.update(new_distances)
                local_distances = dict(latest_distances)

            anchor_positions = get_anchor_positions()
            result = trilaterate(anchor_positions, local_distances)

            timestamp = time.time()
            pos_data = {
                "timestamp":    timestamp,
                "timestamp_us": ts_us,
                "distances":    local_distances,
                "valid":        result is not None,
                "x_cm":         round(result[0], 1) if result else None,
                "y_cm":         round(result[1], 1) if result else None,
            }

            if result:
                x, y = result
                log.info("Position: x=%.1f cm  y=%.1f cm  (distances: %s)",
                         x, y,
                         "  ".join(f"A{k}={v:.0f}" for k, v in sorted(local_distances.items())))

                # Publish computed position back to MQTT
                client.publish("tof/position",
                               json.dumps({"x_cm": round(x, 1), "y_cm": round(y, 1),
                                           "timestamp": timestamp}),
                               qos=0)

                with _lock:
                    global current_position
                    current_position = pos_data
                    position_history.append(pos_data)
                    if len(position_history) > MAX_HISTORY:
                        position_history.pop(0)

            # Push to all connected browser clients
            socketio.emit("position_update", pos_data)

        except (json.JSONDecodeError, KeyError, TypeError) as e:
            log.warning("Bad range payload: %s — %s", payload[:120], e)

    # ── Tag online/offline ────────────────────────────────────────────────────
    elif topic == "tof/tag/status":
        socketio.emit("tag_status", {"status": payload.strip()})


# ─────────────────────────────────────────────────────────────────────────────
#  MQTT Client Thread
# ─────────────────────────────────────────────────────────────────────────────
def mqtt_thread():
    client = mqtt_client.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.on_message    = on_message

    while True:
        try:
            client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=60)
            client.loop_forever()
        except Exception as e:
            log.error("MQTT error: %s — retrying in 5 s", e)
            time.sleep(5)


# ─────────────────────────────────────────────────────────────────────────────
#  Flask Routes
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html",
                           room_width=ROOM_WIDTH_CM,
                           room_height=ROOM_HEIGHT_CM)


@app.route("/api/state")
def api_state():
    """REST endpoint — returns full current state for new browser connections."""
    with _lock:
        anchors = {
            **{aid: {"anchor_id": aid,
                     "x_cm": pos[0], "y_cm": pos[1],
                     "ssid": f"TOF_ANCHOR_{aid}", "bssid": "",
                     "status": anchor_status.get(aid, "unknown")}
               for aid, pos in DEFAULT_ANCHOR_POSITIONS.items()},
        }
        for aid, info in anchor_info.items():
            anchors[aid].update(info)
            anchors[aid]["status"] = anchor_status.get(aid, "unknown")

        return jsonify({
            "room_width_cm":  ROOM_WIDTH_CM,
            "room_height_cm": ROOM_HEIGHT_CM,
            "anchors":        list(anchors.values()),
            "distances":      latest_distances,
            "position":       current_position,
            "history":        position_history[-10:],
        })


@app.route("/api/config", methods=["GET"])
def api_config():
    return jsonify({
        "room_width_cm":  ROOM_WIDTH_CM,
        "room_height_cm": ROOM_HEIGHT_CM,
        "mqtt_broker":    f"{MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}",
    })


# ─────────────────────────────────────────────────────────────────────────────
#  SocketIO Events
# ─────────────────────────────────────────────────────────────────────────────
@socketio.on("connect")
def on_ws_connect():
    log.info("Browser client connected")
    with _lock:
        emit("init", {
            "room_width_cm":  ROOM_WIDTH_CM,
            "room_height_cm": ROOM_HEIGHT_CM,
        })


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    log.info("Starting ToF Dashboard on http://%s:%d", FLASK_HOST, FLASK_PORT)
    log.info("MQTT broker: %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT)
    log.info("Room: %d × %d cm", ROOM_WIDTH_CM, ROOM_HEIGHT_CM)

    t = threading.Thread(target=mqtt_thread, daemon=True, name="mqtt")
    t.start()

    socketio.run(app, host=FLASK_HOST, port=FLASK_PORT, debug=False, allow_unsafe_werkzeug=True)
