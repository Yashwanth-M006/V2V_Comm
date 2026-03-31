# V2V ADAS Dashboard — Raspberry Pi

Real-time vehicle-to-vehicle monitoring dashboard that receives ESP-NOW telemetry
from the ESP32 fleet via UART and renders a live ADAS-style road view.

---

## File Structure

```
v2v_dashboard/
├── dashboard.py          ← Main PyQt5 application (UI, rendering, alerts)
├── vehicle_tracker.py    ← GPS → relative-metre coordinate engine
├── serial_reader.py      ← UART QThread for ESP32 JSON ingestion
├── requirements.txt
├── run.sh                ← One-shot setup & launch
└── v2v-dashboard.service ← systemd autostart (optional)
```

---

## Hardware Wiring

### ESP32 → Raspberry Pi UART

```
ESP32 Pin 17 (TXD2)  →  Pi GPIO 15 (RXD / Pin 10)
ESP32 Pin 16 (RXD2)  →  Pi GPIO 14 (TXD / Pin 8)   [optional, for commands]
ESP32 GND            →  Pi GND (Pin 6 or 9)
```

> ⚠️ **Logic level**: ESP32 is 3.3 V, Pi GPIO is 3.3 V — direct connection is safe.
> Do **not** connect ESP32 TX to a 5 V Pi UART adapter without a level shifter.

### Pi UART Port

| Pi Model   | UART Port      | Notes                                 |
|------------|----------------|---------------------------------------|
| Pi 4B      | `/dev/ttyS0`   | Enable via raspi-config               |
| Pi 3B+     | `/dev/ttyAMA0` | Disable Bluetooth to free full UART   |
| Pi Zero 2W | `/dev/ttyS0`   | Enable via raspi-config               |

---

## Setup

### 1 — Enable UART on Pi

```bash
sudo raspi-config
# → Interface Options → Serial Port
# → "Would you like a login shell to be accessible over serial?" → No
# → "Would you like the serial port hardware to be enabled?"    → Yes
sudo reboot
```

### 2 — Clone / copy files

```bash
mkdir ~/v2v_dashboard
# Copy all files here
chmod +x ~/v2v_dashboard/run.sh
```

### 3 — Run

```bash
cd ~/v2v_dashboard
./run.sh
```

For demo mode (no ESP32 hardware needed):

```bash
python3 dashboard.py --demo
```

---

## CLI Options

```
python3 dashboard.py [options]

  --port  /dev/ttyS0   UART port (default: /dev/ttyS0)
  --baud  115200       Baud rate (default: 115200, must match ESP32 Serial2)
  --demo               Inject synthetic vehicle data — no hardware required
```

---

## Autostart on Boot (optional)

```bash
# Edit the service file — update paths if your user is not 'pi'
nano ~/v2v_dashboard/v2v-dashboard.service

sudo cp ~/v2v_dashboard/v2v-dashboard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable v2v-dashboard
sudo systemctl start  v2v-dashboard

# Check status
sudo systemctl status v2v-dashboard
journalctl -u v2v-dashboard -f
```

---

## Dashboard UI Guide

```
┌──────────────────────────────────────────────────┐
│  V2V ADAS MONITOR              ● CONNECTED  HH:MM:SS  │  ← Top bar
├──────────────────────────────────────────────────┤
│  ACTIVE ALERTS                                   │  ← Alert panel
│  🚨 ACCIDENT DETECTED  [V3]            12:34:01  │
│  ⚠️  Vehicle braking hard ahead [V2]   12:33:58  │
├─────────────────────────────────┬────────────────┤
│                                 │ VEHICLE DETAILS│
│     [Road View — top-down]      │ ID     : V3    │
│                                 │ EVENT  : HAZARD│
│   ●V3(hazard)                   │ SPEED  : 52km/h│
│         ●V2(brake)              │ DIST   : 38.2m │
│                                 │ RSSI   : -72dBm│
│     ──────50m──────             │ SIGNAL ▪▪▪░░   │
│                                 │                │
│          [YOU]                  │ FLEET STATS    │
│                                 │ Vehicles: 3    │
│                                 │ Packets: 8/s   │
└─────────────────────────────────┴────────────────┘
```

### Vehicle Colors

| Color  | Event            |
|--------|------------------|
| 🟢 Green  | STATUS (normal)  |
| 🟡 Yellow | HAZARD           |
| 🔴 Red    | HARSH_BRAKE (blinks) |
| 🟠 Orange | SUDDEN_STOP      |
| 🔵 Blue   | BLIND_SPOT_ALERT |
| 🔴 Red ✖  | ACCIDENT (flashes) |

### Interaction

- **Click any vehicle** on the road to pin its details in the right panel.
- Distance grid rings show 20 m / 40 m / 60 m / 80 m / 100 m ahead.
- Vehicles fade and disappear after 6 seconds with no update (timeout).

---

## Coordinate System

The ESP32 sends floating-point lat/lon. The Pi converts these to metre offsets
from a reference ego position using the Haversine formula:

```
lat_offset_m  →  vertical axis   (positive = ahead)
lon_offset_m  →  horizontal axis (positive = right / east)
```

The ego reference position is defined in `vehicle_tracker.py` (EGO_LAT / EGO_LON).
Update these to your actual deployment location for accurate relative positioning.
If your Pi has a GPS module, call `tracker.set_ego_position(lat, lon)` dynamically.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Permission denied /dev/ttyS0` | `sudo usermod -aG dialout $USER` then re-login |
| Blank screen on Pi | Ensure `DISPLAY=:0` is set; run from desktop terminal |
| No vehicles shown | Check ESP32 baud (115200), wiring TXD2→Pi RXD |
| Vehicles all at origin | EGO_LAT/LON mismatch — update `vehicle_tracker.py` |
| Qt platform error | `export QT_QPA_PLATFORM=xcb` or install `libxcb-xinerama0` |
