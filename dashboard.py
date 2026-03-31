"""
V2V Real-Time Monitoring Dashboard
Raspberry Pi - PyQt5 ADAS-style interface
Receives JSON telemetry from ESP32 over UART (Serial2)
"""

import sys
import json
import math
import time
import random
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QFrame, QScrollArea, QSizePolicy, QGraphicsDropShadowEffect
)
from PyQt5.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QRectF, QPointF, QPropertyAnimation,
    QEasingCurve, pyqtProperty, QObject
)
from PyQt5.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QFontDatabase,
    QLinearGradient, QRadialGradient, QPainterPath, QPolygonF,
    QPalette, QConicalGradient
)

from serial_reader import SerialReader
from vehicle_tracker import VehicleTracker, VehicleState

# ─────────────────────────── THEME ───────────────────────────
THEME = {
    "bg":           "#0a0c0f",
    "road_dark":    "#111418",
    "road_surface": "#1a1e25",
    "lane_mark":    "#ffffff",
    "lane_dim":     "#3a3e45",
    "grid_line":    "#1e2530",
    "grid_label":   "#3a5070",
    "accent":       "#00e5ff",
    "accent2":      "#0077aa",
    "panel_bg":     "#0d1117",
    "panel_border": "#1e2d3d",
    "text_primary": "#e8f4fd",
    "text_muted":   "#4a6080",
    "text_dim":     "#2a3a4a",

    "evt_status":   "#00c853",
    "evt_hazard":   "#ffd600",
    "evt_brake":    "#ff1744",
    "evt_stop":     "#ff6d00",
    "evt_blind":    "#2979ff",
    "evt_accident": "#ff1744",
}

EVENT_COLOR = {
    "STATUS":           THEME["evt_status"],
    "HAZARD":           THEME["evt_hazard"],
    "HARSH_BRAKE":      THEME["evt_brake"],
    "SUDDEN_STOP":      THEME["evt_stop"],
    "BLIND_SPOT_ALERT": THEME["evt_blind"],
    "ACCIDENT":         THEME["evt_accident"],
}

EVENT_LABEL = {
    "STATUS":           "● Normal",
    "HAZARD":           "⚠ Hazard",
    "HARSH_BRAKE":      "⚠ Harsh Brake",
    "SUDDEN_STOP":      "■ Sudden Stop",
    "BLIND_SPOT_ALERT": "◈ Blind Spot",
    "ACCIDENT":         "✖ ACCIDENT",
}

ALERT_MSG = {
    "HAZARD":           "⚠️  Hazard detected nearby",
    "HARSH_BRAKE":      "⚠️  Vehicle braking hard ahead",
    "SUDDEN_STOP":      "🛑  Vehicle sudden stop — collision risk",
    "BLIND_SPOT_ALERT": "🔵  Blind spot alert — check mirrors",
    "ACCIDENT":         "🚨  ACCIDENT DETECTED — slow down",
}

# ─────────────────────────── ROAD WIDGET ───────────────────────────

class RoadWidget(QWidget):
    vehicleSelected = pyqtSignal(object)

    ROAD_W_FRAC  = 0.52   # fraction of widget width for road
    NUM_LANES    = 3
    EGO_Y_FRAC   = 0.88   # ego vehicle vertical position (bottom area)
    RANGE_M      = 120.0  # metres visible ahead

    def __init__(self, parent=None):
        super().__init__(parent)
        self.tracker = None
        self.selected_id = None
        self._blink_state = False
        self._blink_timer = QTimer(self)
        self._blink_timer.timeout.connect(self._toggle_blink)
        self._blink_timer.start(400)
        self.setMinimumSize(500, 600)

    def set_tracker(self, tracker):
        self.tracker = tracker

    def _toggle_blink(self):
        self._blink_state = not self._blink_state
        self.update()

    # ── coordinate helpers ──────────────────────────────────────
    def _road_rect(self):
        w, h = self.width(), self.height()
        rw = int(w * self.ROAD_W_FRAC)
        rx = (w - rw) // 2
        return rx, 0, rw, h

    def _world_to_screen(self, lat_offset_m, lon_offset_m):
        """Convert metre offsets from ego to screen coords."""
        rx, ry, rw, rh = self._road_rect()
        ego_y = rh * self.EGO_Y_FRAC

        # Vertical: forward = up
        # perspective scale: objects far away are smaller & higher
        t = lat_offset_m / self.RANGE_M          # 0..1 (0=ego, 1=far)
        t = max(-0.15, min(1.0, t))

        # perspective y: linear mapping with slight taper
        sy = ego_y - t * ego_y * 0.95

        # perspective x scaling (road narrows at top)
        near_hw = rw * 0.45
        far_hw  = rw * 0.18
        hw = near_hw + (far_hw - near_hw) * max(0, t)

        # lane width at this depth
        lane_hw = hw / (self.NUM_LANES / 2)
        sx = rx + rw / 2 + lon_offset_m / 3.5 * lane_hw  # 3.5 m per lane

        return sx, sy

    def _scale_at(self, lat_offset_m):
        t = max(0, min(1.0, lat_offset_m / self.RANGE_M))
        return 1.0 - t * 0.55   # 1.0 at ego, 0.45 at horizon

    # ── painting ────────────────────────────────────────────────
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        w, h = self.width(), self.height()

        self._draw_background(p, w, h)
        self._draw_road(p, w, h)
        self._draw_distance_grid(p, w, h)
        self._draw_lane_markings(p, w, h)
        self._draw_remote_vehicles(p)
        self._draw_ego(p, w, h)

        p.end()

    def _draw_background(self, p, w, h):
        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0, QColor("#060810"))
        grad.setColorAt(1, QColor("#0a0c0f"))
        p.fillRect(0, 0, w, h, grad)

        # subtle scanlines feel
        p.setPen(QPen(QColor(255, 255, 255, 4), 1))
        for y in range(0, h, 4):
            p.drawLine(0, y, w, y)

    def _draw_road(self, p, w, h):
        rx, _, rw, rh = self._road_rect()
        ego_y = rh * self.EGO_Y_FRAC

        near_hw = rw * 0.45
        far_hw  = rw * 0.18
        cx = rx + rw / 2

        # Road fill with perspective trapezoid
        path = QPainterPath()
        path.moveTo(cx - near_hw, h)
        path.lineTo(cx + near_hw, h)
        path.lineTo(cx + far_hw,  0)
        path.lineTo(cx - far_hw,  0)
        path.closeSubpath()

        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0, QColor("#141820"))
        grad.setColorAt(1, QColor("#1c2028"))
        p.fillPath(path, grad)

        # Road edge lines
        pen = QPen(QColor(THEME["accent2"]), 2)
        pen.setStyle(Qt.SolidLine)
        p.setPen(pen)
        p.drawLine(int(cx - near_hw), h, int(cx - far_hw), 0)
        p.drawLine(int(cx + near_hw), h, int(cx + far_hw), 0)

        # Glow on road edges
        for alpha in [30, 15, 7]:
            gpen = QPen(QColor(0, 180, 255, alpha), 8)
            p.setPen(gpen)
            p.drawLine(int(cx - near_hw), h, int(cx - far_hw), 0)
            p.drawLine(int(cx + near_hw), h, int(cx + far_hw), 0)

    def _draw_distance_grid(self, p, w, h):
        rx, _, rw, rh = self._road_rect()
        cx = rx + rw / 2
        near_hw = rw * 0.45
        far_hw  = rw * 0.18

        distances = [20, 40, 60, 80, 100, 120]
        for dist in distances:
            sx, sy = self._world_to_screen(dist, 0)
            t = dist / self.RANGE_M
            hw = near_hw + (far_hw - near_hw) * t

            alpha = max(20, int(90 - t * 60))
            pen = QPen(QColor(30, 80, 120, alpha), 1, Qt.DashLine)
            p.setPen(pen)
            p.drawLine(int(cx - hw), int(sy), int(cx + hw), int(sy))

            # Label
            lbl_font = QFont("Courier New", 7)
            p.setFont(lbl_font)
            p.setPen(QColor(THEME["grid_label"]))
            p.drawText(int(cx + hw + 6), int(sy + 4), f"{dist}m")

    def _draw_lane_markings(self, p, w, h):
        rx, _, rw, rh = self._road_rect()
        cx = rx + rw / 2
        near_hw = rw * 0.45
        far_hw  = rw * 0.18

        # Draw dashes along each lane divider
        for lane in [-1, 0, 1]:   # two inner dividers + center
            if lane == 0:
                color = QColor(255, 255, 255, 30)
                style = Qt.DotLine
            else:
                color = QColor(255, 255, 255, 55)
                style = Qt.DashLine

            pen = QPen(color, 2, style)
            pen.setDashPattern([12, 18])
            p.setPen(pen)

            # lane divider x at near and far
            frac = lane / (self.NUM_LANES / 2)
            x_near = cx + frac * near_hw
            x_far  = cx + frac * far_hw
            p.drawLine(int(x_near), h, int(x_far), 0)

    def _draw_ego(self, p, w, h):
        rx, _, rw, rh = self._road_rect()
        cx = rx + rw / 2
        ey = rh * self.EGO_Y_FRAC
        self._draw_car(p, cx, ey, 1.0, "#00e5ff", "YOU", None, is_ego=True)

    def _draw_remote_vehicles(self, p):
        if not self.tracker:
            return
        now = time.time()
        vehicles = self.tracker.get_vehicles()
        for vid, vs in vehicles.items():
            sx, sy = self._world_to_screen(vs.lat_offset, vs.lon_offset)
            scale = self._scale_at(vs.lat_offset)
            color = EVENT_COLOR.get(vs.event, THEME["evt_status"])
            blink = vs.event in ("HARSH_BRAKE", "ACCIDENT") and self._blink_state
            self._draw_car(p, sx, sy, scale, color, vs.vid, vs, blink=blink)

    def _draw_car(self, p, cx, cy, scale, color_hex, label, vs, is_ego=False, blink=False):
        color = QColor(color_hex)
        W = int(28 * scale) if not is_ego else 36
        H = int(52 * scale) if not is_ego else 66

        # Glow halo
        if blink or is_ego:
            for r in [W + 16, W + 10, W + 5]:
                alpha = 60 if is_ego else (80 if blink else 30)
                gcolor = QColor(color)
                gcolor.setAlpha(alpha)
                p.setPen(Qt.NoPen)
                p.setBrush(gcolor)
                p.drawEllipse(int(cx - r//2), int(cy - r//2 - H//4), r, r)

        # BLIND_SPOT: draw side highlight
        if vs and vs.event == "BLIND_SPOT_ALERT":
            bpen = QPen(QColor(41, 121, 255, 80), 6)
            p.setPen(bpen)
            p.setBrush(Qt.NoBrush)
            p.drawRect(int(cx - W - 20), int(cy - H//2), 16, H)
            p.drawRect(int(cx + W + 4), int(cy - H//2), 16, H)

        # Car body
        body = QRectF(cx - W//2, cy - H//2, W, H)
        body_path = QPainterPath()
        body_path.addRoundedRect(body, W * 0.25, W * 0.2)

        body_grad = QLinearGradient(cx - W//2, cy, cx + W//2, cy)
        body_grad.setColorAt(0, color.darker(180))
        body_grad.setColorAt(0.4, color.darker(130))
        body_grad.setColorAt(1, color.darker(200))
        p.fillPath(body_path, body_grad)

        # Outline
        pen = QPen(color if not blink else QColor("#ffffff"), 1.5 if not is_ego else 2.5)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawPath(body_path)

        # Windshield
        ws_h = H * 0.25
        ws_rect = QRectF(cx - W * 0.35, cy - H * 0.35, W * 0.7, ws_h)
        ws_path = QPainterPath()
        ws_path.addRoundedRect(ws_rect, 3, 3)
        ws_color = QColor(color)
        ws_color.setAlpha(60)
        p.fillPath(ws_path, ws_color)

        # Headlights / taillights
        hl_size = max(4, int(W * 0.18))
        hl_color = QColor("#ffffff") if not is_ego else QColor("#ffffaa")
        tl_color = QColor("#ff2222")
        p.setPen(Qt.NoPen)
        p.setBrush(hl_color if not is_ego else hl_color)
        p.drawEllipse(int(cx - W//2 + 2), int(cy - H//2 + 2), hl_size, hl_size)
        p.drawEllipse(int(cx + W//2 - hl_size - 2), int(cy - H//2 + 2), hl_size, hl_size)
        p.setBrush(tl_color)
        p.drawEllipse(int(cx - W//2 + 2), int(cy + H//2 - hl_size - 2), hl_size, hl_size)
        p.drawEllipse(int(cx + W//2 - hl_size - 2), int(cy + H//2 - hl_size - 2), hl_size, hl_size)

        # ACCIDENT overlay
        if vs and vs.event == "ACCIDENT":
            p.setPen(QPen(QColor("#ff1744"), 2.5))
            p.drawLine(int(cx - W//2 + 3), int(cy - H//2 + 3),
                       int(cx + W//2 - 3), int(cy + H//2 - 3))
            p.drawLine(int(cx + W//2 - 3), int(cy - H//2 + 3),
                       int(cx - W//2 + 3), int(cy + H//2 - 3))

        # Label below car
        lbl_font = QFont("Courier New", max(7, int(8 * scale)))
        lbl_font.setBold(True)
        p.setFont(lbl_font)
        p.setPen(color)
        lbl_y = int(cy + H//2 + 3 + max(7, int(8 * scale)))
        p.drawText(int(cx - 20), lbl_y, label[:4])

        # Speed label
        if vs and scale > 0.6:
            spd_font = QFont("Courier New", max(6, int(7 * scale)))
            p.setFont(spd_font)
            p.setPen(QColor(THEME["text_muted"]))
            p.drawText(int(cx - 20), lbl_y + 12, f"{int(vs.speed)}km/h")

    def mousePressEvent(self, event):
        if not self.tracker:
            return
        mx, my = event.x(), event.y()
        vehicles = self.tracker.get_vehicles()
        best_id, best_dist = None, 30
        for vid, vs in vehicles.items():
            sx, sy = self._world_to_screen(vs.lat_offset, vs.lon_offset)
            d = math.hypot(mx - sx, my - sy)
            if d < best_dist:
                best_dist = d
                best_id = vid
        if best_id:
            self.selected_id = best_id
            self.vehicleSelected.emit(vehicles[best_id])


# ─────────────────────────── ALERT PANEL ───────────────────────────

class AlertPanel(QWidget):
    MAX_ALERTS = 6

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(130)
        self._alerts = deque(maxlen=self.MAX_ALERTS)
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(8, 6, 8, 6)
        self._layout.setSpacing(3)

        hdr = QLabel("ACTIVE ALERTS")
        hdr.setFont(QFont("Courier New", 8, QFont.Bold))
        hdr.setStyleSheet(f"color: {THEME['text_dim']}; letter-spacing: 2px;")
        self._layout.addWidget(hdr)

        self._list_widget = QWidget()
        self._list_layout = QVBoxLayout(self._list_widget)
        self._list_layout.setContentsMargins(0, 0, 0, 0)
        self._list_layout.setSpacing(2)
        self._layout.addWidget(self._list_widget)
        self._layout.addStretch()

        self.setStyleSheet(f"""
            background: {THEME['panel_bg']};
            border-bottom: 1px solid {THEME['panel_border']};
        """)

    def push_alert(self, event, vid):
        if event not in ALERT_MSG:
            return
        color = EVENT_COLOR.get(event, THEME["text_primary"])
        msg = f"{ALERT_MSG[event]}  [{vid}]"
        ts = time.strftime("%H:%M:%S")
        self._alerts.appendleft((msg, color, ts))
        self._refresh()

    def _refresh(self):
        # clear
        while self._list_layout.count():
            item = self._list_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        for msg, color, ts in list(self._alerts)[:5]:
            row = QWidget()
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)
            rl.setSpacing(6)
            lbl = QLabel(msg)
            lbl.setFont(QFont("Courier New", 8))
            lbl.setStyleSheet(f"color: {color};")
            ts_lbl = QLabel(ts)
            ts_lbl.setFont(QFont("Courier New", 7))
            ts_lbl.setStyleSheet(f"color: {THEME['text_dim']};")
            ts_lbl.setAlignment(Qt.AlignRight)
            rl.addWidget(lbl, 1)
            rl.addWidget(ts_lbl)
            self._list_layout.addWidget(row)


# ─────────────────────────── INFO PANEL ───────────────────────────

class InfoPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(240)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(8)

        # Header
        hdr = QLabel("VEHICLE DETAILS")
        hdr.setFont(QFont("Courier New", 8, QFont.Bold))
        hdr.setStyleSheet(f"color: {THEME['text_dim']}; letter-spacing: 3px;")
        layout.addWidget(hdr)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"border: none; border-top: 1px solid {THEME['panel_border']};")
        layout.addWidget(sep)

        self._fields = {}
        fields = [
            ("ID",       "vid"),
            ("EVENT",    "event"),
            ("SPEED",    "speed"),
            ("DISTANCE", "distance"),
            ("LAT OFF",  "lat_off"),
            ("LON OFF",  "lon_off"),
            ("RSSI",     "rssi"),
            ("SEQ",      "seq"),
            ("TTL",      "ttl"),
        ]
        for label, key in fields:
            row = QWidget()
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)
            rl.setSpacing(4)
            lbl = QLabel(label)
            lbl.setFont(QFont("Courier New", 8))
            lbl.setStyleSheet(f"color: {THEME['text_muted']};")
            lbl.setFixedWidth(72)
            val = QLabel("—")
            val.setFont(QFont("Courier New", 9, QFont.Bold))
            val.setStyleSheet(f"color: {THEME['text_primary']};")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            rl.addWidget(lbl)
            rl.addWidget(val, 1)
            self._fields[key] = val
            layout.addWidget(row)

        # RSSI signal bar
        rssi_lbl = QLabel("SIGNAL")
        rssi_lbl.setFont(QFont("Courier New", 8))
        rssi_lbl.setStyleSheet(f"color: {THEME['text_muted']};")
        layout.addWidget(rssi_lbl)
        self._signal_bar = SignalBar()
        layout.addWidget(self._signal_bar)

        layout.addStretch()

        # Fleet stats
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet(f"border: none; border-top: 1px solid {THEME['panel_border']};")
        layout.addWidget(sep2)

        fleet_lbl = QLabel("FLEET STATS")
        fleet_lbl.setFont(QFont("Courier New", 8, QFont.Bold))
        fleet_lbl.setStyleSheet(f"color: {THEME['text_dim']}; letter-spacing: 3px;")
        layout.addWidget(fleet_lbl)

        self._fleet_count = QLabel("Vehicles: 0")
        self._fleet_count.setFont(QFont("Courier New", 9))
        self._fleet_count.setStyleSheet(f"color: {THEME['accent']};")
        layout.addWidget(self._fleet_count)

        self._pkt_rate = QLabel("Packets: 0/s")
        self._pkt_rate.setFont(QFont("Courier New", 9))
        self._pkt_rate.setStyleSheet(f"color: {THEME['text_muted']};")
        layout.addWidget(self._pkt_rate)

        self.setStyleSheet(f"""
            background: {THEME['panel_bg']};
            border-left: 1px solid {THEME['panel_border']};
        """)

    def update_vehicle(self, vs):
        if vs is None:
            for v in self._fields.values():
                v.setText("—")
            return
        color = EVENT_COLOR.get(vs.event, THEME["text_primary"])
        self._fields["vid"].setText(vs.vid)
        self._fields["event"].setText(vs.event)
        self._fields["event"].setStyleSheet(f"color: {color}; font-weight: bold;")
        self._fields["speed"].setText(f"{vs.speed:.1f} km/h")
        self._fields["distance"].setText(f"{vs.distance:.1f} m")
        self._fields["lat_off"].setText(f"{vs.lat_offset:+.1f} m")
        self._fields["lon_off"].setText(f"{vs.lon_offset:+.1f} m")
        self._fields["rssi"].setText(f"{vs.rssi} dBm")
        self._fields["seq"].setText(str(vs.seq))
        self._fields["ttl"].setText(str(vs.ttl))
        self._signal_bar.set_rssi(vs.rssi)

    def update_fleet(self, count, rate):
        self._fleet_count.setText(f"Vehicles: {count}")
        self._pkt_rate.setText(f"Packets: {rate}/s")


class SignalBar(QWidget):
    def __init__(self):
        super().__init__()
        self._rssi = -100
        self.setFixedHeight(22)

    def set_rssi(self, rssi):
        self._rssi = rssi
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Map RSSI (-90 dBm worst .. -40 dBm best) → 0..1
        strength = max(0, min(1.0, (self._rssi + 90) / 50))
        bars = 5
        bw = (w - (bars - 1) * 3) // bars
        for i in range(bars):
            filled = (i + 1) / bars <= strength
            bh = int(h * (0.3 + 0.7 * (i + 1) / bars))
            by = h - bh
            bx = i * (bw + 3)
            color = QColor(THEME["accent"]) if filled else QColor(THEME["panel_border"])
            p.fillRect(bx, by, bw, bh, color)
        p.end()


# ─────────────────────────── MAIN WINDOW ───────────────────────────

class MainWindow(QMainWindow):
    def __init__(self, port="/dev/ttyS0", baud=115200, demo=False):
        super().__init__()
        self.setWindowTitle("V2V ADAS Monitor")
        self.setMinimumSize(1100, 720)
        self._demo = demo

        self.tracker = VehicleTracker()
        self._pkt_count = 0
        self._pkt_last_ts = time.time()
        self._pkt_rate = 0

        self._setup_ui()
        self._setup_serial(port, baud)
        self._setup_timers()

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Top bar
        topbar = self._make_topbar()
        root.addWidget(topbar)

        # ── Alert panel
        self.alert_panel = AlertPanel()
        root.addWidget(self.alert_panel)

        # ── Main area
        main_row = QHBoxLayout()
        main_row.setContentsMargins(0, 0, 0, 0)
        main_row.setSpacing(0)

        self.road = RoadWidget()
        self.road.set_tracker(self.tracker)
        self.road.vehicleSelected.connect(self._on_vehicle_selected)
        main_row.addWidget(self.road, 1)

        self.info_panel = InfoPanel()
        main_row.addWidget(self.info_panel)

        root.addLayout(main_row, 1)

        self.setStyleSheet(f"background: {THEME['bg']};")

    def _make_topbar(self):
        bar = QWidget()
        bar.setFixedHeight(48)
        bar.setStyleSheet(f"""
            background: {THEME['panel_bg']};
            border-bottom: 1px solid {THEME['panel_border']};
        """)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 0, 16, 0)

        title = QLabel("V2V ADAS MONITOR")
        title.setFont(QFont("Courier New", 13, QFont.Bold))
        title.setStyleSheet(f"color: {THEME['accent']}; letter-spacing: 4px;")
        layout.addWidget(title)

        layout.addStretch()

        self._status_lbl = QLabel("● CONNECTING")
        self._status_lbl.setFont(QFont("Courier New", 9))
        self._status_lbl.setStyleSheet(f"color: {THEME['text_muted']};")
        layout.addWidget(self._status_lbl)

        self._time_lbl = QLabel("")
        self._time_lbl.setFont(QFont("Courier New", 9))
        self._time_lbl.setStyleSheet(f"color: {THEME['text_dim']};")
        layout.addWidget(self._time_lbl)

        return bar

    def _setup_serial(self, port, baud):
        if self._demo:
            self._status_lbl.setText("● DEMO MODE")
            self._status_lbl.setStyleSheet(f"color: {THEME['evt_hazard']};")
            return

        self.serial_reader = SerialReader(port, baud)
        self.serial_reader.data_received.connect(self._on_data)
        self.serial_reader.connection_status.connect(self._on_conn_status)
        self.serial_reader.start()

    def _setup_timers(self):
        # Repaint at ~30fps
        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._tick)
        self._render_timer.start(33)

        # Demo data injection
        if self._demo:
            self._demo_timer = QTimer(self)
            self._demo_timer.timeout.connect(self._inject_demo)
            self._demo_timer.start(600)

        # Packet rate calc
        self._rate_timer = QTimer(self)
        self._rate_timer.timeout.connect(self._calc_rate)
        self._rate_timer.start(1000)

    def _tick(self):
        now = time.strftime("%H:%M:%S")
        self._time_lbl.setText(now)
        self.tracker.expire_vehicles()
        self.road.update()
        count = len(self.tracker.get_vehicles())
        self.info_panel.update_fleet(count, self._pkt_rate)

    def _calc_rate(self):
        self._pkt_rate = self._pkt_count
        self._pkt_count = 0

    def _on_data(self, raw):
        try:
            msg = json.loads(raw.strip())
            changed = self.tracker.update(msg)
            self._pkt_count += 1
            if changed and msg.get("event") in ALERT_MSG:
                self.alert_panel.push_alert(msg["event"], msg.get("id", "?"))
        except Exception:
            pass

    def _on_conn_status(self, ok):
        if ok:
            self._status_lbl.setText("● CONNECTED")
            self._status_lbl.setStyleSheet(f"color: {THEME['evt_status']};")
        else:
            self._status_lbl.setText("● DISCONNECTED")
            self._status_lbl.setStyleSheet(f"color: {THEME['evt_brake']};")

    def _on_vehicle_selected(self, vs):
        self.info_panel.update_vehicle(vs)

    # ── Demo data ───────────────────────────────────────────────
    _demo_vehicles = [
        {"id": "V2", "lat": 13.085, "lon": 80.275, "spd": 45, "event": "STATUS",    "rssi": -62, "ttl": 2, "seq": 0, "priority": 3, "time": 0},
        {"id": "V3", "lat": 13.083, "lon": 80.272, "spd": 60, "event": "HAZARD",    "rssi": -75, "ttl": 2, "seq": 0, "priority": 2, "time": 0},
        {"id": "V4", "lat": 13.086, "lon": 80.278, "spd": 30, "event": "HARSH_BRAKE","rssi": -80,"ttl": 2, "seq": 0, "priority": 1, "time": 0},
        {"id": "V5", "lat": 13.081, "lon": 80.274, "spd": 55, "event": "BLIND_SPOT_ALERT","rssi":-55,"ttl":2,"seq":0,"priority":2,"time":0},
    ]
    _demo_idx = 0
    _demo_events = ["STATUS","HAZARD","HARSH_BRAKE","SUDDEN_STOP","BLIND_SPOT_ALERT","ACCIDENT"]
    _demo_ev_cycle = 0

    def _inject_demo(self):
        self._demo_ev_cycle += 1

        for v in self._demo_vehicles:
            v["lat"] += random.uniform(-0.00003, 0.00005)
            v["lon"] += random.uniform(-0.00002, 0.00002)
            v["spd"] = max(10, min(100, v["spd"] + random.uniform(-3, 3)))
            v["seq"] += 1
            v["time"] = int(time.time() * 1000)

        # Cycle one vehicle through dramatic events
        if self._demo_ev_cycle % 8 == 0:
            pick = self._demo_vehicles[self._demo_idx % len(self._demo_vehicles)]
            ev = self._demo_events[self._demo_ev_cycle // 8 % len(self._demo_events)]
            pick["event"] = ev
            self._demo_idx += 1

        for v in self._demo_vehicles:
            changed = self.tracker.update(v)
            self._pkt_count += 1
            if changed and v["event"] in ALERT_MSG:
                self.alert_panel.push_alert(v["event"], v["id"])


# ─────────────────────────── ENTRY ───────────────────────────

def main():
    import argparse
    ap = argparse.ArgumentParser(description="V2V ADAS Dashboard")
    ap.add_argument("--port",  default="/dev/ttyS0", help="UART port")
    ap.add_argument("--baud",  default=115200, type=int)
    ap.add_argument("--demo",  action="store_true", help="Run with demo data (no hardware)")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window,      QColor(THEME["bg"]))
    palette.setColor(QPalette.WindowText,  QColor(THEME["text_primary"]))
    palette.setColor(QPalette.Base,        QColor(THEME["panel_bg"]))
    palette.setColor(QPalette.Text,        QColor(THEME["text_primary"]))
    app.setPalette(palette)

    win = MainWindow(port=args.port, baud=args.baud, demo=args.demo)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
