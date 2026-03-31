"""
vehicle_tracker.py
Converts absolute GPS coordinates to metre offsets relative to ego vehicle.
Maintains per-vehicle state with expiry.
"""

import math
import time
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional

# Approximate ego vehicle GPS (updated if Pi has GPS; otherwise use first known ref)
# These will be overridden dynamically from the first packet or a GPS module.
EGO_LAT = 13.0820
EGO_LON = 80.2750

# Vehicle expires if no packet received within this many seconds
EXPIRY_S = 6.0


@dataclass
class VehicleState:
    vid: str
    lat: float
    lon: float
    speed: float
    event: str
    rssi: int
    seq: int
    ttl: int
    priority: int
    timestamp: float

    # Computed relative position (metres)
    lat_offset: float = 0.0   # positive = ahead
    lon_offset: float = 0.0   # positive = right
    distance: float   = 0.0

    prev_event: str = ""


def haversine_m(lat1, lon1, lat2, lon2):
    """Return distance in metres between two GPS coords."""
    R = 6_371_000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def lat_to_m(dlat):
    """Latitude difference in degrees → metres (approx)."""
    return dlat * 111_139


def lon_to_m(dlon, lat):
    """Longitude difference in degrees → metres at given latitude."""
    return dlon * 111_139 * math.cos(math.radians(lat))


class VehicleTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self._vehicles: Dict[str, VehicleState] = {}
        self._ego_lat = EGO_LAT
        self._ego_lon = EGO_LON

    def set_ego_position(self, lat, lon):
        with self._lock:
            self._ego_lat = lat
            self._ego_lon = lon

    def update(self, msg: dict) -> bool:
        """
        Ingest a parsed JSON message from ESP32.
        Returns True if event changed (for alert triggering).
        """
        vid = msg.get("id", "?")
        lat = float(msg.get("lat", 0))
        lon = float(msg.get("lon", 0))
        spd = float(msg.get("spd", 0))
        event = msg.get("event", "STATUS")
        rssi = int(msg.get("rssi", -99))
        seq  = int(msg.get("seq", 0))
        ttl  = int(msg.get("ttl", 0))
        prio = int(msg.get("priority", 3))

        with self._lock:
            prev_event = self._vehicles[vid].event if vid in self._vehicles else ""

            # metre offsets from ego
            lat_off = lat_to_m(lat - self._ego_lat)
            lon_off = lon_to_m(lon - self._ego_lon, self._ego_lat)
            dist    = haversine_m(self._ego_lat, self._ego_lon, lat, lon)

            vs = VehicleState(
                vid=vid, lat=lat, lon=lon, speed=spd,
                event=event, rssi=rssi, seq=seq, ttl=ttl, priority=prio,
                timestamp=time.time(),
                lat_offset=lat_off, lon_offset=lon_off, distance=dist,
                prev_event=prev_event,
            )
            self._vehicles[vid] = vs

        return event != prev_event

    def expire_vehicles(self):
        now = time.time()
        with self._lock:
            expired = [k for k, v in self._vehicles.items()
                       if now - v.timestamp > EXPIRY_S]
            for k in expired:
                del self._vehicles[k]

    def get_vehicles(self) -> Dict[str, VehicleState]:
        with self._lock:
            return dict(self._vehicles)

    def get_vehicle(self, vid) -> Optional[VehicleState]:
        with self._lock:
            return self._vehicles.get(vid)
