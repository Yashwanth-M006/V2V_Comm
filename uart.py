#!/usr/bin/env python3
"""
v2v_parser.py  —  Simple terminal display for V2V ESP-NOW data
Run: python3 v2v_parser.py --port /dev/ttyS0
"""

import serial
import json
import argparse
import time

# ANSI colors
R  = "\033[91m"   # red
Y  = "\033[93m"   # yellow
G  = "\033[92m"   # green
B  = "\033[94m"   # blue
O  = "\033[38;5;208m"  # orange
W  = "\033[97m"   # white
DIM = "\033[2m"
RST = "\033[0m"
BOLD = "\033[1m"
CLR  = "\033[2J\033[H"   # clear screen

EVENT_COLOR = {
    "STATUS":           G,
    "HAZARD":           Y,
    "HARSH_BRAKE":      R,
    "SUDDEN_STOP":      O,
    "BLIND_SPOT_ALERT": B,
    "ACCIDENT":         R,
}

def color_event(event):
    c = EVENT_COLOR.get(event, W)
    return f"{c}{BOLD}{event}{RST}"

def rssi_bar(rssi):
    # -40 best, -90 worst
    strength = max(0, min(5, int((rssi + 90) / 10)))
    filled = "█" * strength
    empty  = "░" * (5 - strength)
    return f"{G}{filled}{DIM}{empty}{RST} {rssi}dBm"

def print_header():
    print(f"{BOLD}{W}{'─'*60}{RST}")
    print(f"{BOLD}{W}  V2V TERMINAL MONITOR{RST}  {DIM}{time.strftime('%H:%M:%S')}{RST}")
    print(f"{BOLD}{W}{'─'*60}{RST}")
    print(f"{DIM}  {'ID':<6} {'EVENT':<18} {'SPD':>6} {'LAT':>10} {'LON':>10} {'SIGNAL'}{RST}")
    print(f"{DIM}{'─'*60}{RST}")

def parse_line(line):
    try:
        return json.loads(line.strip())
    except json.JSONDecodeError:
        return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",  default="/dev/ttyS0")
    ap.add_argument("--baud",  default=115200, type=int)
    ap.add_argument("--raw",   action="store_true", help="Also print raw JSON")
    args = ap.parse_args()

    print(f"{G}Opening {args.port} at {args.baud} baud...{RST}")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=2)
    except serial.SerialException as e:
        print(f"{R}Error: {e}{RST}")
        return

    print(f"{G}Connected. Waiting for data...{RST}\n")
    print_header()

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore")
            if not raw.strip():
                continue

            if args.raw:
                print(f"{DIM}{raw.strip()}{RST}")

            msg = parse_line(raw)
            if not msg:
                continue

            vid    = msg.get("id",    "?")
            event  = msg.get("event", "?")
            spd    = msg.get("spd",   0)
            lat    = msg.get("lat",   0)
            lon    = msg.get("lon",   0)
            rssi   = msg.get("rssi",  -99)
            seq    = msg.get("seq",   0)

            ev_str = color_event(event)
            bar    = rssi_bar(rssi)

            print(f"  {BOLD}{W}{vid:<6}{RST} {ev_str:<28} {W}{spd:>3.0f}km/h{RST}  "
                  f"{DIM}{lat:.5f}  {lon:.5f}{RST}  {bar}  {DIM}#{seq}{RST}")

            # Extra alert line for critical events
            if event == "ACCIDENT":
                print(f"  {R}{BOLD}  ⚠  ACCIDENT DETECTED from {vid} — reduce speed!{RST}")
            elif event == "HARSH_BRAKE":
                print(f"  {R}  ⚠  {vid} braking hard ahead{RST}")
            elif event == "SUDDEN_STOP":
                print(f"  {O}  ■  {vid} sudden stop — collision risk{RST}")

        except KeyboardInterrupt:
            print(f"\n{Y}Stopped.{RST}")
            ser.close()
            break
        except Exception as e:
            print(f"{R}Error: {e}{RST}")

if __name__ == "__main__":
    main()