#!/bin/bash
# ─────────────────────────────────────────────
#  V2V ADAS Dashboard — Raspberry Pi Setup
# ─────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║   V2V ADAS Dashboard — Setup & Launch    ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# ── 1. System dependencies ──────────────────────────────────────
echo "[1/4] Installing system packages..."
sudo apt-get update -qq
sudo apt-get install -y \
    python3-pyqt5 \
    python3-serial \
    python3-pip \
    libxcb-xinerama0 \
    --no-install-recommends

# ── 2. Python packages ──────────────────────────────────────────
echo "[2/4] Installing Python packages..."
pip3 install --break-system-packages -r requirements.txt 2>/dev/null || \
pip3 install -r requirements.txt

# ── 3. UART enable on Pi ────────────────────────────────────────
echo "[3/4] Checking UART configuration..."

UART_PORT="/dev/ttyS0"

# Try /dev/ttyAMA0 if ttyS0 doesn't exist
if [ ! -e "$UART_PORT" ]; then
    UART_PORT="/dev/ttyAMA0"
fi

if [ ! -e "$UART_PORT" ]; then
    echo "⚠  No UART port found. Enable UART in raspi-config → Interface Options → Serial."
    echo "   After enabling, reboot and run this script again."
    echo "   For now, launching in DEMO mode..."
    DEMO_FLAG="--demo"
else
    echo "✓  UART port: $UART_PORT"
    DEMO_FLAG=""
    # Add user to dialout group for serial access
    sudo usermod -aG dialout "$USER" 2>/dev/null || true
fi

# ── 4. Launch ───────────────────────────────────────────────────
echo "[4/4] Launching dashboard..."
echo ""

# Set display for Pi (use :0 if running from SSH with DISPLAY forwarded)
export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# Disable Qt warnings that are common on Pi
export QT_LOGGING_RULES="*.debug=false;qt.qpa.*=false"

python3 dashboard.py --port "$UART_PORT" --baud 115200 $DEMO_FLAG "$@"
