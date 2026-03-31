"""
serial_reader.py
QThread that reads newline-terminated JSON from UART (ESP32 Serial2 output).
Emits data_received(str) and connection_status(bool) signals.
"""

import serial
import time
from PyQt5.QtCore import QThread, pyqtSignal


class SerialReader(QThread):
    data_received    = pyqtSignal(str)
    connection_status = pyqtSignal(bool)

    def __init__(self, port="/dev/ttyS0", baud=115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._running = True
        self._ser = None

    def run(self):
        while self._running:
            try:
                self._ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baud,
                    timeout=1.0,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                self.connection_status.emit(True)
                self._read_loop()
            except serial.SerialException as e:
                self.connection_status.emit(False)
                time.sleep(2)   # retry after 2s

    def _read_loop(self):
        buf = b""
        while self._running:
            try:
                chunk = self._ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    decoded = line.strip().decode("utf-8", errors="ignore")
                    if decoded.startswith("{"):
                        self.data_received.emit(decoded)
            except serial.SerialException:
                self.connection_status.emit(False)
                break
            except Exception:
                continue

        if self._ser and self._ser.is_open:
            self._ser.close()

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.wait()
