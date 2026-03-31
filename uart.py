import serial
import json

ser = serial.Serial('/dev/ttyAMA0', 115200)

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()

    if line:
        try:
            data = json.loads(line)
            print("Vehicle:", data["id"])
            print("Event:", data["event"])
            print("Speed:", data["spd"])
            print("----")
        except:
            print("Raw:", line)