import serial
import json
import tkinter as tk

ser = serial.Serial('/dev/ttyUSB0', 115200)

root = tk.Tk()
root.title("V2V Dashboard")

labels = {}

fields = ["id", "event", "spd", "lat", "lon", "rssi", "seq"]

for i, field in enumerate(fields):
    tk.Label(root, text=field.upper(), font=("Arial", 12)).grid(row=i, column=0)
    labels[field] = tk.Label(root, text="--", font=("Arial", 12))
    labels[field].grid(row=i, column=1)

def update():
    if ser.in_waiting:
        line = ser.readline().decode().strip()
        try:
            data = json.loads(line)
            for key in fields:
                labels[key].config(text=str(data.get(key, "--")))
        except:
            pass

    root.after(100, update)

update()
root.mainloop()