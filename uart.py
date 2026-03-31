import serial

# Open UART
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

print("Listening to ESP32...\n")

while True:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line:
            print("Received:", line)

    except KeyboardInterrupt:
        print("\nExiting...")
        break