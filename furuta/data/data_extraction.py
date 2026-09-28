import serial
import csv
import datetime
import os

PORT = "/dev/ttyACM0"
BAUD = 115200
OUT = "/home/davide/pendulum/furuta/data/data.csv"

ser = serial.Serial(PORT,BAUD, timeout=1.0)
with open(OUT, "a", newline="") as f:
    w = csv.writer(f)
    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            w.writerow([datetime.datetime.now().isoformat()] + line.split(","))
            f.flush
    except KeyboardInterrupt:
        print("Stopped. File Saved at: " + str(OUT))