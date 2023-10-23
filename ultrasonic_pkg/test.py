import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)


while True:
    ser.flushInput()
    # read the data from arduino over the serial
    line = ser.readline().decode().strip()
    data = line.split(",")

    print(line)

    left_dist = float(data[0])
    right_dist = float(data[1])
    speed = float(data[2])
    switch = float(data[3])
    emergency = float(data[4])

    time.sleep(0.05)