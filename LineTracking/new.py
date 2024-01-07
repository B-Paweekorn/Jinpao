import serial
import time

arduino = serial.Serial(port='COM13', baudrate=250000, timeout=0.01)

def send_data(x, y, z):
    command = f"{x:.2f};{y:.2f};{z:.2f}\n"
    arduino.write(bytes(command, 'utf-8'))

def read_response():
    # time.sleep(0.1)
    while arduino.in_waiting:
        print(arduino.readline().decode().strip())

# Test sending a command
# while True:
#     send_data(0, 0, 0)
#     # read_response()

# send_data(0.2, 0, 0)
# send_data(0,0,0)