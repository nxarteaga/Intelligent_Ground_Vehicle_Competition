# serial_communication.py

import serial
import argparse
import threading

ser = None  # Make `ser` a global variable

def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

def start_serial_communication(port):
    global ser
    ser = serial.Serial(port, baudrate=115200, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

def send_command(command):
    if ser:
        ser.write(command.encode() + b'\n')
    else:
        print("Serial connection not initialized.")
