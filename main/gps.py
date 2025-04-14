import serial
from pynmeagps import NMEAReader
import time

port = '/dev/ttyACM0'  # Replace with your serial port
baud_rate = 9600
print(f"trying GPS module at {port}:{baud_rate}")

ser = serial.Serial(port, baud_rate)
time = time.sleep(2)

def request_gps():
    try:
    # Receive data
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            print(f'Received: {data}')
            return data
    except serial.SerialException as e:
        print(f"Error: {e}")

    finally:
    # Close the serial port
        ser.close()