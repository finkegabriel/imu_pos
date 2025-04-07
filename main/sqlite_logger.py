import sqlite3
import time
import websocket
import json

conn = sqlite3.connect("./db/sensor_log.db")
cur = conn.cursor()

# Create a wrapper function to handle the path parameter
def create_message_handler(path):        
    def message(ws, message):
        sensor_val = []
        print("Connecting to Gyroscope sensor...")
        values = json.loads(message)['values']
        x = values[0]
        y = values[1]
        z = values[2]
        print("msg ", sensor_val)
        print("x = ", x, "y = ", y, "z = ", z)
        
        #     Insert IMU sample
        cur.execute("""
            INSERT INTO imu_data (timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
            VALUES (?, ?, ?, ?, ?, ?, ?)""",
            (time.time(), 0.1, 0.0, 9.8, x, y, z))

            # Insert GPS sample
            # cur.execute("""
            # INSERT INTO gps_data (timestamp, latitude, longitude, altitude, speed, fix_quality, num_satellites, hdop)
            # VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
            # (time.time(), 37.4219983, -122.084, 15.0, 1.4, 1, 7, 0.9))
    return message

def on_error(ws, error):
    print("error occurred ", error)
    
def on_close(ws, close_code, reason):
    print("connection closed : ", reason)
    
def on_open(ws):
    print("connected")
    
def connect(url, path):
    # Ensure the database is created and ready
    ws = websocket.WebSocketApp(url,
                              on_open=on_open,
                              on_message=create_message_handler(path),
                              on_error=on_error,
                              on_close=on_close)
    ws.run_forever()

connect("ws://192.168.8.183:8098/sensor/connect?type=android.sensor.gyroscope", "gyro")
# connect("ws://10.12.36.194:8098/sensor/connect?type=android.sensor.accelerometer", "accel") 
conn.commit()
conn.close()