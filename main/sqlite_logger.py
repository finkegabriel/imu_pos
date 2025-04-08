import sqlite3
import time
import websocket
import json
import threading

conn = sqlite3.connect("./db/sensor_log.db")
cur = conn.cursor()

host_name = "ws://192.168.8.183:8098"

# Create a wrapper function to handle the path parameter
def create_message_handler(ws,message):
    sensor = ws.url.split(".")[len(ws.url.split("."))-1]
    if(sensor == "gyroscope"):
        sensor_val = []
        print("Connecting to sensor...")
        values = json.loads(message)['values']
        x = values[0]
        y = values[1]
        z = values[2]
        print("msg ", sensor_val)
        print("x = ", x, "y = ", y, "z = ", z)
            
        # Insert IMU sample
        cur.execute("""
                INSERT INTO imu_data (timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
                VALUES (?, ?, ?, ?, ?, ?, ?)""",
                (time.time(), 0.0, 0.0, 0.0, x, y, z))
        
        # Insert GPS sample
        cur.execute("""
            INSERT INTO gps_data (timestamp, latitude, longitude, altitude, speed, fix_quality, num_satellites, hdop)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
            (time.time(), 37.4219983, -122.084, 15.0, 1.4, 1, 7, 0.9))

    if(sensor == "accelerometer"):
        sensor_val = []
        print("Connecting to sensor...")
        values = json.loads(message)['values']
        x = values[0]
        y = values[1]
        z = values[2]
        print("msg ", sensor_val)
        print("x = ", x, "y = ", y, "z = ", z)
            
        # Insert IMU sample
        cur.execute("""
                INSERT INTO imu_data (timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
                VALUES (?, ?, ?, ?, ?, ?, ?)""",
                (time.time(), x, y, z,0.0, 0.0, 0.0))
        
        # Insert GPS sample
        cur.execute("""
            INSERT INTO gps_data (timestamp, latitude, longitude, altitude, speed, fix_quality, num_satellites, hdop)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
            (time.time(), 37.4219983, -122.084, 15.0, 1.4, 1, 7, 0.9))
    

def on_error(ws, error):
    print("error occurred ")
    
def on_close(ws, close_code, reason):
    print("connection closed : ")

def on_message(ws, message):
    print(f"[{ws.url}] Received:")
    create_message_handler(ws,message)
def on_open(ws):
    print(f"Connection opened")

def create_ws(url):
    ws = websocket.WebSocketApp(
        url,
        on_message=on_message,
        on_open=on_open
    )
    ws.url = url  # manually store URL on the object for identification
    ws.run_forever()

def connect(path):
    threads = []
    print(path['gyro'])
    print(path['accel'])
    thread_gyro = threading.Thread(target=create_ws, args=(path['gyro'],))
    thread_gyro.start()
    thread_accel = threading.Thread(target=create_ws, args=(path['accel'],))
    thread_accel.start()
    threads.append(thread_gyro)
    threads.append(thread_accel)

    for t in threads:
        t.join()

connect({
    "gyro": f"{host_name}/sensor/connect?type=android.sensor.gyroscope",
    "accel": f"{host_name}/sensor/connect?type=android.sensor.accelerometer"
})
conn.commit()
conn.close()