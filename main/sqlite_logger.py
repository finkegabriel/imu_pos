import sqlite3
import time
import websocket
import json
import threading
from contextlib import closing
import gps_nema

host_name = "ws://192.168.8.183:8098"

# Database setup
def setup_database():
    with closing(sqlite3.connect("./db/sensor_log.db")) as conn:
        with closing(conn.cursor()) as cur:
            # Create IMU data table
            cur.execute("""
                CREATE TABLE IF NOT EXISTS imu_data (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL,
                    accel_x REAL,
                    accel_y REAL,
                    accel_z REAL,
                    gyro_x REAL,
                    gyro_y REAL,
                    gyro_z REAL
                )
            """)
            
            # Create GPS data table
            cur.execute("""
                CREATE TABLE IF NOT EXISTS gps_data (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL,
                    latitude REAL,
                    longitude REAL,
                    altitude REAL,
                    speed REAL,
                    fix_quality INTEGER,
                    num_satellites INTEGER,
                    hdop REAL
                )
            """)
            conn.commit()

class DatabaseLogger:
    def __init__(self, db_path="./db/sensor_log.db"):
        self.db_path = db_path
        self.lock = threading.Lock()
        
    def log_data(self, sensor_type, values):
        with self.lock:
            with closing(sqlite3.connect(self.db_path)) as conn:
                with closing(conn.cursor()) as cur:
                    try:
                        current_time = time.time()
                        # Get GPS data
                        gps_data = gps_nema.request_gps()
                        
                        if sensor_type == "gyroscope":
                            cur.execute("""
                                INSERT INTO imu_data (timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
                                VALUES (?, ?, ?, ?, ?, ?, ?)
                            """, (current_time, 0.0, 0.0, 0.0, values[0], values[1], values[2]))
                            
                            # GPS insert with actual data
                            cur.execute("""
                                INSERT INTO gps_data (timestamp, latitude, longitude, altitude, speed, fix_quality, num_satellites, hdop)
                                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                            """, (current_time, gps_data.latitude, gps_data.longitude, gps_data.altitude, 
                                 gps_data.speed, gps_data.fix_quality, gps_data.num_satellites, gps_data.hdop))
                        
                        elif sensor_type == "accelerometer":
                            cur.execute("""
                                INSERT INTO imu_data (timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
                                VALUES (?, ?, ?, ?, ?, ?, ?)
                            """, (current_time, values[0], values[1], values[2], 0.0, 0.0, 0.0))

                            # GPS insert with actual data
                            cur.execute("""
                                INSERT INTO gps_data (timestamp, latitude, longitude, altitude, speed, fix_quality, num_satellites, hdop)
                                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                            """, (current_time, gps_data.latitude, gps_data.longitude, gps_data.altitude, 
                                 gps_data.speed, gps_data.fix_quality, gps_data.num_satellites, gps_data.hdop))
                        
                        conn.commit()
                    except sqlite3.Error as e:
                        print(f"Database error: {e}")
                    except Exception as e:
                        print(f"Error getting GPS data: {e}")

# Initialize database and logger
setup_database()
db_logger = DatabaseLogger()

def create_message_handler(ws, message):
    sensor = ws.url.split(".")[-1]
    values = json.loads(message)['values']
    print("Received {} data: {}".format(sensor, values))
    db_logger.log_data(sensor, values)

def on_error(ws, error):
    print("error occurred ")
    
def on_close(ws, close_code, reason):
    print("connection closed : ")

def on_message(ws, message):
    print("[{}] Received:".format(ws.url))
    create_message_handler(ws,message)

def on_open(ws):
    print("Connection opened")

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
    # print(path['gyro'])
    # print(path['accel'])
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