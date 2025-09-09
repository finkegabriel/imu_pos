import sqlite3
import time
import threading
from contextlib import closing
import gps_nema  # assuming you already have this to read from /dev/ttyUSB0

DB_PATH = "./db/sensor_log.db"

# Database setup
def setup_database():
    with closing(sqlite3.connect(DB_PATH)) as conn:
        with closing(conn.cursor()) as cur:
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
    def __init__(self, db_path=DB_PATH):
        self.db_path = db_path
        self.lock = threading.Lock()

    def log_gps(self, gps_data):
        with self.lock:
            with closing(sqlite3.connect(self.db_path)) as conn:
                with closing(conn.cursor()) as cur:
                    try:
                        current_time = time.time()
                        cur.execute("""
                            INSERT INTO gps_data
                            (timestamp, latitude, longitude, altitude, speed, fix_quality, num_satellites, hdop)
                            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                        """, (
                            current_time,
                            gps_data.latitude,
                            gps_data.longitude,
                            gps_data.altitude,
                            gps_data.speed,
                            gps_data.fix_quality,
                            gps_data.num_satellites,
                            gps_data.hdop
                        ))
                        conn.commit()
                        print(f"Logged GPS: lat={gps_data.latitude}, lon={gps_data.longitude}, sats={gps_data.num_satellites}")
                    except sqlite3.Error as e:
                        print(f"Database error: {e}")
                    except Exception as e:
                        print(f"Unexpected error logging GPS: {e}")

def gps_loop(db_logger):
    while True:
        try:
            gps_data = gps_nema.request_gps()  # your function should read/parses from /dev/ttyUSB0
            if gps_data and gps_data.fix_quality > 0:  # only log valid fixes
                db_logger.log_gps(gps_data)
        except Exception as e:
            print(f"GPS read error: {e}")
        time.sleep(1)  # log once per second


setup_database()
db_logger = DatabaseLogger()
gps_loop(db_logger)

