import serial
import pynmea2
import json
import time

GEOJSON_FILE = "gps_log.geojson"

# Initialize empty FeatureCollection
geojson = {
    "type": "FeatureCollection",
    "features": []
}

def save_geojson():
    with open(GEOJSON_FILE, "w") as f:
        json.dump(geojson, f, indent=2)

def add_point(lat, lon, alt, sats, fix, hdop):
    feature = {
        "type": "Feature",
        "geometry": {
            "type": "Point",
            "coordinates": [lon, lat, alt]  # GeoJSON uses [lon, lat, z]
        },
        "properties": {
            "timestamp": time.time(),
            "num_satellites": sats,
            "fix_quality": fix,
            "hdop": hdop
        }
    }
    geojson["features"].append(feature)
    save_geojson()
    print(f"Saved point lat={lat}, lon={lon}, sats={sats}")

def gps_loop():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    while True:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):  # NMEA fix data
                msg = pynmea2.parse(line)
                if msg.gps_qual > 0:  # valid fix
                    add_point(
                        msg.latitude,
                        msg.longitude,
                        float(msg.altitude) if msg.altitude else 0.0,
                        msg.num_sats,
                        msg.gps_qual,
                        msg.horizontal_dil
                    )
        except pynmea2.ParseError:
            continue
        except Exception as e:
            print(f"GPS read error: {e}")
        time.sleep(0.2)

if __name__ == "__main__":
    save_geojson()  # initialize file
    gps_loop()
