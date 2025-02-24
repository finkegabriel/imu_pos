import numpy as np
import pyproj
from filterpy.kalman import KalmanFilter
import websockets
import asyncio
import json

# UTM Projection Setup
utm_proj = pyproj.Proj(proj="utm", zone=33, ellps="WGS84", south=False)

# Initial GPS position in UTM
initial_lat, initial_lon = 37.7749, -122.4194  # Example: San Francisco
initial_x, initial_y = pyproj.transform(pyproj.Proj(proj="latlong", datum="WGS84"), utm_proj, initial_lon, initial_lat)

# Kalman Filter Setup
kf = KalmanFilter(dim_x=4, dim_z=2)
kf.F = np.array([[1, 1, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
kf.H = np.eye(2, 4)
kf.P *= 1000
kf.R = np.eye(2) * 5
kf.Q = np.eye(4) * 0.1
kf.x = np.array([initial_x, 0, initial_y, 0])

# Initial state setup
dt = 1.0
velocity = np.zeros(2, dtype=np.float64)
position = np.array([initial_x, initial_y], dtype=np.float64)

async def process_imu_data(acc_x, acc_y):
    global velocity, position
    
    # Detect motion type
    speed = np.linalg.norm(velocity)
    
    if speed < 2:  # Walking detection
        print("Walking detected, applying Zero-Velocity Update")
        if np.linalg.norm([acc_x, acc_y]) < 0.05:  # Low acceleration -> stationary
            velocity = np.zeros(2, dtype=np.float64)
    
    elif speed > 5:  # Driving detection
        print("Driving detected, applying road constraints")
        kf.Q = np.eye(4) * 0.05
    
    # Update velocity with explicit float type
    velocity += np.array([acc_x * dt, acc_y * dt], dtype=np.float64)
    
    # Update position
    position += velocity * dt
    
    # Kalman Filter prediction & update
    kf.predict()
    kf.update(position)
    
    # Get filtered position
    filtered_x, _, filtered_y, _ = kf.x
    print(f"Estimated UTM Position: Easting={filtered_x:.2f}, Northing={filtered_y:.2f}")

async def connect_to_sensor():
    uri = "ws://10.62.110.115:8080/sensor/connect?type=android.sensor.accelerometer"
    async with websockets.connect(uri) as websocket:
        print("Connected to sensor websocket")
        
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                
                # Extract accelerometer data
                # Assuming the data comes in format {x: value, y: value, z: value}
                acc_x = float(data.get('x', 0))
                acc_y = float(data.get('y', 0))
                
                await process_imu_data(acc_x, acc_y)
                
            except websockets.exceptions.ConnectionClosed:
                print("Connection lost. Attempting to reconnect...")
                break
            except Exception as e:
                print(f"Error processing data: {e}")

async def main():
    while True:
        try:
            await connect_to_sensor()
        except Exception as e:
            print(f"Connection error: {e}")
            print("Retrying in 5 seconds...")
            await asyncio.sleep(5)

if __name__ == "__main__":
    asyncio.run(main())

