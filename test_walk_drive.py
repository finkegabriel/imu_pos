import numpy as np
import pyproj
from filterpy.kalman import KalmanFilter
import websockets
import asyncio
import json
import folium
from datetime import datetime

# UTM Projection Setup
utm_proj = pyproj.Proj(proj="utm", zone=12, ellps="WGS84", south=False)
wgs84 = pyproj.Proj(proj="latlong", datum="WGS84")

# Initial GPS position in UTM
initial_lat, initial_lon = 37.7749, -122.4194  # Example: San Francisco
initial_x, initial_y = pyproj.transform(wgs84, utm_proj, initial_lon, initial_lat)

# Store trajectory points
trajectory = []

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

def create_map(trajectory_points):
    # Create a map centered at the initial position
    m = folium.Map(location=[initial_lat, initial_lon], zoom_start=15)
    
    # Convert trajectory points to lat/lon and create a line
    path_points = []
    for x, y in trajectory_points:
        lon, lat = pyproj.transform(utm_proj, wgs84, x, y)
        path_points.append([lat, lon])
    
    # Add the trajectory line to the map
    folium.PolyLine(
        path_points,
        weight=3,
        color='red',
        opacity=0.8
    ).add_to(m)
    
    # Add markers for start and end points
    if path_points:
        folium.Marker(
            path_points[0],
            popup='Start',
            icon=folium.Icon(color='green')
        ).add_to(m)
        folium.Marker(
            path_points[-1],
            popup='Current',
            icon=folium.Icon(color='red')
        ).add_to(m)
    
    # Save the map
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    map_file = f"trajectory_map_{timestamp}.html"
    m.save(map_file)
    print(f"Map saved as {map_file}")

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
    
    # Store trajectory point
    trajectory.append((filtered_x, filtered_y))
    
    # Convert to lat/lon for display
    lon, lat = pyproj.transform(utm_proj, wgs84, filtered_x, filtered_y)
    print(f"Estimated Position: Lat={lat:.6f}, Lon={lon:.6f}")
    
    # Update map every 10 points
    if len(trajectory) % 10 == 0:
        create_map(trajectory)

async def connect_to_sensor():
    uri = "ws://10.62.110.115:8080/sensor/connect?type=android.sensor.accelerometer"
    
    async with websockets.connect(uri) as websocket:
        print("Connected to sensor websocket")
        
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                
                # Extract accelerometer data
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
    try:
        asyncio.run(main())
    finally:
        # Create final map when program exits
        if trajectory:
            create_map(trajectory)