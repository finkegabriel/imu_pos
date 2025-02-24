import numpy as np
import pyproj
from filterpy.kalman import KalmanFilter
import websockets
import asyncio
import json
import folium
from datetime import datetime
from uszipcode import SearchEngine
from shapely.geometry import Polygon
import requests

def get_zipcode_info(zipcode):
    """Get the center coordinates and boundary for a given zipcode"""
    search = SearchEngine()
    zip_info = search.by_zipcode(zipcode)
    
    if not zip_info or not zip_info.lat or not zip_info.lng:
        raise ValueError(f"Could not find valid coordinates for zipcode {zipcode}")
    
    return {
        'lat': zip_info.lat,
        'lon': zip_info.lng,
        'bounds': zip_info.bounds
    }

def get_zipcode_boundary(zipcode):
    """Get the boundary polygon for a given zipcode"""
    search = SearchEngine()
    zip_info = search.by_zipcode(zipcode)
    
    if not zip_info.bounds:
        print(f"Could not find boundary for zipcode {zipcode}")
        return None
    
    # Create a simple polygon from the bounds
    min_lat, min_lon, max_lat, max_lon = zip_info.bounds
    
    # Convert boundary coordinates to UTM
    corners = [
        transformer.transform(min_lon, min_lat),
        transformer.transform(min_lon, max_lat),
        transformer.transform(max_lon, max_lat),
        transformer.transform(max_lon, min_lat),
        transformer.transform(min_lon, min_lat)  # Close the polygon
    ]
    
    boundary = Polygon(corners)
    return boundary

def create_map(trajectory_points, zipcode=None):
    # Create a map centered at the initial position
    m = folium.Map(location=[initial_lat, initial_lon], zoom_start=15)
    
    # Add zipcode boundary if provided
    if zipcode:
        boundary = get_zipcode_boundary(zipcode)
        if boundary:
            # Convert boundary to list of coordinates
            boundary_coords = list(boundary.exterior.coords)
            folium.Polygon(
                locations=[[lat, lon] for lon, lat in boundary_coords],
                color='blue',
                fill=True,
                fillColor='blue',
                fillOpacity=0.2,
                popup=f'Zipcode {zipcode} boundary'
            ).add_to(m)
    
    # Convert trajectory points to lat/lon and create a line
    path_points = []
    for x, y in trajectory_points:
        # Convert UTM coordinates back to lat/lon
        lon, lat = transformer_back.transform(x, y)
        path_points.append([lat, lon])
    
    # Add the trajectory line to the map
    if path_points:
        folium.PolyLine(
            path_points,
            weight=3,
            color='red',
            opacity=0.8
        ).add_to(m)
        
        # Add markers for start and end points
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

async def process_imu_data(acc_x, acc_y, zipcode):
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
    
    # Convert UTM to lat/lon for display
    lon, lat = transformer_back.transform(filtered_x, filtered_y)
    print(f"Estimated Position: Lat={lat:.6f}, Lon={lon:.6f}")
    print(f"UTM Position: X={filtered_x:.2f}, Y={filtered_y:.2f}")
    
    # Update map every 10 points
    if len(trajectory) % 10 == 0:
        create_map(trajectory, zipcode)

async def connect_to_sensor(zipcode):
    uri = "ws://10.62.110.115:8080/sensor/connect?type=android.sensor.accelerometer"
    
    async with websockets.connect(uri) as websocket:
        print("Connected to sensor websocket")
        
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                
                # Extract accelerometer data
                # Check if data contains the actual values we need
                if isinstance(data, dict) and all(key in data for key in ['x', 'y', 'z']):
                    try:
                        acc_x = float(data['x'])
                        acc_y = float(data['y'])
                        await process_imu_data(acc_x, acc_y, zipcode)
                    except (ValueError, TypeError) as e:
                        print(f"Error parsing accelerometer values: {e}")
                        print(f"Received data: {data}")
                else:
                    print(f"Unexpected data format: {data}")
                
            except websockets.exceptions.ConnectionClosed:
                print("Connection lost. Attempting to reconnect...")
                break
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}")
                print(f"Received message: {message}")
            except Exception as e:
                print(f"Error processing data: {e}")
                print(f"Received message: {message}")

async def main():
    # Get zipcode from user and initialize coordinates
    zipcode = input("Enter your zipcode: ")
    try:
        zip_info = get_zipcode_info(zipcode)
        
        # Set up global variables
        global initial_lat, initial_lon, initial_x, initial_y, transformer, transformer_back, kf, velocity, position, dt
        
        initial_lat = zip_info['lat']
        initial_lon = zip_info['lon']
        
        # UTM Projection Setup
        wgs84 = pyproj.CRS('EPSG:4326')  # Standard GPS coordinates
        utm_zone = int((initial_lon + 180) / 6) + 1  # Calculate UTM zone from longitude
        utm = pyproj.CRS(f"+proj=utm +zone={utm_zone} +datum=WGS84")
        transformer = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)
        transformer_back = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True)
        
        # Convert initial position to UTM
        initial_x, initial_y = transformer.transform(initial_lon, initial_lat)
        
        # Initialize Kalman Filter
        kf = KalmanFilter(dim_x=4, dim_z=2)
        kf.F = np.array([[1, 1, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
        kf.H = np.eye(2, 4)
        kf.P *= 1000
        kf.R = np.eye(2) * 5
        kf.Q = np.eye(4) * 0.1
        kf.x = np.array([initial_x, 0, initial_y, 0])
        
        # Initial state setup
        velocity = np.zeros(2, dtype=np.float64)
        position = np.array([initial_x, initial_y], dtype=np.float64)
        
        print(f"Starting position: Lat={initial_lat:.6f}, Lon={initial_lon:.6f}")
        print(f"UTM Zone: {utm_zone}")
        print(f"UTM Position: X={initial_x:.2f}, Y={initial_y:.2f}")
        
        # Main loop
        while True:
            try:
                # Check for 'q' input
                if input("Press 'q' to quit: ").lower() == 'q':
                    print("Quitting...")
                    break
                await connect_to_sensor(zipcode)
            except Exception as e:
                print(f"Connection error: {e}")
                print("Retrying in 5 seconds...")
                await asyncio.sleep(5)
                
    except ValueError as e:
        print(f"Error: {e}")
        return
    except Exception as e:
        print(f"Unexpected error: {e}")
        return

# Initialize empty lists and variables at module level
trajectory = []
initial_lat = None
initial_lon = None
initial_x = None
initial_y = None
transformer = None
transformer_back = None
kf = None
velocity = None
position = None
dt = 1.0
zipcode = None

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        # Create final map when program exits
        if trajectory:
            create_map(trajectory, zipcode)