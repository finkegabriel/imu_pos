import numpy as np
import pyproj
from filterpy.kalman import KalmanFilter
import websockets
import asyncio
import json
import folium
from datetime import datetime
from uszipcode import SearchEngine
from shapely.geometry import Polygon, Point
import requests
from shapely.ops import nearest_points
import sys

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
    
    # Create a polygon from the bounds
    min_lat, min_lon, max_lat, max_lon = zip_info.bounds
    
    # Create boundary points with more detail
    lat_points = np.linspace(min_lat, max_lat, 10)
    lon_points = np.linspace(min_lon, max_lon, 10)
    
    boundary_points = []
    # Create perimeter points
    for lat in lat_points:
        boundary_points.append((min_lon, lat))
    for lon in lon_points:
        boundary_points.append((lon, max_lat))
    for lat in reversed(lat_points):
        boundary_points.append((max_lon, lat))
    for lon in reversed(lon_points):
        boundary_points.append((lon, min_lat))
    
    # Convert all points to UTM
    utm_points = [transformer.transform(lon, lat) for lon, lat in boundary_points]
    
    boundary = Polygon(utm_points)
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
    
    # Check if point is within zipcode boundary
    boundary = get_zipcode_boundary(zipcode)
    point_utm = (filtered_x, filtered_y)
    if boundary and not boundary.contains(Point(point_utm)):
        print("Warning: Position outside zipcode boundary!")
        # Optionally adjust position to stay within boundary
        nearest_point = nearest_points(Point(point_utm), boundary)[1]
        filtered_x, filtered_y = nearest_point.x, nearest_point.y
    
    # Store trajectory point
    trajectory.append((filtered_x, filtered_y))
    
    # Convert UTM to lat/lon for display
    lon, lat = transformer_back.transform(filtered_x, filtered_y)
    print(f"Estimated Position: Lat={lat:.6f}, Lon={lon:.6f}")
    print(f"UTM Position: X={filtered_x:.2f}, Y={filtered_y:.2f}")
    
    # Get route matching data from OpenStreetMap
    if len(trajectory) > 1:
        try:
            # Get the last two points for route matching
            last_point = transformer_back.transform(trajectory[-1][0], trajectory[-1][1])
            prev_point = transformer_back.transform(trajectory[-2][0], trajectory[-2][1])
            
            # Query OpenStreetMap for nearby roads
            overpass_url = "http://overpass-api.de/api/interpreter"
            query = f"""
                [out:json];
                way["highway"](around:50,{lat},{lon});
                (._;>;);
                out body;
            """
            response = requests.post(overpass_url, data=query)
            data = response.json()
            
            if 'elements' in data:
                roads = [elem for elem in data['elements'] if elem.get('type') == 'way']
                if roads:
                    nearest_road = min(roads, key=lambda r: 
                        abs(r.get('lat', lat) - lat) + abs(r.get('lon', lon) - lon))
                    print(f"Matched to road: {nearest_road.get('tags', {}).get('name', 'unnamed road')}")
        except Exception as e:
            print(f"Route matching error: {e}")
    
    # Update map every 10 points
    if len(trajectory) % 10 == 0:
        create_map(trajectory, zipcode)

async def connect_to_sensor(zipcode, stop_event):
    uri = "ws://10.62.110.115:8080/sensor/connect?type=android.sensor.accelerometer"
    
    async with websockets.connect(uri) as websocket:
        print("Connected to sensor websocket")
        
        while not stop_event.is_set():
            try:
                # Add timeout to websocket.recv()
                message = await asyncio.wait_for(websocket.recv(), timeout=0.1)
                data = json.loads(message)
                
                if isinstance(data, dict) and all(key in data for key in ['x', 'y', 'z']):
                    try:
                        acc_x = float(data['x'])
                        acc_y = float(data['y'])
                        await process_imu_data(acc_x, acc_y, zipcode)
                    except (ValueError, TypeError) as e:
                        print(f"Error parsing accelerometer values: {e}")
                else:
                    print(f"Unexpected data format: {data}")
                
            except asyncio.TimeoutError:
                # Timeout is expected, continue checking stop_event
                continue
            except websockets.exceptions.ConnectionClosed:
                print("Connection lost. Attempting to reconnect...")
                break
            except Exception as e:
                print(f"Error processing data: {e}")

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
        
        # Create an event to signal when to stop
        stop_event = asyncio.Event()
        
        async def input_handler():
            while True:
                # Use asyncio.to_thread for blocking input operation
                user_input = await asyncio.to_thread(input, "Press 'q' to quit and show map: ")
                if user_input.lower() == 'q':
                    stop_event.set()
                    return
        
        async def sensor_handler():
            while not stop_event.is_set():
                try:
                    await connect_to_sensor(zipcode, stop_event)
                except Exception as e:
                    if not stop_event.is_set():
                        print(f"Sensor connection error: {e}")
                        await asyncio.sleep(1)
        
        print("Press 'q' and Enter to quit and show map")
        # Run both handlers concurrently
        await asyncio.gather(
            input_handler(),
            sensor_handler(),
            return_exceptions=True
        )
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
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Creating final map...")
        # Ensure there's an event loop for the final map creation
        if trajectory:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                create_map(trajectory, zipcode)
            finally:
                loop.close()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Only try to create map if not already created by KeyboardInterrupt handler
        if trajectory and not isinstance(sys.exc_info()[1], KeyboardInterrupt):
            try:
                create_map(trajectory, zipcode)
            except Exception as e:
                print(f"Error creating final map: {e}")