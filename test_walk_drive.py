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
from contextlib import contextmanager

# Add this function to manage SearchEngine instances
@contextmanager
def get_search_engine():
    search = SearchEngine()
    try:
        yield search
    finally:
        try:
            search.close()
        except:
            pass

def get_zipcode_info(zipcode):
    """Get the center coordinates and boundary for a given zipcode"""
    with get_search_engine() as search:
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
    with get_search_engine() as search:
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
    global velocity, position, dt, kf
    
    try:
        # Convert inputs to float64 explicitly
        acc_x = np.float64(acc_x)
        acc_y = np.float64(acc_y)
        
        # Initialize position if None
        if position is None:
            position = np.array([initial_x, initial_y], dtype=np.float64)
        
        # Ensure position is a numpy array
        position = np.asarray(position, dtype=np.float64)
        
        # Detect motion type
        speed = np.float64(np.linalg.norm(velocity))
        
        if speed < 2:  # Walking detection
            if np.linalg.norm([acc_x, acc_y]) < 0.05:  # Low acceleration -> stationary
                velocity = np.zeros(2, dtype=np.float64)
                print("Stationary")
            else:
                print("Walking")
        
        elif speed > 5:  # Driving detection
            print("Driving")
            kf.Q = np.eye(4) * 0.05
        
        # Update velocity and position with explicit float type
        velocity = velocity + np.array([acc_x * dt, acc_y * dt], dtype=np.float64)
        position = position + velocity * dt
        
        # Kalman Filter prediction & update
        kf.predict()
        measurement = np.array([position[0], position[1]], dtype=np.float64)
        kf.update(measurement)
        
        # Get filtered position
        filtered_state = kf.x.astype(np.float64)
        filtered_x, _, filtered_y, _ = filtered_state
        
        # Store trajectory point
        trajectory.append((filtered_x, filtered_y))
        
        # Convert UTM to lat/lon for display
        lon, lat = transformer_back.transform(filtered_x, filtered_y)
        print(f"Position: Lat={lat:.6f}, Lon={lon:.6f}")
        
    except Exception as e:
        print(f"Error in process_imu_data: {e}")
        import traceback
        traceback.print_exc()

async def connect_to_sensor(zipcode, stop_event):
    uri = "ws://10.62.110.115:8080/sensor/connect?type=android.sensor.accelerometer"
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to sensor websocket")
            
            while not stop_event.is_set():
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=0.1)
                    data = json.loads(message)
                    
                    if isinstance(data, dict) and 'values' in data and isinstance(data['values'], list):
                        try:
                            # Debug prints for raw data
                            print(f"Raw values types: {[type(v) for v in data['values']]}")
                            print(f"Raw values: {data['values']}")
                            
                            # Convert string values to float if needed
                            values = []
                            for v in data['values']:
                                if isinstance(v, str):
                                    v = v.strip()  # Remove any whitespace
                                values.append(float(v))
                            
                            acc_x = np.float64(values[0])
                            acc_y = np.float64(values[1])
                            
                            print(f"Converted values - x: {acc_x} ({type(acc_x)}), y: {acc_y} ({type(acc_y)})")
                            
                            await process_imu_data(acc_x, acc_y, zipcode)
                        except (ValueError, TypeError, IndexError) as e:
                            print(f"Error parsing accelerometer values: {e}")
                            print(f"Problem value types: {[type(v) for v in data['values']]}")
                            print(f"Problem values: {data['values']}")
                    else:
                        print(f"Unexpected data format: {data}")
                
                except asyncio.TimeoutError:
                    continue
                except websockets.exceptions.ConnectionClosed:
                    print("Connection lost. Attempting to reconnect...")
                    break
                except Exception as e:
                    print(f"Error processing data: {e}")
    except Exception as e:
        if not stop_event.is_set():
            print(f"Connection error: {e}")
    finally:
        # Ensure websocket is closed properly
        if 'websocket' in locals() and not websocket.closed:
            await websocket.close()

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
velocity = np.zeros(2, dtype=np.float64)  # Initialize as numpy array
position = None
dt = np.float64(1.0)  # Initialize as numpy float64
zipcode = None

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\nCtrl+C detected, creating final map...")
        if trajectory:
            create_map(trajectory, zipcode)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            tasks = asyncio.all_tasks(loop)
            for task in tasks:
                task.cancel()
            
            loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        except:
            pass
        finally:
            loop.close()