import asyncio
import websockets
import json

async def receive_gps():
    async with websockets.connect("ws://<ANDROID_IP>:8765") as websocket:
        while True:
            gps_data = await websocket.recv()
            gps = json.loads(gps_data)
            print(f"Latitude: {gps['latitude']}, Longitude: {gps['longitude']}")

asyncio.run(receive_gps())
