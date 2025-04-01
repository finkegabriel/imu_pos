import asyncio
import websockets
import json
import time
from gpsdclient import GPSDClient  # Install with `pip install gpsdclient`

async def send_gps(websocket, path):
    with GPSDClient() as client:
        for result in client.json_stream():
            gps_data = {
                "latitude": result.get("lat"),
                "longitude": result.get("lon"),
                "timestamp": time.time()
            }
            await websocket.send(json.dumps(gps_data))

start_server = websockets.serve(send_gps, "0.0.0.0", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()