import asyncio
import queue
import json
import requests
import websockets
import ssl
import re
import dotenv

# from UnifiWebsockets.decode import decode_packet


# REST login script to get the CSRF token
def get_token(base_url, username, password):
    login_url = f"{base_url}/api/auth/login"
    payload = {"username": username, "password": password}
    headers = {"Content-Type": "application/json"}
    try:
        session = requests.Session()
        response = session.post(login_url, json=payload, headers=headers, verify=False)
        response.raise_for_status()  # Raise an exception for HTTP errors
        return session.cookies
    except requests.exceptions.RequestException as e:
        print(f"Failed to obtain token: {e}")
        return None, None


# Function to connect to the websocket and print the event stream
async def listen_to_event_stream(ws_url, cookies, queue):
    cookie_header = "".join([f"{key}={value}" for key, value in cookies.items()])
    headers = {"Cookie": cookie_header}

    ssl_context = ssl._create_unverified_context()  # Create an unverified SSL context
    try:
        async with websockets.connect(
            ws_url, extra_headers=headers, ssl=ssl_context
        ) as websocket:
            while True:
                try:
                    message = await websocket.recv()
                    message = str(message)

                    queue.put(find_coords(message))
                    # print(find_coords(message))

                    # coords_begin = message.find('coord') + 7
                    # coords_end = message[coords_begin:].find(']')
                    # print(message[coords_begin:])

                    # print(decode_packet(message))
                except websockets.ConnectionClosed:
                    print("Connection closed")
                    break
    except Exception as e:
        print(f"WebSocket connection failed: {e}")


def find_coords(message: str) -> list[str]:
    coords_begin = [m.start() + 8 for m in re.finditer('"coord":', message)]
    coords_end = [m.start() + 1 for m in re.finditer('],"depth":', message)]
    coords = [
        json.loads(message[coords_begin[i] : coords_end[i]])
        for i in range(len(coords_begin))
    ]
    return coords


def run(q: queue.Queue) -> None:
    base_url = "https://172.22.114.176"  # Replace with your UniFi Protect base URL
    username = "engr-ugaif"  # Replace with your username
    password = dotenv.get_key(dotenv.find_dotenv(), "UNIFI_PASSWORD")
    ws_url = "wss://172.22.114.176/proxy/protect/ws/liveDetectTrack?camera=668daa1e019ce603e4002d31"  # Replace with your WebSocket URL

    cookies = get_token(base_url, username, password)
    asyncio.run(listen_to_event_stream(ws_url, cookies, q))
