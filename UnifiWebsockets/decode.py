import zlib
import struct
import json
from typing import Tuple, Union

# Constants
EVENT_PACKET_HEADER_SIZE = 8

# Packet Types
HEADER = 1
PAYLOAD = 2

# Payload Types
JSON_TYPE = 1
STRING_TYPE = 2
BUFFER_TYPE = 3

def decode_packet(packet: bytes) -> Union[Tuple[dict, Union[dict, str, bytes]], None]:
    try:
        data_offset = struct.unpack('>I', packet[ProtectEventPacketHeader.PAYLOAD_SIZE:ProtectEventPacketHeader.PAYLOAD_SIZE + 4])[0] + EVENT_PACKET_HEADER_SIZE
        
        if len(packet) != (data_offset + EVENT_PACKET_HEADER_SIZE + struct.unpack('>I', packet[data_offset + ProtectEventPacketHeader.PAYLOAD_SIZE:data_offset + ProtectEventPacketHeader.PAYLOAD_SIZE + 4])[0]):
            raise ValueError("Packet length doesn't match header information.")
    except Exception as error:
        print(f"Error decoding update packet: {error}")
        return None

    header_frame = decode_frame(packet[:data_offset], HEADER)
    payload_frame = decode_frame(packet[data_offset:], PAYLOAD)

    if not header_frame or not payload_frame:
        return None

    return header_frame, payload_frame

def decode_frame(packet: bytes, packet_type: int) -> Union[dict, str, bytes, None]:
    frame_type = packet[ProtectEventPacketHeader.TYPE]

    if packet_type != frame_type:
        return None

    payload_format = packet[ProtectEventPacketHeader.PAYLOAD_FORMAT]
    is_deflated = packet[ProtectEventPacketHeader.DEFLATED]

    if is_deflated:
        payload = zlib.decompress(packet[EVENT_PACKET_HEADER_SIZE:])
    else:
        payload = packet[EVENT_PACKET_HEADER_SIZE:]

    if frame_type == HEADER:
        if payload_format == JSON_TYPE:
            return json.loads(payload.decode('utf-8'))
        else:
            return None

    if payload_format == JSON_TYPE:
        return json.loads(payload.decode('utf-8'))
    elif payload_format == STRING_TYPE:
        return payload.decode('utf-8')
    elif payload_format == BUFFER_TYPE:
        return payload
    else:
        print(f"Unknown payload packet type received in the realtime events API: {payload_format}")
        return None

def decode_packet_from_mac(packet: bytes, mac: str) -> Union[Tuple[dict, Union[dict, str, bytes]], None]:
    try:
        data_offset = struct.unpack('>I', packet[ProtectEventPacketHeader.PAYLOAD_SIZE:ProtectEventPacketHeader.PAYLOAD_SIZE + 4])[0] + EVENT_PACKET_HEADER_SIZE
        
        if len(packet) != (data_offset + EVENT_PACKET_HEADER_SIZE + struct.unpack('>I', packet[data_offset + ProtectEventPacketHeader.PAYLOAD_SIZE:data_offset + ProtectEventPacketHeader.PAYLOAD_SIZE + 4])[0]):
            raise ValueError("Packet length doesn't match header information.")
    except Exception as error:
        print(f"Error decoding update packet: {error}")
        return None

    header_frame = decode_frame(packet[:data_offset], HEADER)
    
    if not header_frame:
        return None

    # Check if modelKey is 'camera' and mac matches
    if header_frame.get('modelKey') != 'camera' or header_frame.get('mac') != mac:
        return None

    payload_frame = decode_frame(packet[data_offset:], PAYLOAD)

    if not payload_frame:
        return None

    return header_frame, payload_frame

class ProtectEventPacketHeader:
    TYPE = 0
    PAYLOAD_FORMAT = 1
    DEFLATED = 2
    UNKNOWN = 3
    PAYLOAD_SIZE = 4

# Example usage:
# packet = b'...'  # binary string from websocket
# mac_address = 'XX:XX:XX:XX:XX:XX'  # Replace with the desired MAC address
# result = decode_packet_from_mac(packet, mac_address)
# if result:
#     action_frame, data_frame = result
#     print("Action Frame:", action_frame)
#     print("Data Frame:", data_frame)
# else:
#     print("No matching packet found.")