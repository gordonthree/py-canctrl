#####Setup Instructions
##   python3 -m venv venv
##   source venv/bin/activate
##   pip3 install python-can


import can
import time
import struct
from datetime import datetime

# Configuration
CAN_INTERFACE   = 'can0'
ACK_ID          = 0x400
EPOCH_ID        = 0x40C
KNOB_ID         = 0x518
TEMP_ID         = 0x51A

synced_nodes = set()

def send_rtc_sync(bus, node_id):
    """Packs the current Unix time and sends it to the node."""
    now = int(time.time())
    # Payload: [4 bytes Node ID][4 bytes Timestamp]
    payload = struct.pack('>I', node_id) + struct.pack('>I', now)
    sync_msg = can.Message(
        arbitration_id=EPOCH_ID,
        data=payload,
        is_extended_id=False
    )
    bus.send(sync_msg)
    print(f"[{time.strftime('%H:%M:%S')}] RTC SYNC: Sent {now} to Node {hex(node_id)}")

def main():
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        print(f"Master started on {CAN_INTERFACE}...")

        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue

            # --- Handle Heartbeat / RTC Sync (8 Bytes) ---
            if msg.arbitration_id == EPOCH_ID:
                try:
                    # Heartbeat is still 8 bytes
                    node_id, unix_ts = struct.unpack('>II', msg.data)
                    dt_object = datetime.fromtimestamp(unix_ts)
                    print(f"HEARTBEAT from Node 0x{node_id:08X} {dt_object.strftime('%Y-%m-%d %H:%M:%S')}")
                    send_rtc_sync(bus, node_id)
                except struct.error:
                    print(f"Error: Malformed 0x40C packet (Len: {len(msg.data)})")

            # --- Handle Knob Data (7 Bytes) ---
            elif msg.arbitration_id == KNOB_ID:
                try:
                    # Bytes 0-3: Node ID (I)
                    # Bytes 4-5: Sensor Value (H - unsigned short)
                    # We only slice the first 6 bytes to avoid the struct.error
                    node_id, knob_val = struct.unpack('>IH', msg.data[0:6])
                    print(f"[{time.strftime('%H:%M:%S')}] NODE: {hex(node_id)} | KNOB ADC: {knob_val} mV")
                except struct.error:
                    print(f"Error: Malformed 0x518 packet (Len: {len(msg.data)})")

            # --- Handle Temp Data (8 Bytes) ---
            elif msg.arbitration_id == TEMP_ID:
                try:
                    # > : Big Endian
                    # I : 4-byte Unsigned Int (Node ID)
                    # f : 4-byte Float (CPU Temp)
                    # Total = 8 bytes
                    node_id, celsius = struct.unpack('>If', msg.data)
                    print(f"[{time.strftime('%H:%M:%S')}] NODE: {hex(node_id)} | CPU TEMP: {celsius:.2f} °C")
                except struct.error:
                    print(f"Error: Malformed 0x51A packet (Len: {len(msg.data)})")

            # --- Handle Intro/Handshaking (0x700 range) ---
            if 0x700 <= msg.arbitration_id <= 0x7FF:
                if len(msg.data) >= 4:
                    remote_node_id = struct.unpack('>I', msg.data[0:4])[0]
                    print(f"[{time.strftime('%H:%M:%S')}] INTRO PKT: {hex(msg.arbitration_id)} from {hex(remote_node_id)}")
                    
                    ack_payload = struct.pack('>I', remote_node_id) + b'\x00\x00\x00\x00'
                    ack_msg = can.Message(arbitration_id=ACK_ID, data=ack_payload, is_extended_id=False)
                    bus.send(ack_msg)

                    if remote_node_id not in synced_nodes:
                        time.sleep(0.01) 
                        send_rtc_sync(bus, remote_node_id)
                        synced_nodes.add(remote_node_id)

    except KeyboardInterrupt:
        print("\nStopping Master...")
    finally:
        if 'bus' in locals():
            bus.shutdown()

if __name__ == "__main__":
    main()