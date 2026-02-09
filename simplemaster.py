#####Setup Instructions
##   python3 -m venv venv
##   source venv/bin/activate
##   pip3 install python-can


import can
import time
import struct
from datetime import datetime

# Configuration
CAN_INTERFACE   = 'can0'   # Canbus interface in Linux
ACK_ID          = 0x400    # ACK_INTRO_ID from canbus_msg.h
EPOCH_ID        = 0x40C    # DATA_EPOCH_ID
KNOB_ID         = 0x518    # INPUT_ANALOG_KNOB_ID
TEMP_ID         = 0x51A    # NODE_CPU_TEMP_ID

# Track which nodes have been synced this session
synced_nodes = set()

def send_rtc_sync(bus, node_id):
    """Packs the current Unix time and sends it to the node."""
    now = int(time.time())
    # Payload: [4 bytes Timestamp][4 bytes Padding]
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
        # Initialize the bus
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        print(f"Master started on {CAN_INTERFACE}...")
        print(f"Monitoring for Knob ({hex(KNOB_ID)}) and Temp ({hex(TEMP_ID)}) data...")

        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue

            # --- Handle Introduction & Handshaking ---

            if msg.arbitration_id == EPOCH_ID:
                # 1. Unpack the payload
                # Bytes 0-3: Node ID (Big-Endian 'I')
                # Bytes 4-7: Unix Timestamp (Big-Endian 'I')
                # Format '>II' means Big-Endian, two Unsigned Integers
                try:
                    node_id, unix_ts = struct.unpack('>II', msg.data)
                    remote_node_id = struct.unpack('>I', msg.data[0:4])[0]
                    # 2. Convert Unix timestamp to a readable datetime object
                    dt_object = datetime.fromtimestamp(unix_ts)
                    
                    # 3. Pretty-print the result
                    timestamp_str = dt_object.strftime('%Y-%m-%d %H:%M:%S')
                    print(f"HEARTBEAT from Node 0x{node_id:08X} {timestamp_str}")
                    send_rtc_sync(bus, remote_node_id) # Send the node updated time as well
                    
                except struct.error:
                    print(f"Error: Received malformed 0x40C packet from ID 0x{msg.arbitration_id:X}")

            # Check if it is a Data Message or an Intro Message
            if msg.arbitration_id == KNOB_ID: # KNOB_ID
                node_id, knob_val = struct.unpack('>II', msg.data)
                print(f"[{time.strftime('%H:%M:%S')}] NODE: {hex(node_id)} | KNOB ADC: {knob_val}")
            
            elif msg.arbitration_id == TEMP_ID: # TEMP_ID
                # Unpack first 4 bytes as Int, second 4 bytes as Float
                node_id, celsius = struct.unpack('>If', msg.data)
                print(f"[{time.strftime('%H:%M:%S')}] NODE: {hex(node_id)} | CPU TEMP: {celsius:.2f} °C")

            # ACK any 0x700 series message
            if 0x700 <= msg.arbitration_id <= 0x7FF:
                # Generic Handshake for other 0x700 series (Introduction)
                if len(msg.data) >= 4:
                    remote_node_id = struct.unpack('>I', msg.data[0:4])[0]
                    print(f"[{time.strftime('%H:%M:%S')}] INTRO PKT: {hex(msg.arbitration_id)} from {hex(remote_node_id)}")
                    
                    # Send ACK_INTRO_ID (0x400) back to the node
                    ack_payload = struct.pack('>I', remote_node_id) + b'\x00\x00\x00\x00'
                    ack_msg = can.Message(
                        arbitration_id=ACK_ID,
                        data=ack_payload,
                        is_extended_id=False
                    )
                    bus.send(ack_msg)

                    # If this is the FIRST time seeing this node, send the RTC sync
                    if remote_node_id not in synced_nodes:
                        # Small delay to ensure the node processed the first ACK
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