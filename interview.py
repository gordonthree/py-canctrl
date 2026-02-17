#!/usr/bin/env python3
import time
import struct
import threading
import queue
from collections import deque
from datetime import datetime
import argparse
import sys

try:
    import can
except ImportError:
    print("Please install python-can: pip install python-can")
    sys.exit(1)

# --- Definitions from your timeline/main.cpp ---
ACK_INTRO_ID       = 0x400  # Master -> Node: [ID_32]
REQ_NODE_INTRO_ID  = 0x401  # Master -> Bus: [0,0,0,0]
EPOCH_ID           = 0x40C  # Heartbeat / Time Sync
KNOB_ID            = 0x518  # Telemetry
TEMP_ID            = 0x51A  # Telemetry

# Example Node Type IDs (0x700-0x7FF range)
IFACE_ARGB_MULTI_ID = 0x79C 

class CANState:
    def __init__(self):
        self.nodes = {}
        self.lock = threading.Lock()

    def get_node(self, node_id):
        with self.lock:
            if node_id not in self.nodes:
                self.nodes[node_id] = {
                    'heartbeat': None, 'knob': None, 'temp': None, 'last': None,
                    'sub_mod_cnt': 0, 'reported_crc': 0, 'subs': {}, 'interview_ptr': 0
                }
            self.nodes[node_id]['last'] = time.time()
            return self.nodes[node_id]

class App:
    def __init__(self, bus, iface):
        self.bus = bus
        self.iface = iface
        self.state = CANState()
        self.stop_event = threading.Event()
        self.q = queue.Queue()

    def _reader_loop(self):
        while not self.stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg: self.q.put(msg)

    def _send_ack(self, node_id):
        """ Step 3: Master responds with 0x400 and NodeID as Big Endian """
        payload = struct.pack('>I', node_id)
        msg = can.Message(arbitration_id=ACK_INTRO_ID, data=payload, is_extended_id=False)
        self.bus.send(msg)

    def process_messages(self):
        while not self.q.empty():
            msg = self.q.get()
            arb_id = msg.arbitration_id

            # Heartbeat / Telemetry
            if arb_id == EPOCH_ID:
                node_id, ts = struct.unpack('>II', msg.data[:8])
                self.state.get_node(node_id)['heartbeat'] = datetime.fromtimestamp(ts)
            
            elif arb_id == KNOB_ID:
                node_id, val = struct.unpack('>IH', msg.data[:6])
                self.state.get_node(node_id)['knob'] = val
            
            elif arb_id == TEMP_ID:
                node_id, val = struct.unpack('>If', msg.data[:8])
                self.state.get_node(node_id)['temp'] = val

            # --- Introduction / Interview Logic ---
            elif 0x700 <= arb_id <= 0x7FF:
                node_id = struct.unpack('>I', msg.data[0:4])[0]
                node = self.state.get_node(node_id)
                
                # Check if this is the Identity Frame (msgPtr 0) or Sub-module frames
                # If msg.data[4] is the count (as in Step 2 of your timeline)
                if arb_id == IFACE_ARGB_MULTI_ID: # Simplified check for Node Identity
                    node['sub_mod_cnt'] = msg.data[4]
                    node['reported_crc'] = (msg.data[5] << 8) | msg.data[6]
                    print(f"Node 0x{node_id:08X} Identity: {node['sub_mod_cnt']} subs, CRC: 0x{node['reported_crc']:04X}")
                else:
                    # Sub-module frames
                    mod_idx_byte = msg.data[4]
                    mod_idx = mod_idx_byte & 0x7F
                    is_part_b = bool(mod_idx_byte & 0x80)
                    
                    if mod_idx not in node['subs']:
                        node['subs'][mod_idx] = {'cfg': None, 'telemetry': None}
                    
                    if not is_part_b:
                        # Part A: Config
                        node['subs'][mod_idx]['cfg'] = msg.data[5:8]
                    else:
                        # Part B: Telemetry
                        data_msg_id = (msg.data[5] << 8) | msg.data[6]
                        dlc = msg.data[7] & 0x0F
                        save = bool(msg.data[7] & 0x80)
                        node['subs'][mod_idx]['telemetry'] = {'id': data_msg_id, 'dlc': dlc, 'save': save}
                
                # Step 5: Master ACKs every intro frame to trigger the next one
                self._send_ack(node_id)

    def request_broadcast(self):
        """ Step 1: Request all nodes to start intro """
        msg = can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0,0,0,0], is_extended_id=False)
        self.bus.send(msg)
        print("Broadcasted Introduction Request (0x401)")

    def run(self):
        t = threading.Thread(target=self._reader_loop, daemon=True)
        t.start()
        self.request_broadcast()
        
        try:
            while True:
                self.process_messages()
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop_event.set()

if __name__ == "__main__":
    # Example usage
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    app = App(bus, 'can0')
    app.run()