#!/usr/bin/env python3
import time
import struct
import argparse
import sys
import select
import traceback
from collections import deque

from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
from rich.live import Live
from rich.text import Text

try:
    import can
except Exception:
    print("Failed to import python-can. Install with: pip install python-can")
    sys.exit(1)

# --- CAN Constants ---
ACK_ID                = 0x400 # Acknowledgement for identity frames
REQ_INTRO_ID          = 0x401 # Request all nodes to introduce themselves
COMMIT_ID             = 0x41D # Master to Node: Commit configuration
NODE_INTRO_ID         = 0x79C # Standard ESP32 Node Identity
CYD_INTRO_ID          = 0x792 # CYD Touchscreen Identity
SUBMOD_INTRO_START    = 0x700 # Start of submodule range
SUBMOD_INTRO_END      = 0x74F # End of submodule range

def crc16_ccitt_false(data):
    """ Matches the C++ implementation in main.cpp (Poly: 0x1021, Init: 0xFFFF) """
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class RemoteNode:
    # Tracks the discovery state of a physical ESP32 node.
    def __init__(self, node_id):
        self.node_id = node_id      
        self.sub_mod_count = 0      
        self.crc = 0                
        self.discovered_msgs = 0    
        self.sub_modules = {}       
        self.sub_modules_raw = {}   
        self.last_seen = time.time()
        self.type_msg_id = 0
        self.type_dlc = 8

class App:
    def __init__(self, bus, iface="can0"):
        self.bus = bus
        self.iface = iface
        self.nodes = {}  
        self.log_lines = deque(maxlen=15)
        self.running = True

    def log(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_lines.append(f"[{ts}] {msg}")

    def send_ack(self, node_id):
        if self.bus:
            data = struct.pack("<I", node_id) + b'\x00\x00\x00\x00'
            msg = can.Message(arbitration_id=ACK_ID, data=data, is_extended_id=False)
            self.bus.send(msg)

    def process_can_message(self, msg):
        if len(msg.data) < 4: return
        try:
            node_id = struct.unpack("<I", msg.data[0:4])[0]
        except (struct.error, IndexError): return

        if msg.arbitration_id in [NODE_INTRO_ID, CYD_INTRO_ID]:
            if len(msg.data) < 7: return
            if node_id not in self.nodes:
                self.nodes[node_id] = RemoteNode(node_id)
                bus_id = struct.pack("<I", node_id).hex().upper()
                self.log(f"New Node: 0x{bus_id}")
            
            node = self.nodes[node_id]
            node.type_msg_id = msg.arbitration_id
            node.type_dlc = msg.dlc
            node.last_seen = time.time()
            node.sub_mod_count = msg.data[4]
            node.crc = (msg.data[5] << 8) | msg.data[6]
            
            if node.discovered_msgs < (node.sub_mod_count * 2):
                self.send_ack(node_id)

        elif SUBMOD_INTRO_START <= msg.arbitration_id <= SUBMOD_INTRO_END:
            if len(msg.data) < 8: return
            node = self.nodes.get(node_id)
            if node:
                idx_byte = msg.data[4]
                mod_idx = idx_byte & 0x7F
                is_part_b = bool(idx_byte & 0x80)

                if mod_idx not in node.sub_modules_raw:
                    node.sub_modules_raw[mod_idx] = {
                        'intro_id': msg.arbitration_id, 
                        'intro_dlc': msg.dlc,
                        'data_id': 0, 'data_dlc': 0, 'save': 1,
                        'config': b'\x00\x00\x00'
                    }

                sub = node.sub_modules_raw[mod_idx]
                if not is_part_b:
                    sub['config'] = bytes(msg.data[5:8])
                    node.sub_modules[mod_idx] = f"ID:{hex(msg.arbitration_id)}"
                else:
                    sub['data_id'] = (msg.data[5] << 8) | msg.data[6]
                    sub['data_dlc'] = msg.data[7] & 0x0F
                    sub['save'] = 1 if (msg.data[7] & 0x80) else 0

                node.discovered_msgs += 1
                if node.discovered_msgs < (node.sub_mod_count * 2):
                    self.send_ack(node_id)

    def send_commit(self, node_id):
        node = self.nodes.get(node_id)
        if not node: return

        # 1. Reconstruct the 136-byte nodeInfo_t block for CRC calculation
        # This represents the literal byte-by-byte layout in the ESP32's RAM.
        struct_data = bytearray()
        
        for i in range(8):
            sub = node.sub_modules_raw.get(i)
            if sub:
                struct_data += sub['config']               # [0-2] 3 bytes
                struct_data += b'\x00'                     # [3] Alignment byte
                struct_data += b'\x00' * 4                 # [4-7] Operational data (zeroed)
                struct_data += struct.pack("<H", sub['intro_id']) # [8-9] LE
                struct_data += struct.pack("<H", sub['data_id'])  # [10-11] LE
                struct_data += struct.pack("B", sub['intro_dlc']) # [12]
                struct_data += struct.pack("B", sub['data_dlc'])  # [13]
                struct_data += struct.pack("B", sub['save'])      # [14]
                struct_data += b'\x00'                     # [15] Tail padding for 16-byte alignment
            else:
                struct_data += b'\x00' * 16                # Empty 16-byte slot

        # The 8-byte Node Header at the end of the struct
        struct_data += struct.pack("<I", node.node_id)      # [128-131] Node ID (LE in RAM)
        struct_data += struct.pack("<H", node.type_msg_id)  # [132-133] Type ID (LE in RAM)
        struct_data += struct.pack("B", node.type_dlc)      # [134]
        struct_data += struct.pack("B", node.sub_mod_count) # [135]

        # Calculate CRC on the 136-byte RAM image
        calc_crc = crc16_ccitt_false(struct_data)

        # 2. Pack the 8-byte CAN Frame (Arbitration ID 0x41D)
        # The Node extracts the ID and CRC as BIG-ENDIAN from the CAN message.
        # Payload: [ID_MSB, ..., ID_LSB] [CRC_MSB, CRC_LSB] [0, 0]
        id_bytes = struct.pack("<I", node_id) # Big-Endian for the node's targetID check
        crc_bytes = struct.pack(">H", calc_crc) # Big-Endian for the node's masterCrc extraction
        
        payload = list(id_bytes) + list(crc_bytes) + [0x00, 0x00]
        
        # 0x41D = CFG_WRITE_NVS_ID
        self.bus.send(can.Message(arbitration_id=0x41D, data=payload, is_extended_id=False))
        
        self.log(f"SENT COMMIT 0x{node_id:08X} CRC:0x{calc_crc:04X}")

    def run(self):
        layout = Layout()
        layout.split_column(Layout(name="header", size=3), Layout(name="body"), Layout(name="footer", size=10))
        
        import termios, tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            if self.bus: self.bus.send(can.Message(arbitration_id=REQ_INTRO_ID, data=[0]*8))
            with Live(layout, refresh_per_second=4, screen=True):
                while self.running:
                    try:
                        if self.bus:
                            msg = self.bus.recv(0.005)
                            if msg: self.process_can_message(msg)
                    except Exception: pass

                    r, _, _ = select.select([sys.stdin], [], [], 0)
                    if r:
                        char = sys.stdin.read(1).lower()
                        if char == 'q': self.running = False
                        elif char == 'p':
                            for nid in self.nodes: self.send_commit(nid)

                    table = Table(expand=True)
                    table.add_column("Node ID (Bus Order)", style="cyan")
                    table.add_column("Remote CRC", style="magenta")
                    table.add_column("Status", style="green")
                    table.add_column("Last Seen", style="white")
                    
                    for nid, node in sorted(self.nodes.items()):
                        visual_id = "0x" + struct.pack("<I", nid).hex().upper()
                        status = "Synced" if node.crc != 0xFFFF else "[bold red]Dirty[/]"
                        table.add_row(
                            visual_id, f"0x{node.crc:04X}", 
                            f"{node.discovered_msgs//2}/{node.sub_mod_count} Mods",
                            f"{int(time.time() - node.last_seen)}s"
                        )

                    layout["header"].update(Panel(Text("CAN Master - [P] Provision [Q] Quit", justify="center", style="bold blue")))
                    layout["body"].update(Panel(table, title="Network Topology"))
                    layout["footer"].update(Panel("\n".join(list(self.log_lines)), title="System Logs"))
                    time.sleep(0.01)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            if self.bus: self.bus.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="can0")
    args = parser.parse_args()
    try:
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        App(bus, iface=args.iface).run()
    except Exception:
        traceback.print_exc()
        time.sleep(2.0)