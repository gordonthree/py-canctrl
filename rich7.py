#!/usr/bin/env python3
import time
import struct
import threading
import queue
from collections import deque
import platform
import sys
import argparse

from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
from rich.live import Live
from rich.text import Text

try:
    import can
except ImportError:
    print("Failed to import python-can. Install with: pip install python-can")
    sys.exit(1)

# --- Configuration & Message IDs ---
ACK_INTRO_ID         = 0x400
REQ_NODE_INTRO_ID    = 0x401
CFG_REBOOT_ID        = 0x41C
CFG_WRITE_NVS_ID     = 0x41D
DATA_CONFIG_CRC_ID   = 0x526  
DATA_CFGWRITE_FAILED = 0x528  

SUBMODULE_STRUCT_SIZE = 16
NODEINFO_STRUCT_SIZE  = 136
NVS_TIMEOUT_SECONDS   = 5 # Max time to wait for ESP32 flash confirmation

def crc16_ccitt(data: bytes, initial=0xFFFF):
    # Standard CRC-16-CCITT (0x1021) matching ESP32 rom/crc.h crc16_be
    crc = initial
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class CANState:
    def __init__(self, logger):
        self.nodes = {}
        self.lock = threading.Lock()
        self.logger = logger

    def touch(self, node_id: int):
        with self.lock:
            nd = self.nodes.setdefault(node_id, {
                'node_type_msg': 0, 'sub_mod_cnt': 0, 'reported_crc': 0,
                'subs': {}, 'interview_complete': False, 'calculated_crc': 0,
                'last_rx': time.time(), 'nvs_status': "Pending",
                'nvs_timestamp': 0
            })
            nd['last_rx'] = time.time()
            return nd

    def calculate_node_crc(self, node_id: int):
        nd = self.nodes[node_id]
        buf = bytearray(NODEINFO_STRUCT_SIZE) 
        active_mods = min(nd['sub_mod_cnt'], 8)
        
        for i in range(active_mods):
            offset = i * SUBMODULE_STRUCT_SIZE
            if i in nd['subs']:
                s = nd['subs'][i]
                if s['cfg']:
                    buf[offset:offset+3] = s['cfg'][:3]
                
                # Hardware padding/offsets: [9:11] introMsgId, [11:13] dataMsgId
                struct.pack_into('<H', buf, offset + 9, s['intro_id'])
                if s['telemetry']:
                    struct.pack_into('<H', buf, offset + 11, s['telemetry']['id'])
                    buf[offset + 13] = 8 
                    buf[offset + 14] = s['telemetry']['dlc']
                    buf[offset + 15] = 1 if s['telemetry']['save'] else 0

        # Metadata at 0x80
        struct.pack_into('<I', buf, 128, node_id)           
        struct.pack_into('<H', buf, 132, nd['node_type_msg'])
        buf[134] = 8                                        
        buf[135] = nd['sub_mod_cnt']                        

        return crc16_ccitt(buf)

class App:
    def __init__(self, bus, iface):
        self.bus, self.iface = bus, iface
        self.log_lines = deque(maxlen=15)
        self.state = CANState(self)
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.is_windows = (platform.system().lower() == "windows")

    def add_log(self, text):
        self.log_lines.append(f"[{time.strftime('%H:%M:%S')}] {text}")

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg: self.q.put(msg)
            except Exception: pass

    def _process_queue(self):
        # 1. Check for NVS Timeouts
        now = time.time()
        with self.state.lock:
            for nid, n in self.state.nodes.items():
                if n['nvs_status'] == "Writing..." and (now - n['nvs_timestamp'] > NVS_TIMEOUT_SECONDS):
                    n['nvs_status'] = "[bold yellow]Timeout[/]"
                    self.add_log(f"Node 0x{nid:08X}: NVS Write Timed Out")

        # 2. Process incoming messages
        while not self.q.empty():
            msg = self.q.get_nowait()
            arb_id, data = msg.arbitration_id, msg.data

            # Telemetry or Interview Frame - Touch the node for heartbeat
            # We assume node ID is in the first 4 bytes of most relevant frames
            if len(data) >= 4:
                node_id_guess = struct.unpack('>I', data[0:4])[0]
                # Filter for IDs we expect to be valid to avoid ghost nodes
                if 0x10000000 <= node_id_guess <= 0xFFFFFFFF:
                    self.state.touch(node_id_guess)

            if 0x700 <= arb_id <= 0x7FF:
                node_id = struct.unpack('>I', data[0:4])[0]
                node = self.state.touch(node_id)
                if not node['interview_complete']:
                    if node['sub_mod_cnt'] == 0:
                        node['node_type_msg'] = arb_id
                        node['sub_mod_cnt'] = data[4]
                        node['reported_crc'] = (data[5] << 8) | data[6]
                        if node['sub_mod_cnt'] == 0: node['interview_complete'] = True 
                    else:
                        idx, is_b = data[4] & 0x7F, bool(data[4] & 0x80)
                        if idx < 8:
                            if idx not in node['subs']:
                                node['subs'][idx] = {'cfg': None, 'telemetry': None, 'intro_id': arb_id}
                            if not is_b:
                                node['subs'][idx]['cfg'] = bytes(data[5:8])
                            else:
                                node['subs'][idx]['telemetry'] = {'id': (data[5] << 8) | data[6], 'dlc': data[7] & 0x0F, 'save': bool(data[7] & 0x80)}
                                if (idx + 1) >= node['sub_mod_cnt']:
                                    node['interview_complete'] = True
                                    node['calculated_crc'] = self.state.calculate_node_crc(node_id)
                    
                    self.bus.send(can.Message(arbitration_id=ACK_INTRO_ID, data=struct.pack('>I', node_id)))

            elif arb_id == DATA_CONFIG_CRC_ID:
                nid = struct.unpack('>I', data[0:4])[0]
                n = self.state.touch(nid)
                n['nvs_status'] = "[bold green]Success[/]"
                self.add_log(f"Node 0x{nid:08X}: NVS Saved. Sending Reboot...")
                self.bus.send(can.Message(arbitration_id=CFG_REBOOT_ID, data=struct.pack('>I', nid)))
            
            elif arb_id == DATA_CFGWRITE_FAILED:
                nid = struct.unpack('>I', data[0:4])[0]
                self.state.touch(nid)['nvs_status'] = "[bold red]Failed[/]"
                self.add_log(f"Node 0x{nid:08X}: NVS Write Failed")

    def provision_nodes(self):
        now = time.time()
        with self.state.lock:
            for nid, n in self.state.nodes.items():
                if n['interview_complete'] and n['calculated_crc'] == n['reported_crc']:
                    payload = struct.pack('>I H', nid, n['calculated_crc'])
                    self.bus.send(can.Message(arbitration_id=CFG_WRITE_NVS_ID, data=payload))
                    n['nvs_status'] = "Writing..."
                    n['nvs_timestamp'] = now
                    self.add_log(f"Requesting NVS write for 0x{nid:08X}")

    def _build_layout(self):
        table = Table(show_header=True, header_style="bold cyan", expand=True, box=None)
        table.add_column("Node ID", width=12)
        table.add_column("Heartbeat", width=10, justify="right")
        table.add_column("Sub-Module Summary", ratio=1)
        table.add_column("CRC Status", width=18)
        table.add_column("NVS Status", width=12)

        now = time.time()
        with self.state.lock:
            for nid, n in self.state.nodes.items():
                hb = int(now - n['last_rx'])
                hb_str = f"{hb}s" if hb < 999 else ">999s"
                
                mod_summary = [f"M{i}:[dim]0x{s['telemetry']['id']:03X}[/]" for i, s in n['subs'].items() if s['telemetry']]
                
                crc_str = ""
                if n['interview_complete']:
                    m = n['calculated_crc'] == n['reported_crc']
                    crc_str = f"[green]OK:{n['calculated_crc']:04X}[/]" if m else f"[red]ERR:{n['calculated_crc']:04X}[/]"
                
                table.add_row(f"0x{nid:08X}", hb_str, " ".join(mod_summary), crc_str, n['nvs_status'])

        l = Layout()
        l.split(
            Layout(Text(f" CAN Master | {self.iface} | (q)uit (b)roadcast (p)ersist", style="bold reverse green"), size=1),
            Layout(table, name="body"),
            Layout(Panel(Text("\n".join(self.log_lines)), title="System Log", border_style="blue"), size=10)
        )
        return l

    def run(self):
        self.reader.start()
        self.bus.send(can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0]*4))
        if not self.is_windows and sys.stdin.isatty():
            import termios, tty
            orig = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        try:
            with Live(self._build_layout(), refresh_per_second=10, screen=True) as live:
                while not self.stop_event.is_set():
                    self._process_queue()
                    key = self._get_key()
                    if key == 'q': self.stop_event.set()
                    if key == 'b': self.bus.send(can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0]*4))
                    if key == 'p': self.provision_nodes()
                    live.update(self._build_layout())
                    time.sleep(0.05)
        finally:
            if not self.is_windows and sys.stdin.isatty():
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig)

    def _get_key(self):
        if self.is_windows:
            import msvcrt
            return msvcrt.getwch().lower() if msvcrt.kbhit() else None
        else:
            import select
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            return sys.stdin.read(1).lower() if dr else None

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="can0")
    args = parser.parse_args()
    try:
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        App(bus, args.iface).run()
    except Exception as e: print(f"Error: {e}")