#!/usr/bin/env python3
import time
import struct
import threading
import queue
from collections import deque
import platform
import sys
import argparse
import os

from rich.console import Console
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
ACK_INTRO_ID       = 0x400  
REQ_NODE_INTRO_ID  = 0x401  
KNOB_ID            = 0x518  
TEMP_ID            = 0x51A

DEFAULT_CAN_INTERFACE = 'can0'
LOG_MAX_LINES         = 2000

SUBMODULE_STRUCT_SIZE = 16  
NODEINFO_STRUCT_SIZE  = 136 

def crc16_ccitt(data: bytes, initial=0xFFFF):
    """ Standard CRC-16-CCITT (0x1021) matching ESP32 rom/crc.h crc16_be """
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

class LogBuffer:
    def __init__(self, max_lines=LOG_MAX_LINES):
        self.lines = deque(maxlen=max_lines)
        self.lock = threading.Lock()

    def add(self, text: str):
        ts = time.strftime('%H:%M:%S')
        with self.lock:
            self.lines.append(f"[{ts}] {text}")

    def tail(self, n):
        with self.lock:
            return list(self.lines)[-n:]

class CANState:
    def __init__(self, logger):
        self.nodes = {}
        self.lock = threading.Lock()
        self.logger = logger

    def touch(self, node_id: int):
        with self.lock:
            nd = self.nodes.setdefault(node_id, {
                'heartbeat': None, 'knob': None, 'temp': None, 'last': None,
                'node_type_msg': 0, 'sub_mod_cnt': 0, 'reported_crc': 0,
                'subs': {}, 'interview_complete': False, 'calculated_crc': 0,
                'mem_size': 0
            })
            nd['last'] = time.time()
            return nd

    def calculate_node_crc(self, node_id: int):
        nd = self.nodes[node_id]
        buf = bytearray(NODEINFO_STRUCT_SIZE) 
        
        for i in range(8):
            offset = i * SUBMODULE_STRUCT_SIZE
            if i in nd['subs']:
                s = nd['subs'][i]
                # [0:3] Config
                if s['cfg']:
                    buf[offset:offset+3] = s['cfg']
                
                # Based on your Serial Dump: 00 01 07 10 02 08 06 01
                # Byte 8: 00 (Padding)
                # Byte 9: 01 (Intro ID Low?) 
                # Actually, 0x0701 is likely the ID. 
                # Let's pack them exactly as seen in the dump:
                
                # [9:11] introMsgId
                struct.pack_into('<H', buf, offset + 9, s['intro_id'])
                # [11:13] dataMsgId
                if s['telemetry']:
                    struct.pack_into('<H', buf, offset + 11, s['telemetry']['id'])
                    # [13] introMsgDLC
                    buf[offset + 13] = 8 # Default
                    # [14] dataMsgDLC
                    buf[offset + 14] = s['telemetry']['dlc']
                    # [15] saveState
                    buf[offset + 15] = 1 if s['telemetry']['save'] else 0

        # --- Metadata (Starts at 0x80) ---
        # Matches your dump: 84 6D A5 25 9C 07 08 02
        struct.pack_into('<I', buf, 128, node_id)           
        struct.pack_into('<H', buf, 132, nd['node_type_msg'])
        buf[134] = 8                                        
        buf[135] = nd['sub_mod_cnt']                        

        # Log Hexdump
        self.logger.add(f"Memory Reconstructed for 0x{node_id:08X}:")
        for i in range(0, NODEINFO_STRUCT_SIZE, 16):
            chunk = buf[i:i+16]
            hex_str = " ".join(f"{b:02X}" for b in chunk)
            self.logger.add(f"{i:04X}: {hex_str}")

        return crc16_ccitt(buf)

    def snapshot(self):
        with self.lock:
            ids = sorted(self.nodes.keys())
            now = time.time()
            data = []
            for node_id in ids:
                nd = self.nodes[node_id]
                age = int(now - nd['last']) if nd['last'] else 0
                crc_status = ""
                if nd['interview_complete']:
                    if nd['calculated_crc'] == nd['reported_crc']:
                        crc_status = f"[green]MATCH 0x{nd['calculated_crc']:04X}[/]"
                    else:
                        crc_status = f"[red]FAIL C:0x{nd['calculated_crc']:04X} R:0x{nd['reported_crc']:04X}[/]"

                data.append({
                    'id': node_id, 'age': age, 'knob': nd['knob'] or "-",
                    'temp': nd['temp'] or "-", 'crc_str': crc_status,
                    'subs_text': f"{len(nd['subs'])}/{nd['sub_mod_cnt']} mods"
                })
            return data

class App:
    def __init__(self, bus, iface):
        self.bus = bus
        self.iface = iface
        self.log = LogBuffer()
        self.state = CANState(self.log)
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.is_windows = (platform.system().lower() == "windows")

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg: self.q.put(msg)
            except Exception: pass

    def _process_queue(self):
        while not self.q.empty():
            msg = self.q.get_nowait()
            arb_id = msg.arbitration_id
            dlc = len(msg.data)

            if arb_id == KNOB_ID and dlc >= 6:
                node_id, val = struct.unpack('>IH', msg.data[:6])
                self.state.touch(node_id)['knob'] = val
            elif arb_id == TEMP_ID and dlc >= 8:
                node_id, celsius = struct.unpack('>If', msg.data[:8])
                self.state.touch(node_id)['temp'] = celsius
            elif 0x700 <= arb_id <= 0x7FF:
                if dlc < 4: continue
                node_id = struct.unpack('>I', msg.data[0:4])[0]
                node = self.state.touch(node_id)
                if node['interview_complete']: continue 

                if node['sub_mod_cnt'] == 0:
                    if dlc >= 7:
                        node['node_type_msg'] = arb_id
                        node['sub_mod_cnt'] = msg.data[4]
                        node['reported_crc'] = (msg.data[5] << 8) | msg.data[6]
                        self.log.add(f"Started Interview 0x{node_id:08X}")
                else:
                    if dlc >= 5:
                        m_byte = msg.data[4]
                        idx, is_b = m_byte & 0x7F, bool(m_byte & 0x80)
                        if idx not in node['subs']:
                            node['subs'][idx] = {'cfg': None, 'telemetry': None, 'intro_id': arb_id}
                        
                        if not is_b and dlc >= 8:
                            node['subs'][idx]['cfg'] = bytes(msg.data[5:8])
                        elif is_b and dlc >= 8:
                            node['subs'][idx]['telemetry'] = {
                                'id': (msg.data[5] << 8) | msg.data[6],
                                'dlc': msg.data[7] & 0x0F,
                                'save': bool(msg.data[7] & 0x80)
                            }
                            if (idx + 1) >= node['sub_mod_cnt']:
                                node['interview_complete'] = True
                                node['calculated_crc'] = self.state.calculate_node_crc(node_id)

                self.bus.send(can.Message(arbitration_id=ACK_INTRO_ID, data=struct.pack('>I', node_id)))

    def request_broadcast(self):
        self.bus.send(can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0]*4))
        self.log.add("Discovery Request Sent")

    def _build_layout(self):
        layout = Layout()
        table = Table(show_header=True, header_style="bold cyan", expand=True)
        table.add_column("Node ID", width=12)
        table.add_column("Age", width=5, justify="right")
        table.add_column("Interview", width=12)
        table.add_column("CRC Status", ratio=1)

        for n in self.state.snapshot():
            table.add_row(f"0x{n['id']:08X}", f"{n['age']}s", n['subs_text'], n['crc_str'])

        layout.split(
            Layout(Text(f" CAN Master | {self.iface} | (q)uit (b)roadcast", style="bold reverse cyan"), size=1),
            Layout(table, name="body"),
            Layout(Panel(Text("\n".join(self.log.tail(12))), title="Memory Hex Log", border_style="magenta"), size=14)
        )
        return layout

    def run(self):
        self.reader_thread.start()
        self.request_broadcast()
        
        if not self.is_windows and sys.stdin.isatty():
            import termios, tty
            orig_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        
        try:
            with Live(self._build_layout(), refresh_per_second=10, screen=True) as live:
                while not self.stop_event.is_set():
                    self._process_queue()
                    key = self._get_key()
                    if key == 'q': self.stop_event.set()
                    if key == 'b': self.request_broadcast()
                    live.update(self._build_layout())
                    time.sleep(0.05)
        finally:
            if not self.is_windows and sys.stdin.isatty():
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

    def _get_key(self):
        if self.is_windows:
            import msvcrt
            return msvcrt.getwch().lower() if msvcrt.kbhit() else None
        else:
            import select
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            return sys.stdin.read(1).lower() if dr else None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default=DEFAULT_CAN_INTERFACE)
    args = parser.parse_args()
    try:
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        App(bus, args.iface).run()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()