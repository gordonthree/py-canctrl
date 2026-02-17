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
ACK_INTRO_ID       = 0x400  # Master -> Node: [ID_32]
REQ_NODE_INTRO_ID  = 0x401  # Master -> Bus: [0,0,0,0]
KNOB_ID            = 0x518  # Telemetry examples
TEMP_ID            = 0x51A

DEFAULT_CAN_INTERFACE = 'can0'
LOG_MAX_LINES         = 2000

# Constants matching ESP32 debug output
SUBMODULE_STRUCT_SIZE = 16  # sizeof(subModule_t)
NODEINFO_STRUCT_SIZE  = 136 # sizeof(nodeInfo_t)

console = Console()

def crc16_ccitt(data: bytes, initial=0xFFFF):
    """
    Standard CRC-16-CCITT (0x1021) matching ESP32 rom/crc.h crc16_be.
    Initial: 0xFFFF, Poly: 0x1021
    """
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
        """
        Reconstructs the nodeInfo_t structure (136 bytes total).
        """
        nd = self.nodes[node_id]
        buf = bytearray(NODEINFO_STRUCT_SIZE) 
        
        # 1. Pack 8 subModules (16 bytes each)
        for i in range(8):
            offset = i * SUBMODULE_STRUCT_SIZE
            if i in nd['subs']:
                s = nd['subs'][i]
                # [0:2] config personality (3 bytes)
                if s['cfg']:
                    buf[offset:offset+3] = s['cfg']
                # [3] reserved byte (explicit padding)
                buf[offset+3] = 0x00
                # [4:7] data union (operational data, assumed 0 for config CRC)
                # [8:9] introMsgId (Little Endian uint16)
                struct.pack_into('<H', buf, offset + 8, s['intro_id'])
                # [10:11] dataMsgId (Little Endian uint16)
                if s['telemetry']:
                    struct.pack_into('<H', buf, offset + 10, s['telemetry']['id'])
                    # [12] introMsgDLC
                    buf[offset + 12] = 8 # Default
                    # [13] dataMsgDLC
                    buf[offset + 13] = s['telemetry']['dlc']
                    # [14] saveState
                    buf[offset + 14] = 1 if s['telemetry']['save'] else 0
                # [15] Tail padding for 16-byte alignment
                buf[offset + 15] = 0x00

        # 2. Pack Node Metadata (starts at byte 128)
        struct.pack_into('<I', buf, 128, node_id)           # nodeID (uint32_t)
        struct.pack_into('<H', buf, 132, nd['node_type_msg']) # nodeTypeMsg (uint16_t)
        buf[134] = 8                                        # nodeTypeDLC
        buf[135] = nd['sub_mod_cnt']                        # subModCnt

        # Save dump for debugging
        dump_name = f"node_{node_id:08X}_dump.bin"
        with open(dump_name, "wb") as f:
            f.write(buf)
        self.logger.add(f"Memory dump saved: {dump_name}")

        nd['mem_size'] = len(buf)
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
                    sz = f"({nd['mem_size']}B)"
                    if nd['calculated_crc'] == nd['reported_crc']:
                        crc_status = f"[green]MATCH 0x{nd['calculated_crc']:04X} {sz}[/]"
                    else:
                        crc_status = f"[red]FAIL C:0x{nd['calculated_crc']:04X} R:0x{nd['reported_crc']:04X} {sz}[/]"

                data.append({
                    'id': node_id,
                    'age': age,
                    'knob': nd['knob'] if nd['knob'] is not None else "-",
                    'temp': nd['temp'] if nd['temp'] is not None else "-",
                    'crc_str': crc_status,
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

            # Standard Telemetry
            if arb_id == KNOB_ID and dlc >= 6:
                node_id, val = struct.unpack('>IH', msg.data[:6])
                self.state.touch(node_id)['knob'] = val
            elif arb_id == TEMP_ID and dlc >= 8:
                node_id, celsius = struct.unpack('>If', msg.data[:8])
                self.state.touch(node_id)['temp'] = celsius

            # Interview Sequence (0x700-0x7FF)
            elif 0x700 <= arb_id <= 0x7FF:
                if dlc < 4: continue
                # Parse Node ID from Data Payload (Big Endian over wire)
                node_id = struct.unpack('>I', msg.data[0:4])[0]
                node = self.state.touch(node_id)

                if node['interview_complete']: continue 

                if node['sub_mod_cnt'] == 0: # Frame 0: Identity
                    if dlc >= 7:
                        node['node_type_msg'] = arb_id
                        node['sub_mod_cnt'] = msg.data[4]
                        node['reported_crc'] = (msg.data[5] << 8) | msg.data[6]
                        self.log.add(f"Interviewing Node 0x{node_id:08X}...")
                    else: continue
                else: # Frame 1+: Submodules
                    if dlc >= 5:
                        m_byte = msg.data[4]
                        idx, is_b = m_byte & 0x7F, bool(m_byte & 0x80)
                        
                        if idx not in node['subs']:
                            node['subs'][idx] = {'cfg': None, 'telemetry': None, 'intro_id': arb_id}
                        
                        if not is_b and dlc >= 8: # Part A: Config
                            node['subs'][idx]['cfg'] = bytes(msg.data[5:8])
                        elif is_b and dlc >= 8:   # Part B: Telemetry
                            node['subs'][idx]['telemetry'] = {
                                'id': (msg.data[5] << 8) | msg.data[6],
                                'dlc': msg.data[7] & 0x0F,
                                'save': bool(msg.data[7] & 0x80)
                            }
                            if (idx + 1) >= node['sub_mod_cnt']:
                                node['interview_complete'] = True
                                node['calculated_crc'] = self.state.calculate_node_crc(node_id)
                                self.log.add(f"Completed Node 0x{node_id:08X}")
                
                # Acknowledge the frame
                ack = can.Message(arbitration_id=ACK_INTRO_ID, data=struct.pack('>I', node_id))
                self.bus.send(ack)

    def request_broadcast(self):
        msg = can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0]*4)
        self.bus.send(msg)
        self.log.add("Broadcast Discovery Sent (0x401)")

    def _build_layout(self):
        layout = Layout()
        table = Table(show_header=True, header_style="bold cyan", expand=True)
        table.add_column("Node ID", width=12)
        table.add_column("Age", width=5, justify="right")
        table.add_column("Knob", width=8)
        table.add_column("Temp", width=8)
        table.add_column("Interview", width=12)
        table.add_column("CRC Verification", ratio=1)

        for n in self.state.snapshot():
            table.add_row(
                f"0x{n['id']:08X}", f"{n['age']}s", str(n['knob']), 
                f"{n['temp']:.1f}" if isinstance(n['temp'], float) else "-",
                n['subs_text'], n['crc_str']
            )

        layout.split(
            Layout(Text(f" CAN Master | {self.iface} | (q)uit (b)roadcast", style="bold reverse cyan"), size=1),
            Layout(table, name="body"),
            Layout(Panel(Text("\n".join(self.log.tail(8))), title="System Log", border_style="magenta"), size=10)
        )
        return layout

    def run(self):
        self.reader_thread.start()
        time.sleep(0.5)
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