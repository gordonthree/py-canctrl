#!/usr/bin/env python3
import time
import struct
import threading
import queue
from collections import deque
from datetime import datetime
import platform
import sys
import argparse
import traceback
import os

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

# --- Configuration & Message IDs ---
ACK_INTRO_ID       = 0x400  # Master -> Node: [ID_32]
REQ_NODE_INTRO_ID  = 0x401  # Master -> Bus: [0,0,0,0]
EPOCH_ID           = 0x40C  # Heartbeat/Time Sync
KNOB_ID            = 0x518  # Telemetry: Knob
TEMP_ID            = 0x51A  # Telemetry: Temp

DEFAULT_CAN_INTERFACE = 'can0'
LOG_MAX_LINES         = 2000

console = Console()

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

    def write_to_file(self, path: str):
        with self.lock:
            try:
                with open(path, "a", encoding="utf-8") as f:
                    for line in self.lines:
                        f.write(line + "\n")
            except Exception:
                pass

class CANState:
    def __init__(self):
        self.nodes = {}
        self.lock = threading.Lock()
        self.last_sync = {}

    def touch(self, node_id: int):
        with self.lock:
            nd = self.nodes.setdefault(node_id, {
                'heartbeat': None, 
                'knob': None, 
                'temp': None, 
                'last': None,
                'sub_mod_cnt': 0,
                'reported_crc': 0,
                'subs': {}, # Index -> { 'cfg': bytes, 'telemetry': dict }
                'interview_complete': False
            })
            nd['last'] = time.time()
            return nd

    def snapshot(self):
        with self.lock:
            ids = sorted(self.nodes.keys())
            now = time.time()
            data = []
            for node_id in ids:
                nd = self.nodes[node_id]
                age_secs = int(now - nd['last']) if nd['last'] else None
                
                # Format sub-module summary strings
                sub_summaries = []
                for i in sorted(nd['subs'].keys()):
                    s = nd['subs'][i]
                    cfg = s['cfg'].hex().upper() if s['cfg'] else "???"
                    tele = s['telemetry']
                    if tele:
                        sub_summaries.append(f"M{i}[{cfg}|ID:0x{tele['id']:03X}|L:{tele['dlc']}]")
                    else:
                        sub_summaries.append(f"M{i}[{cfg}|...]")

                data.append({
                    'id': node_id,
                    'hb': nd['heartbeat'].strftime('%H:%M:%S') if nd['heartbeat'] else "-",
                    'age': age_secs,
                    'knob': nd['knob'] if nd['knob'] is not None else "-",
                    'temp': nd['temp'] if nd['temp'] is not None else "-",
                    'complete': nd['interview_complete'],
                    'cnt': nd['sub_mod_cnt'],
                    'subs_text': " ".join(sub_summaries)
                })
            return data

class App:
    def __init__(self, bus, iface: str, dry_run: bool = False, debug_file: str | None = None):
        self.bus = bus
        self.iface = iface
        self.dry_run = dry_run
        self.debug_file = debug_file
        self.log = LogBuffer()
        self.state = CANState()
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.log_height = 10
        self.is_windows = (platform.system().lower() == "windows")
        self.stdin_is_tty = sys.stdin.isatty()

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.2)
                if msg: self.q.put(msg)
            except Exception:
                time.sleep(0.1)

    def _process_queue(self):
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            arb_id = msg.arbitration_id
            dlc = len(msg.data)

            # Heartbeat & Telemetry
            if arb_id == EPOCH_ID and dlc >= 8:
                node_id, unix_ts = struct.unpack('>II', msg.data[:8])
                self.state.touch(node_id)['heartbeat'] = datetime.fromtimestamp(unix_ts)
            elif arb_id == KNOB_ID and dlc >= 6:
                node_id, val = struct.unpack('>IH', msg.data[:6])
                self.state.touch(node_id)['knob'] = val
            elif arb_id == TEMP_ID and dlc >= 8:
                node_id, celsius = struct.unpack('>If', msg.data[:8])
                self.state.touch(node_id)['temp'] = celsius

            # Interview Logic (0x700 - 0x7FF)
            elif 0x700 <= arb_id <= 0x7FF:
                if dlc < 4: continue # Length Guard
                
                node_id = struct.unpack('>I', msg.data[0:4])[0]
                node = self.state.touch(node_id) 

                # If already interviewed, do not trigger ACK sequence again
                if node['interview_complete']:
                    continue

                # Process Introduction Frame
                # Identify msgPtr 0: Contains SubMod count and CRC
                if node['sub_mod_cnt'] == 0:
                    if dlc >= 7:
                        node['sub_mod_cnt'] = msg.data[4]
                        node['reported_crc'] = (msg.data[5] << 8) | msg.data[6]
                        self.log.add(f"NEW NODE: 0x{node_id:08X} (Type:0x{arb_id:X})")
                    else:
                        continue # Malformed identity frame
                else:
                    # Sub-module data frames
                    if dlc >= 5:
                        mod_idx_byte = msg.data[4]
                        mod_idx = mod_idx_byte & 0x7F
                        is_part_b = bool(mod_idx_byte & 0x80)
                        
                        if mod_idx not in node['subs']:
                            node['subs'][mod_idx] = {'cfg': None, 'telemetry': None}
                        
                        if not is_part_b and dlc >= 8:
                            # Part A: Raw Config bytes
                            node['subs'][mod_idx]['cfg'] = msg.data[5:8]
                        elif is_part_b and dlc >= 8:
                            # Part B: DataMsgID, DLC, and SaveState
                            data_id = (msg.data[5] << 8) | msg.data[6]
                            data_dlc = msg.data[7] & 0x0F
                            save = bool(msg.data[7] & 0x80)
                            node['subs'][mod_idx]['telemetry'] = {'id': data_id, 'dlc': data_dlc, 'save': save}
                            
                            # Completion check
                            if (mod_idx + 1) >= node['sub_mod_cnt']:
                                node['interview_complete'] = True
                                self.log.add(f"INTERVIEW DONE: 0x{node_id:08X}")
                    else:
                        continue

                # Trigger the next packet in the node's sequence
                self._send_ack(node_id)

    def _send_ack(self, node_id: int):
        if self.dry_run: return
        payload = struct.pack('>I', node_id)
        msg = can.Message(arbitration_id=ACK_INTRO_ID, data=payload, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception: pass

    def request_broadcast(self):
        if self.dry_run: return
        msg = can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0,0,0,0], is_extended_id=False)
        self.bus.send(msg)
        self.log.add("Broadcasted 0x401 Discovery")

    # --- UI Rendering ---
    def _build_table(self):
        table = Table(show_header=True, header_style="bold cyan", expand=True)
        table.add_column("Node ID", width=12)
        table.add_column("HB", width=10)
        table.add_column("Age", width=5, justify="right")
        table.add_column("Knob", width=7, justify="right")
        table.add_column("Temp", width=7, justify="right")
        table.add_column("Interview Details (SubModules)", ratio=1)

        for n in self.state.snapshot():
            age_style = "green" if n['age'] < 10 else "yellow" if n['age'] < 30 else "red"
            status_prefix = "[green]✓[/] " if n['complete'] else "[yellow]⋯[/] "
            
            table.add_row(
                f"0x{n['id']:08X}",
                n['hb'],
                Text(str(n['age']) + "s", style=age_style),
                str(n['knob']),
                f"{n['temp']:.1f}" if isinstance(n['temp'], float) else "-",
                status_prefix + n['subs_text']
            )
        return table

    def _build_layout(self):
        layout = Layout()
        layout.split(
            Layout(Text(f" CAN Master | {self.iface} | {time.strftime('%H:%M:%S')} | (q)uit (b)roadcast", style="bold reverse cyan"), size=1),
            Layout(self._build_table(), name="body"),
            Layout(Panel(Text("\n".join(self.log.tail(self.log_height))), title="System Log", border_style="magenta"), size=self.log_height)
        )
        return layout

    def run(self):
        self.reader_thread.start()
        time.sleep(0.2)
        self.request_broadcast()

        if not self.is_windows and self.stdin_is_tty:
            import termios, tty
            self.orig_term_attrs = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        try:
            with Live(self._build_layout(), refresh_per_second=10, screen=True) as live:
                while not self.stop_event.is_set():
                    self._process_queue()
                    
                    # Manual keyboard check
                    key = None
                    if self.is_windows and msvcrt.kbhit():
                        key = msvcrt.getwch().lower()
                    elif not self.is_windows and self.stdin_is_tty:
                        import select
                        dr, _, _ = select.select([sys.stdin], [], [], 0)
                        if dr: key = sys.stdin.read(1).lower()
                    
                    if key == 'q': self.stop_event.set()
                    if key == 'b': self.request_broadcast()
                    
                    live.update(self._build_layout())
                    time.sleep(0.05)
        finally:
            if not self.is_windows and self.stdin_is_tty:
                import termios
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_attrs)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default=DEFAULT_CAN_INTERFACE)
    args = parser.parse_args()
    try:
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        App(bus, args.iface).run()
    except Exception as e:
        print(f"Terminal Error: {e}")

if __name__ == "__main__":
    main()