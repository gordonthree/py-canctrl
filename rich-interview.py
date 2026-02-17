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
except Exception as e:
    print("Failed to import python-can. Install with: pip install python-can")
    raise

# --- Configuration & Message IDs ---
ACK_INTRO_ID       = 0x400  # Master -> Node: [ID_32]
REQ_NODE_INTRO_ID  = 0x401  # Master -> Bus: [0,0,0,0]
EPOCH_ID           = 0x40C  # Heartbeat/Time Sync
KNOB_ID            = 0x518  # Telemetry: Knob
TEMP_ID            = 0x51A  # Telemetry: Temp

DEFAULT_CAN_INTERFACE = 'can0'
LOG_MAX_LINES         = 2000

console = Console()

def safe_traceback():
    return "".join(traceback.format_exception(*sys.exc_info()))

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
        # node_id -> dict
        self.nodes = {}
        self.lock = threading.Lock()
        self.last_sync = {}

    def touch(self, node_id: int):
        with self.lock:
            # heartbeat, knob, temp, last, sub_mod_cnt, reported_crc, subs, interview_complete
            nd = self.nodes.setdefault(node_id, {
                'heartbeat': None, 
                'knob': None, 
                'temp': None, 
                'last': None,
                'sub_mod_cnt': 0,
                'reported_crc': 0,
                'subs': {},
                'interview_complete': False
            })
            nd['last'] = time.time()
            return nd

    def snapshot(self):
        with self.lock:
            ids = sorted(self.nodes.keys())
            now = time.time()
            rows = []
            for node_id in ids:
                nd = self.nodes[node_id]
                hb = nd['heartbeat'].strftime('%H:%M:%S') if nd['heartbeat'] else "-"
                age_secs = int(now - nd['last']) if nd['last'] else None
                age = f"{age_secs:>3}s" if age_secs is not None else "-"
                knob = f"{nd['knob']}" if nd['knob'] is not None else "-"
                temp = f"{nd['temp']:.2f}" if nd['temp'] is not None else "-"
                
                # Determine status string
                if nd['interview_complete']:
                    status = "[green]Ready[/]"
                elif nd['sub_mod_cnt'] > 0:
                    status = f"[yellow]Mod {len(nd['subs'])}/{nd['sub_mod_cnt']}[/]"
                else:
                    status = "[white]Seen[/]"
                
                rows.append((node_id, hb, age, knob, temp, status, age_secs))
            return rows

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
        self.table_scroll = 0

        # Platform key handling
        self.is_windows = (platform.system().lower() == "windows")
        self.stdin_is_tty = sys.stdin.isatty()
        if self.is_windows:
            try:
                import msvcrt
                self.msvcrt = msvcrt
            except Exception:
                self.msvcrt = None
        else:
            import select, termios, tty
            self.select = select
            self.termios = termios
            self.tty = tty
            self.orig_term_attrs = None

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.2)
                if msg is not None:
                    self.q.put(msg)
            except Exception as e:
                time.sleep(0.1)

    def _process_queue(self):
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            arb_id = msg.arbitration_id

            # 1. Heartbeat
            if arb_id == EPOCH_ID:
                if len(msg.data) >= 8:
                    node_id, unix_ts = struct.unpack('>II', msg.data[:8])
                    dt = datetime.fromtimestamp(unix_ts)
                    self.state.touch(node_id)['heartbeat'] = dt
                    self._send_rtc_sync(node_id) # Optional: keep synced

            # 2. Knob
            elif arb_id == KNOB_ID:
                if len(msg.data) >= 6:
                    node_id, val = struct.unpack('>IH', msg.data[:6])
                    self.state.touch(node_id)['knob'] = val

            # 3. Temp
            elif arb_id == TEMP_ID:
                if len(msg.data) >= 8:
                    node_id, celsius = struct.unpack('>If', msg.data[:8])
                    self.state.touch(node_id)['temp'] = celsius

            # 4. Interview Logic (0x700 - 0x7FF)
            elif 0x700 <= arb_id <= 0x7FF:
                if len(msg.data) >= 7:
                    try:
                        node_id = struct.unpack('>I', msg.data[0:4])[0]
                        node = self.state.touch(node_id)
                        
                        # Check if this is Part A/B or Identity
                        # Identity usually carries the sub_mod_cnt in byte 4
                        # We use a heuristic: if we haven't seen sub_mod_cnt yet, it's Identity
                        if node['sub_mod_cnt'] == 0:
                            node['sub_mod_cnt'] = msg.data[4]
                            node['reported_crc'] = (msg.data[5] << 8) | msg.data[6]
                            self.log.add(f"IDENTITY: 0x{node_id:08X} (Type 0x{arb_id:X}, {node['sub_mod_cnt']} Subs, CRC 0x{node['reported_crc']:04X})")
                        else:
                            # Sub-module data
                            mod_idx_byte = msg.data[4]
                            mod_idx = mod_idx_byte & 0x7F
                            is_part_b = bool(mod_idx_byte & 0x80)
                            
                            if mod_idx not in node['subs']:
                                node['subs'][mod_idx] = {'cfg': None, 'telemetry': None}
                            
                            if not is_part_b:
                                node['subs'][mod_idx]['cfg'] = msg.data[5:8]
                                self.log.add(f"INTERVIEW: 0x{node_id:08X} Mod {mod_idx} Part A (Config)")
                            else:
                                node['subs'][mod_idx]['telemetry'] = msg.data[5:8]
                                self.log.add(f"INTERVIEW: 0x{node_id:08X} Mod {mod_idx} Part B (Tele)")
                                
                            # If we just finished the last Part B of the last module
                            if is_part_b and (mod_idx + 1) >= node['sub_mod_cnt']:
                                node['interview_complete'] = True
                                self.log.add(f"INTERVIEW COMPLETE: 0x{node_id:08X}")

                        # Trigger the next packet in the node's sendIntroduction() routine
                        self._send_ack(node_id)
                        
                    except Exception as e:
                        self.log.add(f"Interview Error: {e}")

    def _send_ack(self, node_id: int):
        if self.dry_run: return
        payload = struct.pack('>I', node_id) # 4 bytes Big Endian
        msg = can.Message(arbitration_id=ACK_INTRO_ID, data=payload, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception: pass

    def _send_rtc_sync(self, node_id: int):
        if self.dry_run: return
        now = time.time()
        last = self.state.last_sync.get(node_id, 0)
        if now - last < 10.0: return
        self.state.last_sync[node_id] = now
        payload = struct.pack('>II', node_id, int(now))
        msg = can.Message(arbitration_id=EPOCH_ID, data=payload, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception: pass

    def request_broadcast(self):
        """ Broadcaster for ID 0x401 to wake up nodes """
        if self.dry_run: return
        msg = can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0,0,0,0], is_extended_id=False)
        self.bus.send(msg)
        self.log.add("Sent Global Interview Request (0x401)")

    # --- UI Rendering ---
    def _build_table(self):
        table = Table(show_header=True, header_style="bold cyan", expand=True)
        table.add_column("Node ID", width=12)
        table.add_column("Heartbeat", width=10)
        table.add_column("Age", width=6, justify="right")
        table.add_column("Knob mV", width=9, justify="right")
        table.add_column("CPU °C", width=9, justify="right")
        table.add_column("Status", width=12)

        rows = self.state.snapshot()
        for node_id, hb, age, knob, temp, status, age_secs in rows:
            style = "on grey15" if age_secs is not None and age_secs < 1 else ""
            table.add_row(f"0x{node_id:08X}", hb, age, knob, temp, status, style=style)
        return table

    def _build_layout(self):
        layout = Layout()
        layout.split(
            Layout(Text(f" CAN Master | {self.iface} | {time.strftime('%H:%M:%S')} (q=quit, b=broadcast)", style="bold cyan"), size=1),
            Layout(self._build_table(), name="body"),
            Layout(Panel(Text("\n".join(self.log.tail(self.log_height))), title="System Log", border_style="magenta"), size=self.log_height)
        )
        return layout

    def _read_key(self):
        if self.is_windows:
            return self.msvcrt.getwch() if self.msvcrt and self.msvcrt.kbhit() else None
        else:
            if not self.stdin_is_tty: return None
            dr, _, _ = self.select.select([sys.stdin], [], [], 0)
            return sys.stdin.read(1) if dr else None

    def run(self):
        self.reader_thread.start()
        # Request intros on startup
        time.sleep(0.5)
        self.request_broadcast()

        if not self.is_windows and self.stdin_is_tty:
            self.orig_term_attrs = self.termios.tcgetattr(sys.stdin)
            self.tty.setcbreak(sys.stdin.fileno())

        try:
            with Live(self._build_layout(), refresh_per_second=10, screen=True) as live:
                while not self.stop_event.is_set():
                    self._process_queue()
                    key = self._read_key()
                    if key:
                        if key.lower() == 'q': self.stop_event.set()
                        if key.lower() == 'b': self.request_broadcast()
                        if key == '+': self.log_height += 1
                        if key == '-': self.log_height = max(5, self.log_height - 1)

                    live.update(self._build_layout())
                    time.sleep(0.05)
        finally:
            if not self.is_windows and self.stdin_is_tty:
                self.termios.tcsetattr(sys.stdin, self.termios.TCSADRAIN, self.orig_term_attrs)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default=DEFAULT_CAN_INTERFACE)
    args = parser.parse_args()

    try:
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        app = App(bus, args.iface)
        app.run()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()