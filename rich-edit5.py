#!/usr/bin/env python3
import time
import struct
import threading
import queue
import json
import os
import csv
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

# --- Default Message IDs ---
ACK_INTRO_ID         = 0x400
REQ_NODE_INTRO_ID    = 0x401
CFG_ERASE_NVS_ID     = 0x434
CFG_REBOOT_ID        = 0x435
CFG_WRITE_NVS_ID     = 0x436
DATA_CONFIG_CRC_ID   = 0x526  
DATA_CFGWRITE_FAILED = 0x528  
CFG_SUB_DATA_MSG_ID  = 0x42C
CFG_SUB_INTRO_MSG_ID = 0x42D
CFG_SUB_RAW_DATA_ID  = 0x429

# --- ColorPicker Management IDs ---
COLORPICKER_WRITE_NVS_ID  = 0x42F
COLORPICKER_ADD_NODE_ID    = 0x433

# Mapping for CSV loading
ID_MAP_KEYS = {
    "ACK_INTRO": "ACK_INTRO_ID",
    "REQ_NODE_INTRO": "REQ_NODE_INTRO_ID",
    "CFG_ERASE_NVS": "CFG_ERASE_NVS_ID",
    "CFG_REBOOT": "CFG_REBOOT_ID",
    "CFG_WRITE_NVS": "CFG_WRITE_NVS_ID",
    "DATA_CONFIG_CRC": "DATA_CONFIG_CRC_ID",
    "COLORPICKER_WRITE_NVS": "COLORPICKER_WRITE_NVS_ID",
    "COLORPICKER_ADD_NODE": "COLORPICKER_ADD_NODE_ID"
}

# --- Constants ---
TARGET_CYD_ID         = 0x792      # CAN ID for the CYD hardware
HISTORY_FILE          = "cmd_history.json"
CSV_HEADER_ROW_START  = 6          # Rows to skip in CSV for message definitions

class CANState:
    def __init__(self, logger):
        self.nodes = {}
        self.lock = threading.Lock()
        self.logger = logger

    def touch(self, node_id: int):
        with self.lock:
            nd = self.nodes.setdefault(node_id, {
                'sub_mod_cnt': 0,
                'last_seen': time.time()
            })
            nd['last_seen'] = time.time()
            return nd

class App:
    def __init__(self, bus, iface, csv_path=None):
        self.bus, self.iface = bus, iface
        self.state = CANState(self)
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.is_windows = (platform.system().lower() == "windows")
        self.selected_idx = 0
        self.interaction_active = False
        self.detail_node = None 
        self.picker_buffer = set()  # Tracks staged nodes for the CYD

        if csv_path:
            self._load_definitions_from_csv(csv_path)

    def _load_definitions_from_csv(self, path):
        """
        @brief Load CAN ID definitions from a CSV file.
        """
        try:
            with open(path, 'r') as f:
                reader = csv.reader(f)
                for _ in range(CSV_HEADER_ROW_START): next(reader)
                for row in reader:
                    if len(row) < 2: continue
                    name, hex_id = row[0], row[1]
                    for key, var_name in ID_MAP_KEYS.items():
                        if key in name:
                            globals()[var_name] = int(hex_id, 16)
        except Exception as e:
            print(f"CSV Load Error: {e}")

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg: self.q.put(msg)
            except Exception: pass

    def _process_queue(self):
        now = time.time()
        while not self.q.empty():
            msg = self.q.get_nowait()
            arb_id, data = msg.arbitration_id, msg.data
            if 0x700 <= arb_id <= 0x7FF and len(data) >= 4:
                node_id = struct.unpack('>I', data[0:4])[0]
                node = self.state.touch(node_id)
                node['sub_mod_cnt'] = data[4] if len(data) > 4 else 0
                self.bus.send(can.Message(arbitration_id=ACK_INTRO_ID, data=struct.pack('>I', node_id)))
            elif len(data) >= 4:
                try:
                    nid = struct.unpack('>I', data[0:4])[0]
                    if nid in self.state.nodes: self.state.nodes[nid]['last_seen'] = now
                except: pass

    def _build_layout(self):
        layout = Layout()
        layout.split_column(Layout(name="header", size=3), Layout(name="body"))
        layout["header"].update(Panel(Text("CAN Master Node Manager", justify="center", style="bold blue")))

        table = Table(title="Discovered Nodes", expand=True)
        table.add_column("ID", style="cyan")
        table.add_column("Status", style="green")
        table.add_column("Last Seen", style="magenta")
        table.add_column("Submodules", style="yellow")

        node_ids = sorted(self.state.nodes.keys())
        for i, node_id in enumerate(node_ids):
            prefix = "> " if i == self.selected_idx else "  "
            marker = "[yellow]*[/yellow]" if node_id in self.picker_buffer else " "
            
            node_data = self.state.nodes[node_id]
            elapsed = time.time() - node_data.get('last_seen', 0)
            status_str = "[green]Online[/green]" if elapsed < 5.0 else "[red]Offline[/red]"
            
            table.add_row(
                f"{prefix}{marker}0x{node_id:03X}",
                status_str,
                f"{elapsed:.1f}s ago",
                str(node_data.get('sub_mod_cnt', 0))
            )

        footer = "[q] Quit | [up/down] Select | [m] Toggle | [c] Custom | [e] Edit | [p] Persist | [+] Stage | [l] Send"
        layout["body"].update(Panel(table, subtitle=Text(footer, justify="center")))
        return layout

    def run(self):
        self.reader.start()
        self.bus.send(can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0]*4))
        
        # Terminal Setup
        self.term_orig = None
        if not self.is_windows and sys.stdin.isatty():
            import tty, termios
            self.term_orig = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        try:
            with Live(self._build_layout(), refresh_per_second=10, screen=True) as live:
                while not self.stop_event.is_set():
                    if not self.interaction_active:
                        self._process_queue()
                        key = self._get_key()
                        node_ids = sorted(self.state.nodes.keys())
                        
                        if key == 'q': self.stop_event.set()
                        elif key == 'up': self.selected_idx -= 1
                        elif key == 'down': self.selected_idx += 1
                        
                        # ColorPicker Features
                        elif key == '+':
                            if node_ids:
                                self.picker_buffer.add(node_ids[self.selected_idx % len(node_ids)])
                        elif key == '-':
                            if node_ids:
                                target = node_ids[self.selected_idx % len(node_ids)]
                                if target in self.picker_buffer: self.picker_buffer.remove(target)
                        elif key == 'l':
                            if node_ids and node_ids[self.selected_idx % len(node_ids)] == TARGET_CYD_ID:
                                for nid in self.picker_buffer:
                                    data = struct.pack(">I", nid) + b'\x00\x00\x00\x00'
                                    self.bus.send(can.Message(arbitration_id=COLORPICKER_ADD_NODE_ID, data=data))
                                    time.sleep(0.02)
                                self.bus.send(can.Message(arbitration_id=COLORPICKER_WRITE_NVS_ID, data=[0]*4))
                        
                        # Original Commands
                        elif key == 'm':
                            if node_ids:
                                current_id = node_ids[self.selected_idx % len(node_ids)]
                                self.detail_node = current_id if self.detail_node != current_id else None
                        elif key == 'c': live.stop(); self.send_custom_command(); live.start()
                        elif key == 'e': live.stop(); self.edit_module(); live.start()
                        elif key == 'p': live.stop(); self.persist_selected(); live.start()
                        elif key == 'x': live.stop(); self.erase_selected(); live.start()
                            
                        live.update(self._build_layout())
                    time.sleep(0.05)
        finally:
            self._restore_terminal()
            if self.bus:
                self.bus.shutdown()

    def _get_key(self):
        if self.is_windows:
            import msvcrt
            if not msvcrt.kbhit(): return None
            ch = msvcrt.getwch()
            if ch == '\xe0':
                ch = msvcrt.getwch()
                return 'up' if ch == 'H' else 'down' if ch == 'P' else None
            return ch.lower()
        else:
            import select
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if not dr: return None
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                seq = sys.stdin.read(2)
                if seq == '[A': return 'up'
                if seq == '[B': return 'down'
            return ch.lower()

    def _restore_terminal(self):
        if not self.is_windows and self.term_orig:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_orig)

    # Placeholders for original functionality
    def edit_module(self): pass
    def send_custom_command(self): pass
    def persist_selected(self): pass
    def erase_selected(self): pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--csv", help="Path to CAN messages CSV file")
    args = parser.parse_args()
    try:
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        App(bus, args.iface, args.csv).run()
    except Exception as e: print(f"Error: {e}")