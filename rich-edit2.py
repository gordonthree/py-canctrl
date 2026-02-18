#!/usr/bin/env python3
import time
import struct
import threading
import queue
import sys
import argparse
import can

from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
from rich.live import Live
from rich.text import Text
from rich.console import Console

# --- Global Constants ---
ACK_INTRO_ID         = 0x400
REQ_NODE_INTRO_ID    = 0x401
CFG_ERASE_NVS_ID     = 0x41B
CFG_REBOOT_ID        = 0x41C
CFG_WRITE_NVS_ID     = 0x41D
DATA_CONFIG_CRC_ID   = 0x526  

MAX_SUB_MODULES      = 8
MAX_ARGB_SUBMODULES   = 4

# Map Intro IDs to Friendly Names (Sync with firmware)
CFG_MAP = {
    0x702: "ARGB Strip",
    0x711: "Digi In",
    0x700: "Anlg Strip",
    0x70A: "AnaBkLt",
    0x70B: "LCD Touch",
    0x744: "Digi Out",
    0x745: "PWM Out",
    0x710: "Anlg In",
    0x715: "Strobe Out"
}

class SystemState:
    def __init__(self):
        self.nodes = {}
        self.lock = threading.Lock()
        self.logs = []

    def touch(self, node_id):
        if node_id not in self.nodes:
            self.nodes[node_id] = {
                'node_id': node_id,
                'node_type_msg': 0,
                'sub_mod_cnt': 0,
                'reported_crc': 0,
                'subs': {},
                'interview_complete': False,
                'last_rx': time.time(),
                'nvs_status': "[grey]Idle[/]",
                'pending_ack': False  # Prevents ACK spamming
            }
        return self.nodes[node_id]

class App:
    def __init__(self, bus):
        self.bus = bus
        self.state = SystemState()
        self.q = queue.Queue()
        self.running = True
        self.selected_idx = 0
        self.console = Console()
        
        # Terminal Setup
        self.is_windows = sys.platform == "win32"
        if not self.is_windows:
            import tty, termios
            self.fd = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(self.fd)

    def add_log(self, msg, node_id=None):
        prefix = f"[0x{node_id:08X}] " if node_id else ""
        self.state.logs.append(f"{prefix}{msg}")
        if len(self.state.logs) > 8: self.state.logs.pop(0)

    def _process_queue(self):
        while not self.q.empty():
            msg = self.q.get_nowait()
            arb_id, data = msg.arbitration_id, msg.data
            
            # 1. Identity Frame (0x700-0x7FF)
            if 0x700 <= arb_id <= 0x7FF and len(data) >= 7:
                node_id = struct.unpack('>I', data[0:4])[0]
                with self.state.lock:
                    node = self.state.touch(node_id)
                    node['last_rx'] = time.time()
                    
                    if not node['interview_complete'] and node['sub_mod_cnt'] == 0:
                        node['node_type_msg'] = arb_id
                        node['sub_mod_cnt'] = data[4]
                        node['reported_crc'] = (data[5] << 8) | data[6]
                        self.add_log("Interview Started", node_id)
                        self.bus.send(can.Message(arbitration_id=ACK_INTRO_ID, data=struct.pack('>I', node_id)))
                        node['pending_ack'] = True

            # 2. Submodule Data (Check NodeID in bytes 0-3)
            elif len(data) >= 5:
                try:
                    node_id = struct.unpack('>I', data[0:4])[0]
                    with self.state.lock:
                        if node_id in self.state.nodes:
                            node = self.state.nodes[node_id]
                            node['last_rx'] = time.time()
                            
                            if not node['interview_complete']:
                                raw_idx = data[4]
                                idx = raw_idx & 0x7F
                                is_part_b = bool(raw_idx & 0x80)

                                if idx not in node['subs']:
                                    node['subs'][idx] = {'type': CFG_MAP.get(arb_id, f"0x{arb_id:X}")}

                                # Move to next state
                                if is_part_b and (idx + 1) >= node['sub_mod_cnt']:
                                    node['interview_complete'] = True
                                    node['pending_ack'] = False
                                    self.add_log("Interview Complete", node_id)
                                else:
                                    self.bus.send(can.Message(arbitration_id=ACK_INTRO_ID, data=struct.pack('>I', node_id)))
                except: pass

    def _build_layout(self):
        layout = Layout()
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="body", ratio=1),
            Layout(name="logs", size=10),
            Layout(name="footer", size=3)
        )
        
        layout["header"].update(Panel(Text("ESP32 NODE MASTER", justify="center", style="bold blue")))
        
        table = Table(expand=True)
        table.add_column("ID", style="cyan", no_wrap=True)
        table.add_column("Type")
        table.add_column("Subs")
        table.add_column("CRC")
        table.add_column("Status")
        table.add_column("HB")

        with self.state.lock:
            nodes = list(self.state.nodes.values())
            for i, n in enumerate(nodes):
                style = "reverse" if i == self.selected_idx else ""
                hb = int(time.time() - n['last_rx'])
                table.add_row(
                    f"0x{n['node_id']:08X}", 
                    f"0x{n['node_type_msg']:03X}", 
                    f"{len(n['subs'])}/{n['sub_mod_cnt']}",
                    f"0x{n['reported_crc']:04X}",
                    n['nvs_status'],
                    f"{hb}s",
                    style=style
                )

        layout["body"].update(Panel(table, title="Nodes"))
        layout["logs"].update(Panel("\n".join(self.state.logs), title="System Logs"))
        layout["footer"].update(Panel(Text("[R] Refresh  [X] Erase  [Q] Quit", justify="center")))
        return layout

    def run(self):
        if not self.is_windows:
            import tty, termios
            tty.setcbreak(self.fd)

        # Receiver Thread
        def rx_thread():
            while self.running:
                m = self.bus.recv(0.1)
                if m: self.q.put(m)
        
        t = threading.Thread(target=rx_thread, daemon=True)
        t.start()

        try:
            with Live(self._build_layout(), console=self.console, screen=True, refresh_per_second=4) as live:
                while self.running:
                    self._process_queue()
                    # Check for basic keyboard input here (non-blocking)
                    # ... (keyboard logic) ...
                    live.update(self._build_layout())
                    time.sleep(0.1)
        finally:
            if not self.is_windows:
                import termios
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="can0")
    args = parser.parse_args()
    try:
        # Use virtual bus for testing if socketcan fails
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        App(bus).run()
    except Exception as e:
        print(f"Error: {e}")