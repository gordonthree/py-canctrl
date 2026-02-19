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

# --- Default Message IDs (Populated dynamically if --csv is used) ---
ACK_INTRO_ID         = 0x400
REQ_NODE_INTRO_ID    = 0x401
CFG_ERASE_NVS_ID     = 0x434
CFG_REBOOT_ID        = 0x435
CFG_WRITE_NVS_ID     = 0x436
DATA_CONFIG_CRC_ID   = 0x526  
DATA_CFGWRITE_FAILED = 0x528  

# Mapping of script variable names to CSV 'c def' names
ID_MAP_KEYS = {
    "ACK_INTRO": "ACK_INTRO_ID",
    "REQ_NODE_INTRO": "REQ_NODE_INTRO_ID",
    "CFG_ERASE_NVS": "CFG_ERASE_NVS_ID",
    "CFG_REBOOT": "CFG_REBOOT_ID",
    "CFG_WRITE_NVS": "CFG_WRITE_NVS_ID",
    "DATA_CONFIG_CRC": "DATA_CONFIG_CRC_ID",
    "DATA_CFGWRITE_FAILED": "DATA_CFGWRITE_FAILED"
}

# Friendly Names for Hardware/Config IDs (Populated from CSV 'Comments')
CFG_MAP = {}

# --- Constants ---
SUBMODULE_STRUCT_SIZE = 16 # Bytes per submodule in ESP32 struct
NODEINFO_STRUCT_SIZE  = 136 # Total bytes for NodeInfo_t struct
NVS_TIMEOUT_SECONDS   = 5
EMPTY_CONFIG_CRC      = 0xFFFF 
MAX_CUSTOM_DATA_LEN   = 4 # Maximum additional data bytes for custom commands
HISTORY_FILE          = "cmd_history.json"
HISTORY_LIMIT         = 10 # Max number of saved commands to display
CSV_HEADER_ROW_START  = 6 # 1-indexed row where headers are located (row 6 in your file)

def crc16_ccitt(data: bytes, initial=0xFFFF):
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
        active_mods = min(nd['sub_mod_cnt'], 8) # CRC only considers up to 8 submodules
        for i in range(active_mods):
            offset = i * SUBMODULE_STRUCT_SIZE
            if i in nd['subs']:
                s = nd['subs'][i]
                if s['cfg']: buf[offset:offset+3] = s['cfg'][:3]
                struct.pack_into('<H', buf, offset + 9, s['intro_id'])
                if s['telemetry']:
                    struct.pack_into('<H', buf, offset + 11, s['telemetry']['id'])
                    buf[offset + 13], buf[offset + 14] = 8, s['telemetry']['dlc']
                    buf[offset + 15] = 1 if s['telemetry']['save'] else 0
        struct.pack_into('<I', buf, 128, node_id)           
        struct.pack_into('<H', buf, 132, nd['node_type_msg'])
        buf[134], buf[135] = 8, nd['sub_mod_cnt']                        
        return crc16_ccitt(buf)

class App:
    def __init__(self, bus, iface, csv_path=None):
        self.bus, self.iface = bus, iface
        self.all_logs = deque(maxlen=100)
        self.state = CANState(self)
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.is_windows = (platform.system().lower() == "windows")
        self.selected_idx = 0
        self.interaction_active = False
        self.filter_enabled = False
        self.detail_node = None 
        self.term_orig = None
        
        if csv_path:
            self._load_definitions_from_csv(csv_path)
            
        self.cmd_history = self._load_history()

    def _load_definitions_from_csv(self, filepath):
        """
        Parses CSV with improved header matching to handle newlines and whitespace.
        """
        if not os.path.exists(filepath):
            self.add_log(f"CSV Error: File not found at {filepath}")
            return

        globals_count = 0
        map_count = 0

        try:
            with open(filepath, 'r', encoding='utf-8-sig') as f:
                reader = list(csv.reader(f))
                if len(reader) < CSV_HEADER_ROW_START:
                    self.add_log(f"CSV Error: File too short ({len(reader)} rows)")
                    return

                # Normalize headers: remove newlines and extra spaces
                header = [h.replace('\n', ' ').strip() for h in reader[CSV_HEADER_ROW_START - 1]]
                
                try:
                    idx_id = next(i for i, v in enumerate(header) if "Message ID" in v)
                    idx_def = next(i for i, v in enumerate(header) if "c def" in v)
                    idx_comm = next(i for i, v in enumerate(header) if "Comments" in v)
                except StopIteration:
                    self.add_log(f"CSV Error: Could not find required columns in header: {header}")
                    return

                for row in reader[CSV_HEADER_ROW_START:]:
                    if len(row) <= max(idx_id, idx_def, idx_comm):
                        continue
                    
                    id_str = row[idx_id].strip()
                    c_def = row[idx_def].strip()
                    comment = row[idx_comm].strip()

                    if not id_str or not id_str.startswith("0x"):
                        continue

                    try:
                        val = int(id_str, 16)
                        # Update global constants
                        if c_def in ID_MAP_KEYS:
                            globals()[ID_MAP_KEYS[c_def]] = val
                            globals_count += 1
                        
                        # Populate CFG_MAP
                        display_name = comment if comment else c_def
                        if display_name:
                            CFG_MAP[val] = display_name.title()
                            map_count += 1
                    except ValueError:
                        continue
            
            self.add_log(f"CSV Loaded: {os.path.basename(filepath)}")
            self.add_log(f"Debug: Mapped {globals_count} globals, {map_count} CFG_MAP entries")

        except Exception as e:
            self.add_log(f"CSV Exception: {str(e)}")

    def _load_history(self):
        if os.path.exists(HISTORY_FILE):
            try:
                with open(HISTORY_FILE, 'r') as f:
                    return json.load(f)
            except Exception: return []
        return []

    def _save_history(self, cmd_id, data_bytes, label=""):
        entry = {'id': cmd_id, 'data': data_bytes, 'label': label}
        self.cmd_history = [c for c in self.cmd_history if not (c['id'] == cmd_id and c['data'] == data_bytes)]
        self.cmd_history.insert(0, entry)
        self.cmd_history = self.cmd_history[:HISTORY_LIMIT]
        try:
            with open(HISTORY_FILE, 'w') as f:
                json.dump(self.cmd_history, f)
        except Exception: pass

    def add_log(self, text, node_id=None):
        tag = f"0x{node_id:08X}" if node_id else "SYSTEM"
        self.all_logs.append((tag, f"[{time.strftime('%H:%M:%S')}] {tag}: {text}"))

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg: self.q.put(msg)
            except Exception: pass

    def _process_queue(self):
        now = time.time()
        with self.state.lock:
            for nid, n in self.state.nodes.items():
                if n['nvs_status'] == "Writing..." and (now - n['nvs_timestamp'] > NVS_TIMEOUT_SECONDS):
                    n['nvs_status'] = "[bold yellow]Timeout[/]"

        while not self.q.empty():
            msg = self.q.get_nowait()
            arb_id, data = msg.arbitration_id, msg.data
            if 0x700 <= arb_id <= 0x7FF and len(data) >= 4:
                node_id = struct.unpack('>I', data[0:4])[0]
                node = self.state.touch(node_id)
                if not node['interview_complete']:
                    if node['sub_mod_cnt'] == 0:
                        node['node_type_msg'], node['sub_mod_cnt'] = arb_id, data[4]
                        node['reported_crc'] = (data[5] << 8) | data[6]
                        self.add_log(f"Started Interview (Subs: {data[4]})", node_id)
                    else:
                        idx, is_b = data[4] & 0x7F, bool(data[4] & 0x80)
                        if idx < 8:
                            if idx not in node['subs']: node['subs'][idx] = {'cfg': None, 'telemetry': None, 'intro_id': arb_id}
                            if not is_b: node['subs'][idx]['cfg'] = bytes(data[5:8])
                            else:
                                node['subs'][idx]['telemetry'] = {'id': (data[5] << 8) | data[6], 'dlc': data[7] & 0x0F, 'save': bool(data[7] & 0x80)}
                                if (idx + 1) >= node['sub_mod_cnt']:
                                    node['interview_complete'] = True
                                    node['calculated_crc'] = self.state.calculate_node_crc(node_id)
                                    self.add_log("Interview Complete", node_id)
                    self.bus.send(can.Message(arbitration_id=ACK_INTRO_ID, data=struct.pack('>I', node_id)))
            elif arb_id == DATA_CONFIG_CRC_ID:
                nid = struct.unpack('>I', data[0:4])[0]
                self.state.touch(nid)['nvs_status'] = "[bold green]Success[/]"
                self.add_log("NVS Commit Successful", nid)
                self.bus.send(can.Message(arbitration_id=CFG_REBOOT_ID, data=struct.pack('>I', nid)))
            elif len(data) >= 4:
                try:
                    nid = struct.unpack('>I', data[0:4])[0]
                    if nid in self.state.nodes: self.state.nodes[nid]['last_rx'] = now
                except: pass

    def _restore_terminal(self):
        if not self.is_windows and self.term_orig:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_orig)

    def _setup_terminal(self):
        if not self.is_windows:
            import tty
            tty.setcbreak(sys.stdin.fileno())

    def edit_module(self):
        self.interaction_active = True
        node_ids = sorted(self.state.nodes.keys())
        if not node_ids: return
        target_id = node_ids[self.selected_idx % len(node_ids)]
        
        self._restore_terminal()
        print(f"\n--- Editing Node 0x{target_id:08X} ---")
        try:
            mod_idx = int(input("Submodule Index (0-7): "))
            new_type = int(input("New Type ID (hex): "), 16)
            pin = int(input("GPIO Pin: "))
            
            payload = struct.pack('>I B B B B', target_id, mod_idx, pin, 0, 0)
            self.bus.send(can.Message(arbitration_id=new_type, data=payload))
            self.add_log(f"Sent Config Update for Index {mod_idx}", target_id)
            
            with self.state.lock:
                n = self.state.nodes[target_id]
                n['interview_complete'], n['sub_mod_cnt'], n['subs'] = False, 0, {}
        except Exception as e: self.add_log(f"Edit Error: {e}")
        
        self._setup_terminal()
        self.interaction_active = False

    def send_custom_command(self):
        self.interaction_active = True
        node_ids = sorted(self.state.nodes.keys())
        if not node_ids: 
            self.interaction_active = False
            return
        
        target_id = node_ids[self.selected_idx % len(node_ids)]
        self._restore_terminal()
        
        print(f"\n--- Custom Command to Node 0x{target_id:08X} ---")
        if self.cmd_history:
            print("History:")
            for i, h in enumerate(self.cmd_history):
                label = f" ({h['label']})" if h.get('label') else ""
                data_hex = " ".join(f"{b:02X}" for b in h['data'])
                print(f" [{i}] ID: 0x{h['id']:X}{label} | Data: {data_hex}")
            print(" [n] New Command Entry")
            choice = input("Select option: ").lower()
        else:
            choice = 'n'

        try:
            if choice == 'n':
                cmd_id = int(input("Enter CAN ID (hex): "), 16)
                data_str = input("Enter up to 4 data bytes (hex, space separated): ").split()
                custom_bytes = [int(b, 16) for b in data_str[:MAX_CUSTOM_DATA_LEN]]
                label = input("Label (optional): ").strip()
                self._save_history(cmd_id, custom_bytes, label)
            else:
                idx = int(choice)
                cmd_id = self.cmd_history[idx]['id']
                custom_bytes = self.cmd_history[idx]['data']
                self._save_history(cmd_id, custom_bytes, self.cmd_history[idx].get('label', ''))
            
            payload = struct.pack('>I', target_id) + bytes(custom_bytes)
            self.bus.send(can.Message(arbitration_id=cmd_id, data=payload))
            self.add_log(f"Sent 0x{cmd_id:X} + {bytes(custom_bytes).hex().upper()}", target_id)
        except Exception as e:
            self.add_log(f"Command Error: {e}")
            
        self._setup_terminal()
        self.interaction_active = False

    def persist_selected(self):
        self.interaction_active = True
        node_ids = sorted(self.state.nodes.keys())
        if not node_ids: return
        target_id = node_ids[self.selected_idx % len(node_ids)]
        node = self.state.nodes[target_id]

        if not node['interview_complete'] or (node['calculated_crc'] == node['reported_crc'] and node['reported_crc'] != EMPTY_CONFIG_CRC):
            self.add_log("No changes to persist.", target_id)
            self.interaction_active = False
            return

        self._restore_terminal()
        confirm = input(f"\nPersist changes to Node 0x{target_id:08X} NVS? (y/n): ").lower()
        if confirm == 'y':
            self.bus.send(can.Message(arbitration_id=CFG_WRITE_NVS_ID, data=struct.pack('>I H', target_id, node['calculated_crc'])))
            node['nvs_status'], node['nvs_timestamp'] = "Writing...", time.time()
            self.add_log("Requested NVS Commit", target_id)
        else:
            self.add_log("Persistence cancelled.")
        
        self._setup_terminal()
        self.interaction_active = False

    def erase_selected(self):
        self.interaction_active = True
        node_ids = sorted(self.state.nodes.keys())
        if not node_ids: return
        target_id = node_ids[self.selected_idx % len(node_ids)]

        self._restore_terminal()
        print(f"\n[WARNING] ERASE NVS on Node 0x{target_id:08X}")
        confirm = input(f"This will wipe NVS and trigger a remote REBOOT. Proceed? (y/n): ").lower()
        if confirm == 'y':
            self.bus.send(can.Message(arbitration_id=CFG_ERASE_NVS_ID, data=struct.pack('>I', target_id)))
            self.add_log("Sent NVS Erase & Reboot Command", target_id)
            
            with self.state.lock:
                n = self.state.nodes[target_id]
                n['interview_complete'] = False
                n['sub_mod_cnt'] = 0
                n['subs'] = {}
                n['nvs_status'] = "Erasing..."
        else:
            self.add_log("Erase cancelled.")
        
        self._setup_terminal()
        self.interaction_active = False

    def _build_layout(self):
        node_ids = sorted(self.state.nodes.keys())
        target_id_val = node_ids[self.selected_idx % len(node_ids)] if node_ids else None
        target_id_str = f"0x{target_id_val:08X}" if target_id_val else None
        
        if self.filter_enabled and target_id_str:
            display_logs = [log[1] for log in self.all_logs if log[0] == target_id_str][-15:]
            filter_status = f"[bold yellow]Filter: ON [{target_id_str}][/]"
        else:
            display_logs = [log[1] for log in self.all_logs][-15:]
            filter_status = "[dim]Filter: OFF[/]"

        table = Table(show_header=True, header_style="bold cyan", expand=True, box=None)
        table.add_column("S", width=2)
        table.add_column("Node ID", width=12)
        table.add_column("HB", width=6, justify="right")
        table.add_column("Sub-Module Summary", ratio=1)
        table.add_column("CRC Status", width=18)
        table.add_column("NVS", width=10)

        now = time.time()
        for i, nid in enumerate(node_ids):
            n = self.state.nodes[nid]
            is_sel = (i == self.selected_idx % len(node_ids))
            row_style = "reverse" if is_sel else ""
            marker = ">" if is_sel else ""
            hb = int(now - n['last_rx'])
            hb_col = "red" if hb > 10 else "white"
            
            sum_color = "cyan" if (self.filter_enabled and is_sel) else "dim"
            mod_summary = [f"M{k}:[{sum_color}]{CFG_MAP.get(s['intro_id'], f'0x{s['intro_id']:03X}')}[/]" for k, s in sorted(n['subs'].items())]
            
            if n['reported_crc'] == EMPTY_CONFIG_CRC:
                crc_str = "[bold red]NEEDS CONFIG[/]"
            elif n['interview_complete'] and n['calculated_crc'] == n['reported_crc']:
                crc_str = "[green]MATCH[/]"
            else:
                crc_str = "[yellow]MODIFIED[/]"
                
            table.add_row(marker, f"0x{nid:08X}", f"[{hb_col}]{hb}s[/]", " ".join(mod_summary), crc_str, n['nvs_status'], style=row_style)

            if self.detail_node == nid:
                detail_text = ""
                for idx, s in sorted(n['subs'].items()):
                    raw_cfg = s['cfg'].hex().upper() if s['cfg'] else "NONE"
                    t_id = f"0x{s['telemetry']['id']:03X}" if s['telemetry'] else "N/A"
                    detail_text += f"  └─ [bold]Idx {idx}[/]: Type: 0x{s['intro_id']:03X} | Raw: {raw_cfg} | Telem ID: {t_id}\n"
                table.add_row("", "", "", detail_text.strip(), "", "")

        l = Layout()
        l.split(
            Layout(Text(f" CAN Master | Arrows: Select | (m)ore | (c)md | (e)dit | (p)ersist | e(x)punge | (f)ilter | (b)roadcast | (q)uit   {filter_status}", style="bold reverse blue"), size=1),
            Layout(table, name="body"),
            Layout(Panel(Text("\n".join(display_logs)), title="System Log", border_style="cyan"), size=10)
        )
        return l

    def run(self):
        self.reader.start()
        self.bus.send(can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0]*4))
        if not self.is_windows and sys.stdin.isatty():
            import termios
            self.term_orig = termios.tcgetattr(sys.stdin)
            self._setup_terminal()
        try:
            with Live(self._build_layout(), refresh_per_second=10, screen=True) as live:
                while not self.stop_event.is_set():
                    if not self.interaction_active:
                        self._process_queue()
                        key = self._get_key()
                        if key == 'q': self.stop_event.set()
                        elif key == 'up': self.selected_idx -= 1
                        elif key == 'down': self.selected_idx += 1
                        elif key == 'f': self.filter_enabled = not self.filter_enabled
                        elif key == 'b': 
                            for n in self.state.nodes.values(): n['interview_complete'], n['sub_mod_cnt'] = False, 0
                            self.bus.send(can.Message(arbitration_id=REQ_NODE_INTRO_ID, data=[0]*4))
                        elif key == 'm':
                            node_ids = sorted(self.state.nodes.keys())
                            if node_ids:
                                current_id = node_ids[self.selected_idx % len(node_ids)]
                                self.detail_node = current_id if self.detail_node != current_id else None
                        elif key == 'c':
                            live.stop(); self.send_custom_command(); live.start()
                        elif key == 'p':
                            live.stop(); self.persist_selected(); live.start()
                        elif key == 'x':
                            live.stop(); self.erase_selected(); live.start()
                        elif key == 'e':
                            live.stop(); self.edit_module(); live.start()
                        live.update(self._build_layout())
                    time.sleep(0.05)
        finally:
            self._restore_terminal()

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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--csv", help="Path to CAN messages CSV file to load definitions")
    args = parser.parse_args()
    try:
        bus = can.interface.Bus(channel=args.iface, interface='socketcan')
        App(bus, args.iface, args.csv).run()
    except Exception as e: print(f"Error: {e}")