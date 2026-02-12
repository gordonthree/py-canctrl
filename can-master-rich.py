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

# --- Configuration defaults ---
DEFAULT_CAN_INTERFACE = 'can0'
ACK_ID                = 0x400
EPOCH_ID              = 0x40C
KNOB_ID               = 0x518
TEMP_ID               = 0x51A

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
                console.print_exception()

class CANState:
    def __init__(self):
        # node_id -> dict
        self.nodes = {}
        self.synced = set()
        self.lock = threading.Lock()
        self.last_sync = {}  # optional: track last RTC sync per node

    def touch(self, node_id: int):
        with self.lock:
            nd = self.nodes.setdefault(node_id, {'heartbeat': None, 'knob': None, 'temp': None, 'last': None})
            nd['last'] = time.time()
            return nd

    def update_heartbeat(self, node_id: int, dt: datetime):
        nd = self.touch(node_id)
        nd['heartbeat'] = dt

    def update_knob(self, node_id: int, val: int):
        nd = self.touch(node_id)
        nd['knob'] = val

    def update_temp(self, node_id: int, celsius: float):
        nd = self.touch(node_id)
        nd['temp'] = celsius

    def snapshot(self):
        """Return a sorted snapshot to render."""
        with self.lock:
            ids = sorted(self.nodes.keys())
            now = time.time()
            rows = []
            for node_id in ids:
                nd = self.nodes[node_id]
                hb = nd['heartbeat'].strftime('%Y-%m-%d %H:%M:%S') if nd['heartbeat'] else "-"
                age_secs = int(now - nd['last']) if nd['last'] else None
                age = f"{age_secs:>3}s" if age_secs is not None else "-"
                knob = f"{nd['knob']}" if nd['knob'] is not None else "-"
                temp = f"{nd['temp']:.2f}" if nd['temp'] is not None else "-"
                rows.append((node_id, hb, age, knob, temp, age_secs))
            return rows

class App:
    def __init__(self, bus, iface: str, dry_run: bool = False, debug_file: str | None = None):
        self.bus = bus
        self.iface = iface
        self.dry_run = dry_run
        self.debug_file = debug_file

        self.log = LogBuffer()
        self.state = CANState()

        # Reader thread
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)

        # UI state
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

    # --- CAN reader ---
    def _reader_loop(self):
        try:
            if self.dry_run:
                # Simulate data in dry-run
                fake_nodes = [0x140B7731, 0xA81B9E74, 0x3B6986B0]
                last = time.time()
                while not self.stop_event.is_set():
                    now = time.time()
                    if now - last > 0.3:
                        for nid in fake_nodes:
                            # simulate heartbeat each ~3 seconds
                            if int(now) % 3 == 0:
                                dt = int(now)
                                self.q.put(("EPOCH", nid, dt))
                            # simulate knob
                            self.q.put(("KNOB", nid, 1000 + (int(now*100) % 1200)))
                            # simulate temp
                            self.q.put(("TEMP", nid, 20.0 + (int(now*10) % 10) / 10.0))
                        last = now
                    time.sleep(0.05)
                return

            while not self.stop_event.is_set():
                try:
                    msg = self.bus.recv(timeout=0.2)
                    if msg is not None:
                        self.q.put(msg)
                except can.CanError as e:
                    self.q.put(("__ERROR__", f"CAN recv error: {e}"))
                except Exception as e:
                    tb = traceback.format_exception_only(type(e), e)
                    self.q.put(("__ERROR__", f"Reader exception: {''.join(tb)}"))
                    time.sleep(0.2)
        except Exception as e:
            msg = f"Reader loop fatal: {e}\n{safe_traceback()}"
            self._debug_log(msg)
            self.q.put(("__ERROR__", msg))

    def _debug_log(self, text: str):
        if self.debug_file:
            try:
                with open(self.debug_file, "a", encoding="utf-8") as f:
                    f.write(text + "\n")
            except Exception:
                pass

    def _process_queue(self):
        while True:
            try:
                item = self.q.get_nowait()
            except queue.Empty:
                break

            if isinstance(item, tuple) and item[0] == "__ERROR__":
                self.log.add(item[1])
                self._debug_log(item[1])
                continue

            # Dry-run synthetic messages
            if isinstance(item, tuple) and item[0] in ("EPOCH", "KNOB", "TEMP"):
                kind, node_id, value = item
                if kind == "EPOCH":
                    dt = datetime.fromtimestamp(value)
                    self.state.update_heartbeat(node_id, dt)
                    self.log.add(f"HEARTBEAT from Node 0x{node_id:08X} {dt.strftime('%Y-%m-%d %H:%M:%S')}")
                elif kind == "KNOB":
                    self.state.update_knob(node_id, int(value))
                elif kind == "TEMP":
                    self.state.update_temp(node_id, float(value))
                continue

            # Real CAN messages
            msg = item
            arb_id = msg.arbitration_id

            if arb_id == EPOCH_ID:
                if len(msg.data) >= 8:
                    try:
                        node_id, unix_ts = struct.unpack('>II', msg.data[:8])
                        dt = datetime.fromtimestamp(unix_ts)
                        self.state.update_heartbeat(node_id, dt)
                        self.log.add(f"HEARTBEAT from Node 0x{node_id:08X} {dt.strftime('%Y-%m-%d %H:%M:%S')}")
                        self._send_rtc_sync(node_id)  # throttled below
                    except struct.error:
                        self.log.add(f"Error: Malformed 0x40C packet (Len: {len(msg.data)})")
                else:
                    self.log.add(f"Error: 0x40C data length {len(msg.data)}")

            elif arb_id == KNOB_ID:
                if len(msg.data) >= 6:
                    try:
                        node_id, knob_val = struct.unpack('>IH', msg.data[:6])
                        self.state.update_knob(node_id, knob_val)
                    except struct.error:
                        self.log.add(f"Error: Malformed 0x518 packet (Len: {len(msg.data)})")
                else:
                    self.log.add(f"Error: 0x518 data length {len(msg.data)}")

            elif arb_id == TEMP_ID:
                if len(msg.data) >= 8:
                    try:
                        node_id, celsius = struct.unpack('>If', msg.data[:8])
                        self.state.update_temp(node_id, celsius)
                    except struct.error:
                        self.log.add(f"Error: Malformed 0x51A packet (Len: {len(msg.data)})")
                else:
                    self.log.add(f"Error: 0x51A data length {len(msg.data)}")

            elif 0x700 <= arb_id <= 0x7FF:
                if len(msg.data) >= 4:
                    try:
                        remote_node_id = struct.unpack('>I', msg.data[:4])[0]
                        self.log.add(f"INTRO PKT: 0x{arb_id:X} from 0x{remote_node_id:X}")
                        self._send_ack(remote_node_id)
                        if remote_node_id not in self.state.synced:
                            time.sleep(0.01)
                            self._send_rtc_sync(remote_node_id)
                            self.state.synced.add(remote_node_id)
                    except struct.error:
                        self.log.add(f"Error: Malformed Intro packet (Len: {len(msg.data)})")
                else:
                    self.log.add(f"Error: Intro data length {len(msg.data)}")

            else:
                self.log.add(f"UNHANDLED ID 0x{arb_id:X} len={len(msg.data)} data={msg.data.hex()}")

    def _send_ack(self, node_id: int):
        if self.dry_run:
            return
        ack_payload = struct.pack('>I', node_id) + b'\x00\x00\x00\x00'
        ack_msg = can.Message(arbitration_id=ACK_ID, data=ack_payload, is_extended_id=False)
        try:
            self.bus.send(ack_msg)
        except Exception as e:
            self.log.add(f"ACK send failed: {e}")
            self._debug_log(f"ACK send failed: {e}")

    def _send_rtc_sync(self, node_id: int):
        if self.dry_run:
            return
        now = time.time()
        last = self.state.last_sync.get(node_id, 0)
        if now - last < 5.0:
            # Skip if we synced within the last 5 seconds
            return
        self.state.last_sync[node_id] = now

        unix_now = int(now)
        payload = struct.pack('>I', node_id) + struct.pack('>I', unix_now)
        sync_msg = can.Message(arbitration_id=EPOCH_ID, data=payload, is_extended_id=False)
        try:
            self.bus.send(sync_msg)
            self.log.add(f"RTC SYNC: Sent {unix_now} to Node 0x{node_id:08X}")
        except Exception as e:
            self.log.add(f"RTC SYNC send failed: {e}")
            self._debug_log(f"RTC SYNC send failed: {e}")

    # --- Renderables ---
    def _age_style(self, age_secs: int | None) -> str:
        if age_secs is None:
            return ""
        if age_secs >= 30:
            return "bold red"
        elif age_secs >= 10:
            return "yellow"
        else:
            return "green"

    def _row_style(self, secs_since_update: int | None) -> str:
        # Flash highlight for very recent updates (< 1s)
        if secs_since_update is None:
            return ""
        return "on grey15" if secs_since_update < 1 else ""

    def _build_table(self):
        table = Table(show_header=True, header_style="bold cyan", expand=True)
        table.add_column("Node ID", width=14, no_wrap=True)
        table.add_column("HEARTBEAT", width=19)
        table.add_column("AGE", width=5, justify="right", no_wrap=True)
        table.add_column("KNOB (mV)", width=9, justify="right", no_wrap=True)
        table.add_column("CPU TEMP (°C)", width=13, justify="right", no_wrap=True)

        rows = self.state.snapshot()
        table_rows_available = max(1, console.height - self.log_height - 2)
        max_scroll = max(0, len(rows) - table_rows_available)
        if self.table_scroll > max_scroll:
            self.table_scroll = max_scroll

        visible_rows = rows[self.table_scroll:self.table_scroll + table_rows_available]

        for node_id, hb, age, knob, temp, age_secs in visible_rows:
            node_cell = Text(f"0x{node_id:08X}")
            hb_cell = Text(hb)
            age_cell = Text(age, style=self._age_style(age_secs))
            knob_cell = Text(knob)
            temp_cell = Text(temp)

            # Optional: tint temp if out of bounds (example thresholds)
            try:
                tval = float(temp) if temp != "-" else None
            except ValueError:
                tval = None
            if tval is not None:
                if tval >= 70.0:
                    temp_cell.stylize("bold red")
                elif tval >= 60.0:
                    temp_cell.stylize("yellow")
                else:
                    temp_cell.stylize("cyan")

            row_style = self._row_style(age_secs)
            table.add_row(node_cell, hb_cell, age_cell, knob_cell, temp_cell, style=row_style)

        return table

    def _build_log_panel(self):
        height = max(5, self.log_height)
        lines = self.log.tail(height)
        text = Text("\n".join(lines))
        return Panel(text, title=f"Log (tail {height})", border_style="magenta")

    def _build_layout(self):
        layout = Layout(name="root")
        header_text = Text(
            f" CAN Master on {self.iface} | Nodes: {len(self.state.snapshot())} | {time.strftime('%Y-%m-%d %H:%M:%S')}   (q=quit, +/- log height, PgUp/PgDn scroll)",
            style="bold cyan"
        )
        layout.split(
            Layout(header_text, name="header", size=1),
            Layout(self._build_table(), name="body"),
            Layout(self._build_log_panel(), name="log", size=self.log_height)
        )
        return layout

    # --- Key handling ---
    def _read_key(self):
        # Non-blocking single-key read
        if self.is_windows:
            if self.msvcrt and self.msvcrt.kbhit():
                ch = self.msvcrt.getwch()
                return ch
            return None
        else:
            if not self.stdin_is_tty:
                return None
            try:
                import select
                dr, _, _ = select.select([sys.stdin], [], [], 0)
                if dr:
                    return sys.stdin.read(1)
            except Exception:
                return None
            return None

    def _start_key_mode(self):
        if not self.is_windows and self.stdin_is_tty:
            try:
                import termios, tty
                self.termios = termios
                self.tty = tty
                self.orig_term_attrs = self.termios.tcgetattr(sys.stdin)
                self.tty.setcbreak(sys.stdin.fileno())
            except Exception as e:
                self._debug_log(f"Failed to set cbreak mode: {e}")

    def _end_key_mode(self):
        if not self.is_windows and self.stdin_is_tty and self.orig_term_attrs:
            try:
                self.termios.tcsetattr(sys.stdin, self.termios.TCSADRAIN, self.orig_term_attrs)
            except Exception as e:
                self._debug_log(f"Failed to restore term attrs: {e}")

    def run(self):
        try:
            self.log.add(f"Master started on {self.iface} (dry_run={self.dry_run})...")
            self.reader_thread.start()
            self._start_key_mode()

            # Keep the screen content after exit (transient=False)
            with Live(self._build_layout(), console=console, refresh_per_second=20, screen=True, transient=False) as live:
                while not self.stop_event.is_set():
                    self._process_queue()

                    # key handling
                    key = self._read_key()
                    if key:
                        if self.is_windows and key == '\xe0' and self.msvcrt:
                            ext = self.msvcrt.getwch()
                            if ext in ('I',):      # Page Up
                                self.table_scroll = max(0, self.table_scroll - 5)
                            elif ext in ('Q',):    # Page Down
                                self.table_scroll += 5
                            elif ext in ('H',):    # Up
                                self.table_scroll = max(0, self.table_scroll - 1)
                            elif ext in ('P',):    # Down
                                self.table_scroll += 1
                        else:
                            if key in ('q', 'Q'):
                                self.stop_event.set()
                            elif key == '+':
                                self.log_height += 1
                            elif key == '-':
                                self.log_height = max(5, self.log_height - 1)
                            elif key in ('w', 'W'):
                                path = os.path.expanduser(f"~/can_master_log_{int(time.time())}.txt")
                                self.log.write_to_file(path)
                                self.log.add(f"Log written to {path}")

                    # redraw
                    live.update(self._build_layout())
                    time.sleep(0.05)
        except Exception as e:
            console.rule("[red]Unhandled Exception")
            console.print_exception(show_locals=False)
            self._debug_log(f"UNHANDLED: {e}\n{safe_traceback()}")
        finally:
            self._end_key_mode()
            self.stop_event.set()
            try:
                self.reader_thread.join(timeout=1.0)
            except RuntimeError:
                pass

def parse_args():
    ap = argparse.ArgumentParser(description="CAN Master Rich TUI")
    ap.add_argument("--iface", default=DEFAULT_CAN_INTERFACE, help="CAN interface (default: can0)")
    ap.add_argument("--dry-run", action="store_true", help="Run UI without CAN; simulate data")
    ap.add_argument("--debug-file", default=os.path.expanduser("~/can_master_rich.log"), help="Write debug log to this file")
    return ap.parse_args()

def open_bus(iface: str):
    try:
        bus = can.interface.Bus(channel=iface, interface='socketcan')
        # Optional: driver filters to reduce CPU
        try:
            bus.set_filters([
                {"can_id": EPOCH_ID, "can_mask": 0x7FF, "extended": False},
                {"can_id": KNOB_ID,  "can_mask": 0x7FF, "extended": False},
                {"can_id": TEMP_ID,  "can_mask": 0x7FF, "extended": False},
            ])
        except Exception as e:
            console.print(f"[yellow]Warning: set_filters failed: {e}")
        return bus
    except Exception as e:
        console.rule("[red]CAN Open Error")
        console.print(f"[red]Failed to open CAN interface '{iface}': {e}")
        console.print_exception()
        return None

def main():
    args = parse_args()

    # If dry-run, skip opening bus
    bus = None
    if not args.dry_run:
        bus = open_bus(args.iface)
        if bus is None:
            return

    app = App(bus, iface=args.iface, dry_run=args.dry_run, debug_file=args.debug_file)
    app.run()

    if bus:
        try:
            bus.shutdown()
        except Exception:
            pass

    # At exit, dump log to file
    if args.debug_file:
        app.log.write_to_file(args.debug_file)
        console.print(f"[green]Log written to {args.debug_file}")

if __name__ == "__main__":
    main()
