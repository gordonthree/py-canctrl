import can
import cantools
import time
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
from rich import box

# ---------------------------------------------------------
# Load DBC
# ---------------------------------------------------------
db = cantools.database.load_file("master_bus.dbc")

# Map arbitration_id → message object
msg_by_id = {msg.frame_id: msg for msg in db.messages}

# Store latest decoded values
decoded_messages = {
    msg.name: {
        "last_seen": None,
        "signals": {sig.name: None for sig in msg.signals}
    }
    for msg in db.messages
}

# ---------------------------------------------------------
# CAN Bus Setup
# ---------------------------------------------------------
bus = can.interface.Bus(channel="can0", bustype="socketcan")

# ---------------------------------------------------------
# Helper: Build CAN Table
# ---------------------------------------------------------
def build_can_table():
    table = Table(title="CAN Bus Messages", box=box.SIMPLE_HEAVY)
    table.add_column("Message")
    table.add_column("Signal")
    table.add_column("Value")
    table.add_column("Last Seen")

    now = time.time()

    for msg_name, entry in decoded_messages.items():
        last_seen = entry["last_seen"]
        age = f"{now - last_seen:0.1f}s" if last_seen else "—"

        for sig_name, value in entry["signals"].items():
            table.add_row(
                msg_name,
                sig_name,
                str(value) if value is not None else "—",
                age
            )

    return table

# ---------------------------------------------------------
# Helper: Build Status Panel (no psutil)
# ---------------------------------------------------------
def build_status_panel(knob_adc, heartbeat_time):
    hb_age = time.time() - heartbeat_time if heartbeat_time else None

    text = (
        f"[bold]Knob ADC:[/bold] {knob_adc}\n"
        f"[bold]Heartbeat:[/bold] {hb_age:.1f}s ago" if hb_age else "Never"
    )

    return Panel(text, title="System Status")

# ---------------------------------------------------------
# Main Loop
# ---------------------------------------------------------
def main():
    knob_adc = 0
    heartbeat_time = None

    layout = Layout()
    layout.split_column(
        Layout(name="upper", ratio=3),
        Layout(name="lower", ratio=1)
    )

    with Live(layout, refresh_per_second=10, screen=True):
        while True:
            msg = bus.recv(timeout=0.1)
            if msg:
                if msg.arbitration_id in msg_by_id:
                    dbc_msg = msg_by_id[msg.arbitration_id]

                    try:
                        decoded = db.decode_message(msg.arbitration_id, msg.data)
                    except Exception:
                        decoded = {}

                    entry = decoded_messages[dbc_msg.name]
                    entry["last_seen"] = time.time()

                    for sig, val in decoded.items():
                        entry["signals"][sig] = val

                    # Preserve your existing special-case logic
                    if dbc_msg.name == "KNOB_ADC":
                        knob_adc = decoded.get("adc", knob_adc)

                    if dbc_msg.name == "HEARTBEAT":
                        heartbeat_time = time.time()

            # Update UI
            layout["upper"].update(build_can_table())
            layout["lower"].update(build_status_panel(knob_adc, heartbeat_time))

# ---------------------------------------------------------
if __name__ == "__main__":
    main()
