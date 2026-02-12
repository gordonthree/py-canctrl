# CAN Bus Master Control - Basic Python Script

Big thanks to Google Gemini for helping with this

Here's the Microsoft contribution, with a tui style interface.
# CAN Master Rich TUI

A Python-based terminal UI for monitoring CAN nodes with a **live-updating table** and **scrolling log**. Built with Rich for a clean, non-scrolling interface.

## Features
- **Top pane**: Table with columns:
  - Node ID
  - HEARTBEAT timestamp
  - AGE (seconds since last update)
  - KNOB ADC (mV)
  - CPU TEMP (°C)
- **Bottom pane**: Scrolling log for events (intro packets, RTC sync, errors).
- Color-coded AGE and CPU TEMP thresholds.
- Keys:
  - `q` — Quit
  - `+` / `-` — Adjust log pane height
  - `PgUp` / `PgDn` or arrow keys — Scroll table
  - `w` — Write log to file (`~/can_master_log_<timestamp>.txt`)

## Requirements
```bash
pip install rich python-can

