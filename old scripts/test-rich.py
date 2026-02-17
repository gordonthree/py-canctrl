# save as test_rich_live.py
from rich.live import Live
from rich.console import Console
from rich.panel import Panel
import time

console = Console()

with Live(Panel("Starting...", title="Test", border_style="green"), console=console, refresh_per_second=10, screen=True) as live:
    for i in range(30):
        live.update(Panel(f"Counter: {i}", title="Test", border_style="green"))
        time.sleep(0.2)
