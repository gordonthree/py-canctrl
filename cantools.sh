#!/bin/bash
candump can0 | python3 -m cantools decode master_bus.dbc
