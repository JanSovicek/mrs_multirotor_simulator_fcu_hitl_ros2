#!/usr/bin/env bash

# Clear the screen at shell launch
clear

# Run ROS 2 topic monitor and format output live
PYTHONUNBUFFERED=1 ros2 topic echo /uav1/hw_api/FCU/cpu_load | python3 -u -c '
import sys

# Erase prompt artifacts
sys.stdout.write("\033[2J\033[1;1H")
sys.stdout.flush()

buf = []
for line in sys.stdin:
    l = line.strip()
    if l.startswith("---"):
        if buf:
            sys.stdout.write("\033[1;1H" + "\n".join(buf) + "\033[J\n")
            sys.stdout.flush()
            buf.clear()
    elif ":" in l:
        k, v = [x.strip() for x in l.split(":", 1)]
        if k == "timestamp":
            buf.append(f"{k:<26} {v}")
        else:
            try:
                buf.append(f"{k:<26} {float(v):.2f}%")
            except ValueError:
                buf.append(f"{k:<26} {v}")
'