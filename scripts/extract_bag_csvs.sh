#!/usr/bin/env bash
# extract_bag_csvs.sh
# Usage: ./extract_bag_csvs.sh <bag_dir_or_file> <duration_sec>
# Example: ./extract_bag_csvs.sh controller_debug_20251004-120000 30

set -euo pipefail
BAG="$1"
DUR="${2:-30}"
OUT_DIR="bag_export_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT_DIR"

echo "Playing bag $BAG for $DUR seconds and extracting topics to $OUT_DIR"

ros2 bag play "$BAG" &
BAG_PID=$!

sleep 0.8

# Topics to extract (edit if your topic names differ)
TOPIC_ODOM_ASV="/model/blueboat/odometry"
TOPIC_ODOM_ROV="/model/bluerov2_heavy/odometry"
TOPIC_PORT="/model/blueboat/joint/motor_port_joint/cmd_thrust"
TOPIC_STBD="/model/blueboat/joint/motor_stbd_joint/cmd_thrust"
TOPIC_ROSOUT="/rosout"

# Use ros2 topic echo -p (plain) and capture for DUR seconds
timeout ${DUR}s ros2 topic echo -p "$TOPIC_ODOM_ASV" > "$OUT_DIR/odom_asv.txt" &
timeout ${DUR}s ros2 topic echo -p "$TOPIC_ODOM_ROV" > "$OUT_DIR/odom_rov.txt" &
timeout ${DUR}s ros2 topic echo -p "$TOPIC_PORT" > "$OUT_DIR/port_thrust.txt" &
timeout ${DUR}s ros2 topic echo -p "$TOPIC_STBD" > "$OUT_DIR/stbd_thrust.txt" &
timeout ${DUR}s ros2 topic echo -p "$TOPIC_ROSOUT" > "$OUT_DIR/rosout.txt" &

# Wait for all child timeouts
wait

# Stop bag player
kill $BAG_PID || true

# Basic conversion helper to CSV for thrust topics (one column: time,data)
python3 - << 'PY'
import re,sys
import json
from pathlib import Path
odir = Path('${OUT_DIR}')
for name in ['port_thrust','stbd_thrust']:
    in_file = odir / f"{name}.txt"
    out_file = odir / f"{name}.csv"
    with in_file.open() as inf, out_file.open('w') as outf:
        outf.write('time,data\n')
        for line in inf:
            line=line.strip()
            if not line: continue
            # ros2 topic echo -p prints Python repr; assume first field is header time or 'header': {'stamp': ...}
            # crude parse: try to find 'stamp': {'sec':..., 'nanosec':...}
            m = re.search(r"'sec':\s*(\d+).*'nanosec':\s*(\d+)", line)
            if m:
                t = int(m.group(1)) + int(m.group(2))/1e9
            else:
                # fallback: use incremental index as time
                t = ''
            # find last number in line
            nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?", line)
            if nums:
                data = nums[-1]
            else:
                data = ''
            outf.write(f"{t},{data}\n")
print('CSV conversion complete in', odir)
PY

echo "Extraction complete. Files in: $OUT_DIR"

# Print summary
ls -lh "$OUT_DIR"
echo "To compress: tar -czf ${OUT_DIR}.tar.gz $OUT_DIR"
