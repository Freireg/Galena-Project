#!/bin/sh
#
# Galena sensor hub — sourced by S99User on boot
#

GALENA_DIR="/root/galena"
LOGFILE="/var/log/galena.log"
PIDFILE="/var/run/galena.pid"

cd "$GALENA_DIR" || exit 1

# Wait for a default route (DHCP done) before starting, so the initial
# weather fetch and mDNS announcement use the correct network state.
i=0
while [ $i -lt 30 ] && ! ip route | grep -q default; do
    sleep 1
    i=$((i + 1))
done

python3 -m uvicorn hub.main:app --host 0.0.0.0 --port 8000 \
    >> "$LOGFILE" 2>&1 &

echo $! > "$PIDFILE"
