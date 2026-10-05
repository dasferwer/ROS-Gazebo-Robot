#!/bin/bash
set -euo pipefail
if ! pgrep Xvfb >/dev/null; then
    Xvfb :1 -screen 0 1600x900x24 +extension GLX +render -noreset > /ws/artifacts/xvfb.log 2>&1 &
    sleep 1
    openbox > /ws/artifacts/openbox.log 2>&1 &
    x11vnc -display :1 -nopw -forever -shared -rfbport 5900 > /ws/artifacts/vnc.log 2>&1 &
    websockify --web=/usr/share/novnc 6080 localhost:5900 > /ws/artifacts/novnc.log 2>&1 &
fi
