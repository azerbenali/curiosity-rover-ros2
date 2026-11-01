#!/usr/bin/env bash
# Sources ROS 2 + the workspace and, unless a host display is given, starts a
# virtual desktop that you open in the browser: http://localhost:${VNC_PORT}
set -e

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

if [ -z "${DISPLAY}" ] && [ "${HEADLESS:-0}" != "1" ]; then
  export DISPLAY=:1
  rm -f /tmp/.X1-lock /tmp/.X11-unix/X1   # stale after "docker compose restart"
  Xvfb :1 -screen 0 "${RESOLUTION}x24" -nolisten tcp >/tmp/xvfb.log 2>&1 &
  for _ in $(seq 1 50); do [ -e /tmp/.X11-unix/X1 ] && break; sleep 0.1; done
  openbox >/tmp/openbox.log 2>&1 &
  x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
  websockify --web /usr/share/novnc "${VNC_PORT}" localhost:5900 >/tmp/novnc.log 2>&1 &
  echo ""
  echo "  Desktop ready: open  http://localhost:${VNC_PORT}/vnc.html?autoconnect=1&resize=scale"
  echo ""
fi

exec "$@"
