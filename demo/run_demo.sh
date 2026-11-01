#!/usr/bin/env bash
# Four-panel demo on the browser desktop (http://localhost:6080):
#
#   RViz (model, lidar, depth cloud) | Gazebo (camera follows the rover)
#   RGB camera                       | depth camera
#
# The rover runs examples/drive_demo.py in a loop. NavCam, MastCam and HazCam are switched off
# (use_cameras:=false): the panels don't show them and the simulation runs about 4x faster without a GPU.
#
#   docker compose up demo
set -e
export DISPLAY=${DISPLAY:-:1}
W=${RESOLUTION%x*}; H=${RESOLUTION#*x}
PW=$((W / 2)); PH=$((H / 2))
HERE=$(dirname "$(readlink -f "$0")")
LOG=/tmp/demo; mkdir -p $LOG

# windows without borders; RViz stays below the other panels, so its status bar can hide under them
mkdir -p ~/.config/openbox
sed 's|<applications>|<applications><application class="*"><decor>no</decor></application><application title="*RViz"><layer>below</layer></application>|' \
    /etc/xdg/openbox/rc.xml > ~/.config/openbox/rc.xml
openbox --reconfigure || true

# 1. simulation
ros2 launch curiosity_description gazebo.launch.xml rviz:=false use_cameras:=false \
    gz_args:="-r --gui-config $HERE/gazebo_gui.config" > $LOG/sim.log 2>&1 &
SIM=$!
trap 'kill $(jobs -p) 2>/dev/null' EXIT
echo "waiting for the controllers..."
until [ "$(ros2 control list_controllers 2>/dev/null | grep -c active)" -ge 4 ]; do sleep 2; done

# 2. viewers
rviz2 -d $HERE/demo.rviz --ros-args -p use_sim_time:=true > $LOG/rviz.log 2>&1 &
python3 $HERE/cam_view.py /depth_camera/image_raw "RGB camera" > $LOG/rgb.log 2>&1 &
python3 $HERE/cam_view.py /depth_camera/depth/image_raw "Depth camera" > $LOG/depth.log 2>&1 &

window_id() {
  until wmctrl -l | grep -q "$1"; do sleep 1; done
  wmctrl -l | awk -v t="$1" 'index($0, t) {print $1; exit}'
}
place() {  # window x y width height
  wmctrl -i -r "$1" -b remove,maximized_vert,maximized_horz
  wmctrl -i -r "$1" -e "0,$2,$3,$4,$5"
}
GZ=$(window_id "Gazebo Sim")
RVIZ=$(window_id "RViz")
RGB=$(window_id "RGB camera")
DEPTH=$(window_id "Depth camera")
sleep 3

# 3. layout: a 2 x 2 grid. Gazebo's title bar and RViz's menu and tool bars sit above the top of the
#    screen; RViz's status bar and dock handles sit under its neighbours.
GZ_BAR=48; RVIZ_TOP=62; RVIZ_BOTTOM=26; RVIZ_SIDE=18
place "$RVIZ" -$RVIZ_SIDE -$RVIZ_TOP $((PW + 2 * RVIZ_SIDE)) $((PH + RVIZ_TOP + RVIZ_BOTTOM))
place "$GZ" $PW -$GZ_BAR $PW $((PH + GZ_BAR))
place "$RGB" 0 $PH $PW $PH
place "$DEPTH" $PW $PH $PW $PH

gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 \
    --req 'data: "curiosity"' > /dev/null || true
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 5000 \
    --req 'x: -7.0, y: 5.0, z: 3.5' > /dev/null || true
echo "demo running: open http://localhost:6080"

# 4. drive
while kill -0 $SIM 2>/dev/null; do
  python3 /ws/examples/drive_demo.py || sleep 2
done
