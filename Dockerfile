# Curiosity rover: ROS 2 Jazzy + Gazebo Harmonic + RViz, with a browser desktop (noVNC).
# Builds on linux/amd64 and linux/arm64 (Intel/AMD PCs, Apple Silicon, ARM boards).
#
#   docker compose up demo          then open http://localhost:6080
#
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ROS 2 / Gazebo packages used by curiosity_description, a small X desktop served over VNC
# and window tiling for the four-panel demo
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rviz2 \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-ros2controlcli \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-rviz-imu-plugin \
      ros-jazzy-pcl-ros \
      ros-jazzy-laser-filters \
      ros-jazzy-rqt-image-view \
      python3-colcon-common-extensions \
      python3-numpy \
      imagemagick \
      xvfb x11vnc novnc websockify openbox xterm \
      wmctrl \
      mesa-utils libgl1-mesa-dri libegl1 \
      fonts-dejavu-core \
    && ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace. --symlink-install lets docker-compose mount the package
# over /ws/src for live editing of URDF / launch / config without rebuilding.
WORKDIR /ws
COPY curiosity_description /ws/src/curiosity_description
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --event-handlers console_direct+

COPY tests /ws/tests
COPY demo /ws/demo
COPY examples /ws/examples
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc

# Docker Desktop (macOS / Windows) has no GPU passthrough: render with Mesa llvmpipe.
# Linux with a GPU: set LIBGL_ALWAYS_SOFTWARE=0 (see docker-compose.gpu.yml).
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    RESOLUTION=1920x1080 \
    VNC_PORT=6080

EXPOSE 6080
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "curiosity_description", "gazebo.launch.xml"]
