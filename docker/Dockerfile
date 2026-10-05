FROM ros:humble-ros-base-jammy@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-tf-transformations python3-pytest python3-yaml \
    xvfb x11vnc novnc websockify openbox wmctrl ffmpeg mesa-utils \
    && dpkg-query -W > /opt/dependencies.lock && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && dpkg-query -W > /opt/dependencies.lock && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 ROS_DOMAIN_ID=42
WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
