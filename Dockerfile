FROM ros:noetic
ENV OVERLAY_WS /root/catkin_ws

RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS

RUN . /opt/ros/$ROS_DISTRO/setup.sh && apt update && apt install -y python3-catkin-tools python3-osrf-pycommon git

COPY . ./src/ros_controllers

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && catkin build

RUN sed --in-place --expression \
    '$isource "$OVERLAY_WS/devel/setup.bash"' \
    /ros_entrypoint.sh