FROM osrf/ros:humble-desktop-full-20250602
RUN apt-get update && apt-get install -y \
    git \
    curl \
    vim \
    tree \
    net-tools \
    iputils-ping \
    ros-humble-rqt-graph \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-joint-state-publisher \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*