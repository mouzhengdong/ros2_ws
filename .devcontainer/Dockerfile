FROM osrf/ros:humble-desktop-full-20250602

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    curl \
    vim \
    tree \
    net-tools \
    iputils-ping \
    python3-pip \
    # ROS 相关插件
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-common-plugins \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-gazebo-* \
    # 音频支持
    libpulse0 \
    pulseaudio \
    alsa-utils \
    libasound2 \
    # GUI 支持
    libgl1-mesa-glx libxrandr2 libxinerama1 libxcursor1 libxi6 \
    lsof \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# 默认进入 bash
CMD ["bash"]