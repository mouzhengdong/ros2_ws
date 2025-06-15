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
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-teleop-twist-keyboard \
    mesa-utils \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    gdb \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 升级cmake版本
# WORKDIR /opt
# COPY third_party/cmake-3.24.0-linux-x86_64.sh /opt/cmake-3.24.0-linux-x86_64.sh
# RUN apt remove cmake -y && \
#     apt-get install -y wget  && \
#     chmod +x cmake-3.24.0-linux-x86_64.sh && \
#     ./cmake-3.24.0-linux-x86_64.sh --skip-license --prefix=/usr/local

COPY third_party/ceres-solver ./ceres-solver    
COPY third_party/Sophus ./Sophus
COPY third_party/g2o ./g2o

# ====== 安装 Ceres Solver ======
RUN cd ceres-solver && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install

# ====== 安装 Sophus ======
RUN cd Sophus && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && make install
# ====== 安装 g2o ======
RUN cd g2o && \
    mkdir build && cd build && \
    cmake .. -DBUILD_SHARED_LIBS=ON -DG2O_BUILD_EXAMPLES=OFF -DG2O_GENERATE_CONFIG_FILE=ON && \
    make -j$(nproc) && make install

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:/workspaces/ros2_ws/third_party/gazebo_models_worlds_collection/models' >> ~/.bashrc && \
    echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/workspaces/ros2_ws/third_party/gazebo_models_worlds_collection/worlds' >> ~/.bashrc && \
    echo "alias colcon_rebuild='rm -rf build install log && colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc

# 默认进入 bash
CMD ["bash"]