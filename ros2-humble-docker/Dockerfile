# (1) JetPack 6.x 공식 베이스
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# (2) 기본 패키지
RUN apt update && apt install -y \
    tzdata \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    git \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# (3) locale 설정
RUN locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# (4) ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# (5) ROS2 apt repo
RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# (6) ROS2 Humble 설치
RUN apt update && apt install -y \
    ros-humble-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# (7) rosdep 초기화
RUN rosdep init || true
RUN rosdep update

# (8) ROS 환경 자동 source
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root
CMD ["/bin/bash"]
