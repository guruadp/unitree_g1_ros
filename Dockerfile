FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ----------------------------
# Base OS deps + locale
# ----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl gnupg2 lsb-release sudo \
    ca-certificates \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# ROS 2 apt repo
# ----------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list

# ----------------------------
# ROS 2 Humble + "everything lab" stack
# ----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core ROS desktop (rviz2 + common tools)
    ros-humble-desktop \
    # Build / dev tools
    build-essential git \
    python3-pip python3-rosdep python3-colcon-common-extensions python3-vcstool \
    bash-completion iputils-ping net-tools iproute2 \
    vim nano \
    # Nav2 stack
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # SLAM / localization
    ros-humble-slam-toolbox \
    ros-humble-nav2-amcl \
    # Common teleop / inputs
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    # Transforms / robot model utilities
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-robot-monitor \
    ros-humble-rqt-image-view \
    # Perception helpers (useful in labs)
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-cv-bridge \
    # Pointcloud / laser tools (common in mapping)
    ros-humble-pointcloud-to-laserscan \
    ros-humble-laser-filters \
    # Simulation: Gazebo classic + ROS interfaces (Humble default)
    # gazebo \
    # ros-humble-gazebo-ros-pkgs \
    # ros-humble-gazebo-ros2-control \
    # Simulation: Gazebo Ignition (Fortress)
    ignition-fortress \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # GUI/OpenGL/Qt sanity for RViz/Gazebo in Docker
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libxkbcommon-x11-0 \
    dbus-x11 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# rosdep init (system-wide). rosdep update usually at runtime.
# ----------------------------
RUN rosdep init || true

# ----------------------------
# Non-root user
# ----------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd  --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# Workspace
RUN mkdir -p /ws \
 && chown -R ${USERNAME}:${USERNAME} /ws

# Auto-source ROS + overlay
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "source /ws/install/setup.bash 2>/dev/null || true" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /ws

CMD ["bash"]
