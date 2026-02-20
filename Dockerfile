FROM ubuntu:22.04

# Avoid interactive apt prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Use bash login shell so "source ..." works in RUN layers when needed
SHELL ["/bin/bash", "-lc"]

# --- Base utilities + UTF-8 locale (ROS tools behave better with proper locale) ---
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release sudo locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# --- Add ROS 2 apt repository key + source list (for Ubuntu 22.04 / Jammy) ---
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list

# --- Install ROS Humble desktop + dev tools (colcon/rosdep/etc.) + QoL packages ---
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    build-essential \
    git vim nano iputils-ping bash-completion \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3* \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (system-wide). "rosdep update" is better done at runtime.
RUN rosdep init || true

# -------------------------------------------------------------------
# Create a NON-ROOT user so mounted files don't become root-owned.
# Default UID/GID = 1000 (typical on Ubuntu hosts). Override at build time if needed.
#   docker build --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) -t ros-humble-dev .
# -------------------------------------------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# Create group + user + passwordless sudo (dev convenience)
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd  --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# Create the workspace root and give ownership to the non-root user
RUN mkdir -p /ws \
 && chown -R ${USERNAME}:${USERNAME} /ws

# Auto-source ROS underlay + workspace overlay for the non-root user's shell
# (overlay line won't error before first build because of "|| true")
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "source /ws/install/setup.bash 2>/dev/null || true" >> /home/${USERNAME}/.bashrc

# Switch to non-root user by default
USER ${USERNAME}
WORKDIR /ws

# Default shell when container starts
CMD ["bash"]

# To run after build

# xhost +local:docker

# docker run -it --rm \
#   --name ros-humble-dev \
#   -v ~/ros2_ws/src:/ws/src \
#   ros-humble-dev

# docker run -it --rm \
#   --net=host \
#   -e DISPLAY=$DISPLAY \
#   -e QT_X11_NO_MITSHM=1 \
#   -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#   -v ~/ros2_ws:/ws \
#   ros-humble-dev
