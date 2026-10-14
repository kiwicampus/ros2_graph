# -----------------------------------------------------------------------------
# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Or your actual UID, GID on Linux if not the default 1000
ARG USERNAME=ada
ARG USER_UID=1000
ARG USER_GID=$USER_UID


# Set environment variables to avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update the system package list and install Python 3, pip, and common dependencies
RUN apt update && apt install -y --no-install-recommends \
    locales \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    software-properties-common \
    git \
    curl \
    clang \
    clang-format \
    iproute2 \
    openssh-client \
    # ------------------------------
    # Audio tools
    pulseaudio \
    alsa-base \
    alsa-tools \
    alsa-utils \
    libasound2-dev \ 
    # ---------------------------------
    && apt autoremove && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
RUN \
    # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
    groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME \
    # ------------------------------
    # [Optional] Add sudo support for non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # ---------------------------------
    && apt autoremove && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# Installation of ROS2 IRON
ENV ROS_DISTRO=iron
RUN \
    add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    # ---------------------------------
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    ros-dev-tools \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-turtlesim \
    libgl1-mesa-glx \
    npm \
    # ---------------------------------
    && apt update \
    && apt upgrade -y \
    && apt autoremove && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY requirements.txt .
RUN pip3 install -r requirements.txt \
    && rm -rf /home/ada/.cache/pip

# ------------------------------------------------------------------------------
# Optionally, set Python 3 as the default Python
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN touch "/home/${USERNAME}/.bashrc" \
    && echo "source /opt/ros/iron/setup.bash " >> "/home/${USERNAME}/.bashrc"

# -----------------------------------------------------------------------------
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND= 
CMD ["/bin/bash"]
