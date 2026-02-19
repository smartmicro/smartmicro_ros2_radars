# ROS 2 distro (default = foxy)
ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}

# Fix repo keys ONLY for Foxy (snapshots issue). 
RUN if [ "$ROS_DISTRO" = "foxy" ]; then \
      rm -f /etc/apt/sources.list.d/ros2-snapshots.list || true && \
      apt-get update && apt-get install -y curl gnupg2 lsb-release && \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
          -o /usr/share/keyrings/ros-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
          | tee /etc/apt/sources.list.d/ros2.list > /dev/null ; \
    fi

# Common dependencies
RUN apt-get update && apt-get install -y \
    iputils-ping \
    python3 \
    python3-pip \
    wget \
    git \
    ca-certificates \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ROS-specific dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-point-cloud-msg-wrapper \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rviz-rendering

WORKDIR /code
