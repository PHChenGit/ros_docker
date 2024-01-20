FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
ARG PUID=1000
ENV PUID ${PUID}
ARG PGID=1000
ENV PGID ${PGID}
ARG USER_NAME=rvlros
ENV USER_NAME ${USER_NAME}
ARG GROUP_NAME=rvlros
ENV GROUP_NAME ${GROUP_NAME}

# always run apt update when start and after add new source list, then clean up at end.
RUN set -xe; \
    apt-get update -yqq && \
    groupadd -g ${PGID} ${USER_NAME} && \
    useradd -l -u ${PUID} -g ${GROUP_NAME} -m ${USER_NAME} && \
    usermod -p "*" ${USER_NAME} -s /bin/bash && \
    apt-get install -yqq \
        software-properties-common \
        curl \
        wget \
        build-essential \
        git \
        zip \
        tar && \
   add-apt-repository ppa:deadsnakes/ppa && \
   apt-get update && \
   apt-get install -y python3 python3-pip

RUN wget https://github.com/Kitware/CMake/releases/download/v3.28.1/cmake-3.28.1-linux-x86_64.tar.gz && \
    tar -zxvf cmake-3.28.1-linux-x86_64.tar.gz && \
    mv cmake-3.28.1-linux-x86_64 cmake-3.28.1 && \
    ln -sf /cmake-3.28.1/bin/* /usr/bin

# Install ROS Noetic
ENV ROS_DISTRO noetic

RUN set -xe; \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-shadow.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    pip3 install -U catkin_tools && \
    pip3 install rosdep rosinstall rosinstall-generator wstool && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc && \
    /bin/bash -c ". /home/${USER_NAME}/.bashrc" && \
    rosdep init && \
    rosdep update && \
    mkdir -p /home/${USER_NAME}/catkin_ws/src && \
    chown ${USER_NAME}:${GROUP_NAME} -R /home/${USER_NAME}/catkin_ws && \
    apt-get autoremove -y

# Install PX4
SHELL ["/bin/bash", "-c"]
ARG INSTALL_PX4=false
ENV INSTALL_PX4 ${INSTALL_PX4}
RUN if [ "${INSTALL_PX4}" = true ]; then \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    /bin/bash -c "./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx" && \
    apt-get install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x ./install_geographiclib_datasets.sh && \
    /bin/bash -c './install_geographiclib_datasets.sh'; \
    fi

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/catkin_ws
