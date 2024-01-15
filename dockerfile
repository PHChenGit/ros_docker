FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive

RUN set -x && \
    apt-get update && \
    apt-get install -y \
        software-properties-common \
        curl \
        wget \
        build-essential \
        git \
        zip \
        unzip \
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

RUN set -x && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-shadow.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    pip3 install -U catkin_tools && \
    pip3 install rosdep rosinstall rosinstall-generator wstool && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    /bin/bash -c ". ~/.bashrc" && \
    rosdep init && \
    rosdep update && \
    mkdir -p catkin_ws/src && \
    apt-get autoremove -y






# # Install PX4
# RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
#     /bin/bash -c './PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx'
# # CMD ["/bin/bash",  "./PX4-Autopilot/Tools/setup/ubuntu.sh", "--no-sim-tools", "--no-nuttx"]

# # Install Mavros
# RUN apt-get install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs && \
#     wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
#     chmod +x ./install_geographiclib_datasets.sh && \
#     /bin/bash -c './install_geographiclib_datasets.sh'


WORKDIR catkin_ws

