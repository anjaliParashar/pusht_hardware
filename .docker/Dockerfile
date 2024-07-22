FROM osrf/ros:noetic-desktop-full

# Install packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    python3-tk \
    ros-noetic-driver-base \
    ros-noetic-sophus \
    ros-noetic-urg-node \
    ros-noetic-realsense2-camera \
    ros-noetic-foxglove-bridge

RUN pip3 install numpy torch pyyaml rospkg transforms3d equinox scipy matplotlib
RUN pip3 install --upgrade numpy

# Install updated cmake
RUN apt-get update \
    && apt-get -y install build-essential \
    && apt-get install -y wget \
    && rm -rf /var/lib/apt/lists/* \
    && wget https://github.com/Kitware/CMake/releases/download/v3.24.1/cmake-3.24.1-Linux-x86_64.sh \
    -q -O /tmp/cmake-install.sh \
    && chmod u+x /tmp/cmake-install.sh \
    && mkdir /opt/cmake-3.24.1 \
    && /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake-3.24.1 \
    && rm /tmp/cmake-install.sh \
    && ln -s /opt/cmake-3.24.1/bin/* /usr/local/bin

# Make a ROS workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
SHELL ["/bin/bash", "-c"]  # so we can use `source`
RUN source /opt/ros/noetic/setup.sh && catkin_init_workspace src

# Build
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y
RUN source /opt/ros/noetic/setup.sh && catkin_make

# Build
WORKDIR /catkin_ws
RUN apt-get update && apt-get install -y 
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y
RUN source /opt/ros/noetic/setup.sh && catkin_make

# Add lines to the bashrc file that source ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
