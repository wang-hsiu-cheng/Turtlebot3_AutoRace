FROM ros:noetic

# each RUN makes the image size bigger
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git \
    cmake \
    ros-noetic-tf* \
    ros-noetic-rviz \
    ros-noetic-diagnostic-updater \
    ros-noetic-cv-bridge \
    ros-noetic-rqt ros-noetic-rqt-common-plugins \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-navigation \
    && rm -rf /var/lib/apt/lists/*

# mkdir 
WORKDIR /root/folder

ENV SHELL /bin/bash
SHELL ["/bin/bash","-ic"]