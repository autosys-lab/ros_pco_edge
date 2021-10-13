ARG ROSDISTRO=foxy

FROM ros:$ROSDISTRO

ARG ROSDISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive


# Install any needed packages then clean up apt cache
# Build tools and general ROS tools
RUN apt-get update && apt-get install -y \
    software-properties-common \
    ssh \
    git \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    udev \
    ros-$ROSDISTRO-rqt-common-plugins \
    ros-$ROSDISTRO-image-pipeline \
    ros-$ROSDISTRO-image-common \
    libusb-1.0-0-dev \
    usbutils

RUN pip3 install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    pyassimp==4.1.3 

# reduce size by removing apt cache
RUN ["/bin/bash", "-c", "rm -rf /var/lib/apt/lists/*"]

# copy packages across
RUN mkdir -p /dev_ws/src/ros_pco_edge
COPY . /dev_ws/src/ros_pco_edge/

# # pco_linux build
# RUN mkdir -p /tmp/pco_linux
# WORKDIR /tmp/pco_linux
# RUN ["/bin/bash", "-c", "wget https://www.pco.de/fileadmin/fileadmin/user_upload/pco-software/pco.linux_usb_pl_1_01_19.tar.gz && \
#                          tar -xzf pco.linux_usb_pl_1_01_19.tar.gz"]
#
# WORKDIR /tmp/pco_linux/pco_camera/pco_usb_pl
# RUN ["/bin/bash", "-c", "make && \
#                          ./symlink_pco -u && \
#                          /sbin/ldconfig && \
#                          cp pco_usb.rules /etc/udev/rules.d/ \
#                          "]
#

WORKDIR /dev_ws/src/ros_pco_edge
RUN ["/bin/bash", "-c", "cp ./.docker/pco_usb.rules /etc/udev/rules.d/ "]

# WORKDIR /tmp/pco_linux/pco_camera/
# RUN ["/bin/bash", "-c", "cp -r pco_common /usr/include/ "]

# pco_edge build 
WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    colcon build --symlink-install \
    "]

CMD ["bash"]


