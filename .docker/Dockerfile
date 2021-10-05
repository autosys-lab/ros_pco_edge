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
    ros-$ROSDISTRO-rqt-common-plugins 

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

# micro_resector_moveit_teleop_stack stack build 
WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    # vcs import src < ./src/micro_resector_moveit_teleop_stack/dependencies.rosinstall &&\
    # vcs pull src || true &&\
    colcon build --symlink-install \
    "]

# RUN ["/bin/bash", "-c", "cp /dev_ws/src/micro_resector_moveit_teleop_stack/.docker/entrypoint.sh / && chmod 777 /entrypoint.sh"]
# ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]


