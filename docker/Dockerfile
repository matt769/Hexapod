FROM ros:melodic

SHELL ["/bin/bash", "-c"]

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
                                        cmake \
                                        python-catkin-tools \
                                        ros-melodic-tf2 \
                                        ros-melodic-tf2-ros \
                                        ros-melodic-tf2-geometry-msgs \
                                        ros-melodic-urdf \
    && rm -rf /var/lib/apt/lists/*

COPY . /catkin_ws/src/hexapod

WORKDIR /catkin_ws

RUN source /opt/ros/melodic/setup.bash && catkin build
