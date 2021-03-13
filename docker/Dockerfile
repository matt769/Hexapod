FROM ros:melodic

SHELL ["/bin/bash", "-c"]

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
