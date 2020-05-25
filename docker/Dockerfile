FROM ros:kinetic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
                                        cmake \
                                        python-catkin-tools \
                                        ros-kinetic-tf2 \
                                        ros-kinetic-tf2-ros \                                        
                                        ros-kinetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY . /catkin_ws/src/hexapod

WORKDIR /catkin_ws

RUN source /opt/ros/kinetic/setup.bash && catkin build
