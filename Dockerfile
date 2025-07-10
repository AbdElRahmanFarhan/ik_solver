FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-xacro \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

RUN python3 -m pip install --no-cache-dir --upgrade pip

WORKDIR /home
RUN mkdir -p workspace/ros2_ws/src
COPY ./robot-urdfs /home/workspace/robot-urdfs

WORKDIR /home/workspace
RUN git clone https://github.com/ros/urdf_parser_py.git
WORKDIR /home/workspace/urdf_parser_py
RUN python3 setup.py install

WORKDIR /home/workspace/ros2_ws
COPY ./robot_description /home/workspace/ros2_ws/src/robot_description
RUN colcon build
RUN echo "source /home/workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/workspace

CMD [ "/bin/bash" ]
    