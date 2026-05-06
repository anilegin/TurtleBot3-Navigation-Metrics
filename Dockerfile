FROM osrf/ros:humble-desktop-full

ENV TURTLEBOT3_MODEL=burger
ENV DEBIAN_FRONTEND=noninteractive
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models

RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    python3-colcon-common-extensions \
    python3-pip \
    mesa-utils \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# create workspace
WORKDIR /root/tbot3_ws

# auto source ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]