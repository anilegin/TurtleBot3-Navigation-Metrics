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
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-pip \
    mesa-utils \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    "numpy==1.26.4" \
    "scipy==1.10.1" \
    "scikit-learn==1.3.2" \
    "joblib==1.3.2" \
    torch \
    torchvision \
    torchaudio \
    flask 

WORKDIR /root/tbot3_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /root/tbot3_ws/install/setup.bash ]; then source /root/tbot3_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]