FROM osrf/ros:iron-desktop

SHELL ["/bin/bash", "-c"]

RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u 1001 ubuntu 

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ubuntu

WORKDIR /home/ubuntu

RUN mkdir -p /home/ubuntu/gz_ws/src

WORKDIR /home/ubuntu/gz_ws

ENV GZ_SIM_RESOURCE_PATH=/home/ubuntu/aprs_ws/install/ur_description/share/
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/ubuntu/aprs_ws/install/gz_ros2_control/lib/
ENV GZ_VERSION=harmonic

RUN source /opt/ros/iron/setup.bash && \
    sudo apt-get update -qq && \
    sudo apt-get install lsb-release gnupg python3-pip wget -y && \
    pip install setuptools==58.2.0 && \
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt-get update -qq && \
    sudo apt-get -y install gz-harmonic && \
    git clone https://github.com/ros-controls/gz_ros2_control.git src/gz_ros2_control -b iron && \
    git clone https://github.com/gazebosim/ros_gz.git src/ros_gz -b humble && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git src/ur_description -b rolling && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    colcon build

RUN mkdir -p /home/ubuntu/moveit_ws/src

WORKDIR /home/ubuntu/moveit_ws

RUN source /opt/ros/iron/setup.bash && \
    git clone https://github.com/ros-planning/moveit2.git src/moveit2 -b main && \
    git clone https://github.com/ros-planning/moveit_msgs.git src/moveit_msgs -b ros2 && \
    git clone https://github.com/ros-planning/moveit_resources.git src/moveit_resources -b ros2 && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    colcon build


RUN mkdir -p /home/ubuntu/aprs_ws/src

WORKDIR /home/ubuntu/aprs_ws

RUN source /opt/ros/iron/setup.bash && \
    source /home/ubuntu/gz_ws/install/setup.bash && \
    source /home/ubuntu/moveit_ws/install/setup.bash && \
    git clone https://github.com/usnistgov/APRS_ros2_demo_sim.git src/APRS_ros2_demo_sim -b main && \
    git clone https://github.com/sparsh-b/ariac_gz_plugins.git src/ariac_gz_plugins -b main && \
    git clone https://github.com/usnistgov/aprs-ros-conveyor.git src/aprs_ros_conveyor -b main && \
    git clone https://github.com/jfernandez37/moveit_test src/moveit_test -b individual_moveit_configs && \
    colcon build

RUN echo 'source /home/ubuntu/moveit_ws/install/setup.bash' >> /home/ubuntu/.bashrc
RUN echo 'source /home/ubuntu/gz_ws/install/setup.bash' >> /home/ubuntu/.bashrc
RUN echo 'source /home/ubuntu/aprs_ws/install/setup.bash' >> /home/ubuntu/.bashrc
RUN echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/ubuntu/gz_ws/install/ur_description/share/:/home/ubuntu/aprs_ws/install/aprs_gz_sim/share/aprs_gz_sim/models/' >> /home/ubuntu/.bashrc
