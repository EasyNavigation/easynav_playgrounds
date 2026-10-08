# Use the official ROS 2 rolling image as base
FROM ros:rolling-ros-base

ENV WORKSPACE=/root/easynav_ws

# Install essential build tools, git, and networking utilities
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    nano \
    unzip \
    ros-rolling-rviz2 \
    && rm -rf /var/lib/apt/lists/*


# Set up the workspace structure
RUN mkdir -p $WORKSPACE/src
WORKDIR $WORKSPACE

# Clone the Easynav project and research repositories
RUN cd src && \
    git clone -b rolling https://github.com/EasyNavigation/EasyNavigation.git  && \
    git clone -b rolling https://github.com/EasyNavigation/NavMap.git  && \
    git clone -b rolling https://github.com/EasyNavigation/easynav_plugins.git  && \
    git clone -b rolling https://github.com/fmrico/yaets.git  && \
    git clone -b rolling https://github.com/EasyNavigation/easynav_playgrounds.git

# Initialize and update rosdep for dependency management
RUN rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Source ROS
RUN . /opt/ros/rolling/setup.sh && \
    colcon build --symlink-install

# Set up the environment to configure automatically
RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc


RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/rolling/setup.bash\n\
source $WORKSPACE/install/setup.bash\n\
ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml > /var/log/easynav.log 2>&1 &\n\
echo "Simulation, EasyNav and rviz started in the background."\n\
exec "$@"' > /usr/local/bin/entrypoint.sh && \
chmod +x /usr/local/bin/entrypoint.sh

# Use the custom script as the entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]