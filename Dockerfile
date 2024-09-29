# Use the official ROS Melodic image from Docker Hub
FROM ros:melodic

# Define the build argument
ARG SSH_PRIVATE_KEY

# Setup keys
RUN sudo apt-get update && apt-get install -y curl 
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Setup sources list
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Update packages again
RUN sudo apt-get update

# Install other dependency packages
RUN apt-get install -y \
    python2.7 \
    python2.7-dev \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    ros-melodic-desktop-full \
    ros-melodic-geographic-msgs \
    ros-melodic-moveit \
    ros-melodic-moveit-* \
    gazebo9 \
    libgazebo9-dev \
    ros-melodic-gazebo9* \
    openssh-client \
    git \
    gdb \
    xvfb

# Initialize rosdep only if the default sources list does not exist
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi
RUN rosdep update

# Set up SSH to use the private key securely
RUN mkdir -p ~/.ssh && \
    echo "$SSH_PRIVATE_KEY" > ~/.ssh/id_rsa && \
    chmod 600 ~/.ssh/id_rsa && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts

# Clone the workspace from the GitHub repository
ARG CACHE_BUSTER
ARG GIT_CLONE_COMMIT=feature/ci_cd_pipeline
RUN echo "Cache Bust: $CACHE_BUSTER" && git clone -b $GIT_CLONE_COMMIT --recurse-submodules git@github.com:John075/Warehouse-Delivery.git /root/catkin_ws
WORKDIR /root/catkin_ws

# Install any more needed dependencies
RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic

# Build the repository
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Source ROS and catkin workspace on container start
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]
