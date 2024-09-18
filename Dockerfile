# Use the official ROS Melodic image from Docker Hub
FROM ros:melodic

# Setup sources list
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
RUN sudo apt install curl # if you haven't already installed curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install packages
RUN apt-get update && apt-get install -y \
    python2.7=2.7.17-1ubuntu0.1 \
    python2.7-dev=2.7.17-1ubuntu0.1 
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    ros-melodic-desktop-full \
    openssh-client \
    git

# Initialize rosdep
RUN sudo rosdep init
RUN rosdep update

# Copy the private SSH key into the container
# This will be passed via GitHub secrets or locally
ARG SSH_PRIVATE_KEY
RUN mkdir -p /root/.ssh
RUN echo "$SSH_PRIVATE_KEY" > /root/.ssh/id_rsa && chmod 600 /root/.ssh/id_rsa

# Ensure SSH knows GitHub's fingerprint to avoid prompts
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# Clone the workspace from the GitHub repository
RUN git clone git@github.com:John075/Warehouse-Delivery.git /root/catkin-ws/
WORKDIR /root/catkin_ws

# Build the repository
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Source ROS and catkin workspace on container start
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]
