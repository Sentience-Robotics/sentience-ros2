FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

# Install some dependencies packages
RUN apt update -q && \
    apt upgrade -q -y && \
    apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    git zsh vim curl wget lsb-release gnupg sudo \
    libboost-all-dev && \
    apt-get clean

# Install ROS2 & Gazebo
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update -q && \
    apt upgrade -q -y && \
    apt install -y ros-humble-desktop ros-humble-ros-gz

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Install gazebo
# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
#     apt update -q && \
#     apt install -y ignition-fortress

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Setup zsh
RUN touch /home/$USERNAME/.zshrc
RUN chsh -s /bin/zsh $USERNAME
ENV SHELL=/bin/zsh

USER $USERNAME

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.zsh' >> /home/$USERNAME/.zshrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.zsh' >> /home/$USERNAME/.zshrc

# Setup entrypoint
#COPY ./ros_entrypoint.sh /
#ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
