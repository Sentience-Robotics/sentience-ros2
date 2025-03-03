FROM ros:humble

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

RUN if id -u $UID ; then userdel `id -un $UID` ; fi

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install some dependencies packages
RUN apt-get update -q \
    && apt-get upgrade -q -y \
    && apt-get install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    neovim git zsh \
    curl wget \
    libboost-all-dev \
    && apt-get clean

# Setup zsh
RUN touch /home/$USERNAME/~.zshrc
RUN chsh -s /bin/zsh $USERNAME
ENV SHELL=/bin/zsh

USER $USERNAME

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.zsh' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.zsh' >> /home/$USERNAME/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
